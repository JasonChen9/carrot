"""macOS USB camera identity/exposure adapters and isolated continuous capture workers."""

import multiprocessing
import pickle
import re
import subprocess
import sys
import time
from functools import cmp_to_key

from .contracts import CAMERAS, Frame, number
from .hardware_config import HardwareConfig


def match_camera_index(device_id: str, unique_ids: list[str]) -> int:
    """Match VID:PID to one physical USB device; never fall back to saved numeric index."""
    suffix = "".join(part[2:].lower() for part in device_id.split(":"))
    matches = [
        index
        for index, value in enumerate(unique_ids)
        if re.fullmatch(r"0x[0-9a-fA-F]{9,}", value) and value.lower().endswith(suffix)
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"USB camera {device_id} has {len(matches)} matches; expected exactly one"
        )
    return matches[0]


def resolve_camera_index(device_id: str) -> int:
    """Use OpenCV 4.13 AVFoundation's Video+Muxed uniqueID ordering."""
    if sys.platform != "darwin":
        raise RuntimeError("This camera adapter currently supports macOS AVFoundation only")
    import Foundation  # noqa: F401 -- registers Foundation types with PyObjC
    import objc

    namespace = {}
    bundle = objc.loadBundle(
        "AVFoundation", namespace, bundle_path="/System/Library/Frameworks/AVFoundation.framework"
    )
    objc.loadBundleVariables(
        bundle, namespace, [("AVMediaTypeVideo", b"@"), ("AVMediaTypeMuxed", b"@")]
    )
    with objc.autorelease_pool():
        device = objc.lookUpClass("AVCaptureDevice")
        devices = list(device.devicesWithMediaType_(namespace["AVMediaTypeVideo"])) + list(
            device.devicesWithMediaType_(namespace["AVMediaTypeMuxed"])
        )
        devices.sort(key=cmp_to_key(lambda a, b: int(a.uniqueID().compare_(b.uniqueID()))))
        return match_camera_index(device_id, [str(d.uniqueID()) for d in devices])


def apply_exposure(tool, device_id: str, exposure: dict, *, run=subprocess.run) -> dict:
    """Restore the shared settings through the existing uvc-util executable, then verify."""

    def invoke(*arguments):
        result = run(
            [str(tool), "-V", device_id, *arguments],
            capture_output=True,
            text=True,
            timeout=3,
            check=False,
        )
        if result.returncode:
            raise RuntimeError(f"UVC command failed for {device_id}: {result.stderr[:400]}")
        return result.stdout

    wanted = 1 if exposure["mode"] == "manual" else 8
    invoke("-s", f"auto-exposure-mode={wanted}")
    ticks = None
    if wanted == 1:
        time.sleep(0.3)  # device firmware needs the mode write to settle
        ticks = round(exposure["exposure_ms"] * 10)
        invoke("-s", f"exposure-time-abs={ticks}")
    parts = invoke("-o", "auto-exposure-mode", "-o", "exposure-time-abs").split()
    if len(parts) != 2 or any(not re.fullmatch(r"(?:0[xX][0-9a-fA-F]+|[0-9]+)", p) for p in parts):
        raise RuntimeError(f"Invalid exposure readback for {device_id}")
    mode, actual = [int(p, 16 if p.lower().startswith("0x") else 10) for p in parts]
    if mode != wanted or (ticks is not None and ticks != actual):
        raise RuntimeError(f"Exposure readback mismatch for {device_id}: {parts}")
    return {"mode": exposure["mode"], "exposure_ms": actual / 10}


class Capture:
    """One camera; all blocking native calls run in its own spawned process.

    Tests inject cv/resolver/exposure without opening devices. Frame timestamps are
    conservative software read-start times, not hardware exposure timestamps.
    """

    def __init__(
        self, config: dict, tool, *, cv, resolver=resolve_camera_index, exposure=apply_exposure
    ):
        self.config = config
        self.cv = cv
        self.camera = None
        self.sequence = 0
        self.index = resolver(config["device_id"])
        self.readback = None
        try:
            self.camera = cv.VideoCapture(self.index, cv.CAP_AVFOUNDATION)
            if not self.camera.isOpened():
                raise RuntimeError(f"Cannot open USB camera {config['device_id']}")
            for prop, value in (
                (cv.CAP_PROP_FRAME_WIDTH, config["width"]),
                (cv.CAP_PROP_FRAME_HEIGHT, config["height"]),
                (cv.CAP_PROP_FPS, config["fps"]),
            ):
                if not self.camera.set(prop, value):
                    raise RuntimeError(f"Camera rejected setting {prop}={value}")
            actual_fps = number(self.camera.get(cv.CAP_PROP_FPS), "camera FPS", positive=True)
            if abs(actual_fps - config["fps"]) > 0.5:
                raise RuntimeError(f"Camera FPS differs from shared settings: {actual_fps}")
            self.readback = exposure(tool, config["device_id"], config["exposure"])
            # Discard warmup frames; software timestamps cannot prove exposure-time freshness.
            for _ in range(3):
                self.read()
        except BaseException:
            self.close()
            raise

    def read(self) -> Frame:
        stamp = time.monotonic()
        ok, image = self.camera.read()
        if not ok or image is None:
            raise RuntimeError(f"No frame from {self.config['device_id']}")
        if image.shape != (self.config["height"], self.config["width"], 3):
            raise RuntimeError(f"Camera size/channels differ from shared settings: {image.shape}")
        rotation = self.config["rotation"]
        if rotation:
            image = self.cv.rotate(
                image,
                {
                    90: self.cv.ROTATE_90_CLOCKWISE,
                    180: self.cv.ROTATE_180,
                    270: self.cv.ROTATE_90_COUNTERCLOCKWISE,
                }[rotation],
            )
        ok, png = self.cv.imencode(".png", image)
        if not ok:
            raise RuntimeError("Camera PNG encoding failed")
        self.sequence += 1
        return Frame(png.tobytes(), stamp, self.sequence, image.shape[1], image.shape[0])

    def close(self) -> None:
        if self.camera is not None:
            self.camera.release()
            self.camera = None


class FrameSlot:
    """One bounded shared-memory message; never wait on a camera process's lock.

    A multiprocessing Queue can block receiving a partial large pipe message, even
    with get_nowait. This slot instead skips a busy/dead writer and lets frames expire.
    Only our private child process can publish serialized Python objects here.
    """

    def __init__(self, context, capacity: int):
        self.buffer = context.RawArray("B", capacity)
        self.length = context.RawValue("I", 0)
        self.lock = context.Lock()

    def publish(self, message) -> None:
        payload = pickle.dumps(message)
        if len(payload) > len(self.buffer):
            raise RuntimeError("Encoded camera message exceeds its shared-memory slot")
        if not self.lock.acquire(False):
            return
        try:
            memoryview(self.buffer).cast("B")[: len(payload)] = payload
            self.length.value = len(payload)
        finally:
            self.lock.release()

    def take(self):
        if not self.lock.acquire(False):
            return None
        try:
            payload = bytes(memoryview(self.buffer).cast("B")[: self.length.value])
            self.length.value = 0
        finally:
            self.lock.release()
        return pickle.loads(payload) if payload else None


def _capture_worker(config: dict, tool: str, channel, stop) -> None:
    capture = None
    try:
        import cv2

        if tuple(cv2.__version__.split(".")[:2]) != ("4", "13"):
            raise RuntimeError("Camera identity ordering is verified for OpenCV 4.13 only")
        capture = Capture(config, tool, cv=cv2)
        while not stop.is_set():
            began = time.monotonic()
            frame = capture.read()
            channel.publish(
                ("frame", frame, {"index": capture.index, "exposure": capture.readback})
            )
            stop.wait(max(0.0, 1 / config["fps"] - (time.monotonic() - began)))
    except BaseException as exc:
        channel.publish(("error", f"{type(exc).__name__}: {exc}", {}))
    finally:
        if capture is not None:
            capture.close()


class ProcessCameras:
    """Bounded startup and nonblocking cached reads; a stuck camera cannot block hold."""

    def __init__(self, config: HardwareConfig, *, context=None, startup_timeout_s: float = 20):
        self.config = config
        self.context = context or multiprocessing.get_context("spawn")
        self.startup_timeout_s = startup_timeout_s
        self.workers = {}
        self.frames: dict[str, Frame] = {}
        self.identities = {}

    def connect(self) -> None:
        if self.workers:
            raise RuntimeError("Cameras already started")
        try:
            for name in CAMERAS:
                camera = self.config.cameras[name]
                channel = FrameSlot(self.context, camera["width"] * camera["height"] * 4 + 65536)
                stop = self.context.Event()
                process = self.context.Process(
                    target=_capture_worker,
                    args=(self.config.cameras[name], str(self.config.uvc_tool), channel, stop),
                    daemon=True,
                )
                self.workers[name] = (process, channel, stop)
                process.start()
            deadline = time.monotonic() + self.startup_timeout_s
            while set(self.frames) != set(CAMERAS):
                self._drain()
                if time.monotonic() >= deadline:
                    raise TimeoutError("Timed out opening both USB cameras")
                time.sleep(0.02)
            if len({v["index"] for v in self.identities.values()}) != len(CAMERAS):
                raise RuntimeError("Both camera identities resolved to the same index")
            self.read_frames()
        except BaseException as exc:
            try:
                self.close()
            except Exception as cleanup:
                exc.add_note(f"Camera cleanup failed: {cleanup}")
            raise

    def _drain(self) -> None:
        for name, (process, channel, _) in self.workers.items():
            message = channel.take()
            if message is not None:
                kind, payload, metadata = message
                if kind == "error":
                    raise RuntimeError(f"Camera {name}: {payload}")
                if kind != "frame" or not isinstance(payload, Frame):
                    raise RuntimeError(f"Invalid camera worker response: {name}")
                self.frames[name], self.identities[name] = payload, metadata
            if not process.is_alive():
                raise RuntimeError(f"Camera {name} worker exited (code {process.exitcode})")

    def read_frames(self) -> dict[str, Frame]:
        self._drain()
        if set(self.frames) != set(CAMERAS):
            raise RuntimeError("Both cameras must have a frame")
        now = time.monotonic()
        for name, frame in self.frames.items():
            if not 0 <= now - frame.captured_at <= 1.5:
                raise RuntimeError(f"Camera {name} frame is stale")
        return self.frames.copy()

    def close(self) -> None:
        errors = []
        for _, _, stop in self.workers.values():
            stop.set()
        for process, _, _ in self.workers.values():
            try:
                if process.pid is None:  # start may have failed
                    process.close()
                    continue
                process.join(timeout=1)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=1)
                if process.is_alive():
                    process.kill()
                    process.join(timeout=1)
                process.close()
            except Exception as exc:
                errors.append(exc)
        self.workers.clear()
        self.frames.clear()
        self.identities.clear()
        if errors:
            raise ExceptionGroup("Camera worker cleanup failed", errors)
