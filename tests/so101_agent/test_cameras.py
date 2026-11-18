import copy
import multiprocessing
import time
from types import SimpleNamespace

import pytest

from examples.so101_agent.contracts import Frame
from examples.so101_agent.mac_cameras import (
    Capture,
    FrameSlot,
    ProcessCameras,
    apply_exposure,
    match_camera_index,
)


@pytest.mark.parametrize(
    "ids,index",
    [
        (["builtin", "0x1234567812340001", "0x9999999912340002"], 1),
        (["0x1234567812340001", "0x9999999912340002"], 0),
    ],
)
def test_usb_identity_follows_reordered_devices(ids, index):
    assert match_camera_index("0x1234:0x0001", ids) == index


@pytest.mark.parametrize(
    "ids", [[], ["builtin-12340001"], ["0x1234567812340001", "0x9999999912340001"]]
)
def test_missing_or_ambiguous_usb_identity_never_uses_numeric_fallback(ids):
    with pytest.raises(RuntimeError, match="matches"):
        match_camera_index("0x1234:0x0001", ids)


@pytest.mark.parametrize("manual,readback", [(True, "1 200\n"), (False, "0x08 0x64\n")])
def test_exposure_restores_and_reads_back_existing_settings(monkeypatch, manual, readback):
    calls = []
    monkeypatch.setattr("examples.so101_agent.mac_cameras.time.sleep", lambda _: None)

    def run(argv, **kwargs):
        calls.append(argv)
        assert kwargs["timeout"] == 3
        return SimpleNamespace(returncode=0, stdout=readback, stderr="")

    settings = {"mode": "manual", "exposure_ms": 20} if manual else {"mode": "auto"}
    result = apply_exposure("/offline/uvc-util", "0x1234:0x0001", settings, run=run)
    assert calls[0][-1] == f"auto-exposure-mode={1 if manual else 8}"
    if manual:
        assert calls[1][-1] == "exposure-time-abs=200"
    assert calls[-1][-4:] == ["-o", "auto-exposure-mode", "-o", "exposure-time-abs"]
    assert result == {"mode": settings["mode"], "exposure_ms": 20 if manual else 10}


@pytest.mark.parametrize("readback", ["1 199", "8 200", "1", "ok 200"])
def test_exposure_mismatch_or_malformed_output_aborts(monkeypatch, readback):
    monkeypatch.setattr("examples.so101_agent.mac_cameras.time.sleep", lambda _: None)

    def run(*a, **kw):
        return SimpleNamespace(returncode=0, stdout=readback, stderr="")

    with pytest.raises(RuntimeError):
        apply_exposure("offline", "0x1234:0x0001", {"mode": "manual", "exposure_ms": 20}, run=run)


@pytest.mark.parametrize("rotation", [0, 90, 180, 270])
def test_capture_uses_resolved_index_and_shared_rotation_without_real_camera(rotation):
    cv2 = pytest.importorskip("cv2")
    np = pytest.importorskip("numpy")
    picture = np.arange(18, dtype=np.uint8).reshape((2, 3, 3))
    calls = []
    camera = SimpleNamespace(
        isOpened=lambda: True,
        set=lambda *a: calls.append(a) or True,
        get=lambda prop: 30,
        read=lambda: (True, picture.copy()),
        release=lambda: calls.append("released"),
    )

    def open_fake(index, backend):
        calls.append(("open_fake", index, backend))
        return camera

    cv = SimpleNamespace(
        **{
            name: getattr(cv2, name)
            for name in (
                "CAP_AVFOUNDATION",
                "CAP_PROP_FRAME_WIDTH",
                "CAP_PROP_FRAME_HEIGHT",
                "CAP_PROP_FPS",
                "ROTATE_90_CLOCKWISE",
                "ROTATE_90_COUNTERCLOCKWISE",
                "ROTATE_180",
                "rotate",
                "imencode",
            )
        },
        VideoCapture=open_fake,
    )
    config = {
        "device_id": "0x1234:0x0001",
        "index_or_path": 0,
        "width": 3,
        "height": 2,
        "fps": 30,
        "rotation": rotation,
        "exposure": {"mode": "auto"},
    }
    capture = Capture(
        config,
        "offline",
        cv=cv,
        resolver=lambda _: 7,
        exposure=lambda *a: {"mode": "auto", "exposure_ms": 10},
    )
    frame = capture.read()
    capture.close()
    assert calls[0] == ("open_fake", 7, cv2.CAP_AVFOUNDATION)
    actual = cv2.imdecode(np.frombuffer(frame.png, dtype=np.uint8), cv2.IMREAD_COLOR)
    expected = np.rot90(picture, k=-rotation // 90)
    assert np.array_equal(actual, expected)
    assert frame.sequence == 4 and (frame.width, frame.height) == (
        expected.shape[1],
        expected.shape[0],
    )
    assert calls[-1] == "released"


def publish_offline(slot):
    slot.publish(("frame", Frame(b"offline", time.monotonic(), 3, 2, 2), {"index": 8}))


@pytest.mark.parametrize("failure", ["open", "setting", "fps", "exposure", "shape"])
def test_capture_initialization_failure_releases_device(failure):
    closed = []
    camera = SimpleNamespace(
        isOpened=lambda: failure != "open",
        set=lambda *a: failure != "setting",
        get=lambda prop: 15 if failure == "fps" else 30,
        read=lambda: (True, SimpleNamespace(shape=(1, 1, 3))),
        release=lambda: closed.append(True),
    )
    cv = SimpleNamespace(
        VideoCapture=lambda *a: camera,
        CAP_AVFOUNDATION=1,
        CAP_PROP_FRAME_WIDTH=2,
        CAP_PROP_FRAME_HEIGHT=3,
        CAP_PROP_FPS=4,
    )
    config = {
        "device_id": "offline",
        "width": 3,
        "height": 2,
        "fps": 30,
        "rotation": 0,
        "exposure": {"mode": "auto"},
    }

    def exposure(*a):
        if failure == "exposure":
            raise RuntimeError("fake exposure failed")
        return {"mode": "auto"}

    with pytest.raises(RuntimeError):
        Capture(config, "offline", cv=cv, resolver=lambda _: 0, exposure=exposure)
    assert closed == [True]


def test_shared_frame_slot_works_across_spawn_and_never_waits_for_busy_writer():
    context = multiprocessing.get_context("spawn")
    slot = FrameSlot(context, 65536)
    worker = context.Process(target=publish_offline, args=(slot,))
    worker.start()
    worker.join(5)
    try:
        assert worker.exitcode == 0
        kind, frame, metadata = slot.take()
        assert kind == "frame" and frame.sequence == 3 and metadata["index"] == 8
        assert slot.take() is None
        slot.lock.acquire()
        try:
            assert slot.take() is None
            slot.publish(("error", "dropped when lock held", {}))
        finally:
            slot.lock.release()
        assert slot.take() is None
    finally:
        if worker.is_alive():
            worker.kill()
            worker.join(2)
        worker.close()


class FakeProcess:
    def __init__(self, kwargs, context):
        self.kwargs, self.context = kwargs, context
        self.pid = None
        self.alive = False
        self.exitcode = None
        self.closed = False
        self.terminated = False

    def start(self):
        self.pid = 123
        self.alive = True
        config, _, channel, _ = self.kwargs["args"]
        if self.context.respond:
            channel.publish(
                (
                    "frame",
                    Frame(b"\x89PNG\r\n\x1a\n", time.monotonic(), 1, 2, 2),
                    {"index": config["index_or_path"]},
                )
            )

    def is_alive(self):
        return self.alive

    def join(self, timeout):
        assert timeout <= 1

    def terminate(self):
        self.terminated = True
        self.alive = False

    def kill(self):
        self.alive = False

    def close(self):
        self.closed = True


class Context:
    def __init__(self, respond=True):
        real = multiprocessing.get_context("spawn")
        self.RawArray, self.RawValue, self.Lock, self.Event = (
            real.RawArray,
            real.RawValue,
            real.Lock,
            real.Event,
        )
        self.respond = respond
        self.processes = []

    def Process(self, **kwargs):
        process = FakeProcess(kwargs, self)
        self.processes.append(process)
        return process


@pytest.fixture
def config():
    return SimpleNamespace(
        uvc_tool="offline",
        cameras={
            name: {"index_or_path": index, "width": 2, "height": 2}
            for index, name in enumerate(("top", "wrist"))
        },
    )


def test_stuck_camera_expires_without_blocking_and_workers_are_reaped(config):
    context = Context()
    cameras = ProcessCameras(config, context=context)
    assert not context.processes
    cameras.connect()
    try:
        assert set(cameras.read_frames()) == {"top", "wrist"}
        # Writer could be permanently stuck inside native VideoCapture.read or while publishing.
        _, channel, _ = cameras.workers["wrist"]
        channel.lock.acquire()
        try:
            cameras.frames["wrist"] = Frame(b"offline", time.monotonic() - 2, 1, 2, 2)
            with pytest.raises(RuntimeError, match="stale"):
                cameras.read_frames()
        finally:
            channel.lock.release()
    finally:
        cameras.close()
    assert all(p.closed and p.terminated for p in context.processes)
    assert not cameras.workers


def test_camera_startup_deadline_cleans_up_both_workers(config):
    context = Context(respond=False)
    cameras = ProcessCameras(config, context=context, startup_timeout_s=0.01)
    with pytest.raises(TimeoutError):
        cameras.connect()
    assert len(context.processes) == 2 and all(p.closed for p in context.processes)


@pytest.mark.parametrize("failure", ["same_index", "dead", "error"])
def test_camera_worker_failures_are_explicit(config, failure):
    context = Context()
    if failure == "same_index":
        config = copy.deepcopy(config)
        config.cameras["wrist"]["index_or_path"] = 0
    cameras = ProcessCameras(config, context=context)
    try:
        if failure == "same_index":
            with pytest.raises(RuntimeError, match="same index"):
                cameras.connect()
        else:
            cameras.connect()
            process, channel, _ = cameras.workers["wrist"]
            if failure == "dead":
                process.alive = False
            else:
                channel.publish(("error", "fake unplug", {}))
            with pytest.raises(RuntimeError, match="exited|fake unplug"):
                cameras.read_frames()
    finally:
        cameras.close()
