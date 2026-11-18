"""SO101 backend lifecycle, injectable at raw motor/camera boundaries for offline tests."""

from collections.abc import Callable
from typing import Protocol

from .contracts import JOINTS, Frame, pose
from .hardware_config import HardwareConfig


class Motors(Protocol):
    def connect(self) -> None: ...
    def close(self) -> None: ...
    def calibrated(self) -> bool: ...
    def read(self, register: str) -> dict[str, int]: ...
    def write_targets(self, raw: dict[str, int]) -> None: ...
    def enable(self, name: str) -> None: ...


class Cameras(Protocol):
    identities: dict

    def connect(self) -> None: ...
    def close(self) -> None: ...
    def read_frames(self) -> dict[str, Frame]: ...


class Lease(Protocol):
    def acquire(self) -> None: ...
    def close(self) -> None: ...


class SO101Backend:
    """Reuse saved calibration/configuration; connect is motor-read-only by default.

    Motion mode explicitly preloads measured positions, verifies the preload, then
    enables only disabled motors. Shutdown preserves torque. Cameras restore saved
    exposure even in motor-read-only mode. All resources are owned by this session.
    """

    name = "so101"
    simulated = False

    def __init__(
        self,
        config: HardwareConfig,
        *,
        motor_factory: Callable[[HardwareConfig], Motors],
        camera_factory: Callable[[HardwareConfig], Cameras],
        lease_factory: Callable[[str], Lease],
    ):
        self.config = config
        self.bounds = config.calibration.bounds
        self.motor_factory = motor_factory
        self.camera_factory = camera_factory
        self.lease_factory = lease_factory
        self.motors: Motors | None = None
        self.cameras: Cameras | None = None
        self.lease: Lease | None = None
        self.connected = False
        self.motion_enabled = False
        self._closed = False

    @classmethod
    def from_config(cls, config: HardwareConfig) -> "SO101Backend":
        # Imports define adapters only; factories/connect defer hardware operations.
        from .feetech import FeetechDriver, SerialLease
        from .mac_cameras import ProcessCameras

        return cls(
            config,
            motor_factory=FeetechDriver,
            camera_factory=ProcessCameras,
            lease_factory=SerialLease,
        )

    def _motor(self) -> Motors:
        if self.motors is None:
            raise RuntimeError("SO101 motor bus is not connected")
        return self.motors

    def _read(self, register: str) -> dict[str, int]:
        values = self._motor().read(register)
        if set(values) != set(JOINTS) or any(type(v) is not int for v in values.values()):
            raise RuntimeError(f"Invalid {register} response")
        return values

    def _check_status(self) -> None:
        if any(self._read("Status").values()):
            raise RuntimeError("SO101 reports a motor fault")

    def connect(self, *, enable_motion: bool = False) -> None:
        if type(enable_motion) is not bool:
            raise ValueError("enable_motion must be boolean")
        if self.connected or self._closed:
            raise RuntimeError("Create a new backend for each connection session")
        wrote = False
        try:
            self.lease = self.lease_factory(self.config.port)
            self.lease.acquire()
            self.motors = self.motor_factory(self.config)
            self.motors.connect()
            if not self.motors.calibrated():
                raise RuntimeError(
                    "Device calibration differs from the saved file; no repair attempted"
                )
            self._check_status()
            if any(self._read("Operating_Mode").values()):
                raise RuntimeError("All motors must already be in position mode")
            if any(v & 0x10 for v in self._read("Phase").values()):
                raise RuntimeError("Continuous-position feedback is unsupported")
            self.config.calibration.decode(self._read("Present_Position"))
            torque = self._read("Torque_Enable")
            if any(v not in (0, 1) for v in torque.values()):
                raise RuntimeError("Invalid motor torque state")
            self.cameras = self.camera_factory(self.config)
            self.cameras.connect()
            self.cameras.read_frames()  # require both live views before any motion enable
            if enable_motion:
                self._check_status()
                torque = self._read("Torque_Enable")
                if any(v not in (0, 1) for v in torque.values()):
                    raise RuntimeError("Invalid motor torque state")
                current = self._read("Present_Position")
                self.config.calibration.decode(current)
                wrote = True
                self.motors.write_targets(current)
                if self._read("Goal_Position") != current:
                    raise RuntimeError(
                        "Hold preload did not read back correctly; refusing torque enable"
                    )
                for name, enabled in torque.items():
                    if enabled == 0:
                        self.motors.enable(name)
                if any(v != 1 for v in self._read("Torque_Enable").values()):
                    raise RuntimeError("Torque enable verification failed")
            self.connected = True
            self.motion_enabled = enable_motion
        except BaseException as exc:
            if wrote:
                try:
                    current = self._read("Present_Position")
                    self.config.calibration.decode(current)
                    self._motor().write_targets(current)
                except Exception as hold_error:
                    exc.add_note(f"Startup hold failed: {hold_error}")
            try:
                self.close()
            except Exception as close_error:
                exc.add_note(f"Cleanup failed: {close_error}")
            raise

    def read_positions(self) -> dict[str, float]:
        if not self.connected:
            raise RuntimeError("SO101 backend is disconnected")
        self._check_status()
        return self.config.calibration.decode(self._read("Present_Position"))

    def read_frames(self) -> dict[str, Frame]:
        if not self.connected or self.cameras is None:
            raise RuntimeError("SO101 cameras are disconnected")
        return self.cameras.read_frames()

    def write_targets(self, targets: dict[str, float]) -> None:
        if not self.connected or not self.motion_enabled:
            raise RuntimeError("SO101 motor writes are disabled in this session")
        target = pose(targets, self.bounds)
        self._check_status()
        if any(v != 1 for v in self._read("Torque_Enable").values()):
            raise RuntimeError("Torque state changed; refusing to re-enable automatically")
        raw = self.config.calibration.encode(target)
        self._motor().write_targets(raw)

    def close(self) -> None:
        """Release all resources, preserving torque. Controller shutdown must hold first."""
        self.connected = self.motion_enabled = False
        self._closed = True
        errors = []
        for resource in (self.cameras, self.motors, self.lease):
            if resource is not None:
                try:
                    resource.close()
                except Exception as exc:
                    errors.append(exc)
        self.cameras = self.motors = self.lease = None
        if errors:
            raise ExceptionGroup("SO101 resource cleanup failed", errors)
