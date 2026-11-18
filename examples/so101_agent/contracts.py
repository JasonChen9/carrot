"""Hardware-independent contracts. All timestamps use the injected monotonic clock."""

import math
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Protocol

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
UNITS = {name: "percent" if name == "gripper" else "degrees" for name in JOINTS}
CAMERAS = ("top", "wrist")


class AgentError(ValueError):
    """Rejected request; no new motion is started. code is stable for tool callers."""

    def __init__(self, code: str, message: str, status: int = 400):
        super().__init__(message)
        self.code = code
        self.status = status


def number(value: object, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AgentError("invalid_argument", f"{label} must be a finite number")
    try:
        result = float(value)
    except OverflowError as exc:
        raise AgentError("invalid_argument", f"{label} is too large") from exc
    if not math.isfinite(result) or (positive and result <= 0):
        raise AgentError(
            "invalid_argument", f"{label} must be finite" + (" and > 0" if positive else "")
        )
    return result


def identifier(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise AgentError(
            "invalid_argument", f"{label} must be a nonempty string of <=128 characters"
        )
    return value


def pose(values: object, bounds: Mapping[str, tuple[float, float]]) -> dict[str, float]:
    if not isinstance(values, Mapping) or set(values) != set(JOINTS):
        raise AgentError("invalid_pose", f"Expected exactly these joint names: {JOINTS}")
    result = {name: number(values[name], name) for name in JOINTS}
    for name, value in result.items():
        low, high = bounds[name]
        if not low <= value <= high:
            raise AgentError(
                "out_of_bounds", f"{name}={value} outside [{low}, {high}] {UNITS[name]}"
            )
    return result


@dataclass(frozen=True)
class Frame:
    """Encoded image and capture metadata, NOT the time the response was assembled."""

    png: bytes
    captured_at: float
    sequence: int
    width: int
    height: int


class Clock(Protocol):
    def now(self) -> float: ...
    def wait(self, cancelled: threading.Event, seconds: float) -> None: ...


class RealClock:
    def now(self) -> float:
        return time.monotonic()

    def wait(self, cancelled: threading.Event, seconds: float) -> None:
        cancelled.wait(seconds)


class Backend(Protocol):
    """Calls are serialized by AgentController, including camera and hold access.

    read_positions must return measured feedback, not the previous command. Methods
    must have bounded I/O timeouts so hold can acquire the controller lock. Bounds
    and degree/percent conversions must be validated by the hardware adapter.
    No method may silently configure motors, reset the arm or disable torque.
    """

    name: str
    simulated: bool
    motion_enabled: bool
    bounds: Mapping[str, tuple[float, float]]

    def read_positions(self) -> dict[str, float]: ...
    def read_frames(self) -> dict[str, Frame]: ...
    def write_targets(self, targets: dict[str, float]) -> None: ...


@dataclass(frozen=True)
class MotionConfig:
    """Degrees for body joints; percentage points for gripper, including rates.

    These are experimental executor bounds, not a collision or force controller.
    Observation TTL covers model thinking; camera freshness is checked separately.
    """

    hz: float = 20.0
    max_speed: float = 8.0
    max_step: float = 0.5
    max_tracking_error: float = 3.0
    tolerance: float = 0.5
    settle_s: float = 2.0
    stall_s: float = 1.0
    max_duration_s: float = 30.0
    observation_ttl_s: float = 30.0
    frame_ttl_s: float = 1.5
    max_commands: int = 1000

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            number(getattr(self, name), name, positive=True)
        if type(self.max_commands) is not int:
            raise AgentError("invalid_argument", "max_commands must be an integer")
        if self.hz > 1000 or self.max_duration_s > 300 or self.settle_s > 60:
            raise AgentError("invalid_argument", "Executor timing exceeds supported session bounds")
        if self.tolerance >= self.max_tracking_error:
            raise AgentError(
                "invalid_argument", "tolerance must be smaller than max_tracking_error"
            )
