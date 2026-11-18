"""Synchronous agent tools with interruptible, locally paced joint execution."""

import base64
import copy
import threading
import uuid
from dataclasses import asdict
from typing import Any

from .contracts import (
    CAMERAS,
    JOINTS,
    UNITS,
    AgentError,
    Backend,
    Clock,
    MotionConfig,
    RealClock,
    identifier,
    number,
    pose,
)


class AgentController:
    """Expose observe/move_joints/set_gripper/hold to Python or an HTTP adapter.

    Commands block until settled, failed or cancelled. All backend I/O uses one
    lock, released between ticks so hold can cancel and replace the active target.
    Duplicate IDs are retained for this process lifetime; a restart is a new session.
    A fault latches motion off until a new controller is created; observe/hold remain
    available. This implements joint bounds, not collision/force/Cartesian control.
    """

    def __init__(
        self, backend: Backend, *, clock: Clock | None = None, config: MotionConfig | None = None
    ):
        self.backend = backend
        self.clock = clock or RealClock()
        self.config = config or MotionConfig()
        if set(backend.bounds) != set(JOINTS):
            raise ValueError("Backend must specify all six joint bounds")
        self.bounds = {}
        for name, pair in backend.bounds.items():
            low, high = (number(value, f"{name} bound") for value in pair)
            if low >= high:
                raise ValueError(f"Invalid backend bounds for {name}")
            self.bounds[name] = (low, high)
        if self.bounds["gripper"][0] < 0 or self.bounds["gripper"][1] > 100:
            raise ValueError("Gripper bounds must be within [0, 100]")
        self.session_id = uuid.uuid4().hex
        self._lock = threading.RLock()
        self._revision = 0
        self._observation: tuple[str, float, int] | None = None
        self._observed_positions: dict[str, float] = {}
        self._commands: dict[str, dict[str, Any]] = {}
        self._active: str | None = None
        self._cancelled = threading.Event()
        self._fault: str | None = None

    def metadata(self) -> dict[str, Any]:
        """Session identity and units; reads no device and issues no commands."""
        with self._lock:
            return {
                "session_id": self.session_id,
                "backend": self.backend.name,
                "simulated": self.backend.simulated,
                "units": UNITS.copy(),
                "bounds": copy.deepcopy(self.bounds),
                "limits": asdict(self.config),
                "active_command": self._active,
                "fault": self._fault,
            }

    def _positions(self) -> dict[str, float]:
        return pose(self.backend.read_positions(), self.bounds)

    def observe(self) -> dict[str, Any]:
        """Return measured joints and top/wrist PNGs with capture times and sequences.

        This tool performs no motor writes. A successful observation replaces the
        previous ID. Motion requires that ID, within TTL, with no intervening action.
        Image data is base64 PNG; timestamps are session-local monotonic seconds.
        """
        with self._lock:
            self._observation = None
            try:
                positions = self._positions()
                state_time = self.clock.now()
                frames = self.backend.read_frames()
                if set(frames) != set(CAMERAS):
                    raise ValueError(f"Expected cameras {CAMERAS}")
                now = self.clock.now()
                images = {}
                for name, frame in frames.items():
                    age = now - number(frame.captured_at, f"{name} capture time")
                    if age < 0 or age > self.config.frame_ttl_s:
                        raise ValueError(f"{name} frame age {age:.3f}s is invalid or stale")
                    if (
                        type(frame.sequence) is not int
                        or frame.sequence < 0
                        or type(frame.width) is not int
                        or frame.width <= 0
                        or type(frame.height) is not int
                        or frame.height <= 0
                        or not isinstance(frame.png, bytes)
                        or not frame.png.startswith(b"\x89PNG\r\n\x1a\n")
                    ):
                        raise ValueError(f"Invalid {name} frame metadata or PNG")
                    images[name] = {
                        "mime_type": "image/png",
                        "data_base64": base64.b64encode(frame.png).decode("ascii"),
                        "captured_at_s": frame.captured_at,
                        "age_s": age,
                        "sequence": frame.sequence,
                        "width": frame.width,
                        "height": frame.height,
                    }
                if now - state_time > self.config.frame_ttl_s:
                    raise ValueError("Joint feedback became stale while capturing images")
            except Exception as exc:
                raise AgentError("observation_failed", str(exc), 503) from exc
            observation_id = uuid.uuid4().hex
            self._observation = (
                observation_id,
                min(state_time, *(frame.captured_at for frame in frames.values())),
                self._revision,
            )
            self._observed_positions = positions.copy()
            return {
                **self.metadata(),
                "observation_id": observation_id,
                "positions": positions,
                "state_captured_at_s": state_time,
                "observed_at_s": now,
                "images": images,
                "camera_skew_s": max(f.captured_at for f in frames.values())
                - min(f.captured_at for f in frames.values()),
            }

    def move_joints(
        self, *, target: dict[str, float], duration_s: float, observation_id: str, command_id: str
    ) -> dict[str, Any]:
        """Move to a complete named pose (five degrees + gripper percent).

        duration_s is the minimum nominal path time, with a separate settling
        budget. Invalid, stale, busy, duplicate-conflicting or too-fast requests
        raise AgentError before movement. Runtime failures return a terminal result,
        attempt measured-position hold, and latch a fault; they never report reached.
        """
        target = pose(target, self.bounds)
        return self._move("move_joints", target, duration_s, observation_id, command_id)

    def set_gripper(
        self, *, value: float, duration_s: float, observation_id: str, command_id: str
    ) -> dict[str, Any]:
        """Set calibrated gripper percentage, holding measured body positions.

        Closing against an object can stall before reaching the requested value.
        Neither reached nor stalled establishes grasp success; observe afterwards.
        """
        value = number(value, "gripper")
        low, high = self.bounds["gripper"]
        if not low <= value <= high:
            raise AgentError("out_of_bounds", f"gripper must be within [{low}, {high}] percent")
        return self._move("set_gripper", {"gripper": value}, duration_s, observation_id, command_id)

    def _move(
        self,
        tool: str,
        request: dict[str, float],
        duration: float,
        observation_id: str,
        command_id: str,
    ) -> dict[str, Any]:
        duration = number(duration, "duration_s", positive=True)
        observation_id = identifier(observation_id, "observation_id")
        command_id = identifier(command_id, "command_id")
        if duration > self.config.max_duration_s:
            raise AgentError("duration_limit", "duration_s exceeds max_duration_s")
        fingerprint = (tool, tuple(sorted(request.items())), duration, observation_id)
        with self._lock:
            if command_id in self._commands:
                record = self._commands[command_id]
                if record["fingerprint"] != fingerprint:
                    raise AgentError(
                        "command_conflict", "command_id was used for another request", 409
                    )
                if record["result"] is None:
                    raise AgentError(
                        "command_in_progress", "Original command is still running", 409
                    )
                return copy.deepcopy(record["result"])
            if self._active is not None:
                raise AgentError("busy", "Another command is running", 409)
            if self._fault:
                raise AgentError("faulted", self._fault, 409)
            if len(self._commands) >= self.config.max_commands:
                raise AgentError(
                    "session_full", "Start a new session; command IDs are not evicted", 409
                )
            if (
                self._observation is None
                or self._observation[0] != observation_id
                or self._observation[2] != self._revision
                or self.clock.now() - self._observation[1] > self.config.observation_ttl_s
            ):
                raise AgentError("stale_observation", "Call observe again before moving", 409)
            try:
                start = self._positions()
            except Exception as exc:
                self._fault = f"Cannot read initial feedback: {exc}"
                raise AgentError("backend_failed", self._fault, 503) from exc
            if any(
                abs(start[n] - self._observed_positions[n]) > self.config.max_tracking_error
                for n in JOINTS
            ):
                raise AgentError("stale_observation", "Joint state changed since observation", 409)
            target = {**start, **request}
            if any(abs(target[n] - start[n]) / duration > self.config.max_speed for n in JOINTS):
                raise AgentError("speed_limit", "Increase duration_s to respect max_speed")
            self._commands[command_id] = {"fingerprint": fingerprint, "result": None}
            self._active = command_id
            self._revision += 1
            self._cancelled.clear()
        return self._execute(command_id, start, target, duration)

    def _try_hold(self) -> dict[str, Any]:
        """Caller holds lock. Failure is explicit, and never triggers torque disable."""
        try:
            current = self._positions()
            self.backend.write_targets(current.copy())
            return {"succeeded": True, "target": current}
        except Exception as exc:
            self._fault = f"Hold failed: {exc}"
            return {"succeeded": False, "error": str(exc)}

    def hold(self) -> dict[str, Any]:
        """Interrupt active motion and replace its target with fresh measured joints.

        Returns whether the write succeeded, not a guarantee of a mechanical stop.
        No subsequent trajectory write can occur for the cancelled command. New
        movement requires a fresh observe; hold does not clear a latched fault.
        """
        with self._lock:
            active = self._active
            self._cancelled.set()
            self._revision += 1
            result = self._try_hold()
            return {
                "session_id": self.session_id,
                "simulated": self.backend.simulated,
                "status": "held" if result["succeeded"] else "failed",
                "interrupted_command": active,
                **result,
            }

    def _execute(
        self, command_id: str, start: dict[str, float], target: dict[str, float], duration: float
    ) -> dict[str, Any]:
        began = previous_time = self.clock.now()
        previous = start.copy()
        progress = 0.0
        stable = 0
        trace = []
        last_progress = dict.fromkeys(JOINTS, began)
        progress_position = start.copy()
        result: dict[str, Any] = {
            "session_id": self.session_id,
            "command_id": command_id,
            "simulated": self.backend.simulated,
            "target": target,
            "start": start,
            "status": "failed",
            "trajectory": trace,
        }
        try:
            while True:
                self.clock.wait(self._cancelled, 1 / self.config.hz)
                with self._lock:
                    now = self.clock.now()
                    current = self._positions()
                    result.update(
                        final_position=current,
                        elapsed_s=now - began,
                        error={n: target[n] - current[n] for n in JOINTS},
                    )
                    if self._cancelled.is_set():
                        result["status"] = "cancelled"
                        break
                    if now - began > duration + self.config.settle_s:
                        result["status"] = "timeout"
                        raise TimeoutError("Motion did not settle before the deadline")
                    reached = progress == 1 and all(
                        abs(target[n] - current[n]) <= self.config.tolerance for n in JOINTS
                    )
                    stable = stable + 1 if reached else 0
                    if stable >= 3:
                        result["status"] = "reached"
                        break
                    for name in JOINTS:
                        remaining = abs(target[name] - current[name])
                        improved = abs(target[name] - progress_position[name]) - remaining
                        if (
                            remaining <= self.config.tolerance
                            or abs(previous[name] - current[name]) <= self.config.tolerance
                            or improved >= self.config.tolerance / 2
                        ):
                            last_progress[name] = now
                            progress_position[name] = current[name]
                        elif now - last_progress[name] >= self.config.stall_s:
                            result["status"] = "stalled"
                            raise RuntimeError(f"{name} stopped tracking the commanded trajectory")
                    step = min(self.config.max_step, self.config.max_speed * (now - previous_time))
                    command, progress = self._next_target(
                        start, current, target, progress, step, min(1.0, (now - began) / duration)
                    )
                    self.backend.write_targets(command.copy())
                    trace.append(
                        {
                            "elapsed_s": now - began,
                            "measured": current,
                            "commanded": command,
                            "progress": progress,
                        }
                    )
                    previous, previous_time = command, now
        except BaseException as exc:
            with self._lock:
                self._fault = f"{type(exc).__name__}: {exc}"
                result.update(failure=self._fault, hold=self._try_hold())
                if isinstance(exc, (KeyboardInterrupt, SystemExit)):
                    result["status"] = "cancelled"
                    raise
        finally:
            with self._lock:
                # Preserve the final result even when its HTTP caller disconnected.
                self._commands[command_id]["result"] = copy.deepcopy(result)
                self._active = None
                self._revision += 1
        return copy.deepcopy(result)

    def _next_target(
        self,
        start: dict[str, float],
        current: dict[str, float],
        target: dict[str, float],
        progress: float,
        step: float,
        scheduled: float,
    ) -> tuple[dict[str, float], float]:
        delta = {name: target[name] - start[name] for name in JOINTS}
        distance = max(map(abs, delta.values()))
        low = progress
        high = min(1.0, scheduled, progress + step / distance) if distance else 1.0
        # One shared path parameter: a lagging joint slows the entire trajectory.
        for name in JOINTS:
            error_bound = self.config.max_tracking_error
            if delta[name] == 0:
                if abs(start[name] - current[name]) > error_bound:
                    raise RuntimeError(f"Stationary joint {name} moved outside tracking bound")
                continue
            interval = sorted(
                (current[name] + offset - start[name]) / delta[name]
                for offset in (-error_bound, error_bound)
            )
            low, high = max(low, interval[0]), min(high, interval[1])
            if low > high + 1e-10:
                raise RuntimeError(f"{name} feedback departed from the commanded path")
        following = max(progress, high)
        command = {
            n: target[n] if following == 1 else start[n] + following * delta[n] for n in JOINTS
        }
        return pose(command, self.bounds), following
