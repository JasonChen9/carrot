import base64
import copy
import json
import struct
import subprocess
import sys
import threading
import zlib
from dataclasses import replace

import pytest

from examples.so101_agent import AgentController, MockBackend
from examples.so101_agent.contracts import JOINTS, AgentError, MotionConfig


class Clock:
    def __init__(self):
        self.time = 100.0

    def now(self):
        return self.time

    def wait(self, cancelled, seconds):
        if not cancelled.is_set():
            self.time += seconds


class RecordingBackend(MockBackend):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.writes = []
        self.written = threading.Event()

    def write_targets(self, targets):
        super().write_targets(targets)
        self.writes.append((self.clock.now(), targets.copy()))
        self.written.set()


@pytest.fixture
def rig():
    clock = Clock()
    backend = RecordingBackend(clock=clock)
    return AgentController(backend, clock=clock), backend, clock


def request(controller, **changes):
    observation = controller.observe()
    return {
        "target": {**observation["positions"], **changes},
        "duration_s": 1.0,
        "observation_id": observation["observation_id"],
        "command_id": "move-1",
    }


def test_observe_contains_decodable_distinct_views_and_measured_state(rig):
    controller, backend, clock = rig
    backend.targets["shoulder_pan"] = 20
    observation = controller.observe()
    assert observation["positions"]["shoulder_pan"] == 0  # goal is NOT feedback
    assert observation["simulated"] is True
    assert observation["units"]["shoulder_pan"] == "degrees"
    assert observation["units"]["gripper"] == "percent"
    assert set(observation["images"]) == {"top", "wrist"}
    assert observation["images"]["top"] != observation["images"]["wrist"]
    for frame in observation["images"].values():
        png = base64.b64decode(frame["data_base64"], validate=True)
        assert png[:8] == b"\x89PNG\r\n\x1a\n"
        assert struct.unpack("!II", png[16:24]) == (frame["width"], frame["height"])
        offset, compressed = 8, b""
        while offset < len(png):
            length = struct.unpack("!I", png[offset : offset + 4])[0]
            kind, body = png[offset + 4 : offset + 8], png[offset + 8 : offset + 8 + length]
            crc = struct.unpack("!I", png[offset + 8 + length : offset + 12 + length])[0]
            assert zlib.crc32(kind + body) & 0xFFFFFFFF == crc
            if kind == b"IDAT":
                compressed += body
            offset += length + 12
        assert len(zlib.decompress(compressed)) == 64 * (1 + 96 * 3)
        assert frame["captured_at_s"] == clock.now()
    assert backend.writes == []


def test_motion_reaches_measured_target_with_bounded_rate_and_shared_progress(rig):
    controller, backend, clock = rig
    arguments = request(controller, shoulder_pan=4, shoulder_lift=-2, wrist_roll=1)
    result = controller.move_joints(**arguments)
    assert result["status"] == "reached"
    assert result["elapsed_s"] >= 1.0
    assert result["final_position"] == pytest.approx(arguments["target"])
    previous, stamp = result["start"], 100.0
    for row in result["trajectory"]:
        for name in JOINTS:
            assert abs(row["commanded"][name] - previous[name]) <= 0.5 + 1e-9
            assert (
                abs(row["commanded"][name] - previous[name])
                <= 8 * (100 + row["elapsed_s"] - stamp) + 1e-9
            )
            assert abs(row["commanded"][name] - row["measured"][name]) <= 3 + 1e-9
        assert row["commanded"]["shoulder_lift"] == pytest.approx(
            -row["commanded"]["shoulder_pan"] / 2
        )
        assert row["commanded"]["wrist_roll"] == pytest.approx(row["commanded"]["shoulder_pan"] / 4)
        previous, stamp = row["commanded"], 100 + row["elapsed_s"]
    assert backend.writes


def test_gripper_preserves_five_measured_body_joints(rig):
    controller, backend, _ = rig
    backend.positions.update(shoulder_pan=12, elbow_flex=-7, wrist_roll=30)
    backend.targets = backend.positions.copy()
    observation = controller.observe()
    result = controller.set_gripper(
        value=45, duration_s=1, observation_id=observation["observation_id"], command_id="grip"
    )
    assert result["status"] == "reached"
    assert result["final_position"]["gripper"] == pytest.approx(45)
    for _, written in backend.writes:
        for name in JOINTS[:-1]:
            assert written[name] == observation["positions"][name]


@pytest.mark.parametrize(
    "change,code",
    [
        ({"duration_s": True}, "invalid_argument"),
        ({"duration_s": 0}, "invalid_argument"),
        ({"duration_s": -1}, "invalid_argument"),
        ({"duration_s": float("inf")}, "invalid_argument"),
        ({"duration_s": 31}, "duration_limit"),
        ({"duration_s": 0.01}, "speed_limit"),
        ({"command_id": ""}, "invalid_argument"),
        ({"observation_id": "unknown"}, "stale_observation"),
        ({"target": [0] * 6}, "invalid_pose"),
        ({"target": {"gripper": 50}}, "invalid_pose"),
    ],
)
def test_invalid_requests_do_not_write(rig, change, code):
    controller, backend, _ = rig
    arguments = request(controller, shoulder_pan=4)
    arguments.update(change)
    with pytest.raises(AgentError) as exc:
        controller.move_joints(**arguments)
    assert exc.value.code == code
    assert backend.writes == []


@pytest.mark.parametrize(
    "name,value",
    [
        ("elbow_flex", 91),
        ("gripper", -1),
        ("gripper", 101),
        ("wrist_roll", float("nan")),
        ("shoulder_pan", True),
        ("shoulder_lift", "3"),
    ],
)
def test_bad_joint_values_are_rejected_instead_of_clipped(rig, name, value):
    controller, backend, _ = rig
    with pytest.raises(AgentError):
        controller.move_joints(**request(controller, **{name: value}))
    assert backend.writes == []


@pytest.mark.parametrize("value", [True, float("nan"), "10", -1, 101])
def test_gripper_validation_does_not_write(rig, value):
    controller, backend, _ = rig
    with pytest.raises(AgentError):
        controller.set_gripper(value=value, duration_s=1, observation_id="x", command_id="g")
    assert backend.writes == []


@pytest.mark.parametrize("invalidate", ["expired", "new_observe", "hold", "external_motion"])
def test_stale_observation_blocks_motion(rig, invalidate):
    controller, backend, clock = rig
    arguments = request(controller, shoulder_pan=4)
    if invalidate == "expired":
        clock.time += 31
    elif invalidate == "new_observe":
        controller.observe()
    elif invalidate == "hold":
        controller.hold()
    else:
        backend.positions["elbow_flex"] = 10
        backend.targets = backend.positions.copy()
    before = len(backend.writes)
    with pytest.raises(AgentError, match="observe|observation"):
        controller.move_joints(**arguments)
    assert len(backend.writes) == before


def test_completed_command_invalidates_previous_observation(rig):
    controller, _, _ = rig
    arguments = request(controller, shoulder_pan=2)
    controller.move_joints(**arguments)
    with pytest.raises(AgentError) as exc:
        controller.move_joints(**{**arguments, "command_id": "second"})
    assert exc.value.code == "stale_observation"


def test_identical_retry_returns_original_result_without_reexecution_or_aliasing(rig):
    controller, backend, _ = rig
    arguments = request(controller, shoulder_pan=4)
    first = controller.move_joints(**arguments)
    count = len(backend.writes)
    expected = copy.deepcopy(first)
    first["target"]["shoulder_pan"] = 42
    first["trajectory"].clear()
    controller.observe()  # cached retry remains valid even after a new observation
    assert controller.move_joints(**arguments) == expected
    assert len(backend.writes) == count
    with pytest.raises(AgentError) as exc:
        controller.move_joints(**{**arguments, "duration_s": 2})
    assert exc.value.code == "command_conflict"


def test_stalled_motion_holds_and_faults_without_claiming_success(rig):
    controller, backend, _ = rig
    backend.following_speed = 1e-10
    arguments = request(controller, shoulder_pan=5)
    result = controller.move_joints(**arguments)
    assert result["status"] == "stalled"
    assert result["hold"]["succeeded"] is True
    assert backend.writes[-1][1] == pytest.approx(backend.read_positions())
    count = len(backend.writes)
    assert controller.move_joints(**arguments) == result  # even failures are deduplicated
    assert len(backend.writes) == count
    fresh = request(controller, shoulder_pan=1)
    fresh["command_id"] = "after-stall"
    with pytest.raises(AgentError) as exc:
        controller.move_joints(**fresh)
    assert exc.value.code == "faulted"
    assert controller.hold()["succeeded"] is True
    assert controller.metadata()["fault"]  # hold cannot silently re-enable movement


def test_timeout_uses_measured_feedback_not_acknowledged_target(rig):
    _, backend, clock = rig
    backend.following_speed = 0.2
    controller = AgentController(
        backend, clock=clock, config=MotionConfig(stall_s=10, settle_s=0.2)
    )
    result = controller.move_joints(**request(controller, shoulder_pan=3))
    assert result["status"] == "timeout"
    assert result["final_position"]["shoulder_pan"] < 1
    assert result["hold"]["succeeded"]


def test_blocked_gripper_reports_stall_without_claiming_a_grasp(rig):
    controller, backend, _ = rig
    backend.following_speed = 1e-10
    observation = controller.observe()
    result = controller.set_gripper(
        value=45, duration_s=1, observation_id=observation["observation_id"], command_id="blocked"
    )
    assert result["status"] == "stalled"
    assert result["final_position"]["gripper"] == pytest.approx(50)
    assert result["hold"]["succeeded"]
    assert "grasp_success" not in result
    assert all(row["commanded"]["shoulder_pan"] == 0 for row in result["trajectory"])


def test_initial_feedback_failure_faults_before_any_write(rig, monkeypatch):
    controller, backend, _ = rig
    arguments = request(controller, shoulder_pan=2)

    def broken_read():
        raise OSError("simulated feedback lost")

    monkeypatch.setattr(backend, "read_positions", broken_read)
    with pytest.raises(AgentError) as exc:
        controller.move_joints(**arguments)
    assert exc.value.code == "backend_failed"
    assert controller.metadata()["fault"]
    assert backend.writes == []


def test_keyboard_interrupt_holds_and_releases_active_command(rig, monkeypatch):
    controller, backend, clock = rig
    arguments = request(controller, shoulder_pan=2)
    original = clock.wait

    def interrupted(cancelled, seconds):
        if backend.writes:
            raise KeyboardInterrupt
        original(cancelled, seconds)

    monkeypatch.setattr(clock, "wait", interrupted)
    with pytest.raises(KeyboardInterrupt):
        controller.move_joints(**arguments)
    assert controller.metadata()["active_command"] is None
    assert backend.targets == backend.read_positions()
    assert controller.move_joints(**arguments)["status"] == "cancelled"


@pytest.mark.parametrize("bad_frame", ["missing_camera", "future", "bad_png", "bad_sequence"])
def test_invalid_camera_data_is_not_an_accepted_observation(rig, monkeypatch, bad_frame):
    controller, backend, clock = rig
    frames = backend.read_frames()
    if bad_frame == "missing_camera":
        del frames["wrist"]
    elif bad_frame == "future":
        frames["top"] = replace(frames["top"], captured_at=clock.now() + 1)
    elif bad_frame == "bad_png":
        frames["top"] = replace(frames["top"], png=b"not a png")
    else:
        frames["top"] = replace(frames["top"], sequence=True)
    monkeypatch.setattr(backend, "read_frames", lambda: frames)
    with pytest.raises(AgentError) as exc:
        controller.observe()
    assert exc.value.code == "observation_failed"
    assert backend.writes == []


def test_observation_failure_invalidates_old_id_and_never_writes(rig, monkeypatch):
    controller, backend, clock = rig
    arguments = request(controller, shoulder_pan=2)
    original = backend.read_frames
    monkeypatch.setattr(
        backend,
        "read_frames",
        lambda: {n: replace(f, captured_at=clock.now() - 10) for n, f in original().items()},
    )
    with pytest.raises(AgentError) as exc:
        controller.observe()
    assert exc.value.code == "observation_failed"
    with pytest.raises(AgentError) as exc:
        controller.move_joints(**arguments)
    assert exc.value.code == "stale_observation"
    assert backend.writes == []


def test_write_and_hold_failure_are_reported_and_cached(rig, monkeypatch):
    controller, backend, _ = rig
    arguments = request(controller, shoulder_pan=2)

    def broken_write(target):
        raise OSError("simulated disconnected bus")

    monkeypatch.setattr(backend, "write_targets", broken_write)
    result = controller.move_joints(**arguments)
    assert result["status"] == "failed"
    assert result["hold"]["succeeded"] is False
    assert "disconnected bus" in result["failure"]
    assert controller.move_joints(**arguments) == result
    assert controller.hold()["status"] == "failed"


def test_hold_interrupts_motion_and_prevents_later_trajectory_writes():
    backend = RecordingBackend()
    controller = AgentController(backend)
    arguments = request(controller, shoulder_pan=8)
    results = []
    worker = threading.Thread(target=lambda: results.append(controller.move_joints(**arguments)))
    worker.start()
    try:
        assert backend.written.wait(2)
        with pytest.raises(AgentError) as exc:
            controller.move_joints(**arguments)
        assert exc.value.code == "command_in_progress"
        with pytest.raises(AgentError) as exc:
            controller.move_joints(**{**arguments, "command_id": "other"})
        assert exc.value.code == "busy"
        hold = controller.hold()
        count = len(backend.writes)
        worker.join(2)
        assert not worker.is_alive()
        assert hold["status"] == "held" and hold["interrupted_command"] == "move-1"
        assert results[0]["status"] == "cancelled"
        assert len(backend.writes) == count
        assert backend.targets == hold["target"]
    finally:
        controller.hold()
        worker.join(2)


def test_command_limit_does_not_evict_ids(rig):
    _, backend, clock = rig
    controller = AgentController(backend, clock=clock, config=MotionConfig(max_commands=1))
    arguments = request(controller, shoulder_pan=2)
    result = controller.move_joints(**arguments)
    second = request(controller, shoulder_pan=1)
    second["command_id"] = "second"
    with pytest.raises(AgentError) as exc:
        controller.move_joints(**second)
    assert exc.value.code == "session_full"
    assert controller.move_joints(**arguments) == result


def test_optimized_python_keeps_validation_and_loads_no_hardware_libraries():
    script = """
import json, sys
from examples.so101_agent import AgentController, MockBackend
from examples.so101_agent.contracts import AgentError
c = AgentController(MockBackend())
o = c.observe()
try:
    c.set_gripper(value=True, duration_s=1, observation_id=o['observation_id'], command_id='g')
except AgentError as e:
    print(json.dumps({'code': e.code, 'modules': list(sys.modules)}))
else:
    raise RuntimeError('validation was optimized away')
"""
    result = subprocess.run(
        [sys.executable, "-O", "-c", script], check=True, text=True, capture_output=True
    )
    data = json.loads(result.stdout)
    assert data["code"] == "invalid_argument"
    assert not any(
        name.split(".")[0] in {"lerobot", "cv2", "serial", "ray", "torch"}
        for name in data["modules"]
    )
