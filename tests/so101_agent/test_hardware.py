"""Raw bus and saved-file contracts, with all device boundaries replaced by fakes."""

import json
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest

from examples.so101_agent import AgentController, MockBackend
from examples.so101_agent.contracts import JOINTS, AgentError
from examples.so101_agent.hardware import SO101Backend
from examples.so101_agent.hardware_config import HardwareConfig


@pytest.fixture
def files(tmp_path):
    robot = tmp_path / "robot.json"
    cameras = tmp_path / "cameras.json"
    calibration = tmp_path / "offline_arm.json"
    tool = tmp_path / "tools" / "uvc-util"
    tool.parent.mkdir()
    tool.write_text("offline placeholder; must never execute")
    robot.write_text(
        json.dumps(
            {
                "robot": {
                    "type": "so101_follower",
                    "port": "/dev/offline-test-only",
                    "id": "offline_arm",
                    "use_degrees": True,
                    "num_read_retries": 2,
                }
            }
        )
    )
    cameras.write_text(
        json.dumps(
            {
                "version": 1,
                "cameras": {
                    name: {
                        "index_or_path": index,
                        "device_id": f"0x1234:0x000{index + 1}",
                        "width": 640,
                        "height": 480,
                        "fps": 30,
                        "rotation": 180 if index == 0 else 0,
                        "exposure": {"mode": "manual", "exposure_ms": 20}
                        if index == 0
                        else {"mode": "auto"},
                    }
                    for index, name in enumerate(("top", "wrist"))
                },
            }
        )
    )
    calibration.write_text(
        json.dumps(
            {
                name: {
                    "id": index,
                    "drive_mode": 0,
                    "homing_offset": 100 - index,
                    "range_min": 1000 + index,
                    "range_max": 3000 - index,
                }
                for index, name in enumerate(JOINTS, 1)
            }
        )
    )
    return robot, cameras, calibration, tool


def load(files):
    robot, cameras, calibration, _ = files
    return HardwareConfig.load(robot, cameras, calibration.parent)


class Motors:
    def __init__(self, config, events):
        self.events = events
        self.raw = dict.fromkeys(JOINTS, 2000)
        self.goal = self.raw.copy()
        self.registers = {
            r: dict.fromkeys(JOINTS, 0)
            for r in ("Status", "Operating_Mode", "Phase", "Torque_Enable")
        }
        self.registers["Torque_Enable"]["shoulder_pan"] = 1
        self.matches = True
        self.writes = []
        self.written = threading.Event()
        self.closed = False
        self.bad_readback = False

    def connect(self):
        self.events.append("bus_open")

    def calibrated(self):
        return self.matches

    def read(self, register):
        if self.closed:
            raise AssertionError("bus read after close")
        self.events.append(register)
        if register == "Present_Position":
            return self.raw.copy()
        if register == "Goal_Position":
            return {**self.goal, "gripper": 1001} if self.bad_readback else self.goal.copy()
        return self.registers[register].copy()

    def write_targets(self, raw):
        if self.closed:
            raise AssertionError("bus write after close")
        self.events.append("write_goal")
        self.goal = self.raw = raw.copy()  # ideal encoder-following bus, still quantized
        self.writes.append(raw.copy())
        self.written.set()

    def enable(self, name):
        self.events.append(f"enable:{name}")
        self.registers["Torque_Enable"][name] = 1

    def close(self):
        self.events.append("bus_close")
        self.closed = True


class Cameras:
    def __init__(self, events):
        self.events = events
        self.clock = SimpleNamespace(now=time.monotonic)
        self.fail = False
        self.identities = {"top": {"index": 1}, "wrist": {"index": 0}}

    def connect(self):
        self.events.append("cameras_open")
        if self.fail:
            raise OSError("fake camera unplugged")

    def read_frames(self):
        self.events.append("camera_frames")
        return MockBackend(clock=self.clock).read_frames()

    def close(self):
        self.events.append("cameras_close")


@pytest.fixture
def rig(files):
    config = load(files)
    events = []
    motors, cameras = Motors(config, events), Cameras(events)
    lease = SimpleNamespace(
        acquire=lambda: events.append("lease"), close=lambda: events.append("release")
    )
    backend = SO101Backend(
        config,
        motor_factory=lambda c: motors,
        camera_factory=lambda c: cameras,
        lease_factory=lambda p: lease,
    )
    yield backend, motors, cameras, events
    backend.close()


def test_config_reads_existing_files_without_changing_them(files):
    before = [p.read_bytes() for p in files]
    config = load(files)
    assert config.robot_id == "offline_arm"
    assert config.cameras["top"]["rotation"] == 180
    assert config.cameras["wrist"]["exposure"] == {"mode": "auto"}
    assert config.summary()["connected"] is False
    assert before == [p.read_bytes() for p in files]


@pytest.mark.parametrize(
    "file_index,path,value",
    [
        (0, ["robot", "use_degrees"], False),
        (0, ["robot", "num_read_retries"], True),
        (0, ["robot", "id"], "../other"),
        (0, ["robot", "type"], "so100_follower"),
        (1, ["cameras", "top", "device_id"], "0x1234:0x0002"),
        (1, ["cameras", "top", "index_or_path"], 1),
        (1, ["cameras", "top", "rotation"], True),
        (1, ["cameras", "top", "fps"], float("nan")),
        (1, ["cameras", "top", "exposure", "exposure_ms"], 20.05),
        (2, ["gripper", "range_min"], -1),
        (2, ["gripper", "range_max"], 100),
        (2, ["gripper", "id"], 5),
        (2, ["wrist_roll", "drive_mode"], 1),
    ],
)
def test_invalid_shared_config_is_rejected(files, file_index, path, value):
    document = json.loads(files[file_index].read_text())
    item = document
    for key in path[:-1]:
        item = item[key]
    item[path[-1]] = value
    files[file_index].write_text(json.dumps(document))
    with pytest.raises(ValueError):
        load(files)


def test_duplicate_config_fields_are_rejected(files):
    files[0].write_text('{"robot": {}, "robot": {}}')
    with pytest.raises(ValueError, match="Duplicate"):
        load(files)


def test_calibration_round_trips_every_in_range_tick_without_hold_drift(files):
    calibration = load(files).calibration
    for offset in range(2001):
        raw = {
            n: min(v["range_max"], v["range_min"] + offset) for n, v in calibration.motors.items()
        }
        assert calibration.encode(calibration.decode(raw)) == raw
    with pytest.raises(ValueError, match="gripper"):
        calibration.decode({**raw, "gripper": calibration.motors["gripper"]["range_max"] + 1})


def test_default_connection_observation_and_shutdown_write_no_motors(rig):
    backend, motors, _, events = rig
    assert not events  # construction has no device side effects
    backend.connect()
    controller = AgentController(backend)
    observed = controller.observe()
    assert not observed["simulated"] and not observed["motion_enabled"]
    assert set(observed["images"]) == {"top", "wrist"}
    with pytest.raises(AgentError, match="disabled"):
        controller.hold()
    with pytest.raises(AgentError, match="disabled"):
        controller.set_gripper(
            value=49, duration_s=1, observation_id=observed["observation_id"], command_id="g"
        )
    with pytest.raises(RuntimeError, match="disabled"):
        backend.write_targets(observed["positions"])
    assert controller.shutdown()["hold"] == {"skipped": "read_only"}
    backend.close()
    assert motors.writes == []
    assert not any(event.startswith("enable:") for event in events)
    assert events[-3:] == ["cameras_close", "bus_close", "release"]


def test_enable_requires_live_cameras_then_raw_hold_readback_before_torque(rig):
    backend, motors, _, events = rig
    backend.connect(enable_motion=True)
    assert motors.writes == [dict.fromkeys(JOINTS, 2000)]
    assert events.index("camera_frames") < events.index("write_goal")
    assert (
        events.index("write_goal") < events.index("Goal_Position") < events.index("enable:gripper")
    )
    assert "enable:shoulder_pan" not in events  # already enabled; never toggle it
    backend.close()
    assert all(v == 1 for v in motors.registers["Torque_Enable"].values())


@pytest.mark.parametrize("failure", ["calibration", "mode", "phase", "status", "raw", "camera"])
def test_preflight_failures_close_resources_without_any_motor_write(rig, failure):
    backend, motors, cameras, events = rig
    if failure == "calibration":
        motors.matches = False
    elif failure == "mode":
        motors.registers["Operating_Mode"]["gripper"] = 1
    elif failure == "phase":
        motors.registers["Phase"]["gripper"] = 0x10
    elif failure == "status":
        motors.registers["Status"]["gripper"] = 4
    elif failure == "raw":
        motors.raw["gripper"] = 4095
    else:
        cameras.fail = True
    with pytest.raises((ValueError, RuntimeError, OSError)):
        backend.connect(enable_motion=True)
    assert not backend.connected and motors.closed and events[-1] == "release"
    assert not motors.writes and not any(e.startswith("enable:") for e in events)


def test_failed_preload_readback_never_enables_torque(rig):
    backend, motors, _, events = rig
    motors.bad_readback = True
    with pytest.raises(RuntimeError, match="preload"):
        backend.connect(enable_motion=True)
    assert not any(e.startswith("enable:") for e in events)
    assert motors.writes == [dict.fromkeys(JOINTS, 2000)] * 2  # initial + recovery hold


def test_torque_loss_is_not_silently_reenabled_and_faults_the_controller(rig):
    backend, motors, _, events = rig
    backend.connect(enable_motion=True)
    controller = AgentController(backend)
    observed = controller.observe()
    motors.registers["Torque_Enable"]["gripper"] = 0
    before = events.count("enable:gripper"), len(motors.writes)
    result = controller.set_gripper(
        value=49, duration_s=1, observation_id=observed["observation_id"], command_id="g"
    )
    assert result["status"] == "failed" and not result["hold"]["succeeded"]
    assert "Torque state changed" in result["failure"]
    assert (events.count("enable:gripper"), len(motors.writes)) == before


def test_four_tools_through_raw_hardware_adapter(rig):
    backend, motors, cameras, _ = rig

    class Clock:
        value = 100.0

        def now(self):
            return self.value

        def wait(self, cancelled, seconds):
            self.value += seconds

    clock = Clock()
    cameras.clock = clock
    backend.connect(enable_motion=True)
    controller = AgentController(backend, clock=clock)
    observed = controller.observe()
    result = controller.move_joints(
        target={**observed["positions"], "shoulder_pan": 2},
        duration_s=1,
        observation_id=observed["observation_id"],
        command_id="m",
    )
    assert result["status"] == "reached"
    assert result["final_position"]["shoulder_pan"] == pytest.approx(2, abs=360 / 4095 / 2)
    observed = controller.observe()
    before = motors.raw.copy()
    result = controller.set_gripper(
        value=45, duration_s=1, observation_id=observed["observation_id"], command_id="g"
    )
    assert result["status"] == "reached"
    assert result["final_position"]["gripper"] == pytest.approx(45, abs=0.03)
    assert {n: motors.raw[n] for n in JOINTS[:-1]} == {n: before[n] for n in JOINTS[:-1]}
    raw = motors.raw.copy()
    for _ in range(5):
        assert controller.hold()["status"] == "held"
    assert motors.raw == raw


def test_shutdown_cancels_active_motion_and_allows_no_io_after_closing(rig):
    backend, motors, _, events = rig
    backend.connect(enable_motion=True)
    motors.written.clear()
    controller = AgentController(backend)
    observed = controller.observe()
    results = []
    worker = threading.Thread(
        target=lambda: results.append(
            controller.set_gripper(
                value=45, duration_s=2, observation_id=observed["observation_id"], command_id="g"
            )
        )
    )
    worker.start()
    try:
        assert motors.written.wait(2)
        shutdown = controller.shutdown()
        backend.close()
        count = len(events)
        worker.join(2)
        assert not worker.is_alive() and results[0]["status"] == "cancelled"
        assert len(events) == count and controller.shutdown() == shutdown
        for method in (controller.hold, controller.observe):
            with pytest.raises(AgentError, match="shut down"):
                method()
        assert controller.metadata()["closed"] and not controller.metadata()["motion_enabled"]
    finally:
        controller.shutdown()
        worker.join(2)


def test_cli_check_config_loads_no_sdk_and_opens_no_devices(files):
    script = """
import sys, json, importlib.abc
class Deny(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, *args):
        if fullname.split('.')[0] in {'serial', 'lerobot', 'cv2', 'objc', 'Foundation'}:
            raise RuntimeError('hardware import forbidden: ' + fullname)
sys.meta_path.insert(0, Deny())
from examples.so101_agent.server import main
main()
"""
    result = subprocess.run(
        [
            sys.executable,
            "-O",
            "-c",
            script,
            "--backend",
            "so101",
            "--robot-config",
            str(files[0]),
            "--camera-settings",
            str(files[1]),
            "--calibration-dir",
            str(files[2].parent),
            "--check-config",
        ],
        check=True,
        text=True,
        capture_output=True,
        timeout=10,
    )
    assert json.loads(result.stdout)["connected"] is False


@pytest.mark.parametrize("enable_motion", [False, True])
def test_server_lifecycle_closes_command_gate_before_bus_and_journals_mode(
    rig, files, monkeypatch, tmp_path, capsys, enable_motion
):
    from examples.so101_agent import server

    backend, motors, _, events = rig
    monkeypatch.setattr(SO101Backend, "from_config", lambda config: backend)
    controllers = []

    def serve(service):
        controllers.append(service.controller)
        assert service.controller.observe()["motion_enabled"] is enable_motion
        raise KeyboardInterrupt

    monkeypatch.setattr(server.AgentServer, "serve_forever", serve)
    output = tmp_path / "session"
    argv = [
        "agent",
        "--backend",
        "so101",
        "--robot-config",
        str(files[0]),
        "--camera-settings",
        str(files[1]),
        "--calibration-dir",
        str(files[2].parent),
        "--port",
        "0",
        "--output-dir",
        str(output),
    ]
    monkeypatch.setattr(sys, "argv", argv + (["--enable-motion"] if enable_motion else []))
    server.main()
    assert json.loads(capsys.readouterr().out)["motion_enabled"] is enable_motion
    assert controllers[0].metadata()["closed"] and motors.closed
    records = [json.loads(line) for line in (output / "events.jsonl").read_text().splitlines()]
    assert [r["tool"] for r in records] == ["session", "shutdown"]
    if enable_motion:
        assert len(motors.writes) == 2  # preload + shutdown hold
        assert events.index("bus_close") > max(i for i, e in enumerate(events) if e == "write_goal")
    else:
        assert not motors.writes


def test_listen_port_conflict_never_connects_devices(rig, files, monkeypatch, tmp_path):
    from examples.so101_agent import server

    backend, _, _, events = rig
    monkeypatch.setattr(SO101Backend, "from_config", lambda config: backend)

    def busy(*a, **kw):
        raise OSError("fake listen port busy")

    monkeypatch.setattr(server, "AgentServer", busy)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "agent",
            "--backend",
            "so101",
            "--robot-config",
            str(files[0]),
            "--camera-settings",
            str(files[1]),
            "--calibration-dir",
            str(files[2].parent),
            "--enable-motion",
            "--output-dir",
            str(tmp_path / "session"),
        ],
    )
    with pytest.raises(OSError, match="listen port busy"):
        server.main()
    assert events == []


def test_raw_driver_matches_installed_sdk_without_opening_serial(files, monkeypatch):
    serial = pytest.importorskip("serial")
    pytest.importorskip("lerobot.motors.feetech")
    from examples.so101_agent.feetech import FeetechDriver

    def forbidden(*args, **kwargs):
        raise AssertionError("Real serial open forbidden in offline tests")

    monkeypatch.setattr(serial.Serial, "open", forbidden)
    config = load(files)
    driver = FeetechDriver(config)
    raw = dict.fromkeys(JOINTS, 2197)
    normalized = driver.bus._normalize({i: raw[n] for i, n in enumerate(JOINTS, 1)})
    assert config.calibration.decode(raw) == pytest.approx(
        {n: normalized[i] for i, n in enumerate(JOINTS, 1)}
    )
    writes = []
    monkeypatch.setattr(driver.bus, "sync_write", lambda *a, **kw: writes.append((a, kw)))
    monkeypatch.setattr(driver.bus, "write", lambda *a, **kw: writes.append((a, kw)))
    driver.write_targets(raw)
    driver.enable("gripper")
    assert writes == [
        (("Goal_Position", raw), {"normalize": False, "num_retry": 0}),
        (("Torque_Enable", "gripper", 1), {"normalize": False, "num_retry": 0}),
    ]
    driver.close()
    assert not driver.bus.is_connected


def test_driver_serial_open_has_deadlines_exclusivity_and_no_drain(files, monkeypatch):
    serial = pytest.importorskip("serial")
    pytest.importorskip("lerobot.motors.feetech")
    from examples.so101_agent import feetech

    calls = []

    class Serial:
        def __init__(self, **kwargs):
            calls.append(kwargs)

        def open(self):
            calls.append("open_fake")

        def fileno(self):
            return 123456

        def reset_input_buffer(self):
            calls.append("clear_fake")

        def close(self):
            calls.append("close_fake")

    monkeypatch.setattr(serial, "Serial", Serial)
    monkeypatch.setattr(feetech.fcntl, "ioctl", lambda *a: calls.append(a))
    driver = feetech.FeetechDriver(load(files))
    handler = driver.bus.port_handler
    assert handler.setupPort(1000000)
    assert calls[0]["port"] is None and calls[0]["write_timeout"] == 0.25
    assert calls[0]["exclusive"] is True and calls[0]["timeout"] == 0
    handler.clearPort()
    handler.setPacketTimeout(10)
    assert handler.packet_timeout == 250
    driver.close()
    assert calls[-1] == "close_fake"


def test_serial_lease_is_alias_shared_and_busy_port_check_is_mocked(monkeypatch, tmp_path):
    from examples.so101_agent import feetech

    monkeypatch.setattr(feetech.tempfile, "gettempdir", lambda: str(tmp_path))
    calls = []

    def run(*a, **kw):
        calls.append(a)
        return SimpleNamespace(stdout="", stderr="", returncode=1)

    monkeypatch.setattr(feetech.subprocess, "run", run)
    lease = feetech.SerialLease("/dev/tty.fake-offline")
    other = feetech.SerialLease("/dev/cu.fake-offline")
    try:
        lease.acquire()
        with pytest.raises(BlockingIOError):
            other.acquire()
        assert len(calls) == 1
    finally:
        lease.close()
        other.close()
    monkeypatch.setattr(
        feetech.subprocess,
        "run",
        lambda *a, **kw: SimpleNamespace(stdout="1234", stderr="", returncode=0),
    )
    with pytest.raises(RuntimeError, match="busy"):
        lease.acquire()
    assert lease.stream is None
