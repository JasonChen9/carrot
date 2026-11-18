"""Read existing SO101 teleop, calibration and camera JSON without loading drivers."""

import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .contracts import CAMERAS, JOINTS, number, pose


def read_json(path: Path) -> dict:
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError(f"Duplicate key {key} in {path}")
            result[key] = value
        return result

    value = json.loads(path.read_text(), object_pairs_hook=pairs)
    if not isinstance(value, dict):
        raise ValueError(f"Expected JSON object: {path}")
    return value


@dataclass(frozen=True)
class Calibration:
    """LeRobot 0.6.1 STS3215 calibration; no hardware registers are changed."""

    motors: dict[str, dict[str, int]]

    def __post_init__(self) -> None:
        if set(self.motors) != set(JOINTS):
            raise ValueError("Calibration must contain the six SO101 motors")
        fields = {"id", "drive_mode", "homing_offset", "range_min", "range_max"}
        for index, name in enumerate(JOINTS, 1):
            item = self.motors[name]
            if (
                not isinstance(item, dict)
                or set(item) != fields
                or any(type(v) is not int for v in item.values())
            ):
                raise ValueError(f"Invalid calibration fields: {name}")
            if item["id"] != index or item["drive_mode"] != 0:
                raise ValueError(f"Expected motor ID {index} and drive_mode=0 for {name}")
            if not 0 <= item["range_min"] < item["range_max"] <= 4095:
                raise ValueError(f"Invalid STS3215 range: {name}")
            if abs(item["homing_offset"]) > 4095:
                raise ValueError(f"Invalid homing offset: {name}")

    @property
    def bounds(self) -> dict[str, tuple[float, float]]:
        return {
            name: (0.0, 100.0)
            if name == "gripper"
            else (
                -(v["range_max"] - v["range_min"]) * 180 / 4095,
                (v["range_max"] - v["range_min"]) * 180 / 4095,
            )
            for name, v in self.motors.items()
        }

    def decode(self, raw: dict[str, int]) -> dict[str, float]:
        if set(raw) != set(JOINTS):
            raise ValueError("Incomplete raw motor feedback")
        result = {}
        for name in JOINTS:
            value = raw[name]
            low, high = self.motors[name]["range_min"], self.motors[name]["range_max"]
            # Checking raw values prevents SDK gripper normalization masking an overrun.
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"Raw {name} position {value} outside calibration [{low}, {high}]")
            result[name] = (
                (value - low) * 100 / (high - low)
                if name == "gripper"
                else (value - (low + high) / 2) * 360 / 4095
            )
        return result

    def encode(self, target: dict[str, float]) -> dict[str, int]:
        target = pose(target, self.bounds)
        result = {}
        for name, value in target.items():
            low, high = self.motors[name]["range_min"], self.motors[name]["range_max"]
            # Nearest encoder tick preserves decode/encode holds without downward drift.
            raw = round(
                value / 100 * (high - low) + low
                if name == "gripper"
                else value * 4095 / 360 + (low + high) / 2
            )
            if not low <= raw <= high:
                raise ValueError(f"Converted {name} target outside calibration")
            result[name] = raw
        return result


@dataclass(frozen=True)
class HardwareConfig:
    port: str
    robot_id: str
    retries: int
    calibration_path: Path
    calibration: Calibration
    cameras: dict[str, dict[str, Any]]
    uvc_tool: Path

    @classmethod
    def load(
        cls,
        robot_config: Path,
        camera_settings: Path,
        calibration_dir: Path | None = None,
        uvc_tool: Path | None = None,
    ) -> "HardwareConfig":
        """Read files only. Never enumerate/open devices or invoke uvc-util."""
        robot_config, camera_settings = robot_config.expanduser(), camera_settings.expanduser()
        robot = read_json(robot_config).get("robot")
        if not isinstance(robot, dict) or robot.get("type") != "so101_follower":
            raise ValueError("robot.type must be so101_follower")
        if robot.get("use_degrees") is not True:
            raise ValueError("robot.use_degrees must explicitly be true")
        port, robot_id = robot.get("port"), robot.get("id")
        if not isinstance(port, str) or not port.startswith("/dev/"):
            raise ValueError("robot.port must be an explicit /dev/ serial path")
        if not isinstance(robot_id, str) or not re.fullmatch(r"[\w-]+", robot_id):
            raise ValueError("Invalid robot calibration ID")
        retries = robot.get("num_read_retries", 2)
        if type(retries) is not int or not 0 <= retries <= 3:
            raise ValueError("num_read_retries must be an integer in [0, 3]")
        directory = calibration_dir or robot.get("calibration_dir")
        if not directory:
            raise ValueError("Specify --calibration-dir or robot.calibration_dir")
        calibration_path = Path(directory).expanduser() / f"{robot_id}.json"
        calibration = Calibration(read_json(calibration_path))
        settings = read_json(camera_settings)
        if type(settings.get("version")) is not int or settings["version"] != 1:
            raise ValueError("Camera settings version must be 1")
        cameras = settings.get("cameras")
        if not isinstance(cameras, dict) or set(cameras) != set(CAMERAS):
            raise ValueError("Camera settings must contain exactly top and wrist")
        devices, indices = set(), set()
        for name, camera in cameras.items():
            required = {
                "device_id",
                "index_or_path",
                "width",
                "height",
                "fps",
                "rotation",
                "exposure",
            }
            if not isinstance(camera, dict) or set(camera) != required:
                raise ValueError(f"Unexpected or missing camera settings: {name}")
            device = camera["device_id"]
            if (
                not isinstance(device, str)
                or not re.fullmatch(r"0x[0-9a-fA-F]{4}:0x[0-9a-fA-F]{4}", device)
                or any(int(v, 16) == 0 for v in device.split(":"))
            ):
                raise ValueError(f"Invalid USB identity for {name}")
            for key in ("index_or_path", "width", "height"):
                if type(camera[key]) is not int or camera[key] < (
                    0 if key == "index_or_path" else 1
                ):
                    raise ValueError(f"Invalid {name}.{key}")
            if camera["width"] > 4096 or camera["height"] > 4096:
                raise ValueError("Camera frame dimensions must be <=4096")
            if not 0 < number(camera["fps"], f"{name}.fps") <= 120:
                raise ValueError("Camera FPS must be in (0, 120]")
            if type(camera["rotation"]) is not int or camera["rotation"] not in (0, 90, 180, 270):
                raise ValueError(f"Invalid {name}.rotation")
            exposure = camera["exposure"]
            if not isinstance(exposure, dict) or exposure.get("mode") not in ("auto", "manual"):
                raise ValueError(f"Invalid {name}.exposure")
            fields = {"mode"} if exposure["mode"] == "auto" else {"mode", "exposure_ms"}
            if set(exposure) != fields:
                raise ValueError(f"Invalid {name}.exposure fields")
            if exposure["mode"] == "manual":
                ticks = number(exposure["exposure_ms"], f"{name}.exposure_ms") * 10
                if not 1 <= ticks <= 5000 or abs(ticks - round(ticks)) > 1e-8:
                    raise ValueError("Manual exposure must be 0.1..500 ms in 0.1 ms steps")
            if device.lower() in devices or camera["index_or_path"] in indices:
                raise ValueError("Camera USB identities and configured indices must be distinct")
            devices.add(device.lower())
            indices.add(camera["index_or_path"])
        tool = (uvc_tool or camera_settings.parent / "tools" / "uvc-util").expanduser()
        if not tool.is_file():
            raise ValueError(f"Existing UVC tool not found: {tool}")
        return cls(port, robot_id, retries, calibration_path, calibration, cameras, tool)

    def summary(self) -> dict:
        return {
            "port": self.port,
            "robot_id": self.robot_id,
            "calibration_path": str(self.calibration_path),
            "bounds": self.calibration.bounds,
            "cameras": self.cameras,
            "uvc_tool": str(self.uvc_tool),
            "motor_configuration": "preserve existing PID/mode/calibration",
            "connected": False,
        }
