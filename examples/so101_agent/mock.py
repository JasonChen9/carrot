"""Synthetic joint feedback and PNG views. No serial, camera, SDK or network imports."""

import struct
import zlib

from .contracts import CAMERAS, JOINTS, Clock, Frame, RealClock, number, pose


def _png(positions: dict[str, float], view: int) -> bytes:
    # Two synthetic dashboards; this is intentionally not a rendered physical scene.
    width, height = 96, 64
    pixels = bytearray(bytes((18, 22, 30)) * width * height)
    for index, name in enumerate(JOINTS):
        length = max(1, min(88, int(44 + positions[name] / 5)))
        for y in range(4 + 10 * index, 10 + 10 * index):
            for x in range(4, 4 + length):
                offset = 3 * (y * width + x)
                pixels[offset : offset + 3] = bytes((70, 180, 240) if view == 0 else (240, 160, 70))
    raw = b"".join(b"\0" + pixels[y * width * 3 : (y + 1) * width * 3] for y in range(height))

    def chunk(kind: bytes, body: bytes) -> bytes:
        return (
            struct.pack("!I", len(body))
            + kind
            + body
            + struct.pack("!I", zlib.crc32(kind + body) & 0xFFFFFFFF)
        )

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack("!2I5B", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


class MockBackend:
    """Rate-limited measured positions, independent from last commanded targets.

    Bounds are synthetic, not SO101 calibration. following_speed is a numerical
    mock rate, not an estimate of real actuator dynamics or payload capacity.
    """

    name = "mock"
    simulated = True

    def __init__(self, *, clock: Clock | None = None, following_speed: float = 24.0):
        self.clock = clock or RealClock()
        self.following_speed = number(following_speed, "following_speed", positive=True)
        self.bounds = {name: (-90.0, 90.0) for name in JOINTS}
        self.bounds.update(wrist_roll=(-180.0, 180.0), gripper=(0.0, 100.0))
        self.positions = dict.fromkeys(JOINTS, 0.0)
        self.positions["gripper"] = 50.0
        self.targets = self.positions.copy()
        self.last_update = self.clock.now()
        self.sequence = 0

    def read_positions(self) -> dict[str, float]:
        now = self.clock.now()
        allowance = self.following_speed * max(0.0, now - self.last_update)
        for name in JOINTS:
            delta = self.targets[name] - self.positions[name]
            self.positions[name] += max(-allowance, min(allowance, delta))
        self.last_update = now
        return self.positions.copy()

    def read_frames(self) -> dict[str, Frame]:
        positions = self.read_positions()
        self.sequence += 1
        stamp = self.clock.now()
        return {
            name: Frame(_png(positions, index), stamp, self.sequence, 96, 64)
            for index, name in enumerate(CAMERAS)
        }

    def write_targets(self, targets: dict[str, float]) -> None:
        self.read_positions()
        self.targets = pose(targets, self.bounds)
