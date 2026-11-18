"""LeRobot 0.6.1 raw-bus adapter. Construction imports SDKs; connect opens the port."""

import fcntl
import hashlib
import importlib.metadata
import os
import subprocess
import tempfile
import termios
import time
from pathlib import Path
from types import MethodType

from .contracts import JOINTS
from .hardware_config import HardwareConfig


class SerialLease:
    """Cooperative lock plus an open-file check before the OS-exclusive serial open.

    tty/cu aliases share a lease. lsof is fail-closed; kernel TIOCEXCL in the driver
    rejects later non-cooperating opens. Existing programs must be stopped first.
    """

    def __init__(self, port: str):
        self.port = port
        self.stream = None

    def acquire(self) -> None:
        canonical = os.path.realpath(self.port).replace("/dev/cu.", "/dev/tty.")
        digest = hashlib.sha256(canonical.encode()).hexdigest()[:24]
        path = Path(tempfile.gettempdir()) / f"carrot-so101-{os.getuid()}-{digest}.lock"
        fd = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        self.stream = os.fdopen(fd, "a")
        try:
            fcntl.flock(self.stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            aliases = {self.port, canonical, canonical.replace("/dev/tty.", "/dev/cu.")}
            result = subprocess.run(
                ["lsof", "-t", "--", *sorted(aliases)], capture_output=True, text=True, timeout=3
            )
            if result.stdout.strip() or result.returncode not in (0, 1) or result.stderr.strip():
                raise RuntimeError(
                    f"Serial port is busy or cannot be checked: {result.stdout} {result.stderr}"
                )
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        if self.stream is not None:
            self.stream.close()
            self.stream = None


class FeetechDriver:
    """Only raw position/torque writes; never SO101Follower.connect/configure.

    SDK packet reads and serial writes have finite deadlines. Native OS failures
    are still not a hard real-time stop guarantee. No port is opened in __init__.
    """

    def __init__(self, config: HardwareConfig):
        if importlib.metadata.version("lerobot") != "0.6.1":
            raise RuntimeError("This adapter requires the verified LeRobot 0.6.1 interface")
        if importlib.metadata.version("feetech-servo-sdk") != "1.0.0":
            raise RuntimeError("This adapter requires the verified Feetech SDK 1.0.0 transport")
        # Optional driver imports remain behind explicit hardware selection.
        import serial
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus

        self.config = config
        self.bus = FeetechMotorsBus(
            config.port,
            {
                n: Motor(
                    i,
                    "sts3215",
                    MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES,
                )
                for i, n in enumerate(JOINTS, 1)
            },
            {n: MotorCalibration(**v) for n, v in config.calibration.motors.items()},
        )

        def setup_port(handler, baud):
            if handler.is_open:
                handler.closePort()
            connection = serial.Serial(
                port=None, baudrate=handler.baudrate, timeout=0, write_timeout=0.25, exclusive=True
            )
            handler.ser = connection
            try:
                connection.port = handler.port_name
                connection.open()
                fcntl.ioctl(connection.fileno(), termios.TIOCEXCL)
                connection.reset_input_buffer()
                handler.tx_time_per_byte = 10000.0 / handler.baudrate
                handler.is_open = True
                return True
            except BaseException:
                connection.close()
                raise

        handler = self.bus.port_handler
        handler.setupPort = MethodType(setup_port, handler)
        # The SDK clearPort uses drain/flush, which lacks a bounded timeout on POSIX.
        handler.clearPort = lambda: handler.ser.reset_input_buffer()
        handler.getCurrentTime = lambda: time.monotonic() * 1000
        handler.setPacketTimeout = lambda length: handler.setPacketTimeoutMillis(250)

    def connect(self) -> None:
        try:
            self.bus.connect()  # handshake pings IDs/models/firmware; no configuration writes
        except BaseException:
            self.close()
            raise

    def close(self) -> None:
        if self.bus.is_connected:
            self.bus.disconnect(disable_torque=False)

    def calibrated(self) -> bool:
        return self.bus.is_calibrated

    def read(self, register: str) -> dict[str, int]:
        values = self.bus.sync_read(register, normalize=False, num_retry=self.config.retries)
        if set(values) != set(JOINTS) or any(type(v) is not int for v in values.values()):
            raise RuntimeError(f"Invalid raw {register} feedback: {values}")
        return values

    def write_targets(self, raw: dict[str, int]) -> None:
        self.config.calibration.decode(raw)  # range validation even for raw hold preloads
        self.bus.sync_write("Goal_Position", raw, normalize=False, num_retry=0)

    def enable(self, name: str) -> None:
        if name not in JOINTS:
            raise ValueError(f"Unknown motor: {name}")
        # enable_torque also writes the EEPROM Lock register; preserve that setting.
        self.bus.write("Torque_Enable", name, 1, normalize=False, num_retry=0)
