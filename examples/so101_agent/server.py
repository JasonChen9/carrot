"""Loopback HTTP transport with mock and explicitly selected SO101 backends."""

import argparse
import json
import signal
import threading
from contextlib import ExitStack, suppress
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from .contracts import JOINTS, AgentError
from .controller import AgentController
from .mock import MockBackend

MAX_BODY = 64 * 1024


def tool_schemas() -> list[dict[str, Any]]:
    """JSON Schema tool definitions; physical units are fixed by the controller."""
    common = {
        "duration_s": {"type": "number", "exclusiveMinimum": 0},
        "observation_id": {"type": "string", "minLength": 1, "maxLength": 128},
        "command_id": {"type": "string", "minLength": 1, "maxLength": 128},
    }
    target = {
        "type": "object",
        "additionalProperties": False,
        "required": list(JOINTS),
        "properties": {n: {"type": "number"} for n in JOINTS},
    }
    definitions = [
        ("observe", "Read measured joints and two PNG views; writes no motors.", {}),
        (
            "move_joints",
            "Move five joints in degrees and gripper in percent; wait for feedback.",
            {**common, "target": target},
        ),
        (
            "set_gripper",
            "Set gripper percent while holding measured body joints.",
            {**common, "value": {"type": "number", "minimum": 0, "maximum": 100}},
        ),
        (
            "hold",
            "Interrupt motion and attempt measured-position hold; does not disable torque.",
            {},
        ),
    ]
    return [
        {
            "name": name,
            "description": description,
            "input_schema": {
                "type": "object",
                "properties": properties,
                "required": list(properties),
                "additionalProperties": False,
            },
        }
        for name, description, properties in definitions
    ]


def _object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def _constant(value: str) -> None:
    raise ValueError(f"Non-finite JSON number: {value}")


class AgentServer(ThreadingHTTPServer):
    """Threaded so hold can run while move_joints waits. Bind only IPv4 loopback."""

    daemon_threads = False

    def __init__(self, controller: AgentController, port: int = 0, journal: Path | None = None):
        self.controller = controller
        self.journal = journal
        self.journal_lock = threading.Lock()
        super().__init__(("127.0.0.1", port), Handler)

    def record(self, tool: str, arguments: dict[str, Any], result: dict[str, Any]) -> None:
        if self.journal is not None:
            with self.journal_lock, self.journal.open("a", encoding="utf-8") as stream:
                stream.write(
                    json.dumps(
                        {"tool": tool, "arguments": arguments, "result": result}, allow_nan=False
                    )
                    + "\n"
                )


class Handler(BaseHTTPRequestHandler):
    server: AgentServer

    def setup(self) -> None:
        super().setup()
        self.connection.settimeout(5.0)

    def log_message(self, format: str, *args: Any) -> None:
        # Responses and motion traces are recorded in the optional JSONL journal.
        pass

    def _reply(self, status: int, value: dict[str, Any]) -> None:
        body = json.dumps(value, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        # Caller disconnection does not retry a command or erase its cached outcome.
        with suppress(BrokenPipeError, ConnectionResetError, TimeoutError):
            self.wfile.write(body)

    def _local(self) -> None:
        expected = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
        if self.headers.get("Origin") is not None or self.headers.get("Host") not in expected:
            raise AgentError(
                "forbidden", "Use a direct loopback client without an Origin header", 403
            )

    def do_GET(self) -> None:
        try:
            self._local()
            if self.path == "/health":
                self._reply(200, self.server.controller.metadata())
            elif self.path == "/tools":
                self._reply(200, {"tools": tool_schemas()})
            else:
                raise AgentError("not_found", "Unknown endpoint", 404)
        except AgentError as exc:
            self._reply(exc.status, {"error": {"code": exc.code, "message": str(exc)}})

    def do_POST(self) -> None:
        try:
            self._local()
            tools = {
                "/observe": self.server.controller.observe,
                "/move_joints": self.server.controller.move_joints,
                "/set_gripper": self.server.controller.set_gripper,
                "/hold": self.server.controller.hold,
            }
            if self.path not in tools:
                raise AgentError("not_found", "Unknown tool", 404)
            if (
                self.headers.get_content_type() != "application/json"
                or self.headers.get("Transfer-Encoding") is not None
            ):
                raise AgentError("invalid_request", "Use application/json with Content-Length")
            lengths = self.headers.get_all("Content-Length", [])
            if len(lengths) != 1 or not lengths[0].isdigit():
                raise AgentError("invalid_request", "Exactly one Content-Length is required")
            size = int(lengths[0])
            if not 0 < size <= MAX_BODY:
                raise AgentError("body_limit", f"Body must contain 1..{MAX_BODY} bytes", 413)
            raw = self.rfile.read(size)
            if len(raw) != size:
                raise AgentError("invalid_request", "Incomplete request body")
            try:
                arguments = json.loads(raw, parse_constant=_constant, object_pairs_hook=_object)
            except (ValueError, UnicodeError, RecursionError) as exc:
                raise AgentError("invalid_json", str(exc)) from exc
            required = next(
                t["input_schema"]["required"]
                for t in tool_schemas()
                if "/" + t["name"] == self.path
            )
            if not isinstance(arguments, dict) or set(arguments) != set(required):
                raise AgentError("invalid_argument", f"Expected exactly fields {required}")
            result = tools[self.path](**arguments)
            self.server.record(self.path[1:], arguments, result)
            self._reply(200, result)
        except AgentError as exc:
            self._reply(exc.status, {"error": {"code": exc.code, "message": str(exc)}})
        except Exception as exc:
            self._reply(500, {"error": {"code": "internal_error", "message": str(exc)}})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=["mock", "so101"], default="mock")
    parser.add_argument(
        "--port", type=int, default=8767, help="Loopback port; 0 selects a free port"
    )
    parser.add_argument(
        "--output-dir", type=Path, help="New directory for events.jsonl (required to serve)"
    )
    parser.add_argument("--robot-config", type=Path, help="Existing teleop/config.json")
    parser.add_argument("--camera-settings", type=Path, help="Existing camera_settings.json")
    parser.add_argument(
        "--calibration-dir", type=Path, help="Directory of saved follower calibration"
    )
    parser.add_argument(
        "--uvc-tool", type=Path, help="Override camera settings sibling tools/uvc-util"
    )
    parser.add_argument(
        "--check-config", action="store_true", help="Read files only; never open devices"
    )
    parser.add_argument(
        "--enable-motion", action="store_true", help="SO101: preload hold and enable torque"
    )
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("port must be within [0, 65535]")
    hardware = None
    if args.backend == "so101":
        if not args.robot_config or not args.camera_settings:
            parser.error("so101 requires --robot-config and --camera-settings")
        from .hardware_config import HardwareConfig

        try:
            config = HardwareConfig.load(
                args.robot_config, args.camera_settings, args.calibration_dir, args.uvc_tool
            )
        except (ValueError, OSError) as exc:
            parser.error(str(exc))
        if args.check_config:
            print(json.dumps(config.summary(), allow_nan=False))
            return
        from .hardware import SO101Backend

        hardware = SO101Backend.from_config(config)
    elif any(
        (
            args.robot_config,
            args.camera_settings,
            args.calibration_dir,
            args.uvc_tool,
            args.check_config,
            args.enable_motion,
        )
    ):
        parser.error("hardware options require --backend so101")
    if args.output_dir is None:
        parser.error("--output-dir is required to serve")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    controller = AgentController(hardware if hardware is not None else MockBackend())

    def terminate(signum, frame):
        raise KeyboardInterrupt

    with ExitStack() as stack:
        if hardware is not None:
            stack.callback(hardware.close)
        # Bind first: a port conflict must not cause hardware connection/torque writes.
        server = stack.enter_context(
            AgentServer(controller, args.port, args.output_dir / "events.jsonl")
        )
        previous = signal.signal(signal.SIGTERM, terminate)
        stack.callback(signal.signal, signal.SIGTERM, previous)
        try:
            if hardware is not None:
                hardware.connect(enable_motion=args.enable_motion)
            info = {"url": f"http://127.0.0.1:{server.server_port}", **controller.metadata()}
            if hardware is not None:
                info["hardware"] = {
                    **config.summary(),
                    "connected": hardware.connected,
                    "camera_readback": hardware.cameras.identities,
                }
            server.record("session", {}, info)
            print(json.dumps(info), flush=True)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            server.record("shutdown", {}, controller.shutdown())


if __name__ == "__main__":
    main()
