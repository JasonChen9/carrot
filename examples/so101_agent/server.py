"""Loopback HTTP transport for the four tools; only the mock backend is shipped."""

import argparse
import json
import threading
from contextlib import suppress
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

    daemon_threads = True

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
    parser.add_argument("--backend", choices=["mock"], default="mock")
    parser.add_argument(
        "--port", type=int, default=8767, help="Loopback port; 0 selects a free port"
    )
    parser.add_argument(
        "--output-dir", type=Path, required=True, help="New directory for events.jsonl"
    )
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("port must be within [0, 65535]")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    controller = AgentController(MockBackend())
    with AgentServer(controller, args.port, args.output_dir / "events.jsonl") as server:
        print(
            json.dumps({"url": f"http://127.0.0.1:{server.server_port}", **controller.metadata()}),
            flush=True,
        )
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            result = controller.hold()
            server.record("hold", {}, result)


if __name__ == "__main__":
    main()
