import http.client
import json
import threading
from contextlib import contextmanager

import pytest

from examples.so101_agent import AgentController, MockBackend
from examples.so101_agent.contracts import JOINTS
from examples.so101_agent.server import MAX_BODY, AgentServer


class Clock:
    def __init__(self):
        self.time = 10.0

    def now(self):
        return self.time

    def wait(self, cancelled, seconds):
        if not cancelled.is_set():
            self.time += seconds


@contextmanager
def serving(controller, journal=None):
    with AgentServer(controller, journal=journal) as server:
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
        thread.start()
        try:
            yield server
        finally:
            controller.hold()
            server.shutdown()
            thread.join(2)


@pytest.fixture
def service(tmp_path):
    clock = Clock()
    controller = AgentController(MockBackend(clock=clock), clock=clock)
    with serving(controller, tmp_path / "events.jsonl") as server:
        yield server


def call(server, path, body=None, *, method="POST", headers=None, raw=None):
    conn = http.client.HTTPConnection("127.0.0.1", server.server_port, timeout=3)
    try:
        conn.request(
            method,
            path,
            body=raw if raw is not None else json.dumps(body or {}),
            headers={"Content-Type": "application/json", **(headers or {})},
        )
        response = conn.getresponse()
        return response.status, json.loads(response.read())
    finally:
        conn.close()


def test_all_four_tools_over_real_loopback_and_persisted_journal(service):
    status, observed = call(service, "/observe")
    assert status == 200 and observed["simulated"]
    arguments = {
        "target": {**observed["positions"], "shoulder_pan": 3},
        "duration_s": 0.5,
        "observation_id": observed["observation_id"],
        "command_id": "http-move",
    }
    status, moved = call(service, "/move_joints", arguments)
    assert status == 200 and moved["status"] == "reached"
    assert moved["final_position"]["shoulder_pan"] == pytest.approx(3)
    assert call(service, "/move_joints", arguments) == (200, moved)
    _, observed = call(service, "/observe")
    status, gripped = call(
        service,
        "/set_gripper",
        {
            "value": 46,
            "duration_s": 1,
            "observation_id": observed["observation_id"],
            "command_id": "http-grip",
        },
    )
    assert status == 200 and gripped["status"] == "reached"
    status, held = call(service, "/hold")
    assert status == 200 and held["status"] == "held"
    assert held["target"]["gripper"] == pytest.approx(46)
    records = [json.loads(line) for line in service.journal.read_text().splitlines()]
    assert [row["tool"] for row in records] == [
        "observe",
        "move_joints",
        "move_joints",
        "observe",
        "set_gripper",
        "hold",
    ]
    assert records[1]["result"]["trajectory"]


def test_tool_schemas_and_metadata_are_discoverable(service):
    status, schemas = call(service, "/tools", method="GET")
    assert status == 200
    assert {t["name"] for t in schemas["tools"]} == {
        "observe",
        "move_joints",
        "set_gripper",
        "hold",
    }
    motion = next(t for t in schemas["tools"] if t["name"] == "move_joints")
    assert motion["input_schema"]["properties"]["target"]["required"] == list(JOINTS)
    status, health = call(service, "/health", method="GET")
    assert status == 200 and health["backend"] == "mock" and health["simulated"]


@pytest.mark.parametrize(
    "path,body,expected",
    [
        ("/observe", '{"unexpected": 1}', "invalid_argument"),
        ("/hold", "[]", "invalid_argument"),
        ("/move_joints", "{}", "invalid_argument"),
        ("/observe", '{"x": NaN}', "invalid_json"),
        ("/observe", '{"x": 1, "x": 2}', "invalid_json"),
        ("/observe", "{", "invalid_json"),
        ("/observe", "[" * 2000, "invalid_json"),
        ("/reset", "{}", "not_found"),
    ],
)
def test_malformed_http_requests_are_explicit_errors(service, path, body, expected):
    status, result = call(service, path, raw=body)
    assert status in {400, 404}
    assert result["error"]["code"] == expected


@pytest.mark.parametrize(
    "headers",
    [
        {"Origin": "http://example.com"},
        {"Origin": "null"},
        {"Host": "example.com"},
    ],
)
def test_browser_origin_and_rebinding_host_are_rejected(service, headers):
    assert call(service, "/hold", headers=headers)[0] == 403


def test_content_type_and_body_size_are_bounded(service):
    assert call(service, "/observe", headers={"Content-Type": "text/plain"})[0] == 400
    assert call(service, "/observe", raw=" " * (MAX_BODY + 1))[0] == 413
    assert call(service, "/observe", headers={"Transfer-Encoding": "chunked"})[0] == 400


def test_http_hold_can_interrupt_a_blocking_motion():
    backend = MockBackend()
    written = threading.Event()
    original = backend.write_targets

    def write(target):
        original(target)
        written.set()

    backend.write_targets = write
    with serving(AgentController(backend)) as server:
        _, observation = call(server, "/observe")
        arguments = {
            "target": {**observation["positions"], "shoulder_pan": 8},
            "duration_s": 1,
            "observation_id": observation["observation_id"],
            "command_id": "interrupt-over-http",
        }
        result = []
        worker = threading.Thread(
            target=lambda: result.append(call(server, "/move_joints", arguments))
        )
        worker.start()
        try:
            assert written.wait(2)
            assert call(server, "/hold")[1]["status"] == "held"
            worker.join(2)
            assert not worker.is_alive()
            assert result[0][1]["status"] == "cancelled"
            assert call(server, "/move_joints", arguments) == result[0]
        finally:
            server.controller.hold()
            worker.join(2)
