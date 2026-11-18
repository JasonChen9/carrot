"""Exercise all four tools over real loopback HTTP, using only synthetic devices."""

import argparse
import base64
import json
import threading
import urllib.request
from pathlib import Path

from .controller import AgentController
from .mock import MockBackend
from .server import AgentServer


def run_demo(directory: Path) -> dict:
    """Create a new artifact directory; no hardware SDK or camera is loaded."""
    directory.mkdir(parents=True, exist_ok=False)
    controller = AgentController(MockBackend())
    with AgentServer(controller, journal=directory / "events.jsonl") as server:
        thread = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": 0.01})
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"

        def call(tool: str, **arguments: object) -> dict:
            request = urllib.request.Request(
                url + "/" + tool,
                data=json.dumps(arguments).encode(),
                headers={"Content-Type": "application/json"},
            )
            with urllib.request.urlopen(request, timeout=5) as response:
                return json.load(response)

        def observe(label: str) -> dict:
            observation = call("observe")
            for name, frame in observation["images"].items():
                (directory / f"{label}_{name}.png").write_bytes(
                    base64.b64decode(frame["data_base64"], validate=True)
                )
            return observation

        try:
            initial = observe("before")
            moved = call(
                "move_joints",
                target={**initial["positions"], "shoulder_pan": 2.0},
                duration_s=0.4,
                observation_id=initial["observation_id"],
                command_id="demo-move",
            )
            if moved["status"] != "reached":
                raise RuntimeError(f"Mock move failed: {moved}")
            middle = observe("after_move")
            gripped = call(
                "set_gripper",
                value=45.0,
                duration_s=0.7,
                observation_id=middle["observation_id"],
                command_id="demo-gripper",
            )
            if gripped["status"] != "reached":
                raise RuntimeError(f"Mock gripper failed: {gripped}")
            held = call("hold")
            if held["status"] != "held":
                raise RuntimeError(f"Mock hold failed: {held}")
            final = observe("after_hold")
            summary = {
                "simulated": True,
                "session_id": controller.session_id,
                "observe": "passed",
                "move_joints": moved["status"],
                "set_gripper": gripped["status"],
                "hold": held["status"],
                "final_positions": final["positions"],
            }
            (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
            return summary
        finally:
            controller.hold()
            server.shutdown()
            thread.join(2)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(run_demo(args.output_dir), indent=2))


if __name__ == "__main__":
    main()
