"""A perception startup exception must still close the session and invalidate frames."""
import asyncio
import json
import threading
import time
from queue import Queue
from types import SimpleNamespace

import pytest
from companionbot_behavior.runtime import session
from companionbot_behavior.core.frames import Stage7FrameBuffer
from companionbot_behavior.runtime.inbox import TextInput


@pytest.mark.parametrize("ros", [False, True])
def test_camera_failure_is_reported_and_all_remaining_cleanup_runs(tmp_path, monkeypatch, ros):
    monkeypatch.setenv("ROS_DOMAIN_ID", "94")
    args = session.parser().parse_args(["--fake", "--output-dir", str(tmp_path)])
    args.ros = ros
    args.ros_args = ["test", "--ros-args", "-r", "__ns:=/runtime_failure"]
    frames, ready = Stage7FrameBuffer(), threading.Event()
    def broken_camera(stop, enqueue, run_id):
        raise RuntimeError("injected_camera_startup_failure")
    with pytest.raises(RuntimeError, match="injected_camera_startup_failure"):
        asyncio.run(session.main(args, frames=frames, camera_runner=broken_camera, camera_ready=ready))
    manifest, = tmp_path.glob("*_runtime.json")
    result = json.loads(manifest.read_text())
    assert result["state"] == "ERROR"
    assert result["inbox"]["closed"] and result["inbox"]["pending"] == 0
    assert {e["resource"] for e in result["errors"]} == {"runtime", "perception"}
    assert frames.latest() is None
    if ros:
        assert result["ros"]["threads_stopped"] and not result["ros"]["executor_errors"]


def test_fresh_at_admission_but_expired_in_queue_cannot_dispatch_a_behavior(tmp_path):
    args = session.parser().parse_args(["--fake", "--output-dir", str(tmp_path)])
    args.exit_after = .15
    ready = threading.Event()
    def camera(stop, enqueue, run_id):
        enqueue(TextInput("你好小柒跟随我", "expired-id", {"source_id": "test"}, time.perf_counter() - 1))
        ready.set()
        stop.wait(timeout=2)
    ui = SimpleNamespace(commands=Queue(), preview_line="", emit=lambda *args: None)
    asyncio.run(session.main(args, camera_runner=camera, camera_ready=ready, ui=ui))
    telemetry, = tmp_path.glob("*.jsonl")
    events = [json.loads(line) for line in telemetry.read_text().splitlines()]
    decision = next(e for e in events if e["record_type"] == "input_decision")
    assert decision["input_id"] == "expired-id" and decision["reason"] == "stale_after_queue"
    assert not any(e.get("intent") == "FOLLOW" for e in events)


def test_finite_lifetime_also_stops_before_camera_first_frame(tmp_path):
    args = session.parser().parse_args(["--fake", "--output-dir", str(tmp_path)])
    args.exit_after = .05
    stopped = threading.Event()
    def loading_camera(stop, enqueue, run_id):
        assert stop.wait(timeout=2)
        stopped.set()
    asyncio.run(session.main(args, camera_runner=loading_camera, camera_ready=threading.Event()))
    manifest, = tmp_path.glob("*_runtime.json")
    assert json.loads(manifest.read_text())["state"] == "CLOSED"
    assert stopped.is_set()
