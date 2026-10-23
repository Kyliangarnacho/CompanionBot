"""Actual launch/Graph/DDS/lifecycle acceptance. Fake/replay; no hardware or GUI.

Run with this repository's Python after sourcing the built ROS workspace.
Every result goes to an explicitly selected engineering session directory.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from diagnostic_msgs.msg import DiagnosticArray
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from companionbot_interfaces.msg import RobotSnapshot, BehaviorFeedback, ColorFrame, MasterStatus, ControlTelemetry
from companionbot_interfaces.srv import UserInput
from companionbot_contracts.paths import repository_root
from companionbot_contracts.stamps import source_stamp


def until(node, predicate, seconds=12):
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)
        if predicate():
            return
    raise TimeoutError("runtime acceptance deadline")


def graph(node):
    def endpoint(item):
        qos = item.qos_profile
        return {"node": item.node_namespace.rstrip("/") + "/" + item.node_name,
                "type": item.topic_type, "reliability": qos.reliability.name,
                "durability": qos.durability.name, "history": qos.history.name, "depth": qos.depth}
    nodes = []
    for name, namespace in sorted(node.get_node_names_and_namespaces()):
        nodes.append({"name": name, "namespace": namespace,
                      "services": node.get_service_names_and_types_by_node(name, namespace),
                      "clients": node.get_client_names_and_types_by_node(name, namespace)})
    return {"nodes": nodes, "topics": [{"name": name, "types": types,
            "publishers": [endpoint(p) for p in node.get_publishers_info_by_topic(name)],
            "subscribers": [endpoint(s) for s in node.get_subscriptions_info_by_topic(name)]}
            for name, types in sorted(node.get_topic_names_and_types())]}


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--output-dir", type=Path, required=True)
    cli.add_argument("--video", type=Path, default=repository_root() /
                     "models/minisegway/stage9/results/architecture/models/canvas_replay.avi")
    args = cli.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = "97"
    os.environ["ROS_LOG_DIR"] = str(output / "ros_logs")
    rclpy.init()
    node = Node("runtime_acceptance_probe")
    launched, handles, records = [], [], {"domain": 97, "hardware_execution_ready": False}
    states, feedback, controls, clocks, transforms = [deque(maxlen=512) for _ in range(5)]
    diagnostics, colors, masters = {}, deque(maxlen=512), deque(maxlen=512)
    def status(message):
        for item in message.status:
            diagnostics[item.name] = {"message": item.message, "level": item.level, "pid": item.hardware_id,
                                      "values": {v.key: v.value for v in item.values}}
    def color(message):
        colors.append({"sequence": message.meta.sequence, "source": message.meta.source_id,
                       "epoch": message.meta.source_epoch, "time": message.meta.source_time_s,
                       "received": message.meta.received_at_s, "width": message.image.width,
                       "height": message.image.height, "encoding": message.image.encoding})
    for namespace in ("companionbot", "runtime_replay"):
        node.create_subscription(DiagnosticArray, f"/{namespace}/diagnostics", status, 10)
    node.create_subscription(RobotSnapshot, "/companionbot/robot/state", lambda m: states.append(m), 10)
    node.create_subscription(BehaviorFeedback, "/companionbot/behavior/feedback", lambda m: feedback.append(m), 32)
    node.create_subscription(ControlTelemetry, "/companionbot/sim/control", lambda m: controls.append(m), qos_profile_sensor_data)
    node.create_subscription(Clock, "/clock", lambda m: clocks.append(m), qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf_static", lambda m: transforms.append(m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(ColorFrame, "/runtime_replay/camera/color", color, qos_profile_sensor_data)
    node.create_subscription(MasterStatus, "/runtime_replay/perception/master", lambda m: masters.append(m), qos_profile_sensor_data)
    def launch(label, filename="interaction", *options):
        log = (output / (label + ".txt")).open("w")
        handles.append(log)
        process = subprocess.Popen([sys.executable, "/opt/ros/jazzy/bin/ros2", "launch", "companionbot_bringup",
                    filename + ".launch.py", "output_dir:=" + str(output / label), *options],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        launched.append(process)
        return process
    def submit(client, text, sequence, age=0):
        request = UserInput.Request(text=text, meta=source_stamp("runtime_acceptance:text", "epoch", sequence,
                       time.perf_counter() - age, time_semantics="text_submitted"))
        future = client.call_async(request)
        until(node, future.done)
        return future.result()
    def finish(process, label, *, failure=False):
        until(node, lambda: process.poll() is not None, seconds=55)
        log = (output / (label + ".txt")).read_text()
        if failure:
            assert process.returncode != 0, log
            return
        assert process.returncode == 0 and "process has finished cleanly" in log, log
        assert "process has died" not in log and "Traceback" not in log, log
        manifest, = (output / label).glob("*_runtime.json")
        value = json.loads(manifest.read_text())
        assert value["state"] == "CLOSED" and not value["errors"] and value["ros"]["threads_stopped"], value
        records[label + "_manifest"] = value
    def save_graph(label):
        value = graph(node)
        (output / (label + "_graph.json")).write_text(json.dumps(value, indent=2))
        return value
    try:
        interaction = launch("text", "interaction", "exit_after:=35.0")
        client = node.create_client(UserInput, "/companionbot/interaction/input")
        cancel = node.create_client(Trigger, "/companionbot/interaction/cancel")
        until(node, lambda: client.service_is_ready() and states and
              diagnostics.get("/companionbot/interaction", {}).get("message") == "RUNNING")
        text_graph = save_graph("text")
        assert not any(n["name"] == "perception" for n in text_graph["nodes"])
        assert not states[-1].master_locked and states[-1].backend == "fake"
        assert not transforms
        assert submit(client, "你好小柒", 1).queued
        duplicate = submit(client, "你好小柒", 1)
        assert not duplicate.queued and duplicate.reason == "duplicate_or_out_of_order"
        stale = submit(client, "带我去服务台", 2, age=3)
        assert not stale.queued and stale.reason == "stale_or_future_input"
        assert submit(client, "跟随我", 2).queued
        until(node, lambda: any(f.status == "REJECT" and f.intent == "FOLLOW" for f in feedback))
        accepted_request = submit(client, "带我去服务台", 3)
        assert accepted_request.queued and accepted_request.input_id
        until(node, lambda: any(f.status == "ACCEPT" and f.input_id == accepted_request.input_id for f in feedback))
        accepted = next(f for f in feedback if f.status == "ACCEPT" and f.input_id == accepted_request.input_id)
        assert accepted.task_id and accepted.turn_id and accepted.destination_id == "service_desk"
        assert accepted.backend == "fake" and not accepted.hardware_execution_ready
        future = cancel.call_async(Trigger.Request())
        until(node, future.done)
        assert future.result().success
        until(node, lambda: any(f.status == "CANCEL" and f.task_id == accepted.task_id for f in feedback))
        next_request = submit(client, "带我去机器人展品", 4)
        until(node, lambda: any(f.status == "ACCEPT" and f.input_id == next_request.input_id for f in feedback))
        assert submit(client, "/quit", 5).queued
        finish(interaction, "text")
        assert any(f.status == "CANCEL" and f.input_id == next_request.input_id for f in feedback)
        records["source_to_feedback_identity"] = {"input_id": accepted.input_id, "turn_id": accepted.turn_id,
                                                   "task_id": accepted.task_id, "shutdown_cancelled": True}
        until(node, lambda: ("interaction", "/companionbot") not in node.get_node_names_and_namespaces())

        params = output / "replay_params.yaml"
        params.write_text("/**/interaction:\n  ros__parameters:\n    diagnostics_period_s: 0.2\n"
                          "/**/perception:\n  ros__parameters:\n    diagnostics_period_s: 0.2\n    publish_period_s: 0.02\n")
        replay = launch("replay", "interaction", "no_camera:=false", "video:=" + str(args.video.resolve()),
                        "namespace:=runtime_replay", "params_file:=" + str(params), "exit_after:=30.0")
        vision = node.create_client(UserInput, "/runtime_replay/interaction/input")
        until(node, lambda: len(colors) >= 2 and masters and vision.service_is_ready() and
              diagnostics.get("/runtime_replay/perception", {}).get("message") == "RUNNING", seconds=45)
        value = save_graph("replay")
        def publishers(topic):
            return next(t["publishers"] for t in value["topics"] if t["name"] == topic)
        assert [p["node"] for p in publishers("/runtime_replay/camera/color")] == ["/runtime_replay/perception"]
        assert [p["node"] for p in publishers("/runtime_replay/robot/state")] == ["/runtime_replay/interaction"]
        assert diagnostics["/runtime_replay/interaction"]["pid"] == diagnostics["/runtime_replay/perception"]["pid"]
        for name in ("interaction", "perception"):
            getter = node.create_client(GetParameters, f"/runtime_replay/{name}/get_parameters")
            until(node, getter.service_is_ready)
            future = getter.call_async(GetParameters.Request(names=["diagnostics_period_s"]))
            until(node, future.done)
            assert future.result().values[0].double_value == .2
        assert submit(vision, "你好小柒", 1).queued
        assert submit(vision, "你前面有什么？", 2).queued
        def captured():
            paths = list((output / "replay").glob("*.jsonl"))
            return bool(paths) and '"record_type": "vision"' in paths[0].read_text()
        until(node, captured)
        assert all(m.state != "LOCKED" for m in masters), "Formal replay must never auto-select a Master"
        assert all(c["width"] == 1280 and c["height"] == 720 and c["encoding"] == "bgr8" for c in colors)
        assert all(not m.hardware_execution_ready for m in masters)
        assert submit(vision, "/quit", 3).queued
        finish(replay, "replay")
        records["replay_counts"] = {"colors": len(colors), "master_updates": len(masters),
                                    "natural_capture_view": True, "auto_selected_master": False}
        until(node, lambda: ("interaction", "/runtime_replay") not in node.get_node_names_and_namespaces())

        sim = launch("simulation", "simulation", "duration_s:=4.0")
        until(node, lambda: len(controls) >= 30 and len(clocks) >= 30, seconds=20)
        save_graph("simulation")
        assert all(c.physics_dt_s == .001 and c.controller_dt_s == .002 and
                   c.meta.clock_domain == "mujoco_sim" and not c.hardware_execution_ready for c in controls)
        until(node, lambda: sim.poll() is not None, seconds=15)
        assert sim.returncode == 0
        runtime = json.loads((output / "simulation/runtime.json").read_text())
        assert runtime["state"] == "CLOSED" and runtime["control_samples"] == 2000
        records["simulation"] = runtime

        tf = launch("explicit_tf", "interaction", "provisional_camera_tf:=true", "exit_after:=3.0")
        until(node, lambda: bool(transforms))
        assert [(t.header.frame_id, t.child_frame_id) for m in transforms for t in m.transforms] == [
            ("base_link", "camera_optical_frame")]
        finish(tf, "explicit_tf")
        bad = launch("invalid_source", "interaction", "no_camera:=false", "video:=" + str(output / "missing.avi"))
        finish(bad, "invalid_source", failure=True)
        records["startup_failure_nonzero_exit"] = bad.returncode
        until(node, lambda: not any(ns in ("/companionbot", "/runtime_replay")
                                   for _, ns in node.get_node_names_and_namespaces()))
        save_graph("after_shutdown")
        records["passed"] = True
    except BaseException as error:
        records["error"] = {"type": type(error).__name__, "message": str(error)}
        raise
    finally:
        for process in launched:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=50)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=8)
        for handle in handles:
            handle.close()
        (output / "summary.json").write_text(json.dumps(records, indent=2, ensure_ascii=False))
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({"passed": True, "output": str(output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
