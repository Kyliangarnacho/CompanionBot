"""Archived migration DDS smoke; superseded by tools/verification/verify_runtime_ros.py.

Historical source only. Its fixed Stage 9 output paths must not be run again.
"""
from companionbot_contracts.paths import repository_root

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from companionbot_interfaces.msg import RobotSnapshot, BehaviorFeedback, ControlTelemetry
from companionbot_interfaces.srv import UserInput
from companionbot_contracts.stamps import source_stamp

ROOT = repository_root()
OUTPUT = ROOT / "models/minisegway/stage9/results/architecture/ros_smoke"


def until(node, condition, seconds=10):
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        if condition():
            return
    raise TimeoutError("ROS smoke deadline")


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    # One isolated localhost graph; no running user robot is addressed.
    os.environ["ROS_DOMAIN_ID"] = "87"
    rclpy.init()
    node = Node("stage9_smoke_client")
    state, feedback, controls, clocks, transforms = [], [], [], [], []
    node.create_subscription(RobotSnapshot,"/companionbot/robot/state",state.append,10)
    node.create_subscription(BehaviorFeedback,"/companionbot/behavior/feedback",feedback.append,32)
    node.create_subscription(ControlTelemetry,"/companionbot/sim/control",controls.append,qos_profile_sensor_data)
    node.create_subscription(Clock,"/clock",clocks.append,qos_profile_sensor_data)
    node.create_subscription(TFMessage,"/tf_static",transforms.append,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = node.create_client(UserInput,"/companionbot/interaction/input")
    cancel = node.create_client(Trigger,"/companionbot/interaction/cancel")
    launched, records = [], {}
    handles = []
    def launch(name, *args):
        log = (OUTPUT / (name + ".txt")).open("w")
        handles.append(log)
        process = subprocess.Popen([sys.executable,"/opt/ros/jazzy/bin/ros2","launch",
                         "companionbot_bringup", name + ".launch.py", *args], stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True)
        launched.append(process)
        return process
    def submit(text, sequence, age=0):
        meta = source_stamp("stage9_smoke:text","smoke-epoch",sequence,time.perf_counter()-age,
                            time_semantics="text_submitted")
        future = client.call_async(UserInput.Request(meta=meta,text=text))
        until(node,future.done)
        return future.result()
    def clean_exit(process, name):
        process.wait(timeout=10)
        assert process.returncode == 0
        log = (OUTPUT / (name + ".txt")).read_text()
        assert "process has finished cleanly" in log
        assert not any(marker in log for marker in ("process has died", "Traceback", "启动/音频错误"))
    try:
        process = launch("interaction", "exit_after:=25.0", "provisional_camera_tf:=true",
                         "output_dir:=" + str(OUTPUT / "interaction"))
        until(node,lambda: client.service_is_ready() and bool(state))
        records["node_names"] = node.get_node_names_and_namespaces()
        until(node, lambda: bool(transforms))
        links = [(t.header.frame_id, t.child_frame_id) for m in transforms for t in m.transforms]
        assert links == [("base_link", "camera_optical_frame")]
        records["explicit_provisional_tf_only"] = links
        assert state[-1].backend == "fake" and not state[-1].hardware_execution_ready and not state[-1].master_locked
        assert submit("你好小柒",1).queued
        until(node,lambda: bool(state) and state[-1].meta.sequence >= 3)
        assert not submit("你好小柒",1).queued  # duplicate cannot re-enter the session
        stale = submit("带我去服务台",2,age=3)
        assert not stale.queued and stale.reason == "stale_or_future_input"
        assert submit("跟随我",2).queued
        until(node,lambda: any(f.status=="REJECT" and f.intent=="FOLLOW" for f in feedback))
        records["follow_without_master"] = next(f.reason for f in feedback if f.intent=="FOLLOW")
        assert submit("带我去服务台",3).queued
        until(node,lambda: any(f.status=="ACCEPT" and f.intent=="GUIDE_TO" for f in feedback))
        accepted = next(f for f in feedback if f.status=="ACCEPT" and f.intent=="GUIDE_TO")
        assert accepted.task_id and accepted.input_id and accepted.turn_id and accepted.destination_id=="service_desk"
        assert accepted.backend=="fake" and not accepted.hardware_execution_ready
        future = cancel.call_async(Trigger.Request())
        until(node,future.done)
        assert future.result().success
        until(node,lambda: any(f.status=="CANCEL" and f.task_id==accepted.task_id for f in feedback))
        records["navigation_fake_task"] = {"task_id":accepted.task_id,"destination_id":accepted.destination_id,
                                           "statuses":[f.status for f in feedback if f.task_id==accepted.task_id]}
        assert submit("/quit",4).queued
        clean_exit(process, "interaction")
        sim = launch("simulation","duration_s:=4.0")
        until(node,lambda: len(controls)>=30 and len(clocks)>=30,seconds=15)
        assert all(m.meta.clock_domain=="mujoco_sim" and m.physics_dt_s==.001 and m.controller_dt_s==.002
                   and not m.hardware_execution_ready for m in controls)
        records["sim_control_samples_received"] = len(controls)
        records["sim_source_time_s"] = controls[-1].meta.source_time_s
        assert controls[-1].header.stamp.sec > 1_000_000_000
        assert all(m.clock.sec < 5 for m in clocks)
        records["ros_publish_and_sim_clock_distinct"] = True
        clean_exit(sim, "simulation")
        records["passed"] = True
    finally:
        for process in launched:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGTERM)
                    process.wait(timeout=5)
        for log in handles:
            log.close()
        node.destroy_node()
        rclpy.try_shutdown()
        (OUTPUT / "summary.json").write_text(json.dumps(records,indent=2))
    print(json.dumps(records,ensure_ascii=False))


if __name__ == "__main__":
    main()
