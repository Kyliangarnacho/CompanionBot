#!/usr/bin/env python3
"""Headless stock Nav2 interfaces against a real stationary sensor simulation.

This is an interface fixture, not physical navigation acceptance: the balance
scenario deliberately has no motion-command consumer. Saved map must describe
the existing indoor-room fixture. Retail physical acceptance is separate.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import psutil
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--map-yaml", required=True, type=Path)
    cli.add_argument("--output-dir", required=True, type=Path)
    cli.add_argument("--domain-id", default=74, type=int)
    args = cli.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOG_DIR"] = str(out/"ros_logs")
    rclpy.init()
    node = Node("nav2_interface_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
    processes, handles, profiles = [], [], []
    counts = {"scan": 0, "odom": 0, "amcl": 0, "cmd_vel": 0}
    latest = {}
    def observe(name, message):
        counts[name] += 1
        latest[name] = message
    for topic, cls, key in (("/companionbot/scan", LaserScan, "scan"),
                            ("/companionbot/odom", Odometry, "odom"),
                            ("/amcl_pose", PoseWithCovarianceStamped, "amcl"),
                            ("/companionbot/navigation/cmd_vel", TwistStamped, "cmd_vel")):
        node.create_subscription(cls, topic, lambda m, key=key: observe(key, m), qos_profile_sensor_data)
    report = {"passed": False, "scope": "stationary room fixture: interfaces only; no actuator consumer",
              "map_yaml": str(args.map_yaml.resolve())}
    def launch(label, filename, options):
        handle = (out/(label+".log")).open("w")
        handles.append(handle)
        process = subprocess.Popen([sys.executable, "/opt/ros/jazzy/bin/ros2", "launch",
            "companionbot_bringup", filename, *options], stdout=handle, stderr=subprocess.STDOUT,
            start_new_session=True)
        processes.append(process)
        return process
    def spin_until(predicate, timeout=30):
        deadline = time.monotonic()+timeout
        last_profile = 0.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("required launch exited; inspect saved logs")
            if time.monotonic()-last_profile > .5:
                last_profile = time.monotonic()
                sample = []
                for p in processes:
                    try:
                        for child in psutil.Process(p.pid).children(recursive=True):
                            sample.append({"name": child.name(), "pid": child.pid,
                                           "rss_bytes": child.memory_info().rss,
                                           "cpu_time_s": sum(child.cpu_times()[:2])})
                    except psutil.Error:
                        pass
                profiles.append({"host_monotonic_s": last_profile, "processes": sample})
            if predicate():
                return
        raise TimeoutError("Nav2 interface deadline")
    def result(future, timeout=30):
        spin_until(future.done, timeout)
        if future.exception():
            raise future.exception()
        return future.result()
    def pose(x, y):
        p = PoseStamped()
        p.header.frame_id = "map"
        p.header.stamp = node.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.w = 1.
        return p
    try:
        launch("navigation", "navigation.launch.py", ["map_yaml:="+str(args.map_yaml.resolve()),
            "set_initial_pose:=true", "use_sim_time:=true"])
        launch("sensors", "spatial_simulation.launch.py", ["scenario:=balance", "duration_s:=45.0",
            "finish_grace_s:=-1.0", "map_authority:=/amcl", "wait_for_scan_consumer:=true",
            "output_dir:="+str(out/"stationary_simulation")])
        spin_until(lambda: counts["amcl"] > 0 and counts["scan"] > 10)
        lifecycle = {}
        for name in ("map_server", "amcl", "planner_server", "controller_server", "bt_navigator"):
            client = node.create_client(GetState, "/"+name+"/get_state")
            spin_until(client.service_is_ready)
            # Managed nodes activate sequentially after scan/TF readiness.
            # A discovered service is not proof of completed activation.
            deadline = time.monotonic()+30.
            state = result(client.call_async(GetState.Request())).current_state
            while state.label != "active" and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                state = result(client.call_async(GetState.Request())).current_state
            assert state.label == "active", (name, state)
            lifecycle[name] = state.label
        report["lifecycle"] = lifecycle
        planner = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
        navigator = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        spin_until(lambda: planner.server_is_ready() and navigator.server_is_ready())
        def plan(x, y):
            goal = ComputePathToPose.Goal()
            goal.goal = pose(x, y)
            goal.planner_id = "GridBased"
            start = time.monotonic()
            handle = result(planner.send_goal_async(goal))
            assert handle.accepted
            planned = result(handle.get_result_async())
            return planned, time.monotonic()-start
        planned, latency = plan(1., 0.)
        assert planned.status == GoalStatus.STATUS_SUCCEEDED, planned
        assert len(planned.result.path.poses) > 2
        report["planning"] = {"status": planned.status, "poses": len(planned.result.path.poses),
            "host_roundtrip_s": latency, "planner_time_s": planned.result.planning_time.sec + planned.result.planning_time.nanosec*1e-9}
        goal = NavigateToPose.Goal()
        goal.pose = pose(1., 0.)
        handle = result(navigator.send_goal_async(goal))
        assert handle.accepted
        spin_until(lambda: counts["cmd_vel"] > 5)
        velocity = latest["cmd_vel"]
        assert velocity.header.frame_id == "base_footprint"
        assert math.isfinite(velocity.twist.linear.x) and 0 < velocity.twist.linear.x <= .2+1e-9
        cancelled = result(handle.cancel_goal_async())
        assert cancelled.goals_canceling
        completion = result(handle.get_result_async())
        assert completion.status == GoalStatus.STATUS_CANCELED, completion
        spin_until(lambda: abs(latest["cmd_vel"].twist.linear.x) < 1e-12)
        report["cancellation"] = {"accepted": True, "status": completion.status, "final_zero_intent": True}
        goal.pose = pose(0., 0.)
        at_goal = result(navigator.send_goal_async(goal))
        assert at_goal.accepted
        completion = result(at_goal.get_result_async())
        assert completion.status == GoalStatus.STATUS_SUCCEEDED, completion
        report["already_at_goal"] = {"status": completion.status, "physical_motion_required": False}
        failed, latency = plan(100., 100.)
        assert failed.status == GoalStatus.STATUS_ABORTED and failed.result.error_code != 0
        report["outside_map_failure"] = {"status": failed.status, "error_code": failed.result.error_code,
                                         "host_roundtrip_s": latency}
        graph = {name for name, _ in node.get_node_names_and_namespaces()}
        assert not graph.intersection({"slam_toolbox", "velocity_smoother", "behavior_server"})
        pubs = node.get_publishers_info_by_topic("/companionbot/navigation/cmd_vel")
        assert len(pubs) == 1 and pubs[0].node_name == "controller_server"
        assert pubs[0].topic_type == "geometry_msgs/msg/TwistStamped"
        report.update({"cmd_vel_type": pubs[0].topic_type, "single_motion_publisher": pubs[0].node_name,
                       "counts": dict(counts), "passed": True})
    except Exception as exc:
        report["error"] = repr(exc)
        raise
    finally:
        for p in processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in processes:
            try:
                p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
        for handle in handles:
            handle.close()
        report["exit_codes"] = [p.returncode for p in processes]
        report["resource_samples"] = len(profiles)
        (out/"report.json").write_text(json.dumps(report, indent=2)+"\n")
        (out/"resources.json").write_text(json.dumps(profiles, indent=2)+"\n")
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
