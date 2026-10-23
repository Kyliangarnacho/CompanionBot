"""Finite headless diagnostic of real Nav2 Actions and task-scoped motion.

Run beside the retail localization launch in an isolated DDS domain. This tool
never publishes velocity, robot TF, sensors, ground truth or a physical pose.
Plan mode records stock Smac reachability before a binding can be approved.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import time
from uuid import uuid4

import psutil
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from rosidl_runtime_py.convert import message_to_ordereddict
from companionbot_interfaces.srv import NavigationControl


class NavigationProbe(Node):
    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def __init__(self, out):
        super().__init__("retail_navigation_acceptance", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.out, self.records, self.counts = out, [], {}
        self.files = {}
        self.last_pose, self.last_odom = None, None
        self.last_command = None
        self.costmaps = {}
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.planner = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.authority = self.create_client(NavigationControl, "/companionbot/navigation/control")
        self.nomotion = self.create_client(Empty, "/request_nomotion_update")
        self.nomotion_requested = False
        self.epoch, self.sequence, self.task = uuid4().hex, 0, None
        self.lease_active = False
        self.renew_due, self.renew_pending, self.renew_error = 0., None, None
        self.resources, self.next_resource = [], 0.
        for key, typ, topic in [
            ("odom", Odometry, "/companionbot/odom"),
            ("pose", PoseWithCovarianceStamped, "/amcl_pose"),
            ("scan", LaserScan, "/companionbot/scan"),
            ("command", TwistStamped, "/companionbot/navigation/cmd_vel"),
            ("path", NavPath, "/plan")]:
            self.files[key] = (out/(key+".jsonl")).open("w")
            self.create_subscription(typ, topic, lambda m, k=key: self.receive(k, m), qos_profile_sensor_data)
        for topic in ("/global_costmap/costmap", "/local_costmap/costmap"):
            self.create_subscription(OccupancyGrid, topic,
                lambda m, t=topic: self.costmaps.__setitem__(t, m), qos_profile_sensor_data)

    def receive(self, key, message):
        row = dict(arrival_monotonic_s=time.perf_counter(), message=message_to_ordereddict(message))
        self.files[key].write(json.dumps(row)+"\n")
        self.counts[key] = self.counts.get(key, 0) + 1
        if key == "pose":
            self.last_pose = message
        if key == "odom":
            self.last_odom = message
        if key == "command":
            self.last_command = message

    def spin(self, timeout=.02):
        rclpy.spin_once(self, timeout_sec=timeout)
        now = time.perf_counter()
        if self.lease_active and self.task and now >= self.renew_due:
            if self.renew_pending is not None and not self.renew_pending.done():
                return
            if self.renew_pending is not None:
                answer = self.renew_pending.result()
                if not answer.accepted:
                    self.renew_error = answer.reason
            self.renew_pending = self.authority.call_async(self.request("RENEW"))
            self.renew_due = now + .4
        if now >= self.next_resource:
            self.next_resource = now + .5
            for process in psutil.process_iter(["pid", "name", "cmdline", "memory_info", "cpu_times"]):
                try:
                    command = " ".join(process.info["cmdline"] or [])
                    if "/opt/ros/jazzy/lib/nav2_" in command or "companionbot_simulation/spatial_sim" in command:
                        cpu = process.info["cpu_times"]
                        self.resources.append(dict(wall_s=now, pid=process.pid, name=process.info["name"],
                            cpu_s=cpu.user+cpu.system, rss_bytes=process.info["memory_info"].rss))
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

    def wait(self, future, timeout):
        end = time.perf_counter()+timeout
        while not future.done() and time.perf_counter() < end:
            self.spin()
        if not future.done():
            raise TimeoutError("ROS operation exceeded diagnostic deadline")
        return future.result()

    def wait_ready(self):
        end=time.perf_counter()+40.
        while time.perf_counter()<end and (self.last_pose is None or self.last_odom is None
                or not self.authority.service_is_ready() or not self.nav.server_is_ready()
                or not self.planner.server_is_ready()):
            if self.last_pose is None and self.nomotion.service_is_ready() and not self.nomotion_requested:
                self.nomotion.call_async(Empty.Request());self.nomotion_requested=True
            self.spin()
        if self.last_pose is None or not self.nav.server_is_ready():
            raise RuntimeError("localization_or_nav2_not_ready")
        lifecycle={}
        for name in ("map_server","amcl","planner_server","controller_server","bt_navigator"):
            client=self.create_client(GetState,"/"+name+"/get_state");deadline=time.perf_counter()+20.
            while not client.service_is_ready() and time.perf_counter()<deadline:self.spin()
            state=self.wait(client.call_async(GetState.Request()),3.).current_state
            while state.label!="active" and time.perf_counter()<deadline:
                self.spin(.1);state=self.wait(client.call_async(GetState.Request()),3.).current_state
            lifecycle[name]=state.label
            if state.label!="active":raise RuntimeError("navigation_lifecycle_not_active:"+name)
        deadline=time.perf_counter()+8.
        while len(self.costmaps)<2 and time.perf_counter()<deadline:self.spin()
        if len(self.costmaps)<2:raise RuntimeError("navigation_costmap_publication_unavailable")
        for topic,message in self.costmaps.items():
            (self.out/(topic.strip("/").replace("/","_")+".json")).write_text(json.dumps(message_to_ordereddict(message))+"\n")
        # Probe subscriptions and the motion owner's subscriptions can become
        # ready on different executor turns. Its actual health CHECK is the
        # authority; lifecycle discovery alone cannot grant motion.
        deadline=time.perf_counter()+8.
        while time.perf_counter()<deadline:
            if self.control("CHECK").accepted:
                return lifecycle
            self.spin(.1)
        raise RuntimeError("navigation_motion_owner_not_ready")

    def request(self, command):
        self.sequence += 1
        request = NavigationControl.Request()
        request.task_id, request.source_epoch, request.sequence = self.task or "diagnostic_stop", self.epoch, self.sequence
        request.command, request.lease_duration_s = command, 1.2
        request.map_id, request.map_version = self.binding["map_id"], self.binding["map_version"]
        return request

    def control(self, command):
        if command in ("STOP", "WAIT", "RELEASE"):
            self.lease_active = False
        if command != "RENEW":
            self.renew_due = time.perf_counter()+.4
        response = self.wait(self.authority.call_async(self.request(command)), 3.)
        if command == "ACQUIRE" and response.accepted:
            self.lease_active = True
        self.records.append(dict(event="authority", command=command, task_id=self.task,
            accepted=response.accepted, reason=response.reason, authority_epoch=response.authority_epoch))
        return response

    def goal_pose(self, target):
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = "map", self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(target["x_m"]), float(target["y_m"])
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(target["yaw_rad"]/2), math.cos(target["yaw_rad"]/2)
        return pose

    def feedback(self, message):
        self.records.append(dict(event="feedback", wall_s=time.perf_counter(), task_id=self.task,
            feedback=message_to_ordereddict(message.feedback)))

    def navigate(self, name, target, timeout, cancel_after=None):
        self.task, self.renew_error = uuid4().hex, None
        self.renew_pending = None  # A reply from an old task cannot fault this lease.
        task = self.task
        self.renew_due = math.inf
        response = self.control("ACQUIRE")
        if not response.accepted:
            self.task = None
            raise RuntimeError("navigation_authority_rejected:"+response.reason)
        started = time.perf_counter()
        started_source = self.get_clock().now().nanoseconds*1e-9
        goal = NavigateToPose.Goal(pose=self.goal_pose(target))
        handle = self.wait(self.nav.send_goal_async(goal, feedback_callback=self.feedback), 8.)
        if not handle.accepted:
            self.control("RELEASE"); self.task = None
            return dict(destination_id=name, accepted=False, status=GoalStatus.STATUS_ABORTED)
        result = handle.get_result_async()
        canceled = False
        try:
            while not result.done() and time.perf_counter()-started < timeout:
                self.spin()
                if self.renew_error:
                    raise RuntimeError("navigation_lease_lost:"+self.renew_error)
                if cancel_after is not None and not canceled and time.perf_counter()-started >= cancel_after:
                    # Clear reference authority before sending the Action cancel.
                    self.control("STOP"); self.task = None
                    answer = self.wait(handle.cancel_goal_async(), 5.)
                    self.records.append(dict(event="cancel_response", goals_canceling=len(answer.goals_canceling)))
                    canceled = True
            if not result.done():
                raise TimeoutError("navigation_goal_deadline")
            answer = result.result()
            row = dict(destination_id=name, task_id=task, accepted=True, status=answer.status,
                elapsed_wall_s=time.perf_counter()-started, result=message_to_ordereddict(answer.result),
                start_source_time_s=started_source, source_time_s=self.get_clock().now().nanoseconds*1e-9,
                target=dict(target),
                localization_error_m=(math.hypot(self.last_pose.pose.pose.position.x-target["x_m"],
                    self.last_pose.pose.pose.position.y-target["y_m"]) if self.last_pose else None))
            self.records.append(dict(event="result", **row))
            return row
        finally:
            if self.task:
                self.control("RELEASE")
                self.task = None
            if not result.done():
                self.wait(handle.cancel_goal_async(), 5.)
            self.renew_pending = None
            # Observe actual braking/settling before another explicit task.
            deadline = time.perf_counter()+3.
            while time.perf_counter() < deadline:
                self.spin()

    def close(self):
        if self.task:
            self.control("STOP")
            self.task = None
        for handle in self.files.values():
            handle.close()
        (self.out/"events.json").write_text(json.dumps(self.records, indent=2)+"\n")
        (self.out/"resources.json").write_text(json.dumps(self.resources)+"\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bindings", type=Path, required=True)
    parser.add_argument("--goals", nargs="+", required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--mode", choices=("plan", "navigate", "cancel"), default="navigate")
    parser.add_argument("--goal-timeout", type=float, default=180.)
    args = parser.parse_args()
    out = args.output_dir.resolve(); out.mkdir(parents=True, exist_ok=True)
    binding = json.loads(args.bindings.read_text())
    (out/"bindings.json").write_text(json.dumps(binding, indent=2)+"\n")
    summary = dict(passed=False, mode=args.mode, results=[], platform=platform.uname()._asdict(),
                   physical_hardware=False, bindings_sha256=hashlib.sha256(args.bindings.read_bytes()).hexdigest())
    rclpy.init(); node = NavigationProbe(out); node.binding = binding
    try:
        summary["lifecycle"] = node.wait_ready()
        for name in args.goals:
            target = binding["destinations"][name]
            if args.mode == "plan":
                goal = ComputePathToPose.Goal(goal=node.goal_pose(target), planner_id="GridBased", use_start=False)
                handle = node.wait(node.planner.send_goal_async(goal), 5.)
                if not handle.accepted:
                    raise RuntimeError("planner_rejected_goal:"+name)
                answer = node.wait(handle.get_result_async(), 8.)
                row = dict(destination_id=name, status=answer.status,
                    planning_time=message_to_ordereddict(answer.result.planning_time),
                    path=message_to_ordereddict(answer.result.path))
                summary["results"].append(row)
                if answer.status != GoalStatus.STATUS_SUCCEEDED or not answer.result.path.poses:
                    raise RuntimeError("unreachable_destination:"+name)
            else:
                if not target.get("reachability_verified"):
                    raise RuntimeError("destination_reachability_not_verified:"+name)
                row = node.navigate(name, target, args.goal_timeout, 3. if args.mode == "cancel" else None)
                summary["results"].append(row)
                expected = GoalStatus.STATUS_CANCELED if args.mode == "cancel" else GoalStatus.STATUS_SUCCEEDED
                if row["status"] != expected:
                    raise RuntimeError("unexpected_navigation_result:"+name)
        summary["passed"] = True
    except BaseException as error:
        summary["error"] = repr(error)
        raise
    finally:
        node.close(); summary["counts"] = node.counts
        (out/"summary.json").write_text(json.dumps(summary, indent=2)+"\n")
        node.destroy_node(); rclpy.try_shutdown()
    print(json.dumps({"passed":summary["passed"],"mode":args.mode,
        "results":[{k:v for k,v in row.items() if k != "path"} for row in summary["results"]],
        "counts":summary["counts"]}), flush=True)


if __name__ == "__main__":
    main()
