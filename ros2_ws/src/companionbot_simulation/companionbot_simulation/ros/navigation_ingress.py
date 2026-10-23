"""Simulation composition of task leases, sensor health and Nav2 intent I/O.

No actuator, reference planner, pose reset or truth interface exists here. DDS
Graph checks enforce the configured controller endpoint in the trusted local
ROS domain; Jazzy rclpy does not expose per-message publisher GIDs.
"""
import math
import time
import json
from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from companionbot_interfaces.srv import NavigationControl
from companionbot_follow.runtime.navigation_motion import NavigationMotionAdapter


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class NavigationIngress:
    def __init__(self, node, *, map_id, map_version, output_dir, controller="/controller_server"):
        self.node, self.controller = node, controller
        self.adapter = NavigationMotionAdapter(map_id=map_id, map_version=map_version)
        self.receipts, self.stamps = {}, {}
        self.pose_covariance_valid = False
        self.controller_gid = None
        self.endpoint_fault = None
        self.last_reason = "navigation_starting"
        out = Path(output_dir);out.mkdir(parents=True, exist_ok=True)
        self.command_log = (out/"navigation_commands.jsonl").open("w")
        self.control_log = (out/"navigation_controls.jsonl").open("w")
        self.command_topic = "/companionbot/navigation/cmd_vel"
        node.create_subscription(TwistStamped, self.command_topic, self.command, 1)
        node.create_subscription(LaserScan, "/companionbot/scan", self.scan, qos_profile_sensor_data)
        node.create_subscription(Odometry, "/companionbot/odom", self.odom, qos_profile_sensor_data)
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose, 10)
        node.create_subscription(TFMessage, "/tf", self.transforms, 100)
        node.create_service(NavigationControl, "/companionbot/navigation/control", self.control)
        node.create_timer(.05, self.health, clock=node.steady)

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def record(self, name, stamp):
        self.receipts[name], self.stamps[name] = time.perf_counter(), seconds(stamp)

    def scan(self, message):
        # -inf is an explicitly invalid near return; +inf is a valid no return.
        valid = (message.header.frame_id == "lidar_link" and len(message.ranges) >= 180
                 and message.range_min > 0 and message.range_max > message.range_min
                 and any(math.isfinite(v) and message.range_min <= v <= message.range_max
                         for v in message.ranges))
        if valid:
            self.record("scan", message.header.stamp)
        else:
            self.adapter.update_health(False, "navigation_invalid_scan", now_s=self.now())

    def odom(self, message):
        values = [message.pose.pose.position.x, message.pose.pose.position.y,
                  message.twist.twist.linear.x, message.twist.twist.angular.z]
        if (message.header.frame_id == "odom" and message.child_frame_id == "base_footprint"
                and all(math.isfinite(v) for v in values)):
            self.record("odom", message.header.stamp)

    def pose(self, message):
        covariance = message.pose.covariance
        self.pose_covariance_valid = (message.header.frame_id == "map"
            and all(math.isfinite(v) for v in covariance)
            and all(0 <= covariance[i] <= .25 for i in (0, 7, 35)))
        if self.pose_covariance_valid:
            self.record("localization_pose", message.header.stamp)
        else:
            self.adapter.update_health(False, "navigation_localization_covariance", now_s=self.now())

    def transforms(self, message):
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.record("localization_tf", transform.header.stamp)

    def endpoint_valid(self):
        endpoints = self.node.get_publishers_info_by_topic(self.command_topic)
        if len(endpoints) != 1:
            if len(endpoints) > 1:
                self.endpoint_fault = "navigation_multiple_command_publishers"
            return False
        endpoint = endpoints[0]
        if endpoint.node_name == "_NODE_NAME_UNKNOWN_" or endpoint.node_namespace == "_NODE_NAMESPACE_UNKNOWN_":
            return False
        name = endpoint.node_namespace.rstrip("/") + "/" + endpoint.node_name
        if name != self.controller or endpoint.topic_type != "geometry_msgs/msg/TwistStamped":
            self.endpoint_fault = "navigation_unapproved_command_publisher"
            return False
        gid = tuple(endpoint.endpoint_gid)
        if self.controller_gid is None:
            self.controller_gid = gid
        elif gid != self.controller_gid:
            self.endpoint_fault = "navigation_controller_restarted_requires_runtime_restart"
        return self.endpoint_fault is None

    def health(self):
        now, wall = self.now(), time.perf_counter()
        reason = self.endpoint_fault
        if reason is None and not self.endpoint_valid():
            reason = self.endpoint_fault or "navigation_controller_endpoint_unavailable"
        if reason is None and not self.pose_covariance_valid:
            reason = "navigation_localization_unavailable"
        # TF is future-dated by AMCL transform_tolerance. No extrapolated pose
        # is invented; source-clock and local receipt freshness are both checked.
        for name, max_age in (("scan", .35), ("odom", .12), ("localization_tf", .35)):
            if reason is None and (name not in self.receipts or wall-self.receipts[name] > max_age
                    or not -.3 <= now-self.stamps[name] <= max_age):
                reason = "navigation_" + name + "_unavailable"
        self.last_reason = reason or "navigation_health_valid"
        self.adapter.update_health(reason is None, self.last_reason, now_s=now)
        return reason is None

    def command(self, message):
        if not self.health():
            self.log_command(message, False, self.last_reason)
            return
        twist = message.twist
        unsupported = (twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y)
        if (message.header.frame_id != "base_footprint"
                or any(not math.isfinite(v) or abs(v) > 1e-9 for v in unsupported)):
            self.adapter.stop("navigation_twist_frame_or_nonplanar_component")
            self.log_command(message, False, "navigation_twist_frame_or_nonplanar_component")
            return
        accepted = self.adapter.receive_cmd(twist.linear.x, twist.angular.z,
            source_time_s=seconds(message.header.stamp), now_s=self.now())
        self.log_command(message, accepted, "accepted" if accepted else self.adapter.snapshot(include_events=False)["reason"])

    def log_command(self, message, accepted, reason):
        authority = self.adapter.snapshot(include_events=False)
        limited = self.adapter.sample(self.now()) if accepted else None
        self.command_log.write(json.dumps(dict(
            received_at_monotonic_s=time.perf_counter(), ros_observation_time_s=self.now(),
            source_ros_time_s=seconds(message.header.stamp), source_frame=message.header.frame_id,
            leased_task_id=authority["task_id"], authority_epoch=authority["authority_epoch"],
            requested_linear_m_s=message.twist.linear.x, requested_yaw_rad_s=message.twist.angular.z,
            limited_linear_m_s=limited.linear_velocity_target_m_s if limited else None,
            limited_yaw_rad_s=limited.yaw_rate_target_rad_s if limited else None,
            accepted=accepted, reason=reason, actual_actuator_feedback="separate_frozen_control_history"))+"\n")

    def control(self, request, response):
        self.health()
        result = self.adapter.control(task_id=request.task_id, source_epoch=request.source_epoch,
            sequence=int(request.sequence), command=request.command,
            lease_duration_s=request.lease_duration_s, map_id=request.map_id,
            map_version=request.map_version, now_s=self.now())
        for name, value in result.items():
            setattr(response, name, value)
        self.control_log.write(json.dumps(dict(received_at_monotonic_s=time.perf_counter(),
            ros_observation_time_s=self.now(), task_id=request.task_id, source_epoch=request.source_epoch,
            sequence=int(request.sequence), command=request.command, lease_duration_s=request.lease_duration_s,
            map_id=request.map_id, map_version=request.map_version, **result))+"\n")
        return response

    def close(self):
        self.adapter.stop("navigation_runtime_shutdown")
        self.command_log.close()
        self.control_log.close()
