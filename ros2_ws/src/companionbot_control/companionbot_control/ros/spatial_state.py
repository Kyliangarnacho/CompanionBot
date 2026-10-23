"""Read-only BalanceState -> Odometry / JointState / dynamic body TF owner.

DDS never schedules MCU control. This node has no actuator publisher, command
subscriber, GT subscription, reference parser or localization/map authority.
"""
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation
from tf2_ros import TransformBroadcaster

from companionbot_interfaces.msg import BalanceState
from companionbot_contracts.ros_diagnostics import publish_status
from companionbot_contracts.spatial_geometry import load_spatial_geometry
from companionbot_control.estimation.spatial import PlanarOdometry, SpatialSample


def stamp_seconds(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid_ros_stamp")
    return stamp.sec + stamp.nanosec*1e-9


def set_rotation(target, quaternion):
    target.x, target.y, target.z, target.w = map(float, quaternion)


class SpatialState(Node):
    def __init__(self):
        super().__init__("spatial_state", namespace="companionbot")
        readonly = ParameterDescriptor(read_only=True)
        def parameter(name, default):
            return self.declare_parameter(name, default, readonly).value
        self.geometry = load_spatial_geometry(parameter("geometry_config", "") or None)
        self.expected_source = parameter("expected_source_id", "mujoco:spatial")
        self.expected_clock = parameter("expected_clock_domain", "mujoco_sim")
        self.mapping_id = parameter("expected_device_mapping_id", "")
        self.simulated = self.expected_clock == "mujoco_sim"
        if bool(self.get_parameter("use_sim_time").value) != self.simulated:
            raise ValueError("spatial ROS time must match the configured source clock")
        if not self.simulated and not self.mapping_id:
            raise ValueError("device spatial data requires an explicit measured ROS-time mapping id")
        self.add_on_set_parameters_callback(self._keep_clock)
        self.odom = PlanarOdometry(**{key: float(parameter(key, value)) for key, value in {
            "velocity_std": .03, "yaw_rate_std": .02, "position_rw": .005, "yaw_rw": .002,
            "max_gap_s": .12}.items()})
        self.odom_pub = self.create_publisher(Odometry, "odom", qos_profile_sensor_data)
        self.joint_pub = self.create_publisher(JointState, "joint_states", qos_profile_sensor_data)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.tf = TransformBroadcaster(self)
        # Jazzy rclpy omits publisher_gid from MessageInfo. The required C++
        # tf_authority node audits actual GIDs; never infer authority here.
        self.create_subscription(BalanceState, "state/balance", self.receive, qos_profile_sensor_data)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.watchdog, clock=self.steady)
        self.error = None
        self.count = self.rejected = 0
        self.identity = None
        self.started = time.monotonic()
        self.last_source_time = None
        self.state = "STARTING"
        self.publish_diagnostics()

    def _keep_clock(self, parameters):
        for parameter in parameters:
            if parameter.name == "use_sim_time" and bool(parameter.value) != self.simulated:
                return SetParametersResult(successful=False, reason="source_clock_requires_restart")
        return SetParametersResult(successful=True)

    def decode(self, message):
        meta = message.meta
        if (message.spatial_contract_version != 1 or meta.contract_version != 2
                or message.header.frame_id != "base_link" or message.geometry_id != self.geometry.geometry_id
                or meta.source_id != self.expected_source or meta.clock_domain != self.expected_clock
                or not meta.source_epoch or not meta.clock_id or not meta.has_sequence or not meta.has_source_time
                or not math.isfinite(meta.source_time_s) or meta.time_semantics != "control_sample"):
            raise ValueError("state_schema_source_frame_or_geometry")
        identity = (meta.source_id, meta.source_epoch, meta.clock_id, message.ros_time_mapping_id)
        if self.identity is not None and identity != self.identity:
            raise ValueError("source_or_time_mapping_changed_restart_required")
        mapping = message.ros_time_mapping_id
        if (not mapping or not math.isfinite(message.ros_time_uncertainty_s)
                or not 0 <= message.ros_time_uncertainty_s <= .002):
            raise ValueError("missing_or_uncertain_source_time_mapping")
        horizon = stamp_seconds(message.header.stamp)
        if self.simulated:
            if (mapping != "identity:" + meta.clock_id or meta.clock_id != "sim:" + meta.source_epoch
                    or message.ros_time_uncertainty_s != 0
                    or not math.isclose(horizon, meta.source_time_s, abs_tol=1e-9)):
                raise ValueError("invalid_simulation_time_mapping")
        elif mapping != self.mapping_id:
            raise ValueError("unapproved_device_time_mapping")
        # /clock and state are different DDS topics: allow a bounded scheduling
        # race, while rejecting old/future epochs instead of restamping to now.
        now = self.get_clock().now().nanoseconds*1e-9
        if horizon > now + .12 or (now > 0 and now - horizon > .12):
            raise ValueError("state_source_time_outside_ros_window")
        sample = SpatialSample(meta.source_epoch, int(meta.sequence), horizon,
            message.traveled_distance_m, message.forward_velocity_m_s,
            message.roll_rad, message.pitch_rad, message.yaw_rad,
            message.roll_rate_rad_s, message.pitch_rate_rad_s, message.yaw_rate_rad_s,
            tuple(message.wheel_relative_angle_rad), tuple(message.wheel_relative_velocity_rad_s),
            message.imu_sample_time_s, message.imu_available_time_s, message.imu_measurement_age_s,
            message.imu_valid, message.imu_stale, message.encoder_valid)
        sample.validate()
        return sample, identity

    def receive(self, message):
        if self.error:
            return
        try:
            sample, identity = self.decode(message)
            estimate = self.odom.update(sample)
            self.identity = identity
            self.last_source_time = sample.time_s
            self.publish_sample(message, sample, estimate)
            self.count += 1
            self.state = "RUNNING"
        except (ValueError, TypeError) as exc:
            self.rejected += 1
            self.fail(exc)

    def publish_sample(self, message, sample, estimate):
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = message.header.stamp, "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = estimate.x_m, estimate.y_m
        set_rotation(odom.pose.pose.orientation, Rotation.from_euler("z", estimate.yaw_rad).as_quat())
        # Unsupported dimensions are not advertised as measured zero variance.
        pose_cov = np.diag([0., 0., 1e6, 1e6, 1e6, 0.])
        pose_cov[np.ix_([0, 1, 5], [0, 1, 5])] = estimate.covariance
        odom.pose.covariance = pose_cov.ravel().tolist()
        odom.twist.twist.linear.x = sample.velocity_m_s
        odom.twist.twist.angular.z = sample.yaw_rate_rad_s
        odom.twist.covariance = np.diag([
            self.odom.velocity_std**2, 1e6, 1e6, 1e6, 1e6, self.odom.yaw_rate_std**2]).ravel().tolist()
        self.odom_pub.publish(odom)

        planar = TransformStamped()
        planar.header, planar.child_frame_id = odom.header, "base_footprint"
        planar.transform.translation.x, planar.transform.translation.y = estimate.x_m, estimate.y_m
        planar.transform.rotation = odom.pose.pose.orientation
        body = TransformStamped()
        body.header.stamp, body.header.frame_id = message.header.stamp, "base_footprint"
        body.child_frame_id = "base_link"
        body.transform.translation.z = self.geometry.wheel_radius_m
        set_rotation(body.transform.rotation, Rotation.from_euler(
            "xyz", [sample.roll_rad, sample.pitch_rad, 0.]).as_quat())
        self.tf.sendTransform([planar, body])

        joints = JointState()
        joints.header = message.header
        joints.name = ["left_wheel_joint", "right_wheel_joint"]
        indices = [self.geometry.ros_left_encoder_index, self.geometry.ros_right_encoder_index]
        joints.position = [sample.wheel_angles_rad[i] for i in indices]
        joints.velocity = [sample.wheel_velocities_rad_s[i] for i in indices]
        # Effort is unavailable in this state contract; never copy requested torque.
        self.joint_pub.publish(joints)

    def fail(self, exc):
        self.error, self.state = exc, "ERROR"
        self.get_logger().error("spatial output inhibited: " + str(exc))
        self.publish_diagnostics()

    def watchdog(self):
        if not self.error:
            if len(self.get_publishers_info_by_topic(self.resolve_topic_name("state/balance"))) > 1:
                self.fail(RuntimeError("multiple_balance_state_publishers"))
            now = self.get_clock().now().nanoseconds*1e-9
            if self.last_source_time is None:
                if time.monotonic() - self.started > 10:
                    self.fail(RuntimeError("spatial_source_startup_timeout"))
            elif now - self.last_source_time > self.odom.max_gap_s:
                self.fail(RuntimeError("spatial_source_timeout_while_clock_advances"))
        self.publish_diagnostics()

    def publish_diagnostics(self):
        publish_status(self, self.diagnostic_pub, self.state, {
            "samples": self.count, "rejected": self.rejected,
            "fault": str(self.error or ""), "geometry_id": self.geometry.geometry_id,
            "source": self.expected_source, "source_clock": self.expected_clock,
            "last_source_time_s": self.last_source_time,
            "odom_model": "planar_support_pitch_corrected_wheel_path",
            "covariance_basis": "declared_uncalibrated_process_assumptions",
            "map_localization_available": False},
            level=DiagnosticStatus.ERROR if self.error else DiagnosticStatus.OK)


def main():
    rclpy.init()
    node = None
    try:
        node = SpatialState()
        while rclpy.ok() and not node.error:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.state = "ERROR" if node.error else "CLOSED"
            if rclpy.ok():
                node.publish_diagnostics()
            node.destroy_node()
        rclpy.try_shutdown()
    if node is not None and node.error:
        raise node.error
