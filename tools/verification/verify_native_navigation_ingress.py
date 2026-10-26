"""Actual DDS software-only acceptance of native command authority ingress.

Publishes explicitly labeled schema fixtures, never launches physics/actuators,
never substitutes observations in a production graph. Run in an isolated domain.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from companionbot_interfaces.srv import NavigationControl
from companionbot_follow import _native_runtime
from companionbot_follow.runtime.navigation_motion import NavigationMotionAdapter


def stamp(value):
    ns = round(value * 1e9)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class Fixture:
    def __init__(self, output, *, follow_config=None):
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node("controller_server", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.cmd = self.node.create_publisher(TwistStamped, "/companionbot/navigation/cmd_vel", 1)
        self.clock = self.node.create_publisher(Clock, "/clock", 10)
        self.scan = self.node.create_publisher(LaserScan, "/companionbot/scan", qos_profile_sensor_data)
        self.odom = self.node.create_publisher(Odometry, "/companionbot/odom", qos_profile_sensor_data)
        self.pose = self.node.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
        self.tf = self.node.create_publisher(TFMessage, "/tf", 100)
        self.client = self.node.create_client(NavigationControl, "/companionbot/navigation/control")
        self.adapter = NavigationMotionAdapter(map_id="software_fixture", map_version="v1", follow_config=follow_config)
        self.adapter.update_mcu_status("NORMAL", "software_fixture_only")
        self.runtime = _native_runtime.NavigationIngressRuntime(
            self.adapter.native_authority, str(output), "/controller_server", True)
        self.runtime.start()
        self.source = 100.
        self.sequence = 0
        self.last_publish = 0.
        self.rogue = None

    def scan_message(self):
        scan = LaserScan()
        scan.header.frame_id = "lidar_link"; scan.header.stamp = stamp(self.source)
        scan.range_min, scan.range_max, scan.ranges = .05, 12., [2.] * 360
        return scan

    def publish_sensors(self):
        self.clock.publish(Clock(clock=stamp(self.source)))
        scan = self.scan_message()
        self.scan.publish(scan)
        odom = Odometry()
        odom.header.frame_id = "odom"; odom.child_frame_id = "base_footprint"
        odom.header.stamp = stamp(self.source)
        self.odom.publish(odom)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"; pose.header.stamp = stamp(self.source)
        pose.pose.covariance[0] = pose.pose.covariance[7] = pose.pose.covariance[35] = .01
        self.pose.publish(pose)
        tf = TransformStamped()
        tf.header.frame_id = "map"; tf.child_frame_id = "odom"; tf.header.stamp = stamp(self.source)
        tf.transform.rotation.w = 1.
        self.tf.publish(TFMessage(transforms=[tf]))

    def pump(self, duration=.05, *, advance=True, commands=False):
        end = time.perf_counter() + duration
        while time.perf_counter() < end:
            now = time.perf_counter()
            if now - self.last_publish >= .02:
                if advance:
                    self.source += .02
                self.publish_sensors()
                if commands:
                    self.command()
                self.last_publish = now
            self.executor.spin_once(timeout_sec=.003)
            self.runtime.check_health()

    def until(self, predicate, timeout=3., **pump_options):
        deadline = time.perf_counter() + timeout
        while time.perf_counter() < deadline:
            self.pump(.02, **pump_options)
            if predicate():
                return
        raise AssertionError("fixture_timeout:" + str(self.runtime.snapshot()))

    def control(self, command, task="fixture_task", sequence=None, **overrides):
        self.sequence += 1
        values = dict(task_id=task, source_epoch="software_fixture", sequence=self.sequence if sequence is None else sequence,
            command=command, lease_duration_s=1.2, map_id="software_fixture", map_version="v1") | overrides
        future = self.client.call_async(NavigationControl.Request(**values))
        self.until(future.done)
        return future.result()

    def command(self, *, source=None, frame="base_footprint", nonplanar=False):
        message = TwistStamped()
        message.header.frame_id, message.header.stamp = frame, stamp(self.source if source is None else source)
        message.twist.linear.x, message.twist.angular.z = .2, .3
        if nonplanar:
            message.twist.linear.y = .1
        self.cmd.publish(message)

    def close(self):
        self.runtime.close()
        snapshot = self.runtime.snapshot()
        assert snapshot["closed"] and snapshot["thread_stopped"]
        self.runtime.close()  # Idempotent cleanup must preserve explicit result.
        if self.rogue:
            self.rogue.destroy_node()
        self.executor.shutdown(timeout_sec=1.)
        self.node.destroy_node()
        self.context.try_shutdown()
        return snapshot


def run(output):
    output.mkdir(parents=True, exist_ok=False)
    checks = {}
    fixture = Fixture(output)
    try:
        fixture.until(lambda: fixture.client.service_is_ready() and
                      fixture.runtime.snapshot()["reason"] == "navigation_health_valid")
        checks["native_node_discovered"] = "navigation_ingress" in fixture.node.get_node_names()
        assert fixture.control("CHECK").accepted
        assert fixture.control("ACQUIRE").accepted
        fixture.pump(.1, commands=True)
        assert fixture.adapter.sample(fixture.source).linear_velocity_target_m_s == .2
        checks["fresh_actual_gid_command_accepted"] = fixture.runtime.snapshot()["commands_accepted"] > 0
        rejected = fixture.runtime.snapshot()["commands_rejected"]
        fixture.command(source=fixture.source - .2)
        fixture.until(lambda: fixture.runtime.snapshot()["commands_rejected"] > rejected)
        checks["stale_source_rejected"] = True
        fixture.command(nonplanar=True)
        fixture.until(lambda: not fixture.adapter.snapshot()["authorized"])
        checks["nonplanar_revokes_authority"] = fixture.adapter.take_stop()[1] == "navigation_twist_frame_or_nonplanar_component"
        assert fixture.control("STOP").accepted
        fixture.adapter.update_mcu_status("SAFE_HOLD", "navigation_stop")
        assert fixture.control("ACQUIRE", task="paused_clock_task").accepted
        fixture.pump(.1, commands=True)
        fixture.pump(.36, advance=False)
        # A reference consumer polls the shared authority; no ROS timer runs
        # high-frequency control. Model that consumer's next read explicitly.
        fixture.adapter.sample(fixture.source)
        checks["receipt_watchdog_with_paused_source_clock"] = (
            not fixture.adapter.snapshot()["authorized"] and
            fixture.adapter.snapshot()["reason"] == "navigation_command_expired")
        assert fixture.control("STOP", task="pause_stop").accepted
        fixture.adapter.update_mcu_status("SAFE_HOLD", "heartbeat_timeout")
        independent = fixture.control("ACQUIRE", task="fault_task")
        checks["independent_mcu_fault_never_rearmed"] = (
            not independent.accepted and independent.reason == "heartbeat_timeout")
        # Explicit fixture-only new MCU feedback, not a ROS recovery command.
        fixture.adapter.update_mcu_status("NORMAL", "software_fixture_only")
        assert fixture.control("ACQUIRE", task="gid_task").accepted
        fixture.pump(.08, commands=True)
        fixture.rogue = Node("rogue_controller", context=fixture.context)
        rogue_pub = fixture.rogue.create_publisher(TwistStamped, "/companionbot/navigation/cmd_vel", 1)
        assert rogue_pub is not None
        fixture.until(lambda: fixture.runtime.snapshot()["reason"] == "navigation_multiple_command_publishers")
        checks["duplicate_command_publisher_revokes"] = not fixture.adapter.snapshot()["authorized"]
        fixture.rogue.destroy_node(); fixture.rogue = None
        fixture.pump(.15)
        checks["publisher_fault_does_not_auto_recover"] = not fixture.control("CHECK", task="post_fault").accepted
        assert fixture.control("STOP", task="shutdown_stop").accepted
        authority = fixture.adapter.snapshot()
    finally:
        runtime = fixture.close()
    checks["native_executor_closed"] = runtime["closed"] and runtime["thread_stopped"]
    report = dict(scope="actual_local_DDS_software_fixture_no_actuator_no_hardware_certification",
                  hardware_execution_ready=False, checks=checks, passed=all(checks.values()),
                  runtime=runtime, authority=authority)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    assert report["passed"], report
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    result = run(parser.parse_args().output_dir.resolve())
    print(json.dumps(result, indent=2))
