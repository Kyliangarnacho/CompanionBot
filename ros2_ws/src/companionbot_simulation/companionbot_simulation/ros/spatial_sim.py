"""Finite spatial sensor diagnostic. No GUI, Agent or external motion input."""
import json
import math
from pathlib import Path
import signal
import threading
import time
from uuid import uuid4
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import Imu, LaserScan
from rosgraph_msgs.msg import Clock as ClockMessage

from companionbot_interfaces.msg import BalanceState
from companionbot_contracts.paths import repository_root
from companionbot_contracts.stamps import source_stamp
from companionbot_contracts.ros_diagnostics import publish_status
from companionbot_simulation.runtime.spatial_session import SpatialSession
from companionbot_simulation.core.virtual_imu import load_imu_hardware_config


def source_time(seconds):
    ns = round(seconds*1_000_000_000)
    return Time(sec=ns//1_000_000_000, nanosec=ns % 1_000_000_000)


class SpatialSimulation(Node):
    def __init__(self):
        super().__init__("spatial_sim", namespace="companionbot")
        readonly = ParameterDescriptor(read_only=True)
        def parameter(name, default):
            return self.declare_parameter(name, default, readonly).value
        if not self.get_parameter("use_sim_time").value:
            raise ValueError("spatial sensor graph requires use_sim_time=true")
        self.add_on_set_parameters_callback(lambda parameters: SetParametersResult(
            successful=all(p.name != "use_sim_time" or bool(p.value) for p in parameters),
            reason="spatial_graph_clock_requires_restart"))
        output = parameter("output_dir", str(repository_root()/"models/minisegway/spatial_slam_foundation/results/sessions"/uuid4().hex))
        factor = float(parameter("realtime_factor", 1.))
        if not math.isfinite(factor) or not .05 <= factor <= 20:
            raise ValueError("realtime_factor must be in [0.05,20]")
        self.state_pub = self.create_publisher(BalanceState, "state/balance", qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", qos_profile_sensor_data)
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.clock_pub = self.create_publisher(ClockMessage, "/clock", 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.lock = threading.Lock()
        self.pending = self.pending_scan = None
        self.state_superseded = self.scan_superseded = 0
        self.state_published = self.imu_published = self.scan_published = 0
        self.stop = threading.Event()
        self.error, self.state = None, "STARTING"
        self.finish_grace_s = float(parameter("finish_grace_s", .25))
        self.scan_dropout_start = float(parameter("scan_dropout_start_s", -1.))
        self.scan_dropout_duration = float(parameter("scan_dropout_duration_s", 0.))
        if (not all(math.isfinite(v) for v in (self.finish_grace_s, self.scan_dropout_start, self.scan_dropout_duration))
                or self.finish_grace_s < -1 or self.scan_dropout_duration < 0):
            raise ValueError("invalid spatial completion/dropout parameters")
        self.scan_dropped = 0
        self.wait_for_scan_consumer = parameter("wait_for_scan_consumer", False)
        self.truth = [] if parameter("record_truth", False) else None
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        scenario = parameter("scenario", "mixed")
        self.navigation = None
        if scenario == "navigation":
            from companionbot_simulation.ros.navigation_ingress import NavigationIngress
            self.navigation = NavigationIngress(self,
                output_dir=output,
                map_id=parameter("navigation_map_id", ""),
                map_version=parameter("navigation_map_version", ""))
        self.session = SpatialSession(output, scenario=scenario,
            duration_s=float(parameter("duration_s", 14.)), on_frame=self.observe, stop=self.stop,
            realtime_factor=factor, imu_dropout_at_s=float(parameter("imu_dropout_at_s", -1.)),
            geometry_config=parameter("geometry_config", "") or None,
            lidar_config=parameter("lidar_config", "") or None,
            world=parameter("world", "indoor"), world_assets=parameter("world_assets", "") or None,
            route_file=parameter("route_file", "") or None,
            viewer=parameter("viewer", False),
            motion_adapter=self.navigation.adapter if self.navigation else None,
            posthoc=self.record_truth if self.truth is not None else None)
        self.imu_config = load_imu_hardware_config()
        self.last_imu_stamp = -1.
        self.create_timer(.005, self.publish_pending, clock=self.steady)
        self.create_timer(.2, self.report, clock=self.steady)
        self.worker = threading.Thread(target=self.run, name="spatial-simulation")
        self.start_deadline = time.monotonic()+10.

    def record_truth(self, sim):
        # Explicit one-way evaluation sink; no return into sensors or control.
        if sim.step_index % 20:
            return
        c = self.session.geometry.mj_to_ros
        body = sim.model.body("chassis").id
        obstacle_contacts = []
        for contact in sim.data.contact[:sim.data.ncon]:
            if {int(sim.model.geom_group[contact.geom1]), int(sim.model.geom_group[contact.geom2])} == {3, 4}:
                obstacle_contacts.append([sim.model.geom(contact.geom1).name, sim.model.geom(contact.geom2).name])
        self.truth.append({"time_s": float(sim.data.time),
            "position_ros": (c @ sim.data.xpos[body]).tolist(),
            "rpy_ros": Rotation.from_matrix(c @ sim.data.xmat[body].reshape(3, 3) @ c.T).as_euler("xyz").tolist(),
            "obstacle_contacts_posthoc": obstacle_contacts})

    def observe(self, frame):
        with self.lock:
            self.state_superseded += self.pending is not None
            self.pending = frame
            if frame["scan"] is not None:
                self.scan_superseded += self.pending_scan is not None
                self.pending_scan = frame["scan"]
            self.state = "RUNNING"

    def publish_pending(self):
        if self.worker.ident is None and not self.error:
            if (self.state_pub.get_subscription_count() > 0 and
                    (not self.wait_for_scan_consumer or self.scan_pub.get_subscription_count() > 0)):
                self.worker.start()
            elif time.monotonic() > self.start_deadline:
                self.error = RuntimeError("spatial_consumer_startup_timeout")
        with self.lock:
            frame, self.pending = self.pending, None
            scan, self.pending_scan = self.pending_scan, None
        if frame is None:
            return
        sample = frame["state"]
        message = BalanceState()
        message.header.stamp, message.header.frame_id = source_time(sample.time_s), "base_link"
        message.meta = source_stamp("mujoco:spatial", sample.epoch, sample.sequence, sample.time_s,
            clock_domain="mujoco_sim", clock_id="sim:"+sample.epoch, time_semantics="control_sample",
            received_at_s=frame["received_at_s"])
        message.spatial_contract_version = 1
        message.geometry_id = self.session.geometry.geometry_id
        message.ros_time_mapping_id = "identity:sim:" + sample.epoch
        message.ros_time_uncertainty_s = 0.
        for name in ("roll_rad", "pitch_rad", "yaw_rad", "roll_rate_rad_s", "pitch_rate_rad_s", "yaw_rate_rad_s",
                     "imu_sample_time_s", "imu_available_time_s", "imu_valid", "imu_stale", "encoder_valid"):
            setattr(message, name, getattr(sample, name))
        message.traveled_distance_m, message.forward_velocity_m_s = sample.distance_m, sample.velocity_m_s
        message.wheel_relative_angle_rad = list(sample.wheel_angles_rad)
        message.wheel_relative_velocity_rad_s = list(sample.wheel_velocities_rad_s)
        message.imu_measurement_age_s = sample.imu_age_s
        # This is the one simulation clock authority. State/IMU/scan keep their
        # own sample times even if queue coalescing delays their publication.
        self.clock_pub.publish(ClockMessage(clock=message.header.stamp))
        self.state_pub.publish(message)
        self.state_published += 1
        if sample.imu_valid and not sample.imu_stale and sample.imu_sample_time_s > self.last_imu_stamp:
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = source_time(sample.imu_sample_time_s), "imu_link"
            imu.orientation.w = 1.
            imu.orientation_covariance[0] = -1.
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = frame["gyro_ros"]
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = frame["accel_ros"]
            for index in (0, 4, 8):
                imu.angular_velocity_covariance[index] = self.imu_config.gyroscope.noise_density**2 / self.imu_config.sample_period_s
                imu.linear_acceleration_covariance[index] = self.imu_config.accelerometer.noise_density**2 / self.imu_config.sample_period_s
            self.imu_pub.publish(imu)
            self.imu_published += 1
            self.last_imu_stamp = sample.imu_sample_time_s
        if scan is not None:
            if (self.scan_dropout_start >= 0 and self.scan_dropout_start <= scan.time_s
                    < self.scan_dropout_start+self.scan_dropout_duration):
                self.scan_dropped += 1
                return
            laser = LaserScan()
            laser.header.stamp, laser.header.frame_id = source_time(scan.time_s), "lidar_link"
            laser.angle_min, laser.angle_increment = scan.angle_min, scan.angle_increment
            laser.angle_max = scan.angle_min + (len(scan.ranges_m)-1)*scan.angle_increment
            laser.time_increment = 0.  # Instantaneous array, not a pretend spinning sensor.
            laser.scan_time, laser.range_min, laser.range_max = scan.scan_time, scan.range_min, scan.range_max
            laser.ranges = list(scan.ranges_m)
            self.scan_pub.publish(laser)
            self.scan_published += 1

    def snapshot(self):
        return {"state": self.state, "error": repr(self.error) if self.error else "",
                "control_samples": self.session.sequence, "state_published": self.state_published,
                "imu_published": self.imu_published, "scan_published": self.scan_published,
                "state_superseded": self.state_superseded, "scan_superseded": self.scan_superseded,
                "scan_dropout_count": self.scan_dropped, "pacing_slip_s": self.session.pacing_slip_s,
                "physics_dt_s": .001, "controller_dt_s": .002, "hardware_execution_ready": False,
                "world": self.session.world,
                "navigation": self.navigation.adapter.snapshot(include_events=False) if self.navigation else None}

    def report(self):
        if not self.error and len(self.get_publishers_info_by_topic("/clock")) > 1:
            self.error = RuntimeError("multiple_simulation_clock_publishers")
            self.stop.set()
        publish_status(self, self.diagnostic_pub, self.state, self.snapshot(),
            level=DiagnosticStatus.ERROR if self.error else DiagnosticStatus.OK)

    def run(self):
        try:
            self.session.run()
            self.state = "COMPLETED"
        except InterruptedError:
            pass
        except Exception as exc:
            self.error = exc
            self.get_logger().error("spatial simulation failed: " + repr(exc))
        finally:
            if self.truth is not None:
                (self.session.output_dir/"truth_posthoc.jsonl").write_text(
                    ''.join(json.dumps(row)+'\n' for row in self.truth))


def main():
    rclpy.init()
    node = None
    try:
        node = SpatialSimulation()
        while rclpy.ok() and (node.worker.ident is None or node.worker.is_alive()) and not node.error:
            rclpy.spin_once(node, timeout_sec=.05)
        if rclpy.ok():
            node.publish_pending()
            # A short steady-time drain delivers the final source sample before
            # the required-process launch policy closes the graph.
            deadline = time.monotonic() + node.finish_grace_s
            while rclpy.ok() and (node.finish_grace_s == -1 or time.monotonic() < deadline) and not node.error:
                rclpy.spin_once(node, timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        # A terminal process-group SIGINT and launch's forwarded SIGINT can
        # both arrive. Finish bounded worker/log cleanup before accepting more.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            if node.navigation is not None:
                node.navigation.close()
                (node.session.output_dir/"navigation_authority.json").write_text(
                    json.dumps(node.navigation.adapter.snapshot(), indent=2)+"\n")
            node.stop.set()
            if node.worker.ident is not None:
                node.worker.join(timeout=5)
            if node.worker.is_alive():
                node.error = RuntimeError("spatial simulation worker failed to stop")
            node.state = "ERROR" if node.error else "CLOSED"
            if rclpy.ok():
                node.report()
            (node.session.output_dir/"runtime.json").write_text(json.dumps(node.snapshot(), indent=2) + "\n")
            node.destroy_node()
        rclpy.try_shutdown()
    if node is not None and node.error:
        raise node.error
