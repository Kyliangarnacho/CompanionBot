"""Diagnostic only: Stage 3 frozen runner with read-only, decimated ROS telemetry.

The original loop owns 1 ms physics/2 ms control. No DDS callback schedules
control, no ROS command input and no perception subscription are provided.
"""
import json
import math
from pathlib import Path
import threading
import time
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as HostClock, ClockType
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rosgraph_msgs.msg import Clock
from companionbot_interfaces.msg import ControlTelemetry
from companionbot_contracts.stamps import source_stamp
from companionbot_contracts.ros_diagnostics import publish_status, require_host_time


class FrozenSimulation(Node):
    def __init__(self):
        super().__init__("frozen_sim", namespace="companionbot")
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("realtime_factor", 1.0)
        from companionbot_contracts.paths import repository_root
        self.declare_parameter("output_dir", str(repository_root() /
                              "models/minisegway/runtime_architecture/results/sessions" / uuid4().hex))
        self.duration = float(self.get_parameter("duration_s").value)
        self.factor = float(self.get_parameter("realtime_factor").value)
        if not 0 < self.duration <= 300 or not math.isfinite(self.factor) or self.factor <= 0:
            raise ValueError("Invalid diagnostic duration/realtime factor")
        require_host_time(self)
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.epoch, self.sequence = uuid4().hex, 0
        self.telemetry = self.create_publisher(ControlTelemetry, "sim/control", qos_profile_sensor_data)
        self.clock_pub = self.create_publisher(Clock, "/clock", qos_profile_sensor_data)
        self.diagnostics = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self._lock, self._pending = threading.Lock(), None
        self.published = self.superseded = 0
        self.state = "STARTING"
        self._steady_clock = HostClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02, self.publish_pending, clock=self._steady_clock)
        self.create_timer(1.0, self.report, clock=self._steady_clock)
        self.stop = threading.Event()
        self.error = None
        self.worker = threading.Thread(target=self._run, name="frozen-simulation")
        self.worker.start()

    def output(self, _context):
        return {}

    def observe(self, context):
        # Only the estimator and applied-command API crosses this adapter.
        # The sim object/GT in the original context is deliberately unused.
        self.sequence += 1
        if self.sequence % 10:
            return {}
        estimate = context["estimate_after"]
        sample = {"sequence": self.sequence, "time_s": context["time_s"], "received_at_s": time.perf_counter(),
                  "position_hat_m": estimate.position_hat_m, "velocity_hat_m_s": estimate.velocity_hat_m_s,
                  "pitch_hat_rad": estimate.theta_hat_rad, "pitch_rate_hat_rad_s": estimate.theta_dot_hat_rad_s,
                  "requested_wheels_nm": context["requested_wheels_nm"].tolist(),
                  "actuator_wheels_nm": context["actual_wheels_nm"].tolist(),
                  "saturated": context["saturated"], "controller_dt_s": context["dt_s"]}
        with self._lock:
            self.superseded += self._pending is not None
            self._pending = sample
            self.state = "RUNNING"
        return {}

    def publish_pending(self):
        with self._lock:
            sample, self._pending = self._pending, None
        if sample is None:
            return
        message = ControlTelemetry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        t = sample["time_s"]
        message.meta = source_stamp("mujoco:estimator", self.epoch, sample["sequence"], t,
                                   clock_domain="mujoco_sim", clock_id="sim:" + self.epoch,
                                   received_at_s=sample["received_at_s"],
                                   time_semantics="control_sample")
        for field in ("position_hat_m", "velocity_hat_m_s", "pitch_hat_rad", "pitch_rate_hat_rad_s",
                      "requested_wheels_nm", "actuator_wheels_nm", "saturated", "controller_dt_s"):
            setattr(message, field, sample[field])
        message.physics_dt_s = 0.001
        message.hardware_execution_ready = False
        self.telemetry.publish(message)
        clock = Clock()
        clock.clock.sec = int(t)
        clock.clock.nanosec = int(round((t - int(t)) * 1e9))
        if clock.clock.nanosec == 1_000_000_000:
            clock.clock.sec += 1
            clock.clock.nanosec = 0
        self.clock_pub.publish(clock)
        self.published += 1

    def snapshot(self):
        with self._lock:
            return {"state": self.state, "control_samples": self.sequence, "published": self.published,
                    "superseded": self.superseded, "pending": self._pending is not None,
                    "error_type": type(self.error).__name__ if self.error else "",
                    "physics_dt_s": .001, "controller_dt_s": .002, "hardware_execution_ready": False}

    def report(self):
        value = self.snapshot()
        publish_status(self, self.diagnostics, value["state"], value,
                       level=DiagnosticStatus.ERROR if self.error else DiagnosticStatus.OK)

    def _run(self):
        try:
            import companionbot_simulation.scenarios.run_stage3b_yaw_control as baseline
            import companionbot_simulation.scenarios.run_stage5_v1_5_pi_mcu_stream as stage5
            # Reuse the current runner's config resolver: archived Stage 3
            # manifest paths predate the results/config layout.
            common, _, _, frozen = stage5.load_common(stage5.load_json(stage5.CONFIG_PATH))
            if not all(frozen.values()):
                raise RuntimeError("frozen baseline check failed")
            manifest = common[0]
            yaw = manifest["yaw"]
            scenario = {"name": "stage9_ros_frozen_smoke", "duration_s": self.duration,
                        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0},
                                                     {"time_s": 1.0, "command": 0.2}],
                        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}]}
            started = time.perf_counter()
            def pace(sim):
                if self.stop.is_set():
                    raise InterruptedError("simulation shutdown")
                target = started + float(sim.data.time) / self.factor
                self.stop.wait(max(0, target - time.perf_counter()))
            result = baseline.run_case(scenario, yaw["K_psi_nm_per_rad"], yaw["K_r_nm_per_rad_s"],
                        yaw_enabled=True, motor_mismatch_enabled=False, common=common,
                        keep_history=True, control_observer=self, physics_step_callback=pace)
            output = self.output_dir
            output.mkdir(parents=True, exist_ok=True)
            (output / "frozen_sim.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        except InterruptedError:
            pass
        except Exception as error:
            self.error = error
            self.get_logger().error("frozen simulation failed: " + repr(error))


def main():
    rclpy.init()
    node = None
    try:
        node = FrozenSimulation()
        while rclpy.ok() and node.worker.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop.set()
            node.worker.join(timeout=5)
            if node.worker.is_alive():
                node.error = RuntimeError("frozen simulation worker did not stop")
            if rclpy.ok():
                node.publish_pending()
            node.state = "ERROR" if node.error else "CLOSED"
            node.output_dir.mkdir(parents=True, exist_ok=True)
            (node.output_dir / "runtime.json").write_text(json.dumps(node.snapshot(), indent=2))
            node.destroy_node()
        rclpy.try_shutdown()
    if node is not None and node.error:
        raise node.error
