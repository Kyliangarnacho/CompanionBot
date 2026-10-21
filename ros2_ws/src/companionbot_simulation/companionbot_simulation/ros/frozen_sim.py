"""Diagnostic only: Stage 3 frozen runner with read-only, decimated ROS telemetry.

The original loop owns 1 ms physics/2 ms control. No DDS callback schedules
control, no ROS command input and no perception subscription are provided.
"""
import json
from pathlib import Path
import threading
import time
from uuid import uuid4

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from companionbot_interfaces.msg import ControlTelemetry
from companionbot_contracts.stamps import source_stamp


class FrozenSimulation(Node):
    def __init__(self):
        super().__init__("frozen_sim", namespace="companionbot")
        self.declare_parameter("duration_s", 8.0)
        self.declare_parameter("realtime_factor", 1.0)
        from companionbot_simulation.scenarios.run_stage3b_yaw_control import ROOT
        self.declare_parameter("output_dir", str(ROOT / "models/minisegway/stage9/results/architecture/ros_sim"))
        self.duration = float(self.get_parameter("duration_s").value)
        self.factor = float(self.get_parameter("realtime_factor").value)
        if not 0 < self.duration <= 300 or self.factor <= 0:
            raise ValueError("Invalid diagnostic duration/realtime factor")
        self.epoch, self.sequence = uuid4().hex, 0
        self.telemetry = self.create_publisher(ControlTelemetry, "sim/control", qos_profile_sensor_data)
        self.clock_pub = self.create_publisher(Clock, "/clock", qos_profile_sensor_data)
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
        message = ControlTelemetry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        t = context["time_s"]
        message.meta = source_stamp("mujoco:estimator", self.epoch, self.sequence, t,
                                   clock_domain="mujoco_sim", clock_id="sim:" + self.epoch,
                                   time_semantics="control_sample")
        message.position_hat_m, message.velocity_hat_m_s = estimate.position_hat_m, estimate.velocity_hat_m_s
        message.pitch_hat_rad, message.pitch_rate_hat_rad_s = estimate.theta_hat_rad, estimate.theta_dot_hat_rad_s
        message.requested_wheels_nm = context["requested_wheels_nm"].tolist()
        message.actuator_wheels_nm = context["actual_wheels_nm"].tolist()
        message.saturated = context["saturated"]
        message.physics_dt_s, message.controller_dt_s = 0.001, context["dt_s"]
        message.hardware_execution_ready = False
        self.telemetry.publish(message)
        clock = Clock()
        clock.clock.sec = int(t)
        clock.clock.nanosec = int(round((t - int(t)) * 1e9))
        if clock.clock.nanosec == 1_000_000_000:
            clock.clock.sec += 1
            clock.clock.nanosec = 0
        self.clock_pub.publish(clock)
        return {}

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
            output = Path(self.get_parameter("output_dir").value)
            output.mkdir(parents=True, exist_ok=True)
            (output / "frozen_sim.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        except InterruptedError:
            pass
        except Exception as error:
            self.error = error
            self.get_logger().error("frozen simulation failed: " + repr(error))


def main():
    rclpy.init()
    node = FrozenSimulation()
    try:
        while rclpy.ok() and node.worker.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.worker.join(timeout=5)
        node.destroy_node()
        rclpy.try_shutdown()
    if node.error:
        raise node.error
