"""Finite, headless spatial diagnostic using the unchanged frozen runner.

Sensor-only state leaves observe(); physical poses stay inside ray casting or
the caller's explicitly post-hoc verification callback. No DDS call is made here.
"""
from dataclasses import asdict
import copy
import json
import math
from pathlib import Path
import time
from uuid import uuid4
from contextlib import nullcontext, ExitStack

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry, load_sim_lidar
from companionbot_control.estimation.longitudinal_estimation import load_longitudinal_estimator_config
from companionbot_control.estimation.spatial import SpatialAttitude, SpatialSample
from companionbot_simulation.core.spatial_scene import prepare_indoor_model, VirtualPlanarLidar


class SpatialSession:
    def __init__(self, output_dir, *, scenario="mixed", duration_s=14., epoch=None,
                 on_frame=None, stop=None, realtime_factor=0., posthoc=None, imu_dropout_at_s=-1.,
                 geometry_config=None, lidar_config=None, world="indoor", world_assets=None,
                 route_file=None, motion_adapter=None, viewer=False):
        if (scenario not in ("balance", "forward", "turn", "mixed", "pitch", "mapping", "navigation")
                or world not in ("indoor", "bookstore")
                or not math.isfinite(duration_s) or not .2 <= duration_s <= (1800 if world == "bookstore" else 300)
                or not math.isfinite(realtime_factor) or realtime_factor < 0
                or not math.isfinite(imu_dropout_at_s)):
            raise ValueError("invalid_spatial_simulation_configuration")
        self.output_dir, self.scenario, self.duration_s = Path(output_dir), scenario, duration_s
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.epoch, self.on_frame, self.stop = epoch or uuid4().hex, on_frame, stop
        self.factor, self.posthoc = realtime_factor, posthoc
        self.dropout_at = imu_dropout_at_s
        self.world, self.world_assets = world, world_assets
        self.route_file, self.motion_adapter = route_file, motion_adapter
        self.viewer_requested, self.viewer = bool(viewer), None
        self._cleanup = ExitStack()
        if (scenario == "navigation") != (motion_adapter is not None):
            raise ValueError("navigation_requires_explicit_motion_adapter")
        if route_file and scenario != "mapping":
            raise ValueError("route_file_requires_mapping_scenario")
        self.geometry = load_spatial_geometry(geometry_config)
        self.lidar_config = load_sim_lidar(lidar_config)
        self.attitude = SpatialAttitude(load_longitudinal_estimator_config().pitch)
        self.sequence = 0
        self.scan = None
        self.samples = []
        self.frame_count = self.scan_count = 0
        self.last_error = ""
        self._log = None
        self.pacing_slip_s = 0.

    def setup(self, sim):
        if self.viewer_requested:
            # User-invoked GUI only. This same model/data is stepped exclusively
            # by the unchanged runner; no second simulator or controller exists.
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(sim.model, sim.data,
                show_left_ui=False, show_right_ui=False)
            self._cleanup.callback(self.viewer.close)
            with self.viewer.lock():
                self.viewer.opt.geomgroup[3] = 0
                self.viewer.opt.geomgroup[4] = 0
                if self.world == "bookstore":
                    self.viewer.opt.geomgroup[0] = 0
                    self.viewer.cam.lookat[:] = [0., 0., .4]
                    self.viewer.cam.distance, self.viewer.cam.azimuth, self.viewer.cam.elevation = 23., 125., -58.
        if self.scenario == "pitch":
            # Deliberate initial condition, not a controller/estimator truth input.
            sim.reset(pitch_rad=self.nominal_pitch + math.radians(5.))
            sim.calibrate_imu_stationary()
        self.lidar = VirtualPlanarLidar(sim.model, self.lidar_config)
        self.scan_period = round(self.lidar.config["period_s"] / sim.physics_dt)
        if not math.isclose(self.scan_period*sim.physics_dt, self.lidar.config["period_s"], abs_tol=1e-12):
            raise ValueError("lidar period must be an integer physics tick count")
        if self.dropout_at >= 0:
            generate = sim._imu_sensor.generate_packet
            def injected_dropout(accel, gyro, sample_time_s):
                if sample_time_s < self.dropout_at:
                    return generate(accel, gyro, sample_time_s)
            sim._imu_sensor.generate_packet = injected_dropout
        return {"spatial_diagnostic": True, "imu_dropout_at_s": self.dropout_at}

    def physics_step(self, sim):
        if self.viewer is not None and sim.step_index % 50 == 0:
            if not self.viewer.is_running():
                raise InterruptedError("user_closed_simulation_viewer")
            self.viewer.sync()
        if self.stop is not None and self.stop.is_set():
            raise InterruptedError("spatial simulation shutdown")
        if sim.step_index % self.scan_period == 0:
            self.scan = self.lidar.sample(sim.model, sim.data)
            self.scan_count += 1
        if self.posthoc is not None:
            # One-way acceptance sink; its return value is never used.
            self.posthoc(sim)
        if self.factor:
            now = time.perf_counter()
            due = self.started + float(sim.data.time)/self.factor
            # Planner/OS stalls must slow wall execution, not release a burst
            # of simulated clock/state that overwhelms the bounded DDS taps.
            if now - due > .02/self.factor:
                self.started += now - due
                self.pacing_slip_s += now - due
                due = now
            delay = due - now
            if delay > 0:
                if self.stop is None:
                    time.sleep(delay)
                else:
                    self.stop.wait(delay)

    def output(self, context):
        if not hasattr(self, "last_attitude"):
            # The original runner has just consumed the held-stationary startup
            # packet. Initialize from that SAME public readout before release;
            # a first free-fall accelerometer packet has no observable tilt.
            readout = context["sim"].last_imu_readout
            accel, gyro = readout.measurement()
            self.last_attitude = self.attitude.update(
                self.geometry.vector_to_ros(accel), self.geometry.vector_to_ros(gyro),
                context["estimate"].theta_measurement_time_rad,
                readout.packet.sample_time_s, readout.measurement_age_s)
        return {}  # No augmentation, no actuator authority.

    def observe(self, context):
        self.sequence += 1
        estimate = context["estimate_after"]
        # Read ONLY the public consumed sensor measurement/timing interface.
        # ImuReadout's hidden noise/ideal sample fields are never accessed here.
        readout = context["sim"].last_imu_readout
        accel, gyro = readout.measurement()
        accel_ros, gyro_ros = self.geometry.vector_to_ros(accel), self.geometry.vector_to_ros(gyro)
        valid = not (readout.imu_invalid or readout.imu_stale)
        if valid:
            attitude = self.attitude.update(accel_ros, gyro_ros, estimate.theta_measurement_time_rad,
                                            readout.packet.sample_time_s, readout.measurement_age_s)
            self.last_attitude = attitude
        elif not hasattr(self, "last_attitude"):
            raise RuntimeError("spatial attitude unavailable at startup")
        else:
            # Carry values only as explicitly INVALID evidence, never publish as
            # current IMU/odom/TF. The spatial consumer must fail closed.
            attitude = self.last_attitude
        sample = SpatialSample(self.epoch, self.sequence, context["time_s"],
            estimate.position_hat_m, estimate.velocity_hat_m_s,
            attitude.roll_rad, estimate.theta_hat_rad, attitude.yaw_rad,
            attitude.roll_rate_rad_s, estimate.theta_dot_hat_rad_s, attitude.yaw_rate_rad_s,
            tuple(map(float, estimate.wheel_relative_angle_hat_rad)),
            tuple(map(float, estimate.wheel_relative_velocity_hat_rad_s)),
            readout.packet.sample_time_s, readout.packet.available_time_s, readout.measurement_age_s,
            valid, readout.imu_stale, True)
        if self.sequence % 10:
            return {}
        frame = {"state": sample, "accel_ros": tuple(map(float, accel_ros)),
                 "gyro_ros": tuple(map(float, gyro_ros)), "scan": self.scan,
                 "received_at_s": time.perf_counter()}
        self.scan = None
        self.frame_count += 1
        record = asdict(sample)
        record["accel_ros"], record["gyro_ros"] = frame["accel_ros"], frame["gyro_ros"]
        self.samples.append(record)
        if self._log is not None:
            self._log.write(json.dumps(record) + "\n")
        if self.on_frame is not None:
            self.on_frame(frame)
        return {}

    def run(self):
        from companionbot_simulation.scenarios import run_stage3b_yaw_control as baseline
        from companionbot_simulation.scenarios import run_stage5_v1_5_pi_mcu_stream as stage5
        common, _, _, frozen = stage5.load_common(stage5.load_json(stage5.CONFIG_PATH))
        if not all(frozen.values()):
            raise RuntimeError("frozen baseline check failed")
        prepare = prepare_indoor_model
        world_options = {}
        if self.world == "bookstore":
            from companionbot_simulation.core.retail_scene import prepare_retail_model
            prepare = prepare_retail_model
            world_options["asset_dir"] = self.world_assets
        model = prepare(repository_root()/"models/minisegway/mini_segway.xml",
                        self.output_dir/"indoor_model.xml", geometry=self.geometry,
                        lidar=self.lidar_config, **world_options)
        scenario = copy.deepcopy(baseline.MIXED_SCENARIO)
        scenario["name"], scenario["duration_s"] = "spatial_" + self.scenario, self.duration_s
        if self.scenario in ("balance", "pitch", "turn", "navigation"):
            scenario["linear_velocity_schedule"] = [{"time_s": 0., "command": 0.}]
        if self.scenario in ("balance", "pitch", "forward", "navigation"):
            scenario["yaw_rate_schedule"] = [{"time_s": 0., "command": 0.}]
        if self.scenario == "turn":
            scenario["yaw_rate_schedule"] = [{"time_s": 0., "command": 0.}, {"time_s": 1., "command": .5},
                                             {"time_s": 6., "command": 0.}]
        (self.output_dir/"configuration.json").write_text(json.dumps({
            "scenario": scenario, "epoch": self.epoch, "geometry_id": self.geometry.geometry_id,
            "physics_dt_s": .001, "control_dt_s": .002, "state_period_s": .02,
            "frozen_checks": frozen, "imu_dropout_at_s": self.dropout_at,
            "world": self.world, "world_assets": str(self.world_assets) if self.world_assets else None,
            "viewer_requested": self.viewer_requested,
            "route_file": str(self.route_file) if self.route_file else None,
            "allocation_policy": "finite_mapping_cyclic_gc_deferred" if self.scenario in ("mapping", "navigation") else "unchanged",
            "Q_actuator": False, "hardware_execution_ready": False}, indent=2) + "\n")
        yaw = common[0]["yaw"]
        reference_options = {}
        allocation_metrics = {}
        allocation_scope = nullcontext()
        if self.scenario in ("mapping", "navigation"):
            from companionbot_simulation.runtime.mapping_motion import mapping_reference, mapping_allocation_scope
            if self.scenario == "mapping":
                arbiter, link, route = mapping_reference(common, stage5, stage5.load_json(stage5.CONFIG_PATH),
                                                       route_file=self.route_file)
            else:
                from companionbot_simulation.runtime.navigation_motion import build_navigation_reference
                arbiter, link, route = build_navigation_reference(common, stage5,
                    stage5.load_json(stage5.CONFIG_PATH), self.motion_adapter)
            allocation_scope = mapping_allocation_scope(allocation_metrics)
            reference_options = dict(reference_source=arbiter, mcu_local_state_callback=link.mcu_local_state,
                pre_reference_tick_callback=link.before_control_tick)
            (self.output_dir/"mapping_route.json").write_text(json.dumps(route, indent=2)+'\n')
        self.nominal_pitch = float(common[7]["parameters"]["theta_eq_rad"])
        self.started = time.perf_counter()
        with (self.output_dir/"sensorized_state.jsonl").open("w") as self._log, allocation_scope, self._cleanup:
            result = baseline.run_case(scenario, yaw["K_psi_nm_per_rad"], yaw["K_r_nm_per_rad_s"],
                yaw_enabled=True, motor_mismatch_enabled=False, common=common, keep_history=True,
                empty_model_path=model, control_observer=self, physics_step_callback=self.physics_step,
                simulation_setup_callback=self.setup, **reference_options)
        self._log = None
        (self.output_dir/"frozen_result_posthoc.json").write_text(json.dumps(result, indent=2) + "\n")
        if self.scenario in ("mapping", "navigation"):
            (self.output_dir/"motion_authority.json").write_text(json.dumps({
                "arbiter_state": arbiter.state, "arbiter_reason": arbiter.reason,
                "safety_events": arbiter.events, "receiver_events": arbiter.receiver.events,
                "producer_events": link.producer.events, "transport_events": link.transport.events,
                "producer_epochs": [{"reference_epoch": p.reference_epoch, "events": p.events}
                    for p in getattr(link, "all_producers", [link.producer])],
                "packet_events": link.packet_events,
                "yaw_events": arbiter.yaw_source.events, "allocation_policy": allocation_metrics}, indent=2)+'\n')
        return result
