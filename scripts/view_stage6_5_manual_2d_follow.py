"""Interactive MuJoCo demo for the complete Stage 6.5 2D follow runtime.

A separate Tk control panel sets continuous forward/lateral Master velocity
in the robot's initial ground-plane frame. The MuJoCo right panel is hidden.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import tempfile
import tkinter as tk
import time
from tkinter import ttk

import mujoco
import mujoco.viewer
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for import_path in (ROOT, ROOT / "scripts"):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

import run_stage3b_yaw_control as stage3b
import run_stage5_v1_5_pi_mcu_stream as stage5
import stage6_synthetic_support as stage6_support
import run_stage6_2_safety_decoupling as stage6_2
import run_stage6_4_2d_follow as stage6_4
import run_stage6_5_radial_kf as stage6_5
from control.follow_governor import FollowGovernor, FollowGovernorConfig
from control.radial_velocity_kf import (
    PiRadialObservationPipeline,
    PiRobotStateHistory,
    RadialVelocityKFConfig,
    RadialVelocityKalmanFilter,
)
from control.safety_brake import SafetyBrakeConfig
from control.stage6_pi_mcu_link import (
    InMemoryPacketTransport,
    MCUSafetyArbiter,
    MCUReferenceBlockReceiver,
    PiReferenceBlockProducer,
)
from control.stage6_yaw_servo import BearingYawServo, MCULatestYawSource
from control.trajectory_feedforward import SparseProjectionFactorizationCache
from sim.slope_estimation import analytic_theta_eq, equilibrium_sum_torque_nm


MODEL_DIR = ROOT / "models" / "minisegway"
MASTER_BODY = "master_visual"
MASTER_SPEED_LIMIT_M_S = 0.5
MASTER_SPEED_STEP_M_S = 0.02
UI_REFRESH_S = 1.0 / 60.0
FLOOR_VISUAL_TILE_RADIUS = 3


def create_manual_model() -> Path:
    """Add a non-contact Master mocap cylinder to the unchanged plant model."""

    spec = mujoco.MjSpec()
    spec.from_file(str(MODEL_DIR / "mini_segway.xml"))

    # Keep the original floor geom intact for contacts. Hide only its rendering
    # group and tile the original visual footprint with non-colliding planes.
    # Each tile keeps the original size, material and texture repeat exactly.
    floor = spec.worldbody.first_geom()
    if floor is None or floor.name != "floor":
        raise RuntimeError("expected the base model's world floor geom")
    old_half_x_m = float(floor.size[0])
    old_half_y_m = float(floor.size[1])
    if min(old_half_x_m, old_half_y_m) <= 0.0:
        raise RuntimeError("expected a finite base floor footprint")
    grid_spacing_m = float(floor.size[2])
    material_name = floor.material
    floor.group = 5
    for tile_x in range(-FLOOR_VISUAL_TILE_RADIUS, FLOOR_VISUAL_TILE_RADIUS + 1):
        for tile_y in range(-FLOOR_VISUAL_TILE_RADIUS, FLOOR_VISUAL_TILE_RADIUS + 1):
            visual_floor = spec.worldbody.add_geom()
            visual_floor.name = (
                f"floor_visual_{tile_x + FLOOR_VISUAL_TILE_RADIUS}_"
                f"{tile_y + FLOOR_VISUAL_TILE_RADIUS}"
            )
            visual_floor.type = mujoco.mjtGeom.mjGEOM_PLANE
            visual_floor.pos[:] = [
                tile_x * 2.0 * old_half_x_m,
                tile_y * 2.0 * old_half_y_m,
                0.0,
            ]
            visual_floor.size[:] = [old_half_x_m, old_half_y_m, grid_spacing_m]
            visual_floor.material = material_name
            visual_floor.contype = 0
            visual_floor.conaffinity = 0
            visual_floor.group = 2

    body = spec.worldbody.add_body()
    body.name = MASTER_BODY
    body.mocap = True
    body.pos[:] = [0.0, 0.0, 0.16]
    geom = body.add_geom()
    geom.name = "master_cylinder_visual"
    geom.type = mujoco.mjtGeom.mjGEOM_CYLINDER
    geom.size[:] = [0.105, 0.16, 0.0]
    geom.rgba[:] = [0.12, 0.76, 0.36, 1.0]
    geom.contype = 0
    geom.conaffinity = 0

    spec.compile()
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        suffix=".xml",
        prefix="_stage6_5_manual_follow_",
        dir=MODEL_DIR,
        delete=False,
    ) as output:
        output.write(spec.to_xml())
        return Path(output.name)


class ManualMasterPanel:
    """Stage 5-style coarse velocity sliders in a separate small window."""

    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("Stage 6.x — manual Master movement")
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self.closed = False

        frame = ttk.Frame(self.root, padding=12)
        frame.grid(sticky="nsew")
        self.forward_value = tk.DoubleVar(value=0.0)
        self.lateral_value = tk.DoubleVar(value=0.0)
        self.forward_text = tk.StringVar(value="+0.00 m/s")
        self.lateral_text = tk.StringVar(value="+0.00 m/s")

        ttk.Label(frame, text="Master forward [m/s]").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(frame, textvariable=self.forward_text, width=11, anchor="e").grid(
            row=0, column=1, sticky="e"
        )
        self.forward_scale = tk.Scale(
            frame,
            variable=self.forward_value,
            from_=-MASTER_SPEED_LIMIT_M_S,
            to=MASTER_SPEED_LIMIT_M_S,
            resolution=MASTER_SPEED_STEP_M_S,
            orient=tk.HORIZONTAL,
            length=380,
            showvalue=False,
            tickinterval=0.1,
            command=self._update_forward_text,
        )
        self.forward_scale.grid(row=1, column=0, columnspan=2, sticky="ew",
                                pady=(0, 8))

        ttk.Label(frame, text="Master lateral [m/s]").grid(
            row=2, column=0, sticky="w"
        )
        ttk.Label(frame, textvariable=self.lateral_text, width=11, anchor="e").grid(
            row=2, column=1, sticky="e"
        )
        self.lateral_scale = tk.Scale(
            frame,
            variable=self.lateral_value,
            from_=-MASTER_SPEED_LIMIT_M_S,
            to=MASTER_SPEED_LIMIT_M_S,
            resolution=MASTER_SPEED_STEP_M_S,
            orient=tk.HORIZONTAL,
            length=380,
            showvalue=False,
            tickinterval=0.1,
            command=self._update_lateral_text,
        )
        self.lateral_scale.grid(row=3, column=0, columnspan=2, sticky="ew",
                                pady=(0, 10))
        ttk.Button(frame, text="Stop Master (both = 0)", command=self.zero).grid(
            row=4, column=0, columnspan=2, sticky="ew"
        )
        ttk.Label(
            frame,
            text="Step: 0.02 m/s. Values are velocity, so Master moves continuously.",
        ).grid(row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))

    def _update_forward_text(self, value: str) -> None:
        self.forward_text.set(f"{float(value):+.2f} m/s")

    def _update_lateral_text(self, value: str) -> None:
        self.lateral_text.set(f"{float(value):+.2f} m/s")

    def _on_close(self) -> None:
        self.closed = True

    def zero(self) -> None:
        self.forward_value.set(0.0)
        self.lateral_value.set(0.0)
        self.forward_text.set("+0.00 m/s")
        self.lateral_text.set("+0.00 m/s")

    @property
    def forward_m_s(self) -> float:
        return float(self.forward_value.get())

    @property
    def lateral_m_s(self) -> float:
        return float(self.lateral_value.get())

    def pump(self) -> None:
        self.root.update_idletasks()
        self.root.update()

    def close(self) -> None:
        try:
            self.root.destroy()
        except tk.TclError:
            pass


class ManualMasterTargetSensor(stage6_support.SyntheticTargetSensor):
    """Synthetic sensor that exposes only a noisy body-relative XY observation."""

    def __init__(self, *, initial_distance_m: float, frequency_hz: float,
                 noise_std_m: float, seed: int) -> None:
        scenario = {
            "name": "manual_viewer",
            "duration_s": 300.0,
            "initial_distance_m": float(initial_distance_m),
            "master_speed_schedule": [{"time_s": 0.0, "value_m_s": 0.0}],
        }
        super().__init__(scenario, frequency_hz)
        self.noise_std_m = float(noise_std_m)
        self.rng = np.random.default_rng(int(seed))
        self.mocap_id: int | None = None
        self.forward_world_xy = np.array([0.0, -1.0])
        self.left_world_xy = np.array([1.0, 0.0])

    def setup(self, sim) -> dict:
        result = super().setup(sim)
        body = sim.model.body(MASTER_BODY)
        mocap_ids = np.asarray(body.mocapid).reshape(-1)
        self.mocap_id = int(mocap_ids[0])
        if self.mocap_id < 0:
            raise RuntimeError("manual Master body is not a mocap body")
        forward = np.array([math.cos(self.master_heading_rad),
                            math.sin(self.master_heading_rad)])
        self.forward_world_xy = forward
        self.left_world_xy = np.array([-forward[1], forward[0]])
        self._write_master_pose(sim)
        # setup() captures the first packet before the mocap position is set.
        # Re-capture so its timestamp and relative geometry match the visible Master.
        self.sequence_id = 0
        self.evaluation_history.clear()
        self._capture(sim)
        self.next_capture_time_s = float(sim.data.time) + self.period_s
        return {
            **result,
            "provider": "ManualMasterSyntheticObservation",
            "master_motion_input": "separate Tk velocity sliders",
            "controller_reads_master_truth": False,
            "truth_boundary": "Master world position stays in sensor/viewer bridge",
        }

    def _write_master_pose(self, sim) -> None:
        assert self.mocap_id is not None
        assert self.master_position_xy_m is not None
        sim.data.mocap_pos[self.mocap_id, :2] = self.master_position_xy_m
        sim.data.mocap_pos[self.mocap_id, 2] = 0.16

    def _capture(self, sim) -> None:
        assert self.master_position_xy_m is not None
        robot_position, local_to_world = self._robot_pose(sim)
        relative_world = np.r_[self.master_position_xy_m - robot_position[:2], 0.0]
        relative_body = local_to_world.T @ relative_world
        x_forward = -float(relative_body[1])
        y_left = float(relative_body[0])
        if self.noise_std_m > 0.0:
            noise = self.rng.normal(0.0, self.noise_std_m, size=2)
            x_forward += float(noise[0])
            y_left += float(noise[1])
        from control.rolling_reference import TargetObservation

        self.latest = TargetObservation(
            capture_time_s=float(sim.data.time),
            x_forward_m=x_forward,
            y_left_m=y_left,
            version=1,
            sequence_id=self.sequence_id,
            valid=True,
            confidence=1.0,
        )
        self.sequence_id += 1
        self.latest_evaluation = None

    def advance_manual_motion(self, sim, forward_m_s: float,
                              lateral_m_s: float) -> None:
        assert self.master_position_xy_m is not None
        time_s = float(sim.data.time)
        dt_s = time_s - self.last_physics_time_s
        if dt_s < -1e-12:
            raise RuntimeError("simulation time moved backwards")
        velocity_world = (float(forward_m_s) * self.forward_world_xy
                          + float(lateral_m_s) * self.left_world_xy)
        self.master_position_xy_m += max(dt_s, 0.0) * velocity_world
        self.last_physics_time_s = time_s
        self._write_master_pose(sim)
        while time_s + 1e-12 >= self.next_capture_time_s:
            self._capture(sim)
            self.next_capture_time_s += self.period_s

    def diagnostic(self, sim) -> dict:
        del sim
        if self.latest is None:
            return {}
        return {
            "target_observation_capture_time_s": self.latest.capture_time_s,
            "target_observation_sequence_id": self.latest.sequence_id,
            "target_observation_x_forward_m": self.latest.x_forward_m,
            "target_observation_y_left_m": self.latest.y_left_m,
        }


class ManualMasterDriver:
    """Read the separate UI, move Master and pace the passive viewer."""

    def __init__(self, sensor: ManualMasterTargetSensor,
                 panel: ManualMasterPanel | None, realtime_factor: float,
                 *, headless_smoke: bool = False) -> None:
        self.sensor = sensor
        self.panel = panel
        self.realtime_factor = float(realtime_factor)
        self.headless_smoke = headless_smoke
        self.context = None
        self.viewer = None
        self.start_wall_s: float | None = None
        self.last_sync_sim_s = -math.inf
        self.last_ui_time_s = -math.inf
        self.closed = False
        self.latest_master_command = (0.0, 0.0)

    @staticmethod
    def _smoke_command(time_s: float) -> tuple[float, float]:
        # One short drive/stop/turn/approach sequence exercises the live data path.
        if time_s < 3.0:
            return 0.22, 0.0
        if time_s < 5.0:
            return 0.0, 0.0
        if time_s < 8.0:
            return 0.04, 0.20
        if time_s < 10.5:
            return -0.28, 0.0
        return 0.18, 0.0

    def _open(self, sim) -> None:
        self.context = mujoco.viewer.launch_passive(
            sim.model, sim.data, show_right_ui=False
        )
        self.viewer = self.context.__enter__()
        self.viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.viewer.cam.trackbodyid = sim.model.body("chassis").id
        self.viewer.cam.distance = 3.4
        self.viewer.cam.azimuth = 135.0
        self.viewer.cam.elevation = -18.0
        self.start_wall_s = time.perf_counter() - (
            float(sim.data.time) / self.realtime_factor
        )

    def __call__(self, sim) -> None:
        if self.headless_smoke:
            forward, lateral = self._smoke_command(float(sim.data.time))
            self.latest_master_command = (forward, lateral)
            self.sensor.advance_manual_motion(sim, forward, lateral)
            return

        if self.closed:
            return
        if self.panel is None or self.panel.closed:
            self.close()
            return
        if self.viewer is None:
            self._open(sim)
        assert self.viewer is not None and self.start_wall_s is not None
        if not self.viewer.is_running():
            self.close()
            return

        target_wall_s = self.start_wall_s + (
            float(sim.data.time) / self.realtime_factor
        )
        remaining_s = target_wall_s - time.perf_counter()
        if remaining_s > 0.0:
            time.sleep(remaining_s)

        forward = self.panel.forward_m_s
        lateral = self.panel.lateral_m_s
        self.latest_master_command = (forward, lateral)
        self.sensor.advance_manual_motion(sim, forward, lateral)
        if float(sim.data.time) - self.last_sync_sim_s >= UI_REFRESH_S:
            self.viewer.sync()
            self.last_sync_sim_s = float(sim.data.time)
        if float(sim.data.time) - self.last_ui_time_s >= UI_REFRESH_S:
            self.panel.pump()
            self.last_ui_time_s = float(sim.data.time)

    def termination_guard(self, _sim) -> str | None:
        return ("viewer_closed"
                if self.closed or (self.panel is not None and self.panel.closed)
                else None)

    def close(self) -> None:
        if self.context is not None:
            self.context.__exit__(None, None, None)
        self.context = None
        self.viewer = None
        self.closed = True


def run_demo(duration_s: float, realtime_factor: float, *, headless_smoke: bool) -> dict:
    config = stage5.load_json(stage6_5.CONFIG_PATH)
    stage6_config = stage5.load_json(stage6_4.CONFIG_PATH)
    stage5_config = stage5.load_json(stage5.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen = stage5.load_common(stage5_config)
    if not all(frozen.values()):
        raise RuntimeError(f"frozen Stage 3/4/5 baseline check failed: {frozen}")

    sensor = ManualMasterTargetSensor(
        initial_distance_m=1.8,
        frequency_hz=stage6_config["observation_frequency_hz"],
        noise_std_m=config["measurement_noise_std_m"],
        seed=config["noise_seed"],
    )
    camera_reader = stage6_2.TargetObservationWireReader(sensor)
    state_history = PiRobotStateHistory(
        retention_s=config["state_history_retention_s"]
    )
    pipeline = PiRadialObservationPipeline(
        RadialVelocityKalmanFilter(RadialVelocityKFConfig(**config["kf"])),
        state_history,
    )
    governor_config = FollowGovernorConfig(**stage6_config["governor"])
    governor = FollowGovernor(governor_config, radial_estimate_provider=pipeline.estimate_for)
    planner = stage5.make_full_planner(
        stage5_config,
        common,
        planning_dt_s=float(stage5_config["full_dynamic"]["planning_dt_s"]),
        backend="cached_kkt",
        cache=SparseProjectionFactorizationCache(),
    )
    source = stage5.make_reference_source(
        stage5_config,
        common,
        pipeline,
        follower=governor,
        full_planner_override=planner,
        scheduler_gate_overrides=stage6_config["follow_scheduler_gate_policy"],
    )
    producer = PiReferenceBlockProducer(source)
    receiver = MCUReferenceBlockReceiver(
        dt_s=producer.dt_s, block_samples=producer.block_samples
    )
    transport = InMemoryPacketTransport(
        stage6_config["safety"]["transport_latency_s"]
    )
    yaw_config = stage6_config["yaw"]
    yaw_source = MCULatestYawSource(
        dt_s=producer.dt_s,
        timeout_s=yaw_config["freshness_timeout_s"],
        decay_rate_rad_s2=yaw_config["decay_rate_rad_s2"],
    )
    safety = stage6_config["safety"]
    arbiter = MCUSafetyArbiter(
        receiver,
        SafetyBrakeConfig(
            safety["safe_max_deceleration_m_s2"],
            safety["safe_max_jerk_m_s3"],
            safety["hidden_reference_handoff_time_s"],
            producer.dt_s,
        ),
        heartbeat_timeout_s=safety["heartbeat_timeout_s"],
        starvation_margin_s=safety["reference_starvation_margin_s"],
        nominal_pitch_rad=float(common[7]["parameters"]["theta_eq_rad"]),
        status_sink=lambda payload, time_s: transport.send(
            "pi", "safety_status", payload, time_s
        ),
        yaw_source=yaw_source,
    )
    servo = BearingYawServo(
        gain_s_inv=yaw_config["gain_s_inv"],
        max_rate_rad_s=yaw_config["max_rate_rad_s"],
        deadband_rad=yaw_config["deadband_rad"],
    )
    scenario = {
        "name": "manual_viewer",
        "duration_s": float(duration_s),
        "initial_distance_m": 1.8,
        "master_speed_schedule": [{"time_s": 0.0, "value_m_s": 0.0}],
        "heading_offset_schedule": [{"time_s": 0.0, "value_rad": 0.0}],
    }
    link = stage6_5.KFPiLink(
        transport=transport,
        producer=producer,
        governor=governor,
        arbiter=arbiter,
        crash_time_s=None,
        hazard_time_s=None,
        camera_reader=camera_reader,
        pipeline=pipeline,
        raw_governor=None,
        processing_delay_s=float(config["camera_processing_delay_s"]),
        jitter_s=0.0,
        servo=servo,
        scenario=scenario,
        stage5_config=stage5_config,
        common=common,
        full_planner=planner,
        governor_config=governor_config,
        scheduler_policy=stage6_config["follow_scheduler_gate_policy"],
    )
    q_adapter, runtime = stage5.make_runtime(
        common, stage5_config, stage4_config, stage4c_config
    )
    driver = ManualMasterDriver(
        sensor,
        None if headless_smoke else ManualMasterPanel(),
        realtime_factor,
        headless_smoke=headless_smoke,
    )
    panel = driver.panel
    model_path = None
    manifest = common[0]
    try:
        model_path = create_manual_model()
        run = stage3b.run_case(
            {
                "name": "manual_stage6_5_2d_follow",
                "duration_s": scenario["duration_s"],
                "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
                "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
            },
            float(manifest["yaw"]["K_psi_nm_per_rad"]),
            float(manifest["yaw"]["K_r_nm_per_rad_s"]),
            yaw_enabled=True,
            motor_mismatch_enabled=False,
            common=common,
            keep_history=True,
            payload_mode="empty",
            common_mode_augmentation=q_adapter,
            physics_step_callback=driver,
            simulation_setup_callback=sensor.setup,
            history_diagnostic_callback=lambda sim: {
                **sensor.diagnostic(sim),
                "mcu_safety_state": arbiter.state,
                "reference_epoch": receiver.reference_epoch,
                "pi_latest_observation_sequence_id": pipeline.last_sequence_id,
            },
            equilibrium_reference_callback=lambda context: analytic_theta_eq(
                runtime.alpha_control_rad, runtime.plant
            ),
            equilibrium_input_callback=lambda context: equilibrium_sum_torque_nm(
                runtime.alpha_control_rad,
                float(context["estimate"].velocity_hat_m_s),
                float(context["estimate"].theta_dot_hat_rad_s),
                runtime.plant,
            ),
            control_observer=runtime,
            reference_source=arbiter,
            mcu_local_state_callback=link.mcu_local_state,
            pre_reference_tick_callback=link.before_control_tick,
            model_path_override=model_path,
            termination_guard=(driver.termination_guard if not headless_smoke else None),
        )
    finally:
        driver.close()
        if panel is not None:
            panel.close()
        if model_path is not None:
            model_path.unlink(missing_ok=True)

    accepted = [row for row in pipeline.rows
                if row["alignment"] in ("EXACT", "INTERPOLATED")]
    full_plans = [event for event in source.replan_events
                  if event.get("planning_path") == "FULL_DYNAMIC"]
    summary = {
        "fell": bool(run["longitudinal"]["fell"]),
        "finite": bool(run["finite"]),
        "observation_packets": len(link.camera_bridge_events),
        "kf_aligned_observations": len(accepted),
        "robot_state_packets": link.robot_state_sequence_id,
        "reference_blocks": producer.next_sequence_id,
        "yaw_packets": link.yaw_sequence_id,
        "full_dynamic_plans": len(full_plans),
        "governor_state": governor.state,
        "safety_state": arbiter.state,
        "master_position_world_xy_m": sensor.master_position_xy_m.tolist(),
        "master_control_forward_lateral_m_s": list(driver.latest_master_command),
        "max_abs_yaw_command_rad_s": max(
            (abs(float(event["yaw_rate_target_rad_s"]))
             for event in link.yaw_generation_events), default=0.0
        ),
        "max_applied_velocity_reference_m_s": max(
            (abs(float(row["rolling_v_ref_applied_m_s"]))
             for row in run["history_50hz"]), default=0.0
        ),
        "viewer_closed_early": run.get("termination_reason") == "viewer_closed",
    }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=300.0,
                        help="maximum simulation time [s] (default: 300)")
    parser.add_argument("--realtime-factor", type=float, default=1.0)
    parser.add_argument("--headless-smoke", action="store_true",
                        help="run a short scripted drive/stop/turn/approach check without GUI")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 2.0:
        parser.error("--duration must be finite and at least 2 seconds")
    if not math.isfinite(args.realtime_factor) or args.realtime_factor <= 0.0:
        parser.error("--realtime-factor must be finite and positive")
    duration = min(args.duration, 13.0) if args.headless_smoke else args.duration
    if args.headless_smoke:
        args.realtime_factor = 100.0
        print("RUN headless Stage 6.5 packet/KF/Governor/full/yaw smoke", flush=True)
    else:
        print("Stage 6.5 manual 2D follow viewer", flush=True)
        print("Use the separate Stage 6.x Master control window: forward/lateral "
              "velocity, ±0.50 m/s range, 0.02 m/s steps.", flush=True)
        print("Either window's close button ends the run; the MuJoCo right panel "
              "and in-scene status text are hidden.", flush=True)

    result = run_demo(duration, args.realtime_factor,
                      headless_smoke=args.headless_smoke)
    print("RESULT " + str(result), flush=True)
    if args.headless_smoke:
        required = (
            result["finite"]
            and result["observation_packets"] > 0
            and result["kf_aligned_observations"] > 0
            and result["robot_state_packets"] > 0
            and result["reference_blocks"] > 0
            and result["yaw_packets"] > 0
            and result["full_dynamic_plans"] > 0
        )
        if not required:
            raise SystemExit("headless smoke did not traverse every required runtime stage")


if __name__ == "__main__":
    main()
