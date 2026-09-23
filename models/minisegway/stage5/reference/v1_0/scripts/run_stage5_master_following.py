"""Stage 5 V1: 20 Hz Master observations to 500 Hz TWIP reference blocks."""

from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import run_stage3b_yaw_control as stage3b  # noqa: E402
import run_stage4_final_closeout as stage4_final  # noqa: E402
import run_stage4a_slope_robustness as stage4a  # noqa: E402
from control.rolling_reference import (  # noqa: E402
    MotionIntent,
    RuckigFixedHorizonVelocityPlanner,
    RuckigRollingVelocityPlanner,
    RollingReferenceBlockPlanner,
    RollingReferenceSource,
    TargetObservation,
)
from sim.slope_estimation import (  # noqa: E402
    analytic_theta_eq,
    equilibrium_sum_torque_nm,
)


MODEL_DIR = ROOT / "models" / "minisegway"
STAGE_DIR = MODEL_DIR / "stage5"
CONFIG_PATH = STAGE_DIR / "config" / "stage5_master_following_config.json"
RESULT_DIR = STAGE_DIR / "results"
PLOT_DIR = RESULT_DIR / "plots"
METRICS_PATH = RESULT_DIR / "stage5_master_following_metrics.json"
HISTORY_PATH = RESULT_DIR / "stage5_master_following_history.json"
REPORT_PATH = RESULT_DIR / "STAGE5_MASTER_FOLLOWING.md"


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )


def schedule_value(schedule: list[dict], time_s: float) -> float:
    return float(np.interp(
        float(time_s),
        [float(item["time_s"]) for item in schedule],
        [float(item["value"]) for item in schedule],
    ))


class TestOnlyMasterObservationProvider:
    """The only Stage 5 object allowed to read simulated Master/robot truth."""

    def __init__(self, config: dict) -> None:
        self.config = config
        self.period_s = 1.0 / float(config["observation_frequency_hz"])
        self.master_config = config["test_master"]
        self.master_xy_m = np.zeros(2, dtype=float)
        self.master_heading_rad = -math.pi / 2.0
        self.last_time_s = 0.0
        self.next_capture_time_s = 0.0
        self.latest_observation: TargetObservation | None = None
        self.observation_history: list[dict] = []

    @staticmethod
    def _robot_pose(sim) -> tuple[np.ndarray, np.ndarray]:
        root = sim.model.joint("root").id
        qpos_adr = int(sim.model.jnt_qposadr[root])
        chassis = sim.model.body("chassis").id
        position = np.asarray(sim.data.qpos[qpos_adr : qpos_adr + 3], dtype=float)
        local_to_world = np.asarray(
            sim.data.xmat[chassis], dtype=float
        ).reshape(3, 3)
        return position, local_to_world

    def setup(self, sim) -> dict:
        robot_position, local_to_world = self._robot_pose(sim)
        forward_world = -local_to_world[:, 1]
        distance = float(self.master_config["initial_forward_distance_m"])
        self.master_xy_m = (
            robot_position[:2] + distance * forward_world[:2]
        )
        self.master_heading_rad = math.atan2(forward_world[1], forward_world[0])
        self.last_time_s = float(sim.data.time)
        self.next_capture_time_s = self.last_time_s
        self._capture(sim)
        self.next_capture_time_s += self.period_s
        return {
            "stage5_test_only_master": True,
            "observation_frequency_hz": 1.0 / self.period_s,
            "GT_boundary": "provider-only",
        }

    def _capture(self, sim) -> None:
        robot_position, local_to_world = self._robot_pose(sim)
        relative_world = np.r_[self.master_xy_m - robot_position[:2], 0.0]
        relative_body = local_to_world.T @ relative_world
        observation = TargetObservation(
            capture_time_s=float(sim.data.time),
            x_forward_m=-float(relative_body[1]),
            y_left_m=-float(relative_body[0]),
        )
        self.latest_observation = observation
        self.observation_history.append({
            "capture_time_s": observation.capture_time_s,
            "x_forward_m": observation.x_forward_m,
            "y_left_m": observation.y_left_m,
        })

    def on_physics_step(self, sim) -> None:
        time_s = float(sim.data.time)
        dt_s = time_s - self.last_time_s
        midpoint = self.last_time_s + 0.5 * dt_s
        speed = schedule_value(
            self.master_config["speed_schedule_m_s"], midpoint
        )
        yaw_rate = schedule_value(
            self.master_config["yaw_rate_schedule_rad_s"], midpoint
        )
        midpoint_heading = self.master_heading_rad + 0.5 * yaw_rate * dt_s
        self.master_xy_m += dt_s * speed * np.asarray([
            math.cos(midpoint_heading), math.sin(midpoint_heading)
        ])
        self.master_heading_rad += yaw_rate * dt_s
        self.last_time_s = time_s
        if time_s + 1e-12 >= self.next_capture_time_s:
            self._capture(sim)
            self.next_capture_time_s += self.period_s

    def read(self) -> TargetObservation:
        if self.latest_observation is None:
            raise RuntimeError("Master observation provider has not been initialized")
        return self.latest_observation

    def diagnostic(self, sim) -> dict:
        del sim
        observation = self.read()
        return {
            "target_observation_capture_time_s": observation.capture_time_s,
            "target_observation_x_forward_m": observation.x_forward_m,
            "target_observation_y_left_m": observation.y_left_m,
        }


class SimpleFollower:
    def __init__(self, config: dict) -> None:
        self.config = config
        self.accepted_velocity_m_s = float(
            config.get("initial_accepted_velocity_m_s", 0.0)
        )

    def __call__(self, observation: TargetObservation) -> MotionIntent:
        distance_error = (
            float(observation.x_forward_m)
            - float(self.config["desired_forward_distance_m"])
        )
        dead_zone = self.config.get("distance_dead_zone_m")
        inside_dead_zone = (
            dead_zone is not None
            and float(dead_zone[0]) <= float(observation.x_forward_m)
            <= float(dead_zone[1])
        )
        if inside_dead_zone:
            velocity = self.accepted_velocity_m_s
        else:
            velocity = float(np.clip(
                float(self.config["linear_distance_gain_s_inv"]) * distance_error,
                -float(self.config["maximum_abs_linear_velocity_m_s"]),
                float(self.config["maximum_abs_linear_velocity_m_s"]),
            ))
            self.accepted_velocity_m_s = velocity
        bearing_left = math.atan2(
            float(observation.y_left_m),
            max(0.05, float(observation.x_forward_m)),
        )
        # CompanionBot forward is body -Y; positive +Z yaw turns body-forward
        # toward body +X (right), so a positive left bearing requests negative yaw.
        yaw_rate = float(np.clip(
            -float(self.config["yaw_bearing_gain_s_inv"]) * bearing_left,
            -float(self.config["maximum_abs_yaw_rate_rad_s"]),
            float(self.config["maximum_abs_yaw_rate_rad_s"]),
        ))
        return MotionIntent(
            source_time_s=float(observation.capture_time_s),
            linear_velocity_target_m_s=velocity,
            yaw_rate_target_rad_s=yaw_rate,
        )


def make_reference_source(
    config: dict, common: tuple, provider: TestOnlyMasterObservationProvider
) -> RollingReferenceSource:
    _, runtime_config, dynamic_config, motion_config, offline, *_ = common
    fit = offline["fit"]
    limits = motion_config["reference_limits"]
    horizon = runtime_config["velocity_lifecycle"]["horizon_search"]
    planner_class = (
        RuckigFixedHorizonVelocityPlanner
        if "fixed_horizon_s" in config["rolling_projection"]
        else RuckigRollingVelocityPlanner
    )
    planner_kwargs = {}
    if planner_class is RuckigFixedHorizonVelocityPlanner:
        planner_kwargs["rolling_horizon_s"] = float(
            config["rolling_projection"]["fixed_horizon_s"]
        )
        planner_kwargs["terminal_hidden_penalty_weight"] = float(
            config["rolling_projection"]["terminal_hidden_penalty_weight"]
        )
    preview = planner_class(
        controller_dt_s=float(motion_config["controller_dt_s"]),
        max_velocity_m_s=float(limits["max_velocity_m_s"]),
        max_acceleration_m_s2=float(limits["max_acceleration_m_s2"]),
        max_jerk_m_s3=float(dynamic_config["max_jerk_m_s3"]),
        A=np.asarray(fit["A_identified"], dtype=float),
        B=np.asarray(fit["B_identified"], dtype=float),
        state_scales=np.asarray(fit["state_scales"], dtype=float),
        input_scale_nm=float(fit["input_scale_nm"]),
        horizon_step_s=float(horizon["candidate_step_s"]),
        maximum_tail_s=float(
            config["rolling_projection"].get("maximum_tail_s", 0.0)
        ),
        normalized_residual_rms_max=float(
            config["rolling_projection"].get(
                "normalized_residual_rms_max",
                horizon["normalized_residual_rms_max"],
            )
        ),
        **planner_kwargs,
    )
    block_config = config["reference_block"]
    if not math.isclose(
        float(block_config["dt_s"]), preview.dt_s, rel_tol=0.0, abs_tol=1e-12
    ):
        raise RuntimeError("Stage 5 reference block dt differs from frozen control dt")
    blocks = RollingReferenceBlockPlanner(
        preview,
        block_samples=int(block_config["sample_count"]),
        feedforward_fade_s=float(
            runtime_config["velocity_lifecycle"]["feedforward_fade_s"]
        ),
    )
    return RollingReferenceSource(
        blocks, provider.read, SimpleFollower(config["follower"])
    )


def metric(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
        "peak_abs": float(np.max(np.abs(values))),
    }


def history_arrays(history: list[dict]) -> dict[str, np.ndarray]:
    keys = (
        "t", "linear_velocity_cmd_m_s", "rolling_v_ref_applied_m_s",
        "v_hat_m_s", "v_GT_m_s", "rolling_a_ref_applied_m_s2",
        "rolling_theta_ref_applied_rad", "theta_hat_rad", "theta_eq_used_rad",
        "theta_GT_rad", "rolling_u_ff_raw_nm",
        "rolling_u_ff_after_lifecycle_nm", "u_ff_used_nm", "u_fb_nm",
        "u_sum_nm", "yaw_rate_cmd_rad_s", "r_hat_rad_s", "r_GT_rad_s",
        "psi_ref_rad", "psi_GT_rad", "rolling_replanned",
        "allocator_differential_clipped", "sum_command_saturated",
    )
    return {
        key: np.asarray([row[key] for row in history])
        for key in keys
    }


def add_replan_lines(axis, times: np.ndarray) -> None:
    for time_s in times:
        axis.axvline(time_s, color="0.75", linewidth=0.45, alpha=0.35)


def make_plots(history: list[dict]) -> list[str]:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    arrays = history_arrays(history)
    time = arrays["t"]
    replans = time[arrays["rolling_replanned"].astype(bool)]
    paths = []

    fig, axis = plt.subplots(figsize=(11, 4.5))
    axis.plot(time, arrays["linear_velocity_cmd_m_s"], label="v_cmd", linewidth=1.4)
    axis.plot(time, arrays["rolling_v_ref_applied_m_s"], label="v_ref", linewidth=1.3)
    axis.plot(time, arrays["v_hat_m_s"], label="v_hat", linewidth=1.0)
    axis.plot(time, arrays["v_GT_m_s"], label="v_GT", linewidth=1.0)
    add_replan_lines(axis, replans)
    axis.set(xlabel="time [s]", ylabel="velocity [m/s]", title="20 Hz intent and velocity response")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=4)
    fig.tight_layout()
    path = PLOT_DIR / "stage5_velocity_response.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path.relative_to(ROOT).as_posix())

    fig, (axis_a, axis_theta) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axis_a.plot(time, arrays["rolling_a_ref_applied_m_s2"], label="a_ref")
    add_replan_lines(axis_a, replans)
    axis_a.set(ylabel="acceleration [m/s²]", title="Reference acceleration and pitch response")
    axis_a.grid(True, alpha=0.25)
    axis_a.legend()
    theta_hat_error = arrays["theta_hat_rad"] - arrays["theta_eq_used_rad"]
    axis_theta.plot(time, np.degrees(arrays["rolling_theta_ref_applied_rad"]), label="theta_ref")
    axis_theta.plot(time, np.degrees(theta_hat_error), label="theta_hat error")
    axis_theta.plot(time, np.degrees(arrays["theta_GT_rad"]), label="theta_GT error")
    add_replan_lines(axis_theta, replans)
    axis_theta.set(xlabel="time [s]", ylabel="pitch error [deg]")
    axis_theta.grid(True, alpha=0.25)
    axis_theta.legend(ncol=3)
    fig.tight_layout()
    path = PLOT_DIR / "stage5_acceleration_pitch_response.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path.relative_to(ROOT).as_posix())

    fig, axis = plt.subplots(figsize=(11, 4.8))
    axis.plot(time, arrays["rolling_u_ff_raw_nm"], label="u_ff_raw", linewidth=1.0)
    axis.plot(time, arrays["u_ff_used_nm"], label="lambda*u_ff_after_lifecycle", linewidth=1.2)
    axis.plot(time, arrays["u_fb_nm"], label="u_fb", linewidth=1.0)
    axis.plot(time, arrays["u_sum_nm"], label="u_total", linewidth=1.2)
    add_replan_lines(axis, replans)
    axis.set(xlabel="time [s]", ylabel="sum torque [N m]", title="Feedforward lifecycle and total torque")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=4)
    fig.tight_layout()
    path = PLOT_DIR / "stage5_feedforward_torque_response.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path.relative_to(ROOT).as_posix())

    fig, (axis_r, axis_psi) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axis_r.plot(time, arrays["yaw_rate_cmd_rad_s"], label="yaw_rate_cmd")
    axis_r.plot(time, arrays["r_hat_rad_s"], label="yaw_rate_hat")
    axis_r.plot(time, arrays["r_GT_rad_s"], label="yaw_rate_GT")
    axis_r.set(ylabel="yaw rate [rad/s]", title="20 Hz yaw command through frozen yaw loop")
    axis_r.grid(True, alpha=0.25)
    axis_r.legend(ncol=3)
    axis_psi.plot(time, arrays["psi_ref_rad"], label="psi_ref")
    axis_psi.plot(time, arrays["psi_GT_rad"], label="psi_GT")
    axis_psi.set(xlabel="time [s]", ylabel="relative yaw [rad]")
    axis_psi.grid(True, alpha=0.25)
    axis_psi.legend(ncol=2)
    fig.tight_layout()
    path = PLOT_DIR / "stage5_yaw_response.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    paths.append(path.relative_to(ROOT).as_posix())

    zoom = (time >= 3.0) & (time <= 4.5)
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(time[zoom], arrays["rolling_a_ref_applied_m_s2"][zoom], label="a_ref")
    axes[1].plot(time[zoom], np.degrees(arrays["rolling_theta_ref_applied_rad"][zoom]), label="theta_ref")
    axes[2].plot(time[zoom], arrays["u_ff_used_nm"][zoom], label="u_ff_applied")
    for axis in axes:
        add_replan_lines(axis, replans[(replans >= 3.0) & (replans <= 4.5)])
        axis.grid(True, alpha=0.25)
        axis.legend()
    axes[0].set(ylabel="m/s²", title="1.5 s rolling-replan seam zoom")
    axes[1].set(ylabel="deg")
    axes[2].set(xlabel="time [s]", ylabel="N m")
    fig.tight_layout()
    path = PLOT_DIR / "stage5_replan_seam_zoom.png"
    fig.savefig(path, dpi=190)
    plt.close(fig)
    paths.append(path.relative_to(ROOT).as_posix())
    return paths


def build_metrics(
    config: dict,
    run: dict,
    source: RollingReferenceSource,
    provider: TestOnlyMasterObservationProvider,
    runtime,
    frozen_checks: dict,
    plots: list[str],
) -> tuple[dict, list[dict]]:
    history = run["history_50hz"]
    arrays = history_arrays(history)
    replanned = arrays["rolling_replanned"].astype(bool)
    replan_indices = np.flatnonzero(replanned)
    adjacent_indices = replan_indices[replan_indices > 0]
    all_step_indices = np.arange(1, len(arrays["t"]))
    nonreplan_indices = all_step_indices[~replanned[1:]]
    acceleration = arrays["rolling_a_ref_applied_m_s2"]
    theta = arrays["rolling_theta_ref_applied_rad"]
    ff_used = arrays["u_ff_used_nm"]
    total = arrays["u_sum_nm"]
    dt_s = float(config["reference_block"]["dt_s"])
    planner_summary = source.summary()
    planning_times = np.asarray([
        item["planning_wall_time_s"]
        for item in planner_summary["replan_events"]
    ], dtype=float)
    seam = {
        "maximum_boundary_p_delta_m": max(
            (abs(item["p_start_delta_m"]) for item in planner_summary["replan_events"]),
            default=0.0,
        ),
        "maximum_boundary_v_delta_m_s": max(
            (abs(item["v_start_delta_m_s"]) for item in planner_summary["replan_events"]),
            default=0.0,
        ),
        "maximum_boundary_theta_delta_rad": max(
            (abs(item["theta_start_delta_rad"]) for item in planner_summary["replan_events"]),
            default=0.0,
        ),
        "maximum_boundary_theta_dot_delta_rad_s": max(
            (abs(item["theta_dot_start_delta_rad_s"]) for item in planner_summary["replan_events"]),
            default=0.0,
        ),
        "maximum_adjacent_a_ref_step_m_s2": float(np.max(np.abs(
            acceleration[adjacent_indices] - acceleration[adjacent_indices - 1]
        ))) if adjacent_indices.size else 0.0,
        "maximum_adjacent_theta_ref_step_rad": float(np.max(np.abs(
            theta[adjacent_indices] - theta[adjacent_indices - 1]
        ))) if adjacent_indices.size else 0.0,
        "maximum_u_ff_applied_step_nm": float(np.max(np.abs(
            ff_used[adjacent_indices] - ff_used[adjacent_indices - 1]
        ))) if adjacent_indices.size else 0.0,
        "maximum_u_total_step_nm": float(np.max(np.abs(
            total[adjacent_indices] - total[adjacent_indices - 1]
        ))) if adjacent_indices.size else 0.0,
        "mean_u_total_step_at_replan_nm": float(np.mean(np.abs(
            total[adjacent_indices] - total[adjacent_indices - 1]
        ))) if adjacent_indices.size else 0.0,
        "mean_u_total_step_without_replan_nm": float(np.mean(np.abs(
            total[nonreplan_indices] - total[nonreplan_indices - 1]
        ))) if nonreplan_indices.size else 0.0,
        "mean_u_ff_applied_step_at_replan_nm": float(np.mean(np.abs(
            ff_used[adjacent_indices] - ff_used[adjacent_indices - 1]
        ))) if adjacent_indices.size else 0.0,
        "mean_u_ff_applied_step_without_replan_nm": float(np.mean(np.abs(
            ff_used[nonreplan_indices] - ff_used[nonreplan_indices - 1]
        ))) if nonreplan_indices.size else 0.0,
    }
    fade_events = planner_summary["fade_events"]
    fade_counts = {
        name: sum(item["event"] == name for item in fade_events)
        for name in ("fade_started", "fade_finished", "fade_interrupted_by_replan")
    }
    jerk = np.diff(acceleration) / dt_s
    metrics = {
        "stage": config["stage"],
        "status": "COMPLETED",
        "upstream": config["upstream"],
        "frozen_baseline_checks": frozen_checks,
        "command_chain": {
            "observation_hz": float(config["observation_frequency_hz"]),
            "reference_block_dt_s": dt_s,
            "reference_block_samples": int(config["reference_block"]["sample_count"]),
            "reference_block_duration_s": dt_s * int(config["reference_block"]["sample_count"]),
            "intent_count": planner_summary["intent_count"],
            "replan_count": planner_summary["replan_count"],
            "block_count": planner_summary["block_count"],
            "rolling_projection_maximum_tail_s": float(
                config["rolling_projection"]["maximum_tail_s"]
            ),
            "maximum_used_lean_recovery_tail_s": max(
                (item["lean_recovery_tail_s"] for item in planner_summary["replan_events"]),
                default=0.0,
            ),
            "planning_wall_time_s": {
                "mean": float(np.mean(planning_times)),
                "p95": float(np.percentile(planning_times, 95.0)),
                "maximum": float(np.max(planning_times)),
            },
        },
        "reference_tracking": {
            "v_ref_minus_v_cmd_m_s": metric(
                arrays["rolling_v_ref_applied_m_s"] - arrays["linear_velocity_cmd_m_s"]
            ),
            "realized_peak_abs_velocity_m_s": float(np.max(np.abs(
                arrays["rolling_v_ref_applied_m_s"]
            ))),
            "realized_peak_abs_acceleration_m_s2": float(np.max(np.abs(acceleration))),
            "realized_peak_abs_jerk_m_s3": float(np.max(np.abs(jerk))),
        },
        "plant_tracking": {
            "v_hat_minus_v_ref_m_s": metric(
                arrays["v_hat_m_s"] - arrays["rolling_v_ref_applied_m_s"]
            ),
            "v_GT_minus_v_ref_m_s": metric(
                arrays["v_GT_m_s"] - arrays["rolling_v_ref_applied_m_s"]
            ),
            "fell": bool(run["longitudinal"]["fell"]),
        },
        "rolling_seams": seam,
        "feedforward_lifecycle": {
            **fade_counts,
            "lambda_ff": float(run["frozen_longitudinal"]["lambda_ff"]),
        },
        "yaw": {
            "yaw_rate_hat_minus_command_rad_s": metric(
                arrays["r_hat_rad_s"] - arrays["yaw_rate_cmd_rad_s"]
            ),
            "yaw_rate_GT_minus_command_rad_s": metric(
                arrays["r_GT_rad_s"] - arrays["yaw_rate_cmd_rad_s"]
            ),
            "differential_clipping_fraction": float(
                run["torque_allocation"]["differential_clipping_fraction"]
            ),
        },
        "actuation": {
            "per_wheel_saturation_fraction": float(
                run["torque_allocation"]["per_wheel_saturation_fraction"]
            ),
            "final_guard_clipping_fraction": float(
                run["torque_allocation"]["final_guard_clipping_fraction"]
            ),
            "requested_wheel_peak_nm": float(
                run["torque_allocation"]["requested_wheel_peak_nm"]
            ),
            "actual_wheel_peak_nm": float(
                run["torque_allocation"]["actual_wheel_peak_nm"]
            ),
        },
        "data_boundary": {
            "GT_provider": "TestOnlyMasterObservationProvider only",
            "follower_inputs": ["capture_time_s", "x_forward_m", "y_left_m"],
            "planner_reads_sim": False,
            "controller_estimator_GT_runtime_dependency": False,
            "observation_count": len(provider.observation_history),
        },
        "stage4_runtime": {
            "final_environment_mode": runtime.supervisor.mode.value,
            "slope_transition_count": len(runtime.transition_log),
            "payload_id_result_count": len(runtime.id_results),
            "payload_lifecycle_events": list(runtime.lifecycle.events),
            "final_payload_mass_kg": runtime.current_model.payload.mass_kg,
            "Q_actuator_enabled": False,
            "slip_enabled": False,
        },
        "fade_events": fade_events,
        "replan_events": planner_summary["replan_events"],
        "plots": plots,
    }
    slim_keys = (
        "t", "linear_velocity_cmd_m_s", "rolling_v_ref_applied_m_s",
        "v_hat_m_s", "v_GT_m_s", "rolling_a_ref_applied_m_s2",
        "rolling_theta_ref_applied_rad", "theta_hat_rad", "theta_eq_used_rad",
        "theta_GT_rad", "rolling_theta_dot_ref_applied_rad_s",
        "rolling_u_ff_raw_nm", "rolling_u_ff_after_lifecycle_nm",
        "u_ff_used_nm", "u_fb_nm", "u_sum_nm", "yaw_rate_cmd_rad_s",
        "r_hat_rad_s", "r_GT_rad_s", "psi_ref_rad", "psi_GT_rad",
        "u_diff_request_nm", "u_diff_used_nm", "u_left_nm", "u_right_nm",
        "actual_left_nm", "actual_right_nm", "sum_command_saturated",
        "allocator_differential_clipped", "rolling_replanned",
        "reference_block_id", "reference_block_sample_index",
        "reference_block_start_time_s", "intent_source_time_s",
        "feedforward_phase", "velocity_lifecycle_phase",
        "target_observation_capture_time_s", "target_observation_x_forward_m",
        "target_observation_y_left_m",
    )
    slim_history = [
        {key: row[key] for key in slim_keys}
        for row in history
    ]
    return metrics, slim_history


def write_report(metrics: dict) -> None:
    ref = metrics["reference_tracking"]["v_ref_minus_v_cmd_m_s"]
    plant = metrics["plant_tracking"]["v_GT_minus_v_ref_m_s"]
    seam = metrics["rolling_seams"]
    fade = metrics["feedforward_lifecycle"]
    yaw = metrics["yaw"]
    actuation = metrics["actuation"]
    chain = metrics["command_chain"]
    lines = [
        "# Stage 5 V1 rolling Master-following",
        "",
        "- OTG: Ruckig 0.19.4 velocity interface; each changed 20 Hz intent replans from the currently executing p/v/a.",
        "- TWIP preview: existing identified-A/B joint projection, with inherited start theta/theta-dot and zero terminal lean manifold.",
        f"- Stage 5-only recovery-tail search cap: {chain['rolling_projection_maximum_tail_s']:.2f} s (maximum used {chain['maximum_used_lean_recovery_tail_s']:.2f} s); the frozen 0.020 residual threshold is unchanged.",
        f"- Planning wall time mean/p95/max: {chain['planning_wall_time_s']['mean']*1000:.1f}/{chain['planning_wall_time_s']['p95']*1000:.1f}/{chain['planning_wall_time_s']['maximum']*1000:.1f} ms on this host.",
        f"- v_ref-v_cmd RMS/peak: {ref['rms']:.4f}/{ref['peak_abs']:.4f} m/s; v_GT-v_ref RMS/peak: {plant['rms']:.4f}/{plant['peak_abs']:.4f} m/s.",
        f"- Replan seams: boundary theta delta {math.degrees(seam['maximum_boundary_theta_delta_rad']):.6f} deg; adjacent a step {seam['maximum_adjacent_a_ref_step_m_s2']:.6f} m/s^2; applied FF/total torque step {seam['maximum_u_ff_applied_step_nm']:.5f}/{seam['maximum_u_total_step_nm']:.5f} N m.",
        f"- Fade start/finish/interrupted: {fade['fade_started']}/{fade['fade_finished']}/{fade['fade_interrupted_by_replan']}.",
        f"- Yaw-rate GT-command RMS: {yaw['yaw_rate_GT_minus_command_rad_s']['rms']:.4f} rad/s; allocator differential clipping {yaw['differential_clipping_fraction']:.4%}.",
        f"- Fall: {metrics['plant_tracking']['fell']}; wheel saturation: {actuation['per_wheel_saturation_fraction']:.4%}; final allocator guard clipping: {actuation['final_guard_clipping_fraction']:.4%}.",
        "- Frozen Stage 4 controller/estimator/yaw/allocator/slope/payload/Q settings were not changed; GT remains inside the test-only observation provider and post-hoc evaluator.",
        "",
        "## Plots",
        "",
    ]
    lines.extend(f"- `{path}`" for path in metrics["plots"])
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    config = load_json(CONFIG_PATH)
    stage4_config = load_json(stage4_final.CONFIG_PATH)
    stage4c_config = load_json(stage4_final.STAGE4C_CONFIG_PATH)
    frozen_checks = stage4_final.validate_frozen_baseline(
        stage4_config, stage4c_config
    )
    common_list = list(stage4_final.make_common(stage4_config, int(config["seed"])))
    common_list[3] = copy.deepcopy(common_list[3])
    # Logging only: keep every 500 Hz control sample for seam inspection.
    common_list[3]["history_frequency_hz"] = 500.0
    common = tuple(common_list)

    provider = TestOnlyMasterObservationProvider(config)
    reference_source = make_reference_source(config, common, provider)
    q_adapter = stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = stage4_final.FinalRuntime(
        reduced=common[7],
        config=stage4_config,
        stage4c_config=stage4c_config,
        q_adapter=q_adapter,
    )
    scenario = {
        "name": "stage5_v1_master_following",
        "duration_s": float(config["episode_duration_s"]),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    manifest = common[0]
    run = stage3b.run_case(
        scenario,
        float(manifest["yaw"]["K_psi_nm_per_rad"]),
        float(manifest["yaw"]["K_r_nm_per_rad_s"]),
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=True,
        payload_mode="empty",
        common_mode_augmentation=q_adapter,
        simulation_setup_callback=provider.setup,
        physics_step_callback=provider.on_physics_step,
        history_diagnostic_callback=provider.diagnostic,
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
        reference_source=reference_source,
    )
    plots = make_plots(run["history_50hz"])
    metrics, history = build_metrics(
        config, run, reference_source, provider, runtime, frozen_checks, plots
    )
    write_json(METRICS_PATH, metrics)
    write_json(HISTORY_PATH, history)
    write_report(metrics)
    print(json.dumps({
        "metrics": str(METRICS_PATH),
        "history": str(HISTORY_PATH),
        "report": str(REPORT_PATH),
        "plots": metrics["plots"],
        "reference_tracking": metrics["reference_tracking"],
        "plant_tracking": metrics["plant_tracking"],
        "rolling_seams": metrics["rolling_seams"],
        "feedforward_lifecycle": metrics["feedforward_lifecycle"],
        "yaw": metrics["yaw"],
        "actuation": metrics["actuation"],
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
