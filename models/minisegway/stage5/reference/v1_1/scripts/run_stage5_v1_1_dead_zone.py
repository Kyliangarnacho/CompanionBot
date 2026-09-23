"""Stage 5 V1.1 dead-zone follower and sparse rolling replans."""

from __future__ import annotations

import copy
import csv
import json
import math
from pathlib import Path
import sys
import time

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import run_stage5_master_following as stage5  # noqa: E402


CONFIG_PATH = (
    stage5.STAGE_DIR / "config" / "stage5_v1_1_dead_zone_config.json"
)
RESULT_JSON_PATH = (
    stage5.RESULT_DIR / "stage5_v1_1_dead_zone_results.json"
)
HISTORY_CSV_PATH = (
    stage5.RESULT_DIR / "stage5_v1_1_dead_zone_history.csv"
)
PLOT_PATHS = {
    "following_velocity": stage5.PLOT_DIR
    / "stage5_v1_1_following_velocity.png",
    "dynamic_reference": stage5.PLOT_DIR
    / "stage5_v1_1_dynamic_reference.png",
    "torque": stage5.PLOT_DIR / "stage5_v1_1_torque.png",
    "yaw": stage5.PLOT_DIR / "stage5_v1_1_yaw.png",
    "rolling_seam_zoom": stage5.PLOT_DIR
    / "stage5_v1_1_rolling_seam_zoom.png",
}


def history_arrays(history: list[dict]) -> dict[str, np.ndarray]:
    keys = (
        "t",
        "target_observation_x_forward_m",
        "linear_velocity_cmd_m_s",
        "rolling_v_ref_applied_m_s",
        "v_hat_m_s",
        "v_GT_m_s",
        "rolling_a_ref_applied_m_s2",
        "rolling_theta_ref_applied_rad",
        "theta_hat_rad",
        "theta_eq_used_rad",
        "theta_GT_rad",
        "rolling_u_ff_raw_nm",
        "u_ff_used_nm",
        "u_fb_nm",
        "u_sum_nm",
        "yaw_rate_cmd_rad_s",
        "r_hat_rad_s",
        "r_GT_rad_s",
        "u_left_nm",
        "u_right_nm",
        "rolling_replanned",
    )
    return {
        key: np.asarray([row[key] for row in history])
        for key in keys
    }


def add_replan_lines(axis, replan_times: np.ndarray) -> None:
    for time_s in replan_times:
        axis.axvline(time_s, color="0.65", linewidth=0.55, alpha=0.45)


def make_plots(history: list[dict], config: dict) -> list[str]:
    stage5.PLOT_DIR.mkdir(parents=True, exist_ok=True)
    arrays = history_arrays(history)
    time_s = arrays["t"].astype(float)
    replans = time_s[arrays["rolling_replanned"].astype(bool)]
    lower, upper = map(float, config["follower"]["distance_dead_zone_m"])

    fig, (distance_axis, velocity_axis) = plt.subplots(
        2, 1, figsize=(11, 7), sharex=True
    )
    distance_axis.plot(
        time_s,
        arrays["target_observation_x_forward_m"],
        label="Master relative distance",
        linewidth=1.3,
    )
    distance_axis.axhspan(lower, upper, color="tab:green", alpha=0.12)
    distance_axis.axhline(lower, color="tab:green", linestyle="--", linewidth=0.8)
    distance_axis.axhline(upper, color="tab:green", linestyle="--", linewidth=0.8)
    add_replan_lines(distance_axis, replans)
    distance_axis.set(ylabel="distance [m]", title="Dead-zone following and velocity response")
    distance_axis.grid(True, alpha=0.25)
    distance_axis.legend()
    velocity_axis.plot(time_s, arrays["linear_velocity_cmd_m_s"], label="v_cmd", linewidth=1.4)
    velocity_axis.plot(time_s, arrays["rolling_v_ref_applied_m_s"], label="v_ref", linewidth=1.3)
    velocity_axis.plot(time_s, arrays["v_hat_m_s"], label="v_hat", linewidth=1.0)
    velocity_axis.plot(time_s, arrays["v_GT_m_s"], label="v_GT", linewidth=1.0)
    add_replan_lines(velocity_axis, replans)
    velocity_axis.set(xlabel="time [s]", ylabel="velocity [m/s]")
    velocity_axis.grid(True, alpha=0.25)
    velocity_axis.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["following_velocity"], dpi=170)
    plt.close(fig)

    fig, (acceleration_axis, pitch_axis) = plt.subplots(
        2, 1, figsize=(11, 7), sharex=True
    )
    acceleration_axis.plot(
        time_s,
        arrays["rolling_a_ref_applied_m_s2"],
        label="a_ref",
    )
    acceleration_axis.set(ylabel="acceleration [m/s²]", title="Dynamic reference")
    acceleration_axis.grid(True, alpha=0.25)
    acceleration_axis.legend()
    theta_hat_error = (
        arrays["theta_hat_rad"].astype(float)
        - arrays["theta_eq_used_rad"].astype(float)
    )
    pitch_axis.plot(
        time_s,
        np.degrees(arrays["rolling_theta_ref_applied_rad"].astype(float)),
        label="theta_ref",
    )
    pitch_axis.plot(
        time_s,
        np.degrees(theta_hat_error),
        label="theta_hat",
        linewidth=1.0,
    )
    pitch_axis.plot(
        time_s,
        np.degrees(arrays["theta_GT_rad"].astype(float)),
        label="theta_GT",
        linewidth=1.0,
    )
    pitch_axis.set(xlabel="time [s]", ylabel="pitch error [deg]")
    pitch_axis.grid(True, alpha=0.25)
    pitch_axis.legend(ncol=3)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["dynamic_reference"], dpi=170)
    plt.close(fig)

    fig, torque_axis = plt.subplots(figsize=(11, 4.8))
    torque_axis.plot(time_s, arrays["rolling_u_ff_raw_nm"], label="u_ff_raw", linewidth=1.0)
    torque_axis.plot(time_s, arrays["u_ff_used_nm"], label="u_ff_applied", linewidth=1.1)
    torque_axis.plot(time_s, arrays["u_fb_nm"], label="u_fb", linewidth=0.9)
    torque_axis.plot(time_s, arrays["u_sum_nm"], label="u_total", linewidth=1.1)
    torque_axis.set(xlabel="time [s]", ylabel="sum torque [N m]", title="Feedforward and feedback torque")
    torque_axis.grid(True, alpha=0.25)
    torque_axis.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["torque"], dpi=170)
    plt.close(fig)

    fig, yaw_axis = plt.subplots(figsize=(11, 4.8))
    yaw_axis.plot(time_s, arrays["yaw_rate_cmd_rad_s"], label="yaw_rate_cmd", linewidth=1.3)
    yaw_axis.plot(time_s, arrays["r_hat_rad_s"], label="yaw_rate_hat", linewidth=1.0)
    yaw_axis.plot(time_s, arrays["r_GT_rad_s"], label="yaw_rate_GT", linewidth=1.0)
    yaw_axis.set(xlabel="time [s]", ylabel="yaw rate [rad/s]", title="Yaw sanity check")
    yaw_axis.grid(True, alpha=0.25)
    yaw_axis.legend(ncol=3)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["yaw"], dpi=170)
    plt.close(fig)

    center_s = float(replans[len(replans) // 2]) if replans.size else 0.4
    zoom = (time_s >= center_s - 0.4) & (time_s <= center_s + 0.4)
    zoom_replans = replans[
        (replans >= center_s - 0.4) & (replans <= center_s + 0.4)
    ]
    fig, zoom_axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    zoom_axes[0].plot(
        time_s[zoom], arrays["rolling_a_ref_applied_m_s2"][zoom], label="a_ref"
    )
    zoom_axes[1].plot(
        time_s[zoom],
        np.degrees(arrays["rolling_theta_ref_applied_rad"][zoom].astype(float)),
        label="theta_ref",
    )
    zoom_axes[2].plot(
        time_s[zoom], arrays["rolling_u_ff_raw_nm"][zoom], label="u_ff_raw"
    )
    for axis in zoom_axes:
        add_replan_lines(axis, zoom_replans)
        axis.grid(True, alpha=0.25)
        axis.legend()
    zoom_axes[0].set(ylabel="m/s²", title="Fixed-horizon rolling seam zoom")
    zoom_axes[1].set(ylabel="deg")
    zoom_axes[2].set(xlabel="time [s]", ylabel="N m")
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["rolling_seam_zoom"], dpi=190)
    plt.close(fig)

    return [path.relative_to(ROOT).as_posix() for path in PLOT_PATHS.values()]


def write_history_csv(history: list[dict]) -> None:
    fields = (
        "t",
        "target_observation_capture_time_s",
        "target_observation_x_forward_m",
        "target_observation_y_left_m",
        "linear_velocity_cmd_m_s",
        "rolling_v_ref_applied_m_s",
        "v_hat_m_s",
        "v_GT_m_s",
        "rolling_a_ref_applied_m_s2",
        "rolling_theta_ref_applied_rad",
        "theta_hat_rad",
        "theta_GT_rad",
        "rolling_u_ff_raw_nm",
        "rolling_u_ff_after_lifecycle_nm",
        "u_ff_used_nm",
        "u_fb_nm",
        "u_sum_nm",
        "yaw_rate_cmd_rad_s",
        "r_hat_rad_s",
        "r_GT_rad_s",
        "u_left_nm",
        "u_right_nm",
        "rolling_replanned",
        "feedforward_phase",
        "velocity_lifecycle_phase",
        "reference_block_id",
        "reference_block_sample_index",
    )
    HISTORY_CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    with HISTORY_CSV_PATH.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in history:
            writer.writerow({key: row[key] for key in fields})


def accepted_velocity_change_count(intents: list[dict]) -> int:
    previous = 0.0
    changes = 0
    for intent in intents:
        target = float(intent["linear_velocity_target_m_s"])
        if not math.isclose(target, previous, rel_tol=0.0, abs_tol=1e-12):
            changes += 1
            previous = target
    return changes


def distribution(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    return {
        "minimum": float(np.min(values)),
        "mean": float(np.mean(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
        "p95": float(np.percentile(values, 95.0)),
        "maximum": float(np.max(values)),
    }


def run_fixed_horizon_regression(config: dict, source) -> dict:
    case = config["fixed_horizon_regression_case"]
    start = np.asarray([
        case["p_ref_m"],
        case["v_ref_m_s"],
        case["theta_ref_rad"],
        case["theta_dot_ref_rad_s"],
    ], dtype=float)
    start_time = time.perf_counter()
    planned = source.planner.preview_planner.plan(
        start,
        float(case["a_ref_m_s2"]),
        float(case["v_cmd_m_s"]),
    )
    planning_wall_time_s = time.perf_counter() - start_time
    plan = planned.plan
    if not np.isfinite(np.r_[
        plan.reference_states.ravel(),
        plan.reference_accelerations_m_s2,
        plan.feedforward_inputs_nm,
    ]).all():
        raise FloatingPointError("fixed-horizon regression produced non-finite output")
    np.testing.assert_allclose(plan.reference_states[0], start, atol=1e-12, rtol=0.0)
    np.testing.assert_allclose(
        plan.reference_accelerations_m_s2[0],
        float(case["a_ref_m_s2"]),
        atol=1e-12,
        rtol=0.0,
    )
    first_block_samples = 25
    return {
        "planning_wall_time_s": planning_wall_time_s,
        "projection_normalized_residual_rms": planned.normalized_residual_rms,
        "terminal_theta_rad": float(plan.reference_states[-1, 2]),
        "terminal_theta_dot_rad_s": float(plan.reference_states[-1, 3]),
        "first_50ms_theta_peak_abs_rad": float(np.max(np.abs(
            plan.reference_states[:first_block_samples, 2]
        ))),
        "first_50ms_u_ff_peak_abs_nm": float(np.max(np.abs(
            plan.feedforward_inputs_nm[:first_block_samples]
        ))),
        "p_v_a_continuous": True,
        "theta_theta_dot_continuous": True,
        "finite": True,
    }


def main() -> None:
    config = stage5.load_json(CONFIG_PATH)
    stage4_config = stage5.load_json(stage5.stage4_final.CONFIG_PATH)
    stage4c_config = stage5.load_json(stage5.stage4_final.STAGE4C_CONFIG_PATH)
    frozen_checks = stage5.stage4_final.validate_frozen_baseline(
        stage4_config, stage4c_config
    )
    common_list = list(stage5.stage4_final.make_common(
        stage4_config, int(config["seed"])
    ))
    common_list[3] = copy.deepcopy(common_list[3])
    common_list[3]["history_frequency_hz"] = 500.0
    common = tuple(common_list)

    provider = stage5.TestOnlyMasterObservationProvider(config)
    source = stage5.make_reference_source(config, common, provider)
    regression = run_fixed_horizon_regression(config, source)
    print(json.dumps({"fixed_horizon_regression": regression}, indent=2))
    q_adapter = stage5.stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = stage5.stage4_final.FinalRuntime(
        reduced=common[7],
        config=stage4_config,
        stage4c_config=stage4c_config,
        q_adapter=q_adapter,
    )
    scenario = {
        "name": "stage5_v1_1_dead_zone_sparse_replan",
        "duration_s": float(config["episode_duration_s"]),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    manifest = common[0]
    run = stage5.stage3b.run_case(
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
        equilibrium_reference_callback=lambda context: stage5.analytic_theta_eq(
            runtime.alpha_control_rad, runtime.plant
        ),
        equilibrium_input_callback=lambda context: stage5.equilibrium_sum_torque_nm(
            runtime.alpha_control_rad,
            float(context["estimate"].velocity_hat_m_s),
            float(context["estimate"].theta_dot_hat_rad_s),
            runtime.plant,
        ),
        control_observer=runtime,
        reference_source=source,
    )

    history = run["history_50hz"]
    arrays = history_arrays(history)
    planner_summary = source.summary()
    intents = planner_summary["intents"]
    replan_events = planner_summary["replan_events"]
    fade_events = planner_summary["fade_events"]
    planning_times = np.asarray([
        event["planning_wall_time_s"] for event in replan_events
    ], dtype=float)
    replan_count = len(replan_events)
    accepted_changes = accepted_velocity_change_count(intents)
    velocity_reference_error = (
        arrays["rolling_v_ref_applied_m_s"].astype(float)
        - arrays["linear_velocity_cmd_m_s"].astype(float)
    )
    velocity_plant_error = (
        arrays["v_GT_m_s"].astype(float)
        - arrays["rolling_v_ref_applied_m_s"].astype(float)
    )
    wheel_limit_nm = float(common[8]["known"]["wheel_torque_hard_peak_nm"])
    wheel_saturated = (
        np.abs(arrays["u_left_nm"].astype(float)) >= wheel_limit_nm - 1e-12
    ) | (
        np.abs(arrays["u_right_nm"].astype(float)) >= wheel_limit_nm - 1e-12
    )
    fade_counts = {
        event_name: sum(
            event["event"] == event_name for event in fade_events
        )
        for event_name in (
            "fade_started",
            "fade_finished",
            "fade_interrupted_by_replan",
        )
    }
    plots = make_plots(history, config)
    write_history_csv(history)
    residuals = np.asarray([
        event["normalized_residual_rms"] for event in replan_events
    ], dtype=float)
    terminal_theta = np.asarray([
        event["terminal_theta_rad"] for event in replan_events
    ], dtype=float)
    terminal_theta_dot = np.asarray([
        event["terminal_theta_dot_rad_s"] for event in replan_events
    ], dtype=float)
    metrics = {
        "observation_count": len(provider.observation_history),
        "accepted_v_cmd_change_count": accepted_changes,
        "expensive_replan_count": replan_count,
        "planner_wall_time_s": {
            "mean": float(np.mean(planning_times)),
            "p95": float(np.percentile(planning_times, 95.0)),
            "maximum": float(np.max(planning_times)),
        },
        "projection_normalized_residual_rms": distribution(residuals),
        "v_ref_minus_v_cmd_m_s": stage5.metric(velocity_reference_error),
        "v_GT_minus_v_ref_m_s": stage5.metric(velocity_plant_error),
        "a_ref_peak_abs_m_s2": float(np.max(np.abs(
            arrays["rolling_a_ref_applied_m_s2"].astype(float)
        ))),
        "theta_ref_peak_abs_rad": float(np.max(np.abs(
            arrays["rolling_theta_ref_applied_rad"].astype(float)
        ))),
        "maximum_replan_seam": {
            "delta_a_ref_m_s2": max(
                abs(event["a_start_delta_m_s2"]) for event in replan_events
            ),
            "delta_theta_ref_rad": max(
                abs(event["theta_start_delta_rad"]) for event in replan_events
            ),
            "delta_theta_dot_ref_rad_s": max(
                abs(event["theta_dot_start_delta_rad_s"])
                for event in replan_events
            ),
            "delta_u_ff_raw_nm": max(
                abs(event["u_ff_raw_start_delta_nm"])
                for event in replan_events
            ),
        },
        "fade": {
            "started": fade_counts["fade_started"],
            "completed": fade_counts["fade_finished"],
            "interrupted": fade_counts["fade_interrupted_by_replan"],
        },
        "wheel_saturation_count": int(np.count_nonzero(wheel_saturated)),
        "fall": bool(run["longitudinal"]["fell"]),
        "fixed_horizon_terminal": {
            "theta_rad": distribution(terminal_theta),
            "theta_dot_rad_s": distribution(terminal_theta_dot),
        },
    }
    result = {
        "stage": config["stage"],
        "status": "COMPLETED",
        "config": config,
        "frozen_baseline_checks": frozen_checks,
        "fixed_horizon_regression": regression,
        "metrics": metrics,
        "observation_history": provider.observation_history,
        "intent_history": intents,
        "replan_events": replan_events,
        "fade_events": fade_events,
        "plots": plots,
        "history_csv": HISTORY_CSV_PATH.relative_to(ROOT).as_posix(),
    }
    stage5.write_json(RESULT_JSON_PATH, result)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
