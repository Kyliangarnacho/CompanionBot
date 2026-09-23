"""Run the single Stage 5 V1.3 three-bug-fix acceptance episode."""

from __future__ import annotations

import copy
import csv
import json
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import run_stage5_master_following as stage5  # noqa: E402
import run_stage5_v1_2_sparse_two_path as stage5_v12  # noqa: E402


CONFIG_PATH = stage5.STAGE_DIR / "config" / "stage5_v1_3_three_bugfixes_config.json"
RESULT_PATH = stage5.RESULT_DIR / "stage5_v1_3_three_bugfixes_results.json"
CSV_PATH = stage5.RESULT_DIR / "stage5_v1_3_three_bugfixes_history.csv"
SUMMARY_PATH = stage5.RESULT_DIR / "STAGE5_V1_3_SUMMARY.md"
PLOTS = {
    "velocity": stage5.PLOT_DIR / "stage5_v1_3_velocity.png",
    "dynamic_environment": stage5.PLOT_DIR / "stage5_v1_3_dynamic_environment.png",
    "torque": stage5.PLOT_DIR / "stage5_v1_3_torque.png",
    "scheduler": stage5.PLOT_DIR / "stage5_v1_3_scheduler.png",
}


def history_arrays(history: list[dict]) -> dict[str, np.ndarray]:
    keys = (
        "t", "raw_linear_velocity_cmd_m_s", "linear_velocity_cmd_m_s",
        "rolling_v_ref_applied_m_s", "v_hat_m_s", "v_GT_m_s",
        "rolling_a_ref_applied_m_s2", "rolling_theta_ref_applied_rad",
        "theta_hat_rad", "theta_eq_used_rad", "theta_GT_rad",
        "environment_mode", "slope_entry_allowed", "rolling_u_ff_raw_nm",
        "u_ff_used_nm", "u_fb_nm", "u_sum_nm", "u_left_nm", "u_right_nm",
        "scheduler_mode", "candidate_delta_v_m_s", "candidate_target_m_s",
        "accepted_velocity_changed",
    )
    return {key: np.asarray([row[key] for row in history]) for key in keys}


def accepted_lines(axis, events: list[dict]) -> None:
    labels_seen: set[str] = set()
    for event in events:
        mode = event["planning_path"]
        color = "tab:purple" if mode == "FULL_DYNAMIC" else "tab:gray"
        label = mode if mode not in labels_seen else None
        labels_seen.add(mode)
        axis.axvline(event["time_s"], color=color, linewidth=0.8, alpha=0.55, label=label)


def make_plots(history: list[dict], events: list[dict], wheel_limit_nm: float) -> list[str]:
    stage5.PLOT_DIR.mkdir(parents=True, exist_ok=True)
    value = history_arrays(history)
    t = value["t"].astype(float)

    fig, axis = plt.subplots(figsize=(12, 5.2))
    axis.plot(t, value["raw_linear_velocity_cmd_m_s"], label="raw_v_cmd")
    axis.plot(t, value["linear_velocity_cmd_m_s"], label="accepted_v_cmd")
    axis.plot(t, value["rolling_v_ref_applied_m_s"], label="v_ref")
    axis.plot(t, value["v_hat_m_s"], label="v_hat", linewidth=0.9)
    axis.plot(t, value["v_GT_m_s"], label="v_GT", linewidth=0.9)
    accepted_lines(axis, events)
    axis.set(xlabel="time [s]", ylabel="velocity [m/s]", title="V1.3 command classification and velocity response")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(PLOTS["velocity"], dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, value["rolling_a_ref_applied_m_s2"], label="a_ref")
    axes[0].axhspan(-0.03, 0.03, color="tab:green", alpha=0.10, label="entry quiet band")
    axes[0].set(ylabel="m/s²", title="Slope-entry gate during dynamic reference")
    axes[0].legend()
    theta_hat_error = value["theta_hat_rad"].astype(float) - value["theta_eq_used_rad"].astype(float)
    axes[1].plot(t, np.degrees(value["rolling_theta_ref_applied_rad"].astype(float)), label="theta_ref")
    axes[1].plot(t, np.degrees(theta_hat_error), label="theta_hat")
    axes[1].plot(t, np.degrees(value["theta_GT_rad"].astype(float)), label="theta_GT")
    axes[1].set(ylabel="pitch error [deg]")
    axes[1].legend(ncol=3)
    environment = np.asarray([1 if mode == "SLOPE" else 0 for mode in value["environment_mode"]])
    allowed = value["slope_entry_allowed"].astype(bool).astype(float)
    axes[2].step(t, environment, where="post", label="environment_mode")
    axes[2].step(t, allowed, where="post", label="slope_entry_allowed", alpha=0.75)
    axes[2].set(xlabel="time [s]", ylabel="mode", yticks=[0, 1], yticklabels=["FLAT / blocked", "SLOPE / allowed"])
    axes[2].legend(ncol=2)
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS["dynamic_environment"], dpi=180)
    plt.close(fig)

    saturated = (
        np.abs(value["u_left_nm"].astype(float)) >= wheel_limit_nm - 1e-12
    ) | (
        np.abs(value["u_right_nm"].astype(float)) >= wheel_limit_nm - 1e-12
    )
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, height_ratios=[4, 1])
    axes[0].plot(t, value["rolling_u_ff_raw_nm"], label="u_ff_raw")
    axes[0].plot(t, value["u_ff_used_nm"], label="u_ff_applied")
    axes[0].plot(t, value["u_fb_nm"], label="u_fb")
    axes[0].plot(t, value["u_sum_nm"], label="u_total")
    axes[0].set(ylabel="sum torque [N m]", title="Torque and wheel saturation")
    axes[0].legend(ncol=4)
    axes[1].fill_between(t, 0.0, saturated.astype(float), step="post", alpha=0.65, label="wheel saturation")
    axes[1].set(xlabel="time [s]", ylabel="sat", yticks=[0, 1])
    axes[1].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS["torque"], dpi=180)
    plt.close(fig)

    modes = np.asarray([{"HOLD": 0, "LIGHTWEIGHT": 1, "FULL_DYNAMIC": 2}[str(mode)] for mode in value["scheduler_mode"]])
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axes[0].step(t, modes, where="post")
    axes[0].set(ylabel="mode", yticks=[0, 1, 2], yticklabels=["HOLD", "LIGHT", "FULL"], title="Scheduler candidate evidence")
    axes[1].plot(t, value["candidate_delta_v_m_s"], label="candidate_delta_v")
    axes[1].axhline(0.30, color="0.5", linestyle="--", linewidth=0.8)
    axes[1].axhline(-0.30, color="0.5", linestyle="--", linewidth=0.8)
    axes[1].set(ylabel="m/s")
    axes[1].legend()
    axes[2].plot(t, value["candidate_target_m_s"].astype(float), label="candidate_target")
    axes[2].set(xlabel="time [s]", ylabel="m/s")
    axes[2].legend()
    for axis in axes:
        accepted_lines(axis, events)
        axis.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS["scheduler"], dpi=180)
    plt.close(fig)
    return [path.relative_to(ROOT).as_posix() for path in PLOTS.values()]


def distribution(values: list[float]) -> dict:
    array = np.asarray(values, dtype=float)
    if not len(array):
        return {"count": 0}
    return {
        "count": int(len(array)), "minimum": float(np.min(array)),
        "mean": float(np.mean(array)), "p95": float(np.percentile(array, 95)),
        "maximum": float(np.max(array)),
    }


def write_csv(history: list[dict]) -> None:
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    with CSV_PATH.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(history[0]), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(history)


def main() -> None:
    config = stage5.load_json(CONFIG_PATH)
    stage4_config = stage5.load_json(stage5.stage4_final.CONFIG_PATH)
    stage4c_config = stage5.load_json(stage5.stage4_final.STAGE4C_CONFIG_PATH)
    frozen = stage5.stage4_final.validate_frozen_baseline(stage4_config, stage4c_config)
    common_list = list(stage5.stage4_final.make_common(stage4_config, int(config["seed"])))
    common_list[3] = copy.deepcopy(common_list[3])
    common_list[3]["history_frequency_hz"] = 500.0
    common = tuple(common_list)
    provider = stage5.TestOnlyMasterObservationProvider(config)
    source = stage5_v12.make_source(config, common, provider)
    q_adapter = stage5.stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = stage5.stage4_final.FinalRuntime(
        reduced=common[7], config=stage4_config, stage4c_config=stage4c_config,
        q_adapter=q_adapter,
        slope_enter_persistence_override_s=float(config["slope_entry"]["enter_persistence_s"]),
        slope_entry_acceleration_quiet_m_s2=float(config["slope_entry"]["acceleration_quiet_abs_m_s2"]),
    )
    scenario = {
        "name": "stage5_v1_3_three_bugfixes",
        "duration_s": float(config["episode_duration_s"]),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    run = stage5.stage3b.run_case(
        scenario,
        float(common[0]["yaw"]["K_psi_nm_per_rad"]),
        float(common[0]["yaw"]["K_r_nm_per_rad_s"]),
        yaw_enabled=True, motor_mismatch_enabled=False, common=common,
        keep_history=True, payload_mode="empty", common_mode_augmentation=q_adapter,
        simulation_setup_callback=provider.setup,
        physics_step_callback=provider.on_physics_step,
        history_diagnostic_callback=provider.diagnostic,
        equilibrium_reference_callback=lambda context: stage5.analytic_theta_eq(runtime.alpha_control_rad, runtime.plant),
        equilibrium_input_callback=lambda context: stage5.equilibrium_sum_torque_nm(
            runtime.alpha_control_rad, float(context["estimate"].velocity_hat_m_s),
            float(context["estimate"].theta_dot_hat_rad_s), runtime.plant,
        ),
        control_observer=runtime, reference_source=source,
    )
    history = run["history_50hz"]
    value = history_arrays(history)
    summary = source.summary()
    events = summary["replan_events"]
    wheel_limit = float(common[8]["known"]["wheel_torque_hard_peak_nm"])
    plots = make_plots(history, events, wheel_limit)
    write_csv(history)
    light = [event for event in events if event["planning_path"] == "LIGHTWEIGHT"]
    full = [event for event in events if event["planning_path"] == "FULL_DYNAMIC"]
    accepted_modes = {"LIGHTWEIGHT": len(light), "FULL_DYNAMIC": len(full)}
    transition_modes = []
    times = value["t"].astype(float)
    for transition in runtime.transition_log:
        index = int(np.argmin(np.abs(times - float(transition["time_s"]))))
        transition_modes.append({**transition, "scheduler_mode": str(value["scheduler_mode"][index])})
    saturated = (
        np.abs(value["u_left_nm"].astype(float)) >= wheel_limit - 1e-12
    ) | (np.abs(value["u_right_nm"].astype(float)) >= wheel_limit - 1e-12)
    metrics = {
        "observation_count": len(provider.observation_history),
        "accepted_count": accepted_modes,
        "planning_wall_time_s": {
            mode: distribution([event["planning_wall_time_s"] for event in selected])
            for mode, selected in (("LIGHTWEIGHT", light), ("FULL_DYNAMIC", full))
        },
        "full_projection_residual_rms": distribution([event["normalized_residual_rms"] for event in full]),
        "full_residual_above_0_020_count": sum(event["normalized_residual_rms"] > 0.020 for event in full),
        "full_solver_failure_count": sum(event["projection_solver_converged"] is False for event in full),
        "slope_transitions": transition_modes,
        "flat_to_slope_during_lightweight_count": sum(
            item["to"] == "SLOPE" and item["scheduler_mode"] == "LIGHTWEIGHT"
            for item in transition_modes
        ),
        "quiet_snap_count": len(summary["quiet_events"]),
        "fade_completed_count": sum(item["event"] == "fade_finished" for item in summary["fade_events"]),
        "wheel_saturation_count": int(np.count_nonzero(saturated)),
        "fall": bool(run["longitudinal"]["fell"]),
    }
    result = {
        "stage": config["stage"], "status": "COMPLETED", "config": config,
        "frozen_baseline_checks": frozen, "metrics": metrics,
        "scheduler_events": summary["scheduler_events"], "replan_events": events,
        "quiet_events": summary["quiet_events"], "fade_events": summary["fade_events"],
        "observation_history": provider.observation_history,
        "intent_history": summary["intents"], "history": history,
        "plots": plots, "history_csv": CSV_PATH.relative_to(ROOT).as_posix(),
    }
    stage5.write_json(RESULT_PATH, result)
    SUMMARY_PATH.write_text(
        "# Stage 5 V1.3 summary\n\n"
        f"- Accepted: {accepted_modes['LIGHTWEIGHT']} LIGHTWEIGHT / {accepted_modes['FULL_DYNAMIC']} FULL_DYNAMIC.\n"
        f"- FLAT→SLOPE during LIGHTWEIGHT: {metrics['flat_to_slope_during_lightweight_count']}.\n"
        f"- FULL residual > 0.020: {metrics['full_residual_above_0_020_count']}; solver failures: {metrics['full_solver_failure_count']}.\n"
        f"- Quiet/fade completed: {metrics['quiet_snap_count']}/{metrics['fade_completed_count']}.\n"
        f"- Wheel saturation samples: {metrics['wheel_saturation_count']}; fall: {str(metrics['fall']).lower()}.\n"
        f"- Raw JSON: `{RESULT_PATH.relative_to(ROOT).as_posix()}`\n"
        f"- Raw CSV: `{CSV_PATH.relative_to(ROOT).as_posix()}`\n",
        encoding="utf-8",
    )
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
