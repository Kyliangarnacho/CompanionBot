"""Run the Stage 5 V1.4 synthetic command-lifecycle episode once."""

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
from control.rolling_reference import MotionIntent, TargetObservation  # noqa: E402


CONFIG_PATH = stage5.STAGE_DIR / "config" / "stage5_v1_4_candidate_lifecycle_config.json"
RESULT_PATH = stage5.RESULT_DIR / "stage5_v1_4_candidate_lifecycle_results.json"
CSV_PATH = stage5.RESULT_DIR / "stage5_v1_4_candidate_lifecycle_history.csv"
SUMMARY_PATH = stage5.RESULT_DIR / "STAGE5_V1_4_SUMMARY.md"
PLOTS = {
    "command_lifecycle": stage5.PLOT_DIR / "stage5_v1_4_command_lifecycle.png",
    "reference_dynamics": stage5.PLOT_DIR / "stage5_v1_4_reference_dynamics.png",
    "torque": stage5.PLOT_DIR / "stage5_v1_4_torque.png",
    "scheduler_lifecycle": stage5.PLOT_DIR / "stage5_v1_4_scheduler_lifecycle.png",
}


def schedule_value(schedule: list[dict], time_s: float) -> float:
    return float(np.interp(
        float(time_s),
        [float(item["time_s"]) for item in schedule],
        [float(item["value"]) for item in schedule],
    ))


class SyntheticCommandProvider:
    """20 Hz synthetic raw command source; it reads no simulated ground truth."""

    def __init__(self, config: dict) -> None:
        self.period_s = 1.0 / float(config["observation_frequency_hz"])
        self.schedule = config["synthetic_raw_v_cmd_schedule_m_s"]
        self.index = 0
        self.latest: TargetObservation | None = None
        self.observation_history: list[dict] = []

    def setup(self, sim) -> dict:
        del sim
        self.index = 0
        self.latest = None
        self.observation_history.clear()
        return {
            "provider": "SyntheticCommandProvider",
            "frequency_hz": 1.0 / self.period_s,
            "sim_GT_read": False,
        }

    def read(self) -> TargetObservation:
        time_s = self.index * self.period_s
        self.index += 1
        raw = schedule_value(self.schedule, time_s)
        self.latest = TargetObservation(
            capture_time_s=time_s,
            x_forward_m=raw,
            y_left_m=0.0,
        )
        self.observation_history.append({
            "capture_time_s": time_s,
            "raw_v_cmd_m_s": raw,
        })
        return self.latest

    def diagnostic(self, sim) -> dict:
        del sim
        observation = self.latest
        if observation is None:
            return {}
        return {
            "target_observation_capture_time_s": observation.capture_time_s,
            "target_observation_raw_v_cmd_m_s": observation.x_forward_m,
        }


def synthetic_follower(observation: TargetObservation) -> MotionIntent:
    return MotionIntent(
        source_time_s=float(observation.capture_time_s),
        linear_velocity_target_m_s=float(observation.x_forward_m),
        yaw_rate_target_rad_s=0.0,
    )


def history_arrays(history: list[dict]) -> dict[str, np.ndarray]:
    keys = (
        "t", "raw_linear_velocity_cmd_m_s", "candidate_target_m_s",
        "linear_velocity_cmd_m_s", "rolling_v_ref_applied_m_s", "v_hat_m_s",
        "v_GT_m_s", "rolling_a_ref_applied_m_s2",
        "rolling_theta_ref_applied_rad", "theta_hat_rad", "theta_eq_used_rad",
        "theta_GT_rad", "environment_mode", "rolling_u_ff_raw_nm",
        "u_ff_used_nm", "u_fb_nm", "u_sum_nm", "actual_left_nm",
        "actual_right_nm", "scheduler_mode", "candidate_stable",
        "pending_command_active", "latest_pending_raw_v_cmd_m_s",
        "candidate_delta_v_m_s", "accepted_velocity_changed",
        "sum_command_saturated",
    )
    return {key: np.asarray([row[key] for row in history]) for key in keys}


def event_markers(axis, scheduler_events: list[dict], full_exits: list[dict]) -> None:
    shown: set[str] = set()
    styles = {
        "candidate_started": ("0.5", "candidate start"),
        "candidate_stable": ("tab:cyan", "candidate stable"),
    }
    for event in scheduler_events:
        name = event["event"]
        if name in styles:
            color, label = styles[name]
        elif name == "accepted":
            color = "tab:purple" if event["mode"] == "FULL_DYNAMIC" else "tab:green"
            label = f"{event['mode']} accept"
        else:
            continue
        axis.axvline(
            float(event["time_s"]), color=color, linewidth=0.85,
            alpha=0.62, label=(label if label not in shown else None),
        )
        shown.add(label)
    for event in full_exits:
        axis.axvline(
            float(event["time_s"]), color="tab:red", linewidth=1.0,
            linestyle="--", alpha=0.75,
            label=("FULL exit" if "FULL exit" not in shown else None),
        )
        shown.add("FULL exit")


def make_plots(
    history: list[dict],
    scheduler_events: list[dict],
    full_exit_events: list[dict],
    full_events: list[dict],
    wheel_limit_nm: float,
) -> list[str]:
    stage5.PLOT_DIR.mkdir(parents=True, exist_ok=True)
    value = history_arrays(history)
    t = value["t"].astype(float)

    fig, axis = plt.subplots(figsize=(12, 5.4))
    for key, label, width in (
        ("raw_linear_velocity_cmd_m_s", "raw_v_cmd", 1.2),
        ("candidate_target_m_s", "candidate/latest target", 1.0),
        ("linear_velocity_cmd_m_s", "accepted_v_cmd", 1.3),
        ("rolling_v_ref_applied_m_s", "v_ref", 1.3),
        ("v_hat_m_s", "v_hat", 0.9),
        ("v_GT_m_s", "v_GT", 0.9),
    ):
        axis.plot(t, value[key].astype(float), label=label, linewidth=width)
    event_markers(axis, scheduler_events, full_exit_events)
    axis.set(xlabel="time [s]", ylabel="velocity [m/s]", title="V1.4 command lifecycle")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(PLOTS["command_lifecycle"], dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True)
    axes[0].plot(t, value["rolling_a_ref_applied_m_s2"], label="a_ref")
    axes[0].set(ylabel="m/s²", title="Reference dynamics and environment supervisor")
    axes[0].legend()
    theta_hat_error = value["theta_hat_rad"].astype(float) - value["theta_eq_used_rad"].astype(float)
    axes[1].plot(t, np.degrees(value["rolling_theta_ref_applied_rad"].astype(float)), label="theta_ref")
    axes[1].plot(t, np.degrees(theta_hat_error), label="theta_hat")
    axes[1].plot(t, np.degrees(value["theta_GT_rad"].astype(float)), label="theta_GT")
    axes[1].set(ylabel="pitch error [deg]")
    axes[1].legend(ncol=3)
    env_mode = np.asarray([1.0 if item == "SLOPE" else 0.0 for item in value["environment_mode"]])
    axes[2].step(t, env_mode, where="post", label="environment_mode")
    axes[2].set(xlabel="time [s]", ylabel="mode", yticks=[0, 1], yticklabels=["FLAT", "SLOPE"])
    axes[2].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS["reference_dynamics"], dpi=180)
    plt.close(fig)

    actual_left = value["actual_left_nm"].astype(float)
    actual_right = value["actual_right_nm"].astype(float)
    saturated = (
        (np.abs(actual_left) >= wheel_limit_nm - 1e-12)
        | (np.abs(actual_right) >= wheel_limit_nm - 1e-12)
        | value["sum_command_saturated"].astype(bool)
    )
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, height_ratios=[4, 1])
    axes[0].plot(t, value["rolling_u_ff_raw_nm"], label="u_ff_raw")
    axes[0].plot(t, value["u_ff_used_nm"], label="u_ff_applied")
    axes[0].plot(t, value["u_fb_nm"], label="u_fb")
    axes[0].plot(t, value["u_sum_nm"], label="u_total")
    axes[0].set(ylabel="sum torque [N m]", title="Torque and saturation")
    axes[0].legend(ncol=4)
    axes[1].fill_between(t, 0.0, saturated.astype(float), step="post", alpha=0.65, label="wheel/command saturation")
    axes[1].set(xlabel="time [s]", ylabel="sat", yticks=[0, 1])
    axes[1].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOTS["torque"], dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(12, 9.5), sharex=True)
    modes = np.asarray([
        {"NORMAL": 0, "CANDIDATE": 1, "LIGHTWEIGHT": 2, "FULL_DYNAMIC": 3}[str(mode)]
        for mode in value["scheduler_mode"]
    ])
    axes[0].step(t, modes, where="post")
    axes[0].set(ylabel="state", yticks=[0, 1, 2, 3], yticklabels=["NORMAL", "CANDIDATE", "LIGHTWEIGHT", "FULL"])
    h_full = np.zeros_like(t)
    t_ruckig = np.zeros_like(t)
    for event in full_events:
        start = float(event["time_s"])
        stop = float(event.get("full_exit_time_s") or t[-1])
        active = (t >= start) & (t <= stop)
        h_full[active] = float(event["horizon_s"])
        t_ruckig[active] = float(event["ruckig_duration_s"])
    axes[1].step(t, t_ruckig, where="post", label="T_ruckig")
    axes[1].step(t, h_full, where="post", label="H_full")
    axes[1].set(ylabel="seconds")
    axes[1].legend(ncol=2)
    axes[2].step(t, value["pending_command_active"].astype(float), where="post", label="pending active")
    axes[2].plot(t, value["latest_pending_raw_v_cmd_m_s"].astype(float), label="latest pending raw")
    axes[2].set(ylabel="pending")
    axes[2].legend(ncol=2)
    axes[3].plot(t, value["candidate_delta_v_m_s"].astype(float), label="candidate delta")
    axes[3].plot(t, value["candidate_target_m_s"].astype(float), label="candidate/latest target")
    axes[3].set(xlabel="time [s]", ylabel="m/s")
    axes[3].legend(ncol=2)
    for axis in axes:
        event_markers(axis, scheduler_events, full_exit_events)
        axis.grid(True, alpha=0.25)
    fig.suptitle("Scheduler lifecycle and FULL timing")
    fig.tight_layout()
    fig.savefig(PLOTS["scheduler_lifecycle"], dpi=180)
    plt.close(fig)
    return [path.relative_to(ROOT).as_posix() for path in PLOTS.values()]


def write_csv(history: list[dict]) -> None:
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    with CSV_PATH.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(history[0]), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(history)


def distribution(values: list[float]) -> dict:
    array = np.asarray(values, dtype=float)
    if not array.size:
        return {"count": 0}
    return {
        "count": int(array.size), "minimum": float(np.min(array)),
        "mean": float(np.mean(array)), "p95": float(np.percentile(array, 95)),
        "maximum": float(np.max(array)),
    }


def main() -> None:
    config = stage5.load_json(CONFIG_PATH)
    stage4_config = stage5.load_json(stage5.stage4_final.CONFIG_PATH)
    stage4c_config = stage5.load_json(stage5.stage4_final.STAGE4C_CONFIG_PATH)
    frozen = stage5.stage4_final.validate_frozen_baseline(stage4_config, stage4c_config)
    common_list = list(stage5.stage4_final.make_common(stage4_config, int(config["seed"])))
    common_list[3] = copy.deepcopy(common_list[3])
    common_list[3]["history_frequency_hz"] = 500.0
    common = tuple(common_list)
    provider = SyntheticCommandProvider(config)
    source = stage5_v12.make_source(
        config, common, provider, follower=synthetic_follower
    )
    q_adapter = stage5.stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = stage5.stage4_final.FinalRuntime(
        reduced=common[7], config=stage4_config, stage4c_config=stage4c_config,
        q_adapter=q_adapter,
        slope_enter_persistence_override_s=float(config["slope_entry"]["enter_persistence_s"]),
        slope_entry_acceleration_quiet_m_s2=float(config["slope_entry"]["acceleration_quiet_abs_m_s2"]),
    )
    scenario = {
        "name": "stage5_v1_4_candidate_command_lifecycle",
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
        history_diagnostic_callback=provider.diagnostic,
        equilibrium_reference_callback=lambda context: stage5.analytic_theta_eq(
            runtime.alpha_control_rad, runtime.plant
        ),
        equilibrium_input_callback=lambda context: stage5.equilibrium_sum_torque_nm(
            runtime.alpha_control_rad, float(context["estimate"].velocity_hat_m_s),
            float(context["estimate"].theta_dot_hat_rad_s), runtime.plant,
        ),
        control_observer=runtime, reference_source=source,
    )
    history = run["history_50hz"]
    summary = source.summary()
    scheduler_events = summary["scheduler_events"]
    full_exits = summary["full_exit_events"]
    full_events = [
        event for event in summary["replan_events"]
        if event["planning_path"] == "FULL_DYNAMIC"
    ]
    light_events = [
        event for event in summary["replan_events"]
        if event["planning_path"] == "LIGHTWEIGHT"
    ]
    values = history_arrays(history)
    wheel_limit = float(common[8]["known"]["wheel_torque_hard_peak_nm"])
    plots = make_plots(
        history, scheduler_events, full_exits, full_events, wheel_limit
    )
    write_csv(history)
    actual_left = values["actual_left_nm"].astype(float)
    actual_right = values["actual_right_nm"].astype(float)
    saturation = (
        (np.abs(actual_left) >= wheel_limit - 1e-12)
        | (np.abs(actual_right) >= wheel_limit - 1e-12)
        | values["sum_command_saturated"].astype(bool)
    )
    accepted = [event for event in scheduler_events if event["event"] == "accepted"]
    metrics = {
        "observation_count": len(provider.observation_history),
        "candidate_count": sum(event["event"] == "candidate_started" for event in scheduler_events),
        "cancelled_candidate_count": sum(event["event"] == "candidate_cancelled" for event in scheduler_events),
        "candidate_stable_count": sum(event["event"] == "candidate_stable" for event in scheduler_events),
        "lightweight_accept_count": len(light_events),
        "full_accept_count": len(full_events),
        "mid_trajectory_lightweight_replan_count": sum(bool(event["mid_trajectory_replan"]) for event in light_events),
        "full_planning_wall_time_s": distribution([event["planning_wall_time_s"] for event in full_events]),
        "full_runs": [
            {
                "start_time_s": event["time_s"],
                "T_ruckig_s": event["ruckig_duration_s"],
                "H_full_s": event["horizon_s"],
                "planning_wall_time_s": event["planning_wall_time_s"],
                "residual_rms": event["normalized_residual_rms"],
                "quiet_time_s": event["quiet_time_s"],
                "fade_completion_time_s": event["fade_completion_time_s"],
                "full_exit_time_s": event["full_exit_time_s"],
                "solver_status": event["projection_solver_status"],
            }
            for event in full_events
        ],
        "quiet_snap_count": len(summary["quiet_events"]),
        "fade_completion_count": sum(event["event"] == "fade_finished" for event in summary["fade_events"]),
        "pending_commands_processed_after_full_count": sum(bool(event.get("processed_after_full_exit")) for event in accepted),
        "slope_false_trigger_count": sum(
            event["to"] == "SLOPE" for event in runtime.transition_log
        ),
        "saturation_count": int(np.count_nonzero(saturation)),
        "fall": bool(run["longitudinal"]["fell"]),
    }
    result = {
        "stage": config["stage"], "status": "COMPLETED", "config": config,
        "frozen_baseline_checks": frozen, "metrics": metrics,
        "scheduler_events": scheduler_events,
        "replan_events": summary["replan_events"],
        "full_exit_events": full_exits,
        "quiet_events": summary["quiet_events"],
        "fade_events": summary["fade_events"],
        "slope_transition_events": runtime.transition_log,
        "observation_history": provider.observation_history,
        "intent_history": summary["intents"], "history": history,
        "plots": plots, "history_csv": CSV_PATH.relative_to(ROOT).as_posix(),
    }
    stage5.write_json(RESULT_PATH, result)
    lines = [
        "# Stage 5 V1.4 summary", "",
        f"- Observations: {metrics['observation_count']}; candidates: {metrics['candidate_count']}; cancelled: {metrics['cancelled_candidate_count']}",
        f"- Accepted: {metrics['lightweight_accept_count']} LIGHTWEIGHT / {metrics['full_accept_count']} FULL_DYNAMIC; mid-trajectory lightweight replans: {metrics['mid_trajectory_lightweight_replan_count']}",
        f"- Quiet snaps / fade completions: {metrics['quiet_snap_count']} / {metrics['fade_completion_count']}; pending commands processed after FULL: {metrics['pending_commands_processed_after_full_count']}",
        f"- Slope false triggers: {metrics['slope_false_trigger_count']}; saturation samples: {metrics['saturation_count']}; fall: {str(metrics['fall']).lower()}",
        "", "FULL maneuvers:",
    ]
    if full_events:
        for index, item in enumerate(metrics["full_runs"], start=1):
            lines.append(
                f"- {index}: T_ruckig={item['T_ruckig_s']:.3f}s, H_full={item['H_full_s']:.3f}s, "
                f"planning={item['planning_wall_time_s']:.3f}s, residual={item['residual_rms']:.5f}, "
                f"quiet={item['quiet_time_s']}, exit={item['full_exit_time_s']}"
            )
    else:
        lines.append("- None")
    lines.extend([
        "", f"- JSON: `{RESULT_PATH.relative_to(ROOT).as_posix()}`",
        f"- CSV: `{CSV_PATH.relative_to(ROOT).as_posix()}`",
    ])
    SUMMARY_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
