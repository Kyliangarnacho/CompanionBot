"""Run the single Stage 5 V1.2 sparse-scheduler two-path episode."""

from __future__ import annotations

import copy
import csv
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

import run_stage5_master_following as stage5  # noqa: E402
from control.rolling_reference import (  # noqa: E402
    RuckigFullHorizonVelocityPlanner,
    RuckigLightweightVelocityPlanner,
    SparseTwoPathReferenceSource,
    SparseVelocityCommandScheduler,
)


CONFIG_PATH = stage5.STAGE_DIR / "config" / "stage5_v1_2_sparse_two_path_config.json"
RESULT_PATH = stage5.RESULT_DIR / "stage5_v1_2_sparse_two_path_results.json"
CSV_PATH = stage5.RESULT_DIR / "stage5_v1_2_sparse_two_path_history.csv"
PLOT_PATHS = {
    "velocity": stage5.PLOT_DIR / "stage5_v1_2_velocity.png",
    "dynamic": stage5.PLOT_DIR / "stage5_v1_2_dynamic_reference.png",
    "torque": stage5.PLOT_DIR / "stage5_v1_2_torque.png",
    "scheduler": stage5.PLOT_DIR / "stage5_v1_2_scheduler_mode.png",
    "yaw": stage5.PLOT_DIR / "stage5_v1_2_yaw.png",
}


def make_source(
    config: dict, common: tuple, provider, *, follower=None,
    full_planner_override: RuckigFullHorizonVelocityPlanner | None = None,
) -> SparseTwoPathReferenceSource:
    _, runtime_config, dynamic_config, motion_config, offline, *_ = common
    fit = offline["fit"]
    limits = motion_config["reference_limits"]
    horizon = runtime_config["velocity_lifecycle"]["horizon_search"]
    shared = {
        "controller_dt_s": float(motion_config["controller_dt_s"]),
        "max_velocity_m_s": float(limits["max_velocity_m_s"]),
        "max_acceleration_m_s2": float(limits["max_acceleration_m_s2"]),
        "max_jerk_m_s3": float(dynamic_config["max_jerk_m_s3"]),
        "A": np.asarray(fit["A_identified"], dtype=float),
        "B": np.asarray(fit["B_identified"], dtype=float),
        "state_scales": np.asarray(fit["state_scales"], dtype=float),
        "input_scale_nm": float(fit["input_scale_nm"]),
        "horizon_step_s": float(horizon["candidate_step_s"]),
        "maximum_tail_s": 0.0,
        "normalized_residual_rms_max": float(
            config["full_dynamic"]["diagnostic_residual_rms"]
        ),
    }
    lightweight = RuckigLightweightVelocityPlanner(**shared)
    full_config = config["full_dynamic"]
    full = full_planner_override or RuckigFullHorizonVelocityPlanner(
        minimum_horizon_s=float(full_config.get(
            "minimum_horizon_s", full_config.get("fixed_horizon_s", 1.5)
        )),
        dynamic_settle_margin_s=float(
            full_config.get("dynamic_settle_margin_s", 0.0)
        ),
        planning_dt_s=full_config.get("planning_dt_s"),
        horizon_quantum_s=float(full_config.get("horizon_quantum_s", 0.0)),
        projection_backend=str(full_config.get("projection_backend", "lsqr")),
        **shared,
    )
    scheduler_config = config["scheduler"]
    scheduler = SparseVelocityCommandScheduler(
        accept_delta_v_m_s=float(scheduler_config.get(
            "T_accept_delta_v_m_s",
            scheduler_config.get("T1_accept_delta_v_m_s", 0.12),
        )),
        full_delta_v_m_s=float(
            scheduler_config.get(
                "T_full_delta_v_m_s",
                scheduler_config.get("TF_full_delta_v_m_s", 0.30),
            )
        ),
        stable_window_s=float(scheduler_config.get(
            "candidate_stable_window_s", scheduler_config.get("T3_persist_s", 0.20)
        )),
        stable_range_m_s=float(
            scheduler_config.get("candidate_stable_range_m_s", 0.03)
        ),
        initial_accepted_velocity_m_s=float(
            scheduler_config["initial_accepted_velocity_m_s"]
        ),
    )
    block = config["reference_block"]
    if not math.isclose(
        float(block["dt_s"]), lightweight.dt_s, rel_tol=0.0, abs_tol=1e-12
    ):
        raise RuntimeError("Stage 5 V1.2 block dt differs from frozen control dt")
    return SparseTwoPathReferenceSource(
        lightweight_planner=lightweight,
        full_planner=full,
        scheduler=scheduler,
        block_samples=int(block["sample_count"]),
        feedforward_fade_s=float(
            runtime_config["velocity_lifecycle"]["feedforward_fade_s"]
        ),
        observation_reader=provider.read,
        follower=(
            stage5.SimpleFollower(config["follower"])
            if follower is None else follower
        ),
        quiet_theta_rad=math.radians(
            float(full_config["terminal_theta_abs_deg"])
        ),
        quiet_theta_dot_rad_s=math.radians(
            float(full_config["terminal_theta_dot_abs_deg_s"])
        ),
    )


def arrays(history: list[dict]) -> dict[str, np.ndarray]:
    keys = (
        "t", "raw_linear_velocity_cmd_m_s", "linear_velocity_cmd_m_s",
        "rolling_v_ref_applied_m_s", "v_GT_m_s", "rolling_a_ref_applied_m_s2",
        "rolling_theta_ref_applied_rad", "theta_GT_rad", "rolling_u_ff_raw_nm",
        "u_ff_used_nm", "u_fb_nm", "u_sum_nm", "yaw_rate_cmd_rad_s",
        "r_hat_rad_s", "r_GT_rad_s", "scheduler_mode", "rolling_replanned",
        "accepted_velocity_changed", "u_left_nm", "u_right_nm",
    )
    return {key: np.asarray([row[key] for row in history]) for key in keys}


def make_plots(history: list[dict]) -> list[str]:
    stage5.PLOT_DIR.mkdir(parents=True, exist_ok=True)
    value = arrays(history)
    time_s = value["t"].astype(float)
    replans = time_s[value["rolling_replanned"].astype(bool)]

    fig, axis = plt.subplots(figsize=(11, 5))
    axis.plot(time_s, value["raw_linear_velocity_cmd_m_s"], label="raw_v_cmd")
    axis.plot(time_s, value["linear_velocity_cmd_m_s"], label="accepted_v_cmd")
    axis.plot(time_s, value["rolling_v_ref_applied_m_s"], label="v_ref")
    axis.plot(time_s, value["v_GT_m_s"], label="v_GT")
    for event_time in replans:
        axis.axvline(event_time, color="0.7", linewidth=0.6, alpha=0.45)
    axis.set(xlabel="time [s]", ylabel="velocity [m/s]", title="Sparse command and velocity response")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["velocity"], dpi=180)
    plt.close(fig)

    fig, (axis_a, axis_theta) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axis_a.plot(time_s, value["rolling_a_ref_applied_m_s2"], label="a_ref")
    axis_a.set(ylabel="m/s²", title="Dynamic reference")
    axis_a.grid(True, alpha=0.25)
    axis_a.legend()
    axis_theta.plot(time_s, np.degrees(value["rolling_theta_ref_applied_rad"].astype(float)), label="theta_ref")
    axis_theta.plot(time_s, np.degrees(value["theta_GT_rad"].astype(float)), label="theta_GT")
    axis_theta.set(xlabel="time [s]", ylabel="pitch [deg]")
    axis_theta.grid(True, alpha=0.25)
    axis_theta.legend()
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["dynamic"], dpi=180)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(11, 5))
    axis.plot(time_s, value["rolling_u_ff_raw_nm"], label="u_ff_raw")
    axis.plot(time_s, value["u_ff_used_nm"], label="u_ff_applied")
    axis.plot(time_s, value["u_fb_nm"], label="u_fb")
    axis.plot(time_s, value["u_sum_nm"], label="u_total")
    axis.set(xlabel="time [s]", ylabel="sum torque [N m]", title="Torque response")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=4)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["torque"], dpi=180)
    plt.close(fig)

    mode_values = np.asarray([
        {"HOLD": 0, "NORMAL": 0, "CANDIDATE": 0, "LIGHTWEIGHT": 1, "FULL_DYNAMIC": 2}[str(mode)]
        for mode in value["scheduler_mode"]
    ])
    fig, axis = plt.subplots(figsize=(11, 3.4))
    axis.step(time_s, mode_values, where="post")
    axis.set(
        xlabel="time [s]", ylabel="scheduler mode",
        yticks=[0, 1, 2], yticklabels=["HOLD", "LIGHTWEIGHT", "FULL_DYNAMIC"],
        title="Scheduler and active planning path",
    )
    axis.grid(True, alpha=0.25)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["scheduler"], dpi=180)
    plt.close(fig)

    fig, axis = plt.subplots(figsize=(11, 4.5))
    axis.plot(time_s, value["yaw_rate_cmd_rad_s"], label="yaw_rate_cmd")
    axis.plot(time_s, value["r_hat_rad_s"], label="yaw_rate_hat")
    axis.plot(time_s, value["r_GT_rad_s"], label="yaw_rate_GT")
    axis.set(xlabel="time [s]", ylabel="yaw rate [rad/s]", title="Frozen yaw-loop sanity check")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=3)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["yaw"], dpi=180)
    plt.close(fig)
    return [path.relative_to(ROOT).as_posix() for path in PLOT_PATHS.values()]


def write_csv(history: list[dict]) -> None:
    CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
    fields = list(history[0])
    with CSV_PATH.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(history)


def distribution(values: np.ndarray) -> dict:
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {"count": 0}
    return {
        "count": int(values.size),
        "minimum": float(np.min(values)),
        "mean": float(np.mean(values)),
        "p95": float(np.percentile(values, 95.0)),
        "maximum": float(np.max(values)),
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
    provider = stage5.TestOnlyMasterObservationProvider(config)
    source = make_source(config, common, provider)
    q_adapter = stage5.stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = stage5.stage4_final.FinalRuntime(
        reduced=common[7], config=stage4_config,
        stage4c_config=stage4c_config, q_adapter=q_adapter,
    )
    scenario = {
        "name": "stage5_v1_2_sparse_two_path",
        "duration_s": float(config["episode_duration_s"]),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    run = stage5.stage3b.run_case(
        scenario,
        float(common[0]["yaw"]["K_psi_nm_per_rad"]),
        float(common[0]["yaw"]["K_r_nm_per_rad_s"]),
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
    summary = source.summary()
    value = arrays(history)
    plots = make_plots(history)
    write_csv(history)
    events = summary["replan_events"]
    light = [item for item in events if item["planning_path"] == "LIGHTWEIGHT"]
    full = [item for item in events if item["planning_path"] == "FULL_DYNAMIC"]
    full_residuals = np.asarray([item["normalized_residual_rms"] for item in full])
    planning = {
        name: distribution(np.asarray([
            item["planning_wall_time_s"] for item in selected
        ]))
        for name, selected in (("LIGHTWEIGHT", light), ("FULL_DYNAMIC", full))
    }
    wheel_limit = float(common[8]["known"]["wheel_torque_hard_peak_nm"])
    saturated = (
        np.abs(value["u_left_nm"].astype(float)) >= wheel_limit - 1e-12
    ) | (
        np.abs(value["u_right_nm"].astype(float)) >= wheel_limit - 1e-12
    )
    metrics = {
        "observation_count": len(provider.observation_history),
        "accepted_v_cmd_change_count": sum(
            item["event"] == "accepted" for item in summary["scheduler_events"]
        ),
        "planning_count": {
            "LIGHTWEIGHT": len(light), "FULL_DYNAMIC": len(full)
        },
        "planning_wall_time_s": planning,
        "full_projection_residual_rms_diagnostic": distribution(full_residuals),
        "full_projection_residual_0_020_exceedance_count": int(np.count_nonzero(
            full_residuals > float(config["full_dynamic"]["diagnostic_residual_rms"])
        )),
        "reference_peaks": {
            "a_ref_abs_m_s2": float(np.max(np.abs(value["rolling_a_ref_applied_m_s2"].astype(float)))),
            "theta_ref_abs_deg": float(np.max(np.abs(np.degrees(value["rolling_theta_ref_applied_rad"].astype(float))))),
        },
        "fade": {
            "started": sum(item["event"] == "fade_started" for item in summary["fade_events"]),
            "completed": sum(item["event"] == "fade_finished" for item in summary["fade_events"]),
        },
        "quiet_snap_count": len(summary["quiet_events"]),
        "wheel_saturation_count": int(np.count_nonzero(saturated)),
        "fall": bool(run["longitudinal"]["fell"]),
    }
    result = {
        "stage": config["stage"],
        "status": "COMPLETED",
        "config": config,
        "frozen_baseline_checks": frozen,
        "metrics": metrics,
        "scheduler_events": summary["scheduler_events"],
        "replan_events": events,
        "quiet_events": summary["quiet_events"],
        "fade_events": summary["fade_events"],
        "observation_history": provider.observation_history,
        "intent_history": summary["intents"],
        "history": history,
        "plots": plots,
        "history_csv": CSV_PATH.relative_to(ROOT).as_posix(),
    }
    stage5.write_json(RESULT_PATH, result)
    print(json.dumps(metrics, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
