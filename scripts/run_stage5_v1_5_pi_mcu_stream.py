"""Benchmark Stage 5 FULL projection paths and a deterministic Pi/MCU block stream."""

from __future__ import annotations

import argparse
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

import run_stage3b_yaw_control as stage3b  # noqa: E402
import run_stage4_final_closeout as stage4_final  # noqa: E402
import run_stage4a_slope_robustness as stage4a  # noqa: E402
from control.rolling_reference import (  # noqa: E402
    DeterministicReferenceBlockStream,
    MotionIntent,
    ReferenceBlockUnderrun,
    RuckigFullHorizonVelocityPlanner,
    RuckigLightweightVelocityPlanner,
    SparseTwoPathReferenceSource,
    SparseVelocityCommandScheduler,
    TargetObservation,
)
from control.trajectory_feedforward import SparseProjectionFactorizationCache  # noqa: E402
from sim.slope_estimation import (  # noqa: E402
    analytic_theta_eq,
    equilibrium_sum_torque_nm,
)


STAGE_DIR = ROOT / "models" / "minisegway" / "stage5"
RESULT_DIR = STAGE_DIR / "results"
PLOT_DIR = RESULT_DIR / "plots"
REFERENCE_DIR = STAGE_DIR / "reference"
CONFIG_PATH = STAGE_DIR / "config" / "stage5_v1_5_pi_mcu_stream_config.json"
RESULT_PATH = RESULT_DIR / "stage5_v1_5_pi_mcu_stream_results.json"
BENCHMARK_PATH = RESULT_DIR / "stage5_v1_5_planner_benchmark.json"
REPLAY_PATH = RESULT_DIR / "stage5_v1_5_closed_loop_comparison.json"
SUMMARY_PATH = RESULT_DIR / "STAGE5_V1_5_SUMMARY.md"
V14_RESULT_PATH = (
    REFERENCE_DIR / "v1_4" / "results" / "stage5_v1_4_candidate_lifecycle_results.json"
)
PLOT_PATHS = {
    "velocity": PLOT_DIR / "stage5_v1_5_velocity.png",
    "dynamic": PLOT_DIR / "stage5_v1_5_dynamic_ff.png",
    "stream": PLOT_DIR / "stage5_v1_5_block_stream.png",
    "planning": PLOT_DIR / "stage5_v1_5_planning_timeline.png",
}

SPLICE_STATE = np.asarray([1.0788539999999929, 0.38, 0.0, 0.0], dtype=float)
SPLICE_ACCELERATION = 0.0
SPLICE_TARGET = -0.10


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


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
        if self.latest is None:
            return {}
        return {
            "target_observation_capture_time_s": self.latest.capture_time_s,
            "target_observation_raw_v_cmd_m_s": self.latest.x_forward_m,
        }


def synthetic_follower(observation: TargetObservation) -> MotionIntent:
    return MotionIntent(
        source_time_s=float(observation.capture_time_s),
        linear_velocity_target_m_s=float(observation.x_forward_m),
        yaw_rate_target_rad_s=0.0,
    )


def make_reference_source(
    config: dict,
    common: tuple,
    provider,
    *,
    follower,
    full_planner_override: RuckigFullHorizonVelocityPlanner | None = None,
    scheduler_gate_overrides: dict | None = None,
) -> SparseTwoPathReferenceSource:
    _, runtime_config, dynamic_config, motion_config, offline, *_ = common
    fit = offline["fit"]
    limits = motion_config["reference_limits"]
    shared = {
        "controller_dt_s": float(motion_config["controller_dt_s"]),
        "max_velocity_m_s": float(limits["max_velocity_m_s"]),
        "max_acceleration_m_s2": float(limits["max_acceleration_m_s2"]),
        "max_jerk_m_s3": float(dynamic_config["max_jerk_m_s3"]),
        "A": np.asarray(fit["A_identified"], dtype=float),
        "B": np.asarray(fit["B_identified"], dtype=float),
        "state_scales": np.asarray(fit["state_scales"], dtype=float),
        "input_scale_nm": float(fit["input_scale_nm"]),
    }
    lightweight = RuckigLightweightVelocityPlanner(**shared)
    full_config = config["full_dynamic"]
    full = full_planner_override or RuckigFullHorizonVelocityPlanner(
        minimum_horizon_s=float(full_config["minimum_horizon_s"]),
        dynamic_settle_margin_s=float(
            full_config.get("dynamic_settle_margin_s", 0.0)
        ),
        planning_dt_s=full_config.get("planning_dt_s"),
        horizon_quantum_s=float(full_config["horizon_bucket_s"]),
        projection_backend=str(full_config.get("projection_backend", "lsqr")),
        **shared,
    )
    scheduler_config = config["scheduler"]
    scheduler = SparseVelocityCommandScheduler(
        accept_delta_v_m_s=float(scheduler_config["T_accept_delta_v_m_s"]),
        full_delta_v_m_s=float(scheduler_config["T_full_delta_v_m_s"]),
        stable_window_s=float(scheduler_config["candidate_stable_window_s"]),
        stable_range_m_s=float(scheduler_config["candidate_stable_range_m_s"]),
        initial_accepted_velocity_m_s=float(
            scheduler_config["initial_accepted_velocity_m_s"]
        ),
        require_stable_candidate=bool(
            (scheduler_gate_overrides or {}).get(
                "require_stable_candidate", True
            )
        ),
        minimum_delta_enabled=bool(
            (scheduler_gate_overrides or {}).get(
                "minimum_delta_enabled", True
            )
        ),
        force_full_dynamic=bool(
            (scheduler_gate_overrides or {}).get(
                "follow_force_full_dynamic", False
            )
        ),
    )
    block = config["reference_block"]
    if not math.isclose(
        float(block["dt_s"]), lightweight.dt_s, rel_tol=0.0, abs_tol=1e-12
    ):
        raise RuntimeError("Stage 5 reference block dt differs from frozen control dt")
    return SparseTwoPathReferenceSource(
        lightweight_planner=lightweight,
        full_planner=full,
        scheduler=scheduler,
        block_samples=int(block["sample_count"]),
        feedforward_fade_s=float(
            runtime_config["velocity_lifecycle"]["feedforward_fade_s"]
        ),
        observation_reader=provider.read,
        follower=follower,
        quiet_theta_rad=math.radians(float(full_config["terminal_theta_abs_deg"])),
        quiet_theta_dot_rad_s=math.radians(
            float(full_config["terminal_theta_dot_abs_deg_s"])
        ),
        defer_full_accept_until_observed_exit=bool(
            (scheduler_gate_overrides or {}).get("follow_defer_until_full_exit", False)
        ),
    )


def write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False),
        encoding="utf-8",
    )


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def recorded_v14_full_planning_time_s() -> float:
    prior = load_json(V14_RESULT_PATH)
    return float(prior["metrics"]["full_runs"][0]["planning_wall_time_s"])


def load_common(config: dict) -> tuple:
    stage4_config = load_json(stage4_final.CONFIG_PATH)
    stage4c_config = load_json(stage4_final.STAGE4C_CONFIG_PATH)
    frozen_checks = stage4_final.validate_frozen_baseline(
        stage4_config, stage4c_config
    )
    common_items = list(
        stage4_final.make_common(stage4_config, int(config["seed"]))
    )
    common_items[3] = copy.deepcopy(common_items[3])
    common_items[3]["history_frequency_hz"] = 500.0
    return tuple(common_items), stage4_config, stage4c_config, frozen_checks


def make_full_planner(
    config: dict,
    common: tuple,
    *,
    planning_dt_s: float,
    backend: str,
    cache: SparseProjectionFactorizationCache | None = None,
) -> RuckigFullHorizonVelocityPlanner:
    _, runtime_config, dynamic_config, motion_config, offline, *_ = common
    fit = offline["fit"]
    limits = motion_config["reference_limits"]
    full = config["full_dynamic"]
    return RuckigFullHorizonVelocityPlanner(
        controller_dt_s=float(motion_config["controller_dt_s"]),
        max_velocity_m_s=float(limits["max_velocity_m_s"]),
        max_acceleration_m_s2=float(limits["max_acceleration_m_s2"]),
        max_jerk_m_s3=float(dynamic_config["max_jerk_m_s3"]),
        A=np.asarray(fit["A_identified"], dtype=float),
        B=np.asarray(fit["B_identified"], dtype=float),
        state_scales=np.asarray(fit["state_scales"], dtype=float),
        input_scale_nm=float(fit["input_scale_nm"]),
        minimum_horizon_s=float(full["minimum_horizon_s"]),
        dynamic_settle_margin_s=float(full["dynamic_settle_margin_s"]),
        planning_dt_s=float(planning_dt_s),
        horizon_quantum_s=float(full["horizon_bucket_s"]),
        projection_backend=backend,
        factorization_cache=cache,
    )


def plan_summary(plan) -> dict:
    diagnostics = plan.planning_diagnostics
    return {
        "planning_backend": diagnostics["projection_backend"],
        "planning_frequency_hz": 1.0 / float(diagnostics["planning_dt_s"]),
        "raw_horizon_s": float(diagnostics["raw_horizon_s"]),
        "horizon_bucket_s": float(diagnostics["horizon_bucket_s"]),
        "projected_horizon_s": float(diagnostics["projected_horizon_s"]),
        "control_interval_count": int(diagnostics["control_interval_count"]),
        "projection_interval_count": int(diagnostics["projection_interval_count"]),
        "matrix_build_time_s": float(diagnostics.get("matrix_build_time_s", 0.0)),
        "factorization_time_s": float(diagnostics.get("factorization_time_s", 0.0)),
        "rhs_build_time_s": float(diagnostics.get(
            "rhs_build_time_s", diagnostics.get("rhs_build_time_s", 0.0)
        )),
        "solve_time_s": float(diagnostics.get("solve_time_s", 0.0)),
        "planning_wall_time_s": float(diagnostics["planning_wall_time_s"]),
        "residual_500hz_post_interpolation_rms": float(
            diagnostics["residual_500hz_post_interpolation_rms"]
        ),
        "theta_ref_peak_rad": float(diagnostics["theta_ref_peak_rad"]),
        "theta_dot_ref_peak_rad_s": float(diagnostics["theta_dot_ref_peak_rad_s"]),
        "u_ff_peak_nm": float(diagnostics["u_ff_peak_nm"]),
        "ruckig_duration_s": float(plan.ruckig_duration_s),
        "terminal_theta_rad": float(plan.plan.reference_states[-1, 2]),
        "terminal_theta_dot_rad_s": float(plan.plan.reference_states[-1, 3]),
        "cache_hit": diagnostics.get("cache_hit"),
        "kkt_residual_relative": diagnostics.get("kkt_residual_relative"),
        "diagonal_pivot_ratio": diagnostics.get("diagonal_pivot_ratio"),
        "lsqr_matrix_and_rhs_build_time_s": diagnostics.get(
            "matrix_and_rhs_build_time_s"
        ),
    }


def array_parity(reference, candidate) -> dict:
    ref_states = np.asarray(reference.plan.reference_states, dtype=float)
    cand_states = np.asarray(candidate.plan.reference_states, dtype=float)
    ref_u = np.asarray(reference.plan.feedforward_inputs_nm, dtype=float)
    cand_u = np.asarray(candidate.plan.feedforward_inputs_nm, dtype=float)
    return {
        "theta_ref_max_abs_difference_rad": float(
            np.max(np.abs(ref_states[:, 2] - cand_states[:, 2]))
        ),
        "theta_dot_ref_max_abs_difference_rad_s": float(
            np.max(np.abs(ref_states[:, 3] - cand_states[:, 3]))
        ),
        "u_ff_max_abs_difference_nm": float(np.max(np.abs(ref_u - cand_u))),
        "normalized_residual_rms_difference": float(abs(
            reference.normalized_residual_rms - candidate.normalized_residual_rms
        )),
    }


def run_planner_benchmarks(config: dict, common: tuple) -> dict:
    paths = {
        "lsqr_500hz": {"dt": 0.002, "backend": "lsqr"},
        "cached_kkt_500hz": {"dt": 0.002, "backend": "cached_kkt"},
        "cached_kkt_250hz": {"dt": 0.004, "backend": "cached_kkt"},
        "lsqr_250hz_reference": {"dt": 0.004, "backend": "lsqr"},
    }
    planners = {}
    first_and_warm = {}
    caches = {}
    for name, path in paths.items():
        cache = SparseProjectionFactorizationCache() if path["backend"] == "cached_kkt" else None
        planner = make_full_planner(
            config, common, planning_dt_s=path["dt"],
            backend=path["backend"], cache=cache,
        )
        print(f"BENCHMARK cold {name}", flush=True)
        first = planner.plan(SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET)
        print(f"BENCHMARK warm {name}", flush=True)
        warm = planner.plan(SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET)
        planners[name] = planner
        caches[name] = cache
        first_and_warm[name] = {
            "cold": plan_summary(first),
            "warm": plan_summary(warm),
            "repeat_parity": array_parity(first, warm),
        }

    parity = {
        "cached_kkt_500hz_vs_lsqr_500hz": array_parity(
            # Compare repeated/settled outputs; both use the same frozen 2 ms A/B.
            planners["lsqr_500hz"].plan(
                SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET
            ),
            planners["cached_kkt_500hz"].plan(
                SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET
            ),
        ),
        "cached_kkt_250hz_vs_lsqr_250hz": array_parity(
            planners["lsqr_250hz_reference"].plan(
                SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET
            ),
            planners["cached_kkt_250hz"].plan(
                SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET
            ),
        ),
        "cached_kkt_250hz_vs_lsqr_500hz": array_parity(
            planners["lsqr_500hz"].plan(
                SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET
            ),
            planners["cached_kkt_250hz"].plan(
                SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET
            ),
        ),
    }
    for name, parity_data in parity.items():
        parity_data["kkt_parity_pass"] = bool(
            max(
                parity_data["theta_ref_max_abs_difference_rad"],
                parity_data["theta_dot_ref_max_abs_difference_rad_s"],
                parity_data["u_ff_max_abs_difference_nm"],
            ) <= 1e-6
        ) if "vs_lsqr_500hz" in name or "vs_lsqr_250hz" in name else None
    return {
        "splice": {
            "start_reference_p_v_theta_theta_dot": SPLICE_STATE.tolist(),
            "start_acceleration_m_s2": SPLICE_ACCELERATION,
            "target_velocity_m_s": SPLICE_TARGET,
        },
        "paths": first_and_warm,
        "parity": parity,
        "recorded_v1_4_full_planning_wall_time_s": recorded_v14_full_planning_time_s(),
        "planner_objects": planners,
        "factorization_caches": caches,
    }


def make_runtime(common: tuple, config: dict, stage4_config: dict, stage4c_config: dict):
    q_adapter = stage4a.make_q_adapter(common, actuator_enabled=False)
    runtime = stage4_final.FinalRuntime(
        reduced=common[7], config=stage4_config, stage4c_config=stage4c_config,
        q_adapter=q_adapter,
        slope_enter_persistence_override_s=float(
            config["slope_entry"]["enter_persistence_s"]
        ),
        slope_entry_acceleration_quiet_m_s2=float(
            config["slope_entry"]["acceleration_quiet_abs_m_s2"]
        ),
    )
    return q_adapter, runtime


def run_episode(
    config: dict,
    common: tuple,
    stage4_config: dict,
    stage4c_config: dict,
    planner: RuckigFullHorizonVelocityPlanner,
    *,
    stream: DeterministicReferenceBlockStream | None = None,
):
    provider = SyntheticCommandProvider(config)
    source = make_reference_source(
        config,
        common,
        provider,
        follower=synthetic_follower,
        full_planner_override=planner,
    )
    q_adapter, runtime = make_runtime(
        common, config, stage4_config, stage4c_config
    )
    scenario = {
        "name": "stage5_v1_5_pi_mcu_reference_stream" if stream else "stage5_v1_5_full_path_replay",
        "duration_s": float(config["episode_duration_s"]),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    run = stage3b.run_case(
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
        reference_source=stream if stream is not None else source,
    )
    return run, source, provider, runtime


def replay_metrics(run: dict, source: SparseTwoPathReferenceSource, runtime, wheel_limit_nm: float) -> dict:
    history = run["history_50hz"]
    v_ref = np.asarray([row["rolling_v_ref_applied_m_s"] for row in history], dtype=float)
    v_gt = np.asarray([row["v_GT_m_s"] for row in history], dtype=float)
    theta_ref = np.asarray([row["rolling_theta_ref_applied_rad"] for row in history], dtype=float)
    theta_gt_error = np.asarray([
        row["theta_GT_rad"] - row["theta_eq_used_rad"] for row in history
    ], dtype=float)
    theta_hat_error = np.asarray([
        row["theta_hat_rad"] - row["theta_eq_used_rad"] for row in history
    ], dtype=float)
    v_error = v_gt - v_ref
    theta_error = theta_gt_error - theta_ref
    left = np.asarray([row["actual_left_nm"] for row in history], dtype=float)
    right = np.asarray([row["actual_right_nm"] for row in history], dtype=float)
    saturation = (
        (np.abs(left) >= wheel_limit_nm - 1e-12)
        | (np.abs(right) >= wheel_limit_nm - 1e-12)
        | np.asarray([row["sum_command_saturated"] for row in history], dtype=bool)
    )
    full_events = [event for event in source.replan_events if event["planning_path"] == "FULL_DYNAMIC"]
    overshoot = []
    for event in full_events:
        event_time = float(event["execution_start_time_s"])
        target = float(event["target_velocity_m_s"])
        delta = float(event.get("candidate_delta_v_m_s", target))
        direction = 1.0 if delta >= 0.0 else -1.0
        times = np.asarray([row["t"] for row in history], dtype=float)
        mask = (times >= event_time) & (times <= event_time + 2.0)
        if np.any(mask):
            overshoot.append(float(max(0.0, np.max(direction * (v_gt[mask] - target)))))
    summary = source.summary()
    return {
        "v_ref_minus_v_GT_rms_m_s": float(np.sqrt(np.mean(v_error**2))),
        "v_ref_minus_v_GT_peak_abs_m_s": float(np.max(np.abs(v_error))),
        "theta_GT_tracking_rms_rad": float(np.sqrt(np.mean(theta_error**2))),
        "theta_GT_tracking_peak_abs_rad": float(np.max(np.abs(theta_error))),
        "theta_hat_tracking_rms_rad": float(np.sqrt(np.mean((theta_hat_error - theta_ref)**2))),
        "full_target_overshoot_peak_m_s": float(max(overshoot, default=0.0)),
        "full_planning_wall_times_s": [float(event["planning_wall_time_s"]) for event in full_events],
        "full_post_interpolation_residuals": [float(event.get(
            "residual_500hz_post_interpolation_rms", event["normalized_residual_rms"]
        )) for event in full_events],
        "quiet_snap_count": len(summary["quiet_events"]),
        "fade_completion_count": sum(event["event"] == "fade_finished" for event in summary["fade_events"]),
        "saturation_count": int(np.count_nonzero(saturation)),
        "fall": bool(run["longitudinal"]["fell"]),
        "false_slope_count": sum(
            event.get("to") == "SLOPE" for event in runtime.transition_log
        ),
        "full_maneuver_count": len(full_events),
    }


def write_replay_csv(name: str, history: list[dict]) -> str:
    path = RESULT_DIR / f"stage5_v1_5_replay_{name}.csv"
    write_csv(path, history)
    return path.relative_to(ROOT).as_posix()


def run_replays(config: dict, common: tuple, stage4_config: dict, stage4c_config: dict, planners: dict) -> dict:
    wheel_limit = float(common[8]["known"]["wheel_torque_hard_peak_nm"])
    metrics = {}
    csv_paths = {}
    histories = {}
    for name in ("lsqr_500hz", "cached_kkt_500hz", "cached_kkt_250hz"):
        print(f"CLOSED LOOP replay {name}", flush=True)
        run, source, _provider, runtime = run_episode(
            config, common, stage4_config, stage4c_config, planners[name]
        )
        history = run["history_50hz"]
        metrics[name] = replay_metrics(run, source, runtime, wheel_limit)
        csv_paths[name] = write_replay_csv(name, history)
        histories[name] = history
    return {
        "metrics": metrics,
        "csv_paths": csv_paths,
        "histories": histories,
    }


def run_synthetic_delay_case(config: dict, common: tuple, planner: RuckigFullHorizonVelocityPlanner) -> dict:
    quiet_config = copy.deepcopy(config)
    quiet_config["synthetic_raw_v_cmd_schedule_m_s"] = [
        {"time_s": 0.0, "value": 0.0},
        {"time_s": float(config["episode_duration_s"]), "value": 0.0},
    ]
    provider = SyntheticCommandProvider(quiet_config)
    source = make_reference_source(
        quiet_config, common, provider,
        follower=synthetic_follower,
        full_planner_override=planner,
    )
    stream = DeterministicReferenceBlockStream(
        source,
        link_latency_s=float(config["stream"]["synthetic_delay_latency_s"]),
    )
    for tick in range(500):
        stream.command(tick * source.dt_s)
        _ = stream.next_reference_state
    result = stream.summary()
    return {
        "link_latency_s": result["link_latency_s"],
        "control_ticks": len(result["consumer_events"]),
        "generated_block_count": result["generated_block_count"],
        "underrun_count": len(result["underrun_events"]),
        "stale_block_count": len(result["stale_events"]),
        "sequence_gap_count": len(result["sequence_gap_events"]),
        "next_ready_at_swap_count": result["next_ready_at_swap_count"],
        "event_log": result,
    }


def stream_metrics(
    run: dict,
    source: SparseTwoPathReferenceSource,
    stream: DeterministicReferenceBlockStream,
    runtime,
    wheel_limit_nm: float,
) -> dict:
    replay = replay_metrics(run, source, runtime, wheel_limit_nm)
    state = stream.summary()
    replay.update({
        "reference_block_underrun_count": len(state["underrun_events"]),
        "stale_block_count": len(state["stale_events"]),
        "sequence_gap_count": len(state["sequence_gap_events"]),
        "block_count": state["generated_block_count"],
        "active_next_swaps": state["next_ready_at_swap_count"],
    })
    return replay


def make_stream_plots(history: list[dict], stream_summary: dict, source_summary: dict) -> list[str]:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    t = np.asarray([row["t"] for row in history], dtype=float)
    fig, axis = plt.subplots(figsize=(12, 5.5))
    for key, label in (
        ("raw_linear_velocity_cmd_m_s", "raw_v_cmd"),
        ("linear_velocity_cmd_m_s", "accepted_v_cmd"),
        ("rolling_v_ref_applied_m_s", "v_ref"),
        ("v_hat_m_s", "v_hat"),
        ("v_GT_m_s", "v_GT"),
    ):
        axis.plot(t, [row[key] for row in history], label=label)
    for event in source_summary["scheduler_events"]:
        if event.get("event") == "accepted":
            axis.axvline(float(event["time_s"]), color="0.45", linewidth=0.7, alpha=0.4)
    axis.set(xlabel="time [s]", ylabel="velocity [m/s]", title="V1.5 streamed command and response")
    axis.grid(True, alpha=0.25)
    axis.legend(ncol=5)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["velocity"], dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(t, np.degrees([row["rolling_theta_ref_applied_rad"] for row in history]), label="theta_ref")
    axes[0].plot(t, np.degrees([row["theta_hat_rad"] - row["theta_eq_used_rad"] for row in history]), label="theta_hat error")
    axes[0].plot(t, np.degrees([row["theta_GT_rad"] - row["theta_eq_used_rad"] for row in history]), label="theta_GT error")
    axes[0].set(ylabel="pitch error [deg]")
    axes[0].legend(ncol=3)
    axes[1].plot(t, [row["u_ff_used_nm"] for row in history], label="u_ff_applied")
    axes[1].set(xlabel="time [s]", ylabel="torque [N m]")
    axes[1].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.suptitle("V1.5 dynamic reference and feedforward")
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["dynamic"], dpi=180)
    plt.close(fig)

    consumed = stream_summary["consumer_events"]
    sc_t = np.asarray([row["time_s"] for row in consumed], dtype=float)
    active = np.asarray([row["active_sequence_id"] for row in consumed], dtype=float)
    next_ready = np.asarray([row["next_block_ready"] for row in consumed], dtype=float)
    remaining = np.asarray([row["buffer_remaining_s"] for row in consumed], dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(12, 8.5), sharex=True)
    axes[0].step(sc_t, active, where="post", label="ACTIVE sequence")
    axes[0].plot(sc_t, np.where(next_ready > 0.0, active + 1, np.nan), ".", ms=1.5, label="NEXT ready")
    axes[0].set(ylabel="sequence")
    axes[0].legend()
    blocks = stream_summary["block_events"]
    generated = [row for row in blocks if row["event"] == "block_generated"]
    axes[1].scatter(
        [row["generated_time_s"] for row in generated],
        [row["available_time_s"] for row in generated],
        s=13, label="generated → available",
    )
    axes[1].plot([0, sc_t[-1]], [0, sc_t[-1]], color="0.6", linewidth=0.8)
    axes[1].set(ylabel="available time [s]")
    axes[1].legend()
    axes[2].plot(sc_t, remaining * 1000.0, label="ACTIVE buffer remaining")
    axes[2].set(xlabel="consumer time [s]", ylabel="ms")
    axes[2].legend()
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.suptitle("ACTIVE/NEXT reference block stream")
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["stream"], dpi=180)
    plt.close(fig)

    plans = [row for row in stream_summary["planning_events"] if row["planning_path"] == "FULL_DYNAMIC"]
    fig, axis = plt.subplots(figsize=(12, 4.6))
    for index, event in enumerate(plans):
        y = 2 - index * 0.08
        axis.scatter(float(event["planning_request_time_s"]), y, marker="o", label="planning request" if index == 0 else None)
        axis.scatter(float(event["planning_complete_time_s"]), y - 0.03, marker="s", label="planning complete" if index == 0 else None)
        axis.scatter(float(event["execution_start_time_s"]), y - 0.06, marker="^", label="FULL execution start" if index == 0 else None)
        axis.plot(
            [float(event["planning_request_time_s"]), float(event["planning_complete_time_s"])],
            [y, y - 0.03], color="0.5", linewidth=0.8,
        )
    axis.set(xlabel="simulation time [s]", yticks=[], title="FULL plan request, completion, and execution")
    axis.grid(True, axis="x", alpha=0.25)
    if plans:
        axis.legend(ncol=3)
    fig.tight_layout()
    fig.savefig(PLOT_PATHS["planning"], dpi=180)
    plt.close(fig)
    return [str(path.relative_to(ROOT).as_posix()) for path in PLOT_PATHS.values()]


def run_stream_final(config: dict, common: tuple, stage4_config: dict, stage4c_config: dict, planner) -> dict:
    provider = SyntheticCommandProvider(config)
    source = make_reference_source(
        config, common, provider,
        follower=synthetic_follower,
        full_planner_override=planner,
    )
    stream = DeterministicReferenceBlockStream(
        source, link_latency_s=float(config["stream"]["main_link_latency_s"])
    )
    q_adapter, runtime = make_runtime(common, config, stage4_config, stage4c_config)
    scenario = {
        "name": "stage5_v1_5_pi_mcu_reference_stream",
        "duration_s": float(config["episode_duration_s"]),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    try:
        run = stage3b.run_case(
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
            reference_source=stream,
        )
    except ReferenceBlockUnderrun as error:
        state = stream.summary()
        return {
            "status": "UNDERRUN",
            "error": str(error),
            "stream_summary": state,
            "source_summary": source.summary(),
        }
    history = run["history_50hz"]
    stream_state = stream.summary()
    metrics = stream_metrics(
        run, source, stream, runtime,
        float(common[8]["known"]["wheel_torque_hard_peak_nm"]),
    )
    csv_path = RESULT_DIR / "stage5_v1_5_stream_final_history.csv"
    write_csv(csv_path, history)
    consumer_csv = RESULT_DIR / "stage5_v1_5_stream_consumption.csv"
    write_csv(consumer_csv, stream_state["consumer_events"])
    block_csv = RESULT_DIR / "stage5_v1_5_stream_blocks.csv"
    write_csv(block_csv, stream_state["block_events"])
    plots = make_stream_plots(history, stream_state, source.summary())
    return {
        "status": "COMPLETED",
        "selected_planner": planner.projection_backend,
        "metrics": metrics,
        "stream_summary": {
            key: value for key, value in stream_state.items()
            if key != "consumer_events"
        },
        "source_summary": source.summary(),
        "history_csv": csv_path.relative_to(ROOT).as_posix(),
        "consumer_csv": consumer_csv.relative_to(ROOT).as_posix(),
        "block_csv": block_csv.relative_to(ROOT).as_posix(),
        "plots": plots,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("benchmark", "stream", "finalize"), default="benchmark")
    parser.add_argument(
        "--solver", choices=("lsqr_500hz", "cached_kkt_500hz", "cached_kkt_250hz"),
        default="cached_kkt_250hz",
    )
    args = parser.parse_args()
    config = load_json(CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen_checks = load_common(config)

    if args.phase == "finalize":
        benchmark = load_json(BENCHMARK_PATH)
        result = load_json(RESULT_PATH)
        original_v14_time = float(
            benchmark.get(
                "recorded_v1_4_full_planning_wall_time_s",
                recorded_v14_full_planning_time_s(),
            )
        )
        benchmark["recorded_v1_4_full_planning_wall_time_s"] = original_v14_time
        write_json(BENCHMARK_PATH, benchmark)
        result["original_v1_4_full_planning_wall_time_s"] = original_v14_time
        write_json(RESULT_PATH, result)
        final = result["final_stream"]
        selected_name = result["selected_planner"]
        selected_benchmark = benchmark["paths"][selected_name]
        measured_stream_time = float(final["metrics"]["full_planning_wall_times_s"][0])
        cache_build_s = (
            float(selected_benchmark["cold"]["matrix_build_time_s"])
            + float(selected_benchmark["cold"]["factorization_time_s"])
        )
        synthetic = final.get("synthetic_delay_case", {})
        replay = load_json(REPLAY_PATH)
        path_labels = {
            "lsqr_500hz": "LSQR 500 Hz",
            "cached_kkt_500hz": "cached KKT 500 Hz",
            "cached_kkt_250hz": "cached KKT 250 Hz",
        }
        summary_lines = [
            "# Stage 5 V1.5 — Pi/MCU reference stream and FULL planner acceleration",
            "",
            f"- Selected path: `{selected_name}`; final status: **{final['status']}**.",
            f"- Stored V1.4 FULL planning time: {original_v14_time * 1000:.2f} ms; final stream warm plan: {measured_stream_time * 1000:.2f} ms ({original_v14_time / max(measured_stream_time, 1e-12):.2f}× faster).",
            f"- Selected warm benchmark: {selected_benchmark['warm']['planning_wall_time_s'] * 1000:.2f} ms; one-time matrix build + LU factorization: {cache_build_s * 1000:.2f} ms.",
            f"- Horizon: bucket {selected_benchmark['warm']['horizon_bucket_s']:.3f} s; actual projection {selected_benchmark['warm']['projected_horizon_s']:.3f} s; post-interpolation 500 Hz residual {selected_benchmark['warm']['residual_500hz_post_interpolation_rms']:.5f} (V1.4 was 0.01750).",
            f"- Stream: underruns {final['metrics']['reference_block_underrun_count']}, stale {final['metrics']['stale_block_count']}, sequence gaps {final['metrics']['sequence_gap_count']}; saturation {final['metrics']['saturation_count']}; false SLOPE {final['metrics']['false_slope_count']}; fall {str(final['metrics']['fall']).lower()}.",
            f"- 4 ms synthetic-delay check: {synthetic.get('link_latency_s', 'n/a')} s; underruns {synthetic.get('underrun_count', 'n/a')}, stale {synthetic.get('stale_block_count', 'n/a')}, sequence gaps {synthetic.get('sequence_gap_count', 'n/a')}.",
            "",
            "Closed-loop replay (same V1.4 episode):",
            "| Path | Warm planning | 500 Hz residual | v error RMS / peak | theta tracking RMS | sat / fall |",
            "|---|---:|---:|---:|---:|---:|",
        ]
        for name in ("lsqr_500hz", "cached_kkt_500hz", "cached_kkt_250hz"):
            item = replay["metrics"][name]
            timing = benchmark["paths"][name]["warm"]["planning_wall_time_s"]
            residual = benchmark["paths"][name]["warm"]["residual_500hz_post_interpolation_rms"]
            summary_lines.append(
                f"| {path_labels[name]} | {timing * 1000:.2f} ms | {residual:.5f} | "
                f"{item['v_ref_minus_v_GT_rms_m_s']:.5f} / {item['v_ref_minus_v_GT_peak_abs_m_s']:.5f} m/s | "
                f"{item['theta_GT_tracking_rms_rad']:.5f} rad | "
                f"{item['saturation_count']} / {str(item['fall']).lower()} |"
            )
        replay_metrics = replay["metrics"]
        summary_lines.append(
            "- Replay quiet snaps / fade completions: "
            + "; ".join(
                f"{path_labels[name]} {replay_metrics[name]['quiet_snap_count']} / "
                f"{replay_metrics[name]['fade_completion_count']} (overshoot "
                f"{replay_metrics[name]['full_target_overshoot_peak_m_s']:.5f} m/s)"
                for name in ("lsqr_500hz", "cached_kkt_500hz", "cached_kkt_250hz")
            )
            + "."
        )
        summary_lines.extend([
            "",
            f"- Benchmark JSON: `{BENCHMARK_PATH.relative_to(ROOT).as_posix()}`.",
            f"- Replay comparison JSON: `{REPLAY_PATH.relative_to(ROOT).as_posix()}`.",
            f"- Final JSON: `{RESULT_PATH.relative_to(ROOT).as_posix()}`.",
            f"- Summary: `{SUMMARY_PATH.relative_to(ROOT).as_posix()}`.",
        ])
        if final["status"] == "COMPLETED":
            summary_lines.extend([
                f"- Raw histories: `{final['history_csv']}`, `{final['consumer_csv']}`, `{final['block_csv']}`.",
                f"- Plots: {', '.join(f'`{item}`' for item in final['plots'])}.",
            ])
        SUMMARY_PATH.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
        print(json.dumps({
            "result_json": str(RESULT_PATH),
            "summary": str(SUMMARY_PATH),
            "original_v1_4_full_planning_wall_time_s": original_v14_time,
            "final_stream_full_planning_wall_time_s": measured_stream_time,
        }, indent=2, ensure_ascii=False), flush=True)
        return

    if args.phase == "benchmark":
        benchmark = run_planner_benchmarks(config, common)
        planners = benchmark.pop("planner_objects")
        caches = benchmark.pop("factorization_caches")
        write_json(BENCHMARK_PATH, benchmark)
        replay = run_replays(
            config, common, stage4_config, stage4c_config, planners
        )
        histories = replay.pop("histories")
        write_json(REPLAY_PATH, replay)
        print(json.dumps({
            "benchmark_json": str(BENCHMARK_PATH),
            "replay_json": str(REPLAY_PATH),
            "benchmark_paths": benchmark["paths"],
            "parity": benchmark["parity"],
            "replay_metrics": replay["metrics"],
            "replay_csv_paths": replay["csv_paths"],
            "frozen_baseline_checks": frozen_checks,
        }, indent=2, ensure_ascii=False), flush=True)
        return

    benchmark = json.loads(BENCHMARK_PATH.read_text(encoding="utf-8"))
    cache = SparseProjectionFactorizationCache()
    planner_config = {
        "lsqr_500hz": (0.002, "lsqr"),
        "cached_kkt_500hz": (0.002, "cached_kkt"),
        "cached_kkt_250hz": (0.004, "cached_kkt"),
    }
    planning_dt, backend = planner_config[args.solver]
    planner = make_full_planner(
        config, common, planning_dt_s=planning_dt, backend=backend,
        cache=(cache if backend == "cached_kkt" else None),
    )
    # Warm only the exact Stage 5 splice/cache shape before the streamed replay.
    planner.plan(SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET)
    planner.plan(SPLICE_STATE, SPLICE_ACCELERATION, SPLICE_TARGET)
    print(f"RUN final streamed episode using {args.solver}", flush=True)
    final = run_stream_final(
        config, common, stage4_config, stage4c_config, planner
    )
    if final["status"] == "COMPLETED":
        delay = run_synthetic_delay_case(config, common, planner)
        final["synthetic_delay_case"] = {
            key: value for key, value in delay.items() if key != "event_log"
        }
        delay_csv = RESULT_DIR / "stage5_v1_5_synthetic_delay_blocks.csv"
        write_csv(delay_csv, delay["event_log"]["block_events"])
        final["synthetic_delay_case"]["block_csv"] = delay_csv.relative_to(ROOT).as_posix()
    result = {
        "stage": config["stage"],
        "status": final["status"],
        "config": config,
        "selected_planner": args.solver,
        "benchmark_json": BENCHMARK_PATH.relative_to(ROOT).as_posix(),
        "closed_loop_comparison_json": REPLAY_PATH.relative_to(ROOT).as_posix(),
        "frozen_baseline_checks": frozen_checks,
        "original_v1_4_full_planning_wall_time_s": float(benchmark.get(
            "recorded_v1_4_full_planning_wall_time_s",
            recorded_v14_full_planning_time_s(),
        )),
        "final_stream": final,
    }
    write_json(RESULT_PATH, result)
    summary_lines = [
        "# Stage 5 V1.5 — Pi/MCU reference stream and FULL planner acceleration",
        "",
        f"- Selected path: `{args.solver}`; final status: **{final['status']}**.",
        f"- Benchmark: `{BENCHMARK_PATH.relative_to(ROOT).as_posix()}`.",
        f"- Closed-loop comparison: `{REPLAY_PATH.relative_to(ROOT).as_posix()}`.",
        f"- Final JSON: `{RESULT_PATH.relative_to(ROOT).as_posix()}`.",
    ]
    if final["status"] == "COMPLETED":
        metrics = final["metrics"]
        summary_lines.extend([
            f"- V1.4 FULL planning: {result['original_v1_4_full_planning_wall_time_s'] * 1000:.1f} ms; selected stream FULL plan: {final['metrics']['full_planning_wall_times_s'][0] * 1000:.1f} ms.",
            f"- Speedup: {result['original_v1_4_full_planning_wall_time_s'] / max(final['metrics']['full_planning_wall_times_s'][0], 1e-12):.2f}×; 500 Hz post-interpolation residual: {benchmark['paths'][args.solver]['warm']['residual_500hz_post_interpolation_rms']:.5f}.",
            f"- Stream: underruns {metrics['reference_block_underrun_count']}, stale {metrics['stale_block_count']}, sequence gaps {metrics['sequence_gap_count']}; saturation {metrics['saturation_count']}; fall {str(metrics['fall']).lower()}.",
            f"- History CSV: `{final['history_csv']}`; consumer CSV: `{final['consumer_csv']}`; block CSV: `{final['block_csv']}`.",
            f"- Plots: {', '.join(f'`{item}`' for item in final['plots'])}.",
        ])
    else:
        summary_lines.append(f"- Underrun detail: `{final.get('error')}`")
    SUMMARY_PATH.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    print(json.dumps({
        "result_json": str(RESULT_PATH),
        "summary": str(SUMMARY_PATH),
        "final_status": final["status"],
        "metrics": final.get("metrics"),
        "benchmark_paths": benchmark["paths"],
    }, indent=2, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
