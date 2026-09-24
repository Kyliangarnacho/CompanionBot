"""Archived Stage 6.1 all-FULL comparison runner for historical reproduction."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = next(
    parent for parent in Path(__file__).resolve().parents
    if (parent / "scripts").is_dir() and (parent / "models" / "minisegway").is_dir()
)
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

import run_stage3b_yaw_control as stage3b  # noqa: E402
import run_stage4_final_closeout as stage4_final  # noqa: E402
import run_stage5_v1_5_pi_mcu_stream as stage5_v15  # noqa: E402
from control.follow_governor import FollowGovernor, FollowGovernorConfig  # noqa: E402
from control.rolling_reference import (  # noqa: E402
    DeterministicReferenceBlockStream,
    TargetObservation,
)
from control.trajectory_feedforward import SparseProjectionFactorizationCache  # noqa: E402
from sim.slope_estimation import (  # noqa: E402
    analytic_theta_eq,
    equilibrium_sum_torque_nm,
)


STAGE_DIR = ROOT / "models" / "minisegway" / "stage6"
REFERENCE_DIR = STAGE_DIR / "reference" / "stage6_1"
CONFIG_PATH = REFERENCE_DIR / "config" / "stage6_1_dynamic_braking_all_full_config.json"
RESULT_DIR = REFERENCE_DIR / "results" / "dynamic_braking_all_full"
RESULT_PATH = RESULT_DIR / "stage6_1_dynamic_braking_all_full_results.json"


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    return value


def _schedule_value(schedule: list[dict], time_s: float) -> float:
    value = float(schedule[0]["value_m_s"])
    for entry in schedule:
        if float(entry["time_s"]) > time_s + 1e-12:
            break
        value = float(entry["value_m_s"])
    return value


class SyntheticTargetSensor:
    """Sim-only world truth adapter that publishes body-relative packets."""

    def __init__(self, scenario: dict, observation_frequency_hz: float) -> None:
        self.scenario = scenario
        self.period_s = 1.0 / float(observation_frequency_hz)
        self.sequence_id = 0
        self.latest: TargetObservation | None = None
        self.latest_evaluation: dict | None = None
        self.evaluation_history: list[dict] = []
        self.master_position_xy_m: np.ndarray | None = None
        self.master_heading_rad = 0.0
        self.last_physics_time_s = 0.0
        self.next_capture_time_s = 0.0

    @staticmethod
    def _robot_pose(sim) -> tuple[np.ndarray, np.ndarray]:
        root = sim.model.joint("root").id
        qpos_adr = int(sim.model.jnt_qposadr[root])
        chassis = sim.model.body("chassis").id
        position = np.asarray(sim.data.qpos[qpos_adr : qpos_adr + 3], dtype=float)
        local_to_world = np.asarray(sim.data.xmat[chassis], dtype=float).reshape(3, 3)
        return position, local_to_world

    def setup(self, sim) -> dict:
        robot_position, local_to_world = self._robot_pose(sim)
        forward_world = -local_to_world[:, 1]
        initial_distance = float(self.scenario["initial_distance_m"])
        self.master_position_xy_m = (
            robot_position[:2] + initial_distance * forward_world[:2]
        )
        self.master_heading_rad = math.atan2(
            float(forward_world[1]), float(forward_world[0])
        )
        self.last_physics_time_s = float(sim.data.time)
        self.next_capture_time_s = self.last_physics_time_s
        self._capture(sim)
        self.next_capture_time_s += self.period_s
        return {
            "provider": "Stage6SyntheticTargetSensor",
            "observation_frequency_hz": 1.0 / self.period_s,
            "packet_version": 1,
            "controller_reads_sim_truth": False,
            "truth_boundary": "SyntheticTargetSensor internal; evaluation fields are post-hoc",
        }

    def _capture(self, sim) -> None:
        assert self.master_position_xy_m is not None
        robot_position, local_to_world = self._robot_pose(sim)
        relative_world = np.r_[self.master_position_xy_m - robot_position[:2], 0.0]
        relative_body = local_to_world.T @ relative_world
        observation = TargetObservation(
            capture_time_s=float(sim.data.time),
            x_forward_m=-float(relative_body[1]),
            y_left_m=-float(relative_body[0]),
            version=1,
            sequence_id=self.sequence_id,
            valid=True,
            confidence=1.0,
        )
        self.sequence_id += 1
        self.latest = observation
        # The following GT values are held by this evaluation side channel only.
        self.latest_evaluation = {
            "capture_time_s": observation.capture_time_s,
            "sequence_id": observation.sequence_id,
            "x_forward_GT_m": observation.x_forward_m,
            "master_velocity_GT_m_s": _schedule_value(
                self.scenario["master_speed_schedule"], observation.capture_time_s
            ),
        }
        self.evaluation_history.append(self.latest_evaluation.copy())

    def on_physics_step(self, sim) -> None:
        assert self.master_position_xy_m is not None
        time_s = float(sim.data.time)
        dt_s = time_s - self.last_physics_time_s
        midpoint = self.last_physics_time_s + 0.5 * dt_s
        speed = _schedule_value(self.scenario["master_speed_schedule"], midpoint)
        direction = np.asarray([
            math.cos(self.master_heading_rad), math.sin(self.master_heading_rad)
        ])
        self.master_position_xy_m += dt_s * speed * direction
        self.last_physics_time_s = time_s
        while time_s + 1e-12 >= self.next_capture_time_s:
            self._capture(sim)
            self.next_capture_time_s += self.period_s

    def read(self) -> TargetObservation:
        if self.latest is None:
            raise RuntimeError("synthetic target sensor is not initialized")
        return self.latest

    def diagnostic(self, sim) -> dict:
        del sim
        if self.latest is None or self.latest_evaluation is None:
            return {}
        return {
            "target_observation_version": self.latest.version,
            "target_observation_sequence_id": self.latest.sequence_id,
            "target_observation_capture_time_s": self.latest.capture_time_s,
            "target_observation_valid": self.latest.valid,
            "target_observation_confidence": self.latest.confidence,
            "target_observation_x_forward_m": self.latest.x_forward_m,
            "target_observation_y_left_m": self.latest.y_left_m,
            **self.latest_evaluation,
        }


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                fields.append(key)
                seen.add(key)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(_json_safe(rows))


def _rmse(values: list[float]) -> float | None:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        return None
    return float(np.sqrt(np.mean(array**2)))


def _response_delays(history: list[dict], accepted_events: list[dict]) -> list[dict]:
    rows = []
    for index, event in enumerate(accepted_events):
        start_time = float(event["time_s"])
        stop_time = (float(accepted_events[index + 1]["time_s"])
                     if index + 1 < len(accepted_events) else math.inf)
        previous = float(event["previous_accepted_velocity_m_s"])
        target = float(event["accepted_velocity_m_s"])
        threshold = previous + 0.9 * (target - previous)
        response = {"event_time_s": start_time, "target_velocity_m_s": target,
                    "mode": event["mode"]}
        for key, output in (("rolling_v_ref_applied_m_s", "v_ref_90pct_delay_s"),
                            ("v_hat_m_s", "v_hat_90pct_delay_s")):
            response[output] = next((
                float(row["t"]) - start_time for row in history
                if start_time - 1e-12 <= float(row["t"]) < stop_time - 1e-12
                and ((float(row[key]) >= threshold) if target > previous
                     else (float(row[key]) <= threshold))
            ), None)
        rows.append(response)
    return rows


def _make_plot(
    scenario_name: str,
    history: list[dict],
    governor_observations: list[dict],
    governor_events: list[dict],
    full_starts: list[dict],
    d1_m: float,
    d2_m: float,
    plot_path: Path,
) -> None:
    time_s = np.asarray([row["t"] for row in history], dtype=float)
    obs_time = [row["capture_time_s"] for row in governor_observations]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(time_s, [row["target_observation_x_forward_m"] for row in history],
                 label="relative distance", color="tab:blue")
    axes[0].axhline(d1_m, color="tab:red", linestyle=":", label="d1")
    axes[0].axhline(d2_m, color="tab:green", linestyle=":", label="d2 (target)")
    axes[0].plot(obs_time,
                 [row["switch_distance_m"] for row in governor_observations],
                 "--", color="tab:orange", label="dynamic switch distance")
    axes[0].set_ylabel("relative distance [m]")
    axes[0].legend(ncol=4, fontsize=8)

    axes[1].plot(obs_time,
                 [row["master_velocity_hat_m_s"] for row in governor_observations],
                 label="Master v estimated", color="tab:purple")
    axes[1].step(obs_time,
                 [row["latched_v_cmd_m_s"] for row in governor_observations],
                 where="post", label="Governor latched v_cmd", color="tab:red")
    axes[1].plot(time_s, [row["rolling_v_ref_applied_m_s"] for row in history],
                 label="Stage 5 v_ref", color="tab:orange")
    axes[1].plot(time_s, [row["v_hat_m_s"] for row in history],
                 label="robot v_hat", color="tab:blue")
    axes[1].set(xlabel="time [s]", ylabel="velocity [m/s]")
    axes[1].legend(ncol=4, fontsize=8)

    for index, event in enumerate(governor_events):
        axes[0].axvline(float(event["time_s"]), color="0.25", alpha=0.35,
                        linewidth=1.0, label="Governor event" if index == 0 else None)
    for index, event in enumerate(full_starts):
        axes[1].axvline(float(event["time_s"]), color="0.25", alpha=0.35,
                        linestyle=":", linewidth=1.0,
                        label="FULL start" if index == 0 else None)
    if governor_events:
        axes[0].legend(ncol=5, fontsize=8)
    if full_starts:
        axes[1].legend(ncol=5, fontsize=8)
    fig.suptitle(f"Stage 6.1 response — {scenario_name}")
    for axis in axes:
        axis.grid(True, alpha=0.25)
    fig.tight_layout()
    plot_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(plot_path, dpi=160)
    plt.close(fig)


def run_scenario(
    config: dict,
    stage5_config: dict,
    common: tuple,
    stage4_config: dict,
    stage4c_config: dict,
    scenario: dict,
    result_dir: Path,
    make_plot: bool,
) -> dict:
    governor_config = FollowGovernorConfig(**config["governor"])
    stage5_acceleration_limit = float(
        common[3]["reference_limits"]["max_acceleration_m_s2"]
    )
    if governor_config.a_brake_effective_m_s2 > stage5_acceleration_limit:
        raise ValueError("effective braking acceleration exceeds Stage 5 limit")
    governor = FollowGovernor(governor_config)
    sensor = SyntheticTargetSensor(
        scenario, float(config["observation_frequency_hz"])
    )
    cache = SparseProjectionFactorizationCache()
    full_planner = stage5_v15.make_full_planner(
        stage5_config,
        common,
        planning_dt_s=float(stage5_config["full_dynamic"]["planning_dt_s"]),
        backend="cached_kkt",
        cache=cache,
    )
    source = stage5_v15.make_reference_source(
        stage5_config,
        common,
        sensor,
        follower=governor,
        full_planner_override=full_planner,
        scheduler_gate_overrides=config["follow_scheduler_gate_policy"],
    )
    stream = DeterministicReferenceBlockStream(
        source,
        link_latency_s=float(stage5_config["stream"]["main_link_latency_s"]),
    )
    q_adapter, runtime = stage5_v15.make_runtime(
        common, stage5_config, stage4_config, stage4c_config
    )
    scenario_input = {
        "name": scenario["name"],
        "duration_s": float(scenario["duration_s"]),
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
    }
    manifest = common[0]
    run = stage3b.run_case(
        scenario_input,
        float(manifest["yaw"]["K_psi_nm_per_rad"]),
        float(manifest["yaw"]["K_r_nm_per_rad_s"]),
        yaw_enabled=True,
        motor_mismatch_enabled=False,
        common=common,
        keep_history=True,
        payload_mode="empty",
        common_mode_augmentation=q_adapter,
        physics_step_callback=sensor.on_physics_step,
        simulation_setup_callback=sensor.setup,
        history_diagnostic_callback=sensor.diagnostic,
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
        reference_context_callback=lambda time_s, estimate: (
            governor.set_robot_velocity_hat(estimate.velocity_hat_m_s)
        ),
    )

    history = run["history_50hz"]
    governor_rows = governor.observation_history
    governor_index = 0
    latest_governor_state = None
    for row in history:
        observation_sequence = int(row["target_observation_sequence_id"])
        while (
            governor_index < len(governor_rows)
            and int(governor_rows[governor_index]["sequence_id"])
            <= observation_sequence
        ):
            latest_governor_state = governor_rows[governor_index]
            governor_index += 1
        if latest_governor_state is not None:
            state = latest_governor_state
            row.update({
                "governor_processed_sequence_id": state["sequence_id"],
                "governor_state": state["governor_state"],
                "master_velocity_hat_m_s": state["master_velocity_hat_m_s"],
                "robot_velocity_hat_for_master_estimate_m_s": state[
                    "robot_velocity_hat_m_s"
                ],
                "governor_latched_v_cmd_m_s": state["latched_v_cmd_m_s"],
                "governor_command_changed": state["command_changed"],
                "x_rel_dot_hat_m_s": state["x_rel_dot_hat_m_s"],
                "d2_m": state["d2_m"],
                "closing_speed_m_s": state["closing_speed_m_s"],
                "transition_distance_m": state["transition_distance_m"],
                "switch_distance_m": state["switch_distance_m"],
                "unable_to_close": state["unable_to_close"],
            })
    stem = str(scenario["name"])
    plot_dir = result_dir / "plots"
    _write_csv(result_dir / f"{stem}_history.csv", history)
    _write_csv(
        result_dir / f"{stem}_governor_observations.csv",
        governor.observation_history,
    )
    _write_csv(result_dir / f"{stem}_sensor_evaluation.csv", sensor.evaluation_history)
    _write_csv(result_dir / f"{stem}_governor_events.csv", governor.events)
    if make_plot:
        _make_plot(
            stem, history, governor.observation_history, governor.events,
            [event for event in source.replan_events
             if event["planning_path"] == "FULL_DYNAMIC"],
            float(config["governor"]["d1_m"]),
            float(config["governor"]["d2_m"]),
            plot_dir / f"{stem}.png",
        )

    observation_pairs = list(zip(governor.observation_history, sensor.evaluation_history))
    velocity_errors = [
        float(estimate["master_velocity_hat_m_s"] - truth["master_velocity_GT_m_s"])
        for estimate, truth in observation_pairs
        if estimate["x_rel_dot_hat_m_s"] is not None
    ]
    scheduler_events = source.scheduler.events
    accepted_events = [event for event in scheduler_events if event["event"] == "accepted"]
    planning_paths = [event["planning_path"] for event in source.replan_events]
    full_plans = [event for event in source.replan_events
                  if event["planning_path"] == "FULL_DYNAMIC"]
    overlap_events = [
        event for event in governor.events
        if any(
            float(plan["time_s"]) + 1e-12 < float(event["time_s"])
            and (plan["full_exit_time_s"] is None
                 or float(event["time_s"]) < float(plan["full_exit_time_s"]) - 1e-12)
            for plan in full_plans
        )
    ]
    stream_summary = stream.summary()
    environment_modes = [row.get("environment_mode") for row in history]
    result = {
        "scenario": scenario,
        "simulation": {
            "actual_duration_s": run["simulation_termination"]["actual_duration_s"],
            "terminated_early": run["simulation_termination"]["terminated_early"],
            "termination_reason": run["simulation_termination"]["reason"],
            "fell": run["longitudinal"]["fell"],
            "finite": run["finite"],
            "saturation_fraction": run["torque_allocation"]["per_wheel_saturation_fraction"],
            "differential_clipping_fraction": run["torque_allocation"][
                "differential_clipping_fraction"
            ],
            "final_guard_clipping_fraction": run["torque_allocation"][
                "final_guard_clipping_fraction"
            ],
            "sum_command_saturated_samples": sum(
                bool(row["sum_command_saturated"]) for row in history
            ),
            "allocator_guard_clipped_samples": sum(
                bool(row["allocator_guard_clipped"]) for row in history
            ),
            "pitch_peak_GT_deg": run["longitudinal"]["pitch_peak_GT_deg"],
            "false_slope_sample_count": sum(mode == "SLOPE" for mode in environment_modes),
        },
        "governor": {
            "events": governor.events,
            "observation_count": len(governor.observation_history),
            "duplicate_observation_count": governor.duplicate_observation_count,
            "master_velocity_estimate_rmse_m_s": _rmse(velocity_errors),
            "final_state": governor.state,
            "final_latched_v_cmd_m_s": governor.latched_velocity_m_s,
            "final_distance_m": (
                None if sensor.latest is None else sensor.latest.x_forward_m
            ),
            "min_distance_m": min(
                float(row["target_observation_x_forward_m"]) for row in history
            ),
            "catch_retrigger_count": sum(
                event["event"] == "catch_up_retriggered" for event in governor.events
            ),
            "unable_to_close_sample_count": governor.unable_to_close_sample_count,
        },
        "stage5": {
            "accepted_command_count": len(accepted_events),
            "accepted_commands": accepted_events,
            "planning_paths": planning_paths,
            "lightweight_replan_count": planning_paths.count("LIGHTWEIGHT"),
            "full_replan_count": planning_paths.count("FULL_DYNAMIC"),
            "full_exit_count": len(source.full_exit_events),
            "full_exit_events": source.full_exit_events,
            "response_90pct_delays": _response_delays(history, accepted_events),
            "governor_events_during_active_full": overlap_events,
            "pending_accepted_after_full_exit_count": sum(
                bool(event.get("processed_after_full_exit")) for event in accepted_events
            ),
            "pending_observation_count": sum(
                bool(row["full_locked"])
                and abs(float(row["latest_pending_raw_v_cmd_m_s"])
                        - float(row["accepted_linear_velocity_target_m_s"])) > 1e-12
                for row in source.intent_history
            ),
            "v_ref_command_iae_m": sum(
                abs(float(row["rolling_v_ref_applied_m_s"])
                    - float(row["raw_linear_velocity_cmd_m_s"])) * 0.02
                for row in history
            ),
            "v_hat_command_iae_m": sum(
                abs(float(row["v_hat_m_s"])
                    - float(row["raw_linear_velocity_cmd_m_s"])) * 0.02
                for row in history
            ),
            "scheduler_gate_policy": config["follow_scheduler_gate_policy"],
            "scheduler_events": scheduler_events,
        },
        "stream": {
            "underrun_count": len(stream_summary["underrun_events"]),
            "stale_count": len(stream_summary["stale_events"]),
            "sequence_gap_count": len(stream_summary["sequence_gap_events"]),
            "next_ready_at_swap_count": stream_summary["next_ready_at_swap_count"],
            "block_count": stream_summary["generated_block_count"],
        },
        "source_summary": source.summary(),
        "baseline_runtime": {
            "GT_runtime_dependency": run["GT_runtime_dependency"],
            "Q_disturbance_rejection_enabled": run["frozen_longitudinal"]["Q_disturbance_rejection_enabled"],
            "controller_dt_s": float(common[3]["controller_dt_s"]),
            "physics_dt_s": 0.001,
        },
        "files": {
            "history_csv": (result_dir / f"{stem}_history.csv").relative_to(ROOT).as_posix(),
            "governor_observations_csv": (result_dir / f"{stem}_governor_observations.csv").relative_to(ROOT).as_posix(),
            "sensor_evaluation_csv": (result_dir / f"{stem}_sensor_evaluation.csv").relative_to(ROOT).as_posix(),
            "governor_events_csv": (result_dir / f"{stem}_governor_events.csv").relative_to(ROOT).as_posix(),
            "plot": ((plot_dir / f"{stem}.png").relative_to(ROOT).as_posix()
                     if make_plot else None),
        },
    }
    return _json_safe(result)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--scenario", help="Run only one configured scenario name")
    parser.add_argument("--planner-mode", choices=("full", "mixed"), default="full",
                        help="Mixed uses the same governor for a planner-only comparison")
    args = parser.parse_args()
    config = stage5_v15.load_json(args.config)
    if args.planner_mode == "mixed":
        config["follow_scheduler_gate_policy"]["follow_force_full_dynamic"] = False
    stage5_config = stage5_v15.load_json(stage5_v15.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen_checks = stage5_v15.load_common(
        stage5_config
    )
    if not all(frozen_checks.values()):
        raise RuntimeError(f"frozen Stage 4 baseline validation failed: {frozen_checks}")

    result_dir = (RESULT_DIR if args.planner_mode == "full"
                  else REFERENCE_DIR / "results" / "dynamic_braking_mixed_ablation")
    result_path = (RESULT_PATH if args.planner_mode == "full"
                   else result_dir / "stage6_1_dynamic_braking_mixed_results.json")
    result_dir.mkdir(parents=True, exist_ok=True)
    if args.planner_mode == "full":
        (result_dir / "plots").mkdir(parents=True, exist_ok=True)
    scenarios = config["scenarios"]
    if args.scenario is not None:
        scenarios = [item for item in scenarios if item["name"] == args.scenario]
        if not scenarios:
            raise ValueError(f"unknown configured scenario: {args.scenario}")
    results = []
    for scenario in scenarios:
        print(f"RUN {scenario['name']}", flush=True)
        results.append(run_scenario(
            config,
            stage5_config,
            common,
            stage4_config,
            stage4c_config,
            scenario,
            result_dir,
            args.planner_mode == "full",
        ))
        current = {
            "stage": config["stage"],
            "status": "IN_PROGRESS",
            "frozen_baseline_checks": frozen_checks,
            "configuration": config,
            "planner_mode": args.planner_mode,
            "scenarios": results,
        }
        stage5_v15.write_json(result_path, _json_safe(current))

    final = {
        "stage": config["stage"],
        "status": "COMPLETED",
        "frozen_baseline_checks": frozen_checks,
        "configuration": config,
        "planner_mode": args.planner_mode,
        "scenarios": results,
        "conclusion": {
            "full_triggered": any(
                item["stage5"]["full_replan_count"] > 0 for item in results
            ),
            "all_streams_clean": all(
                item["stream"]["underrun_count"] == 0
                and item["stream"]["stale_count"] == 0
                and item["stream"]["sequence_gap_count"] == 0
                for item in results
            ),
        },
    }
    stage5_v15.write_json(result_path, _json_safe(final))
    print(f"RESULT {result_path}", flush=True)


if __name__ == "__main__":
    main()
