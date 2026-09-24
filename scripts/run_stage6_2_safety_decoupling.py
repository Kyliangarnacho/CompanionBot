"""Four Stage 6 Pi/MCU packet, pending, and local safety stress cases."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT, ROOT / "scripts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import run_stage3b_yaw_control as stage3b  # noqa: E402
import run_stage5_v1_5_pi_mcu_stream as stage5  # noqa: E402
import stage6_synthetic_support as stage6_support  # noqa: E402
from control.follow_governor import FollowGovernor, FollowGovernorConfig  # noqa: E402
from control.pi_mcu_packets import (  # noqa: E402
    PROTOCOL_VERSION, TargetObservationPacket,
)
from control.rolling_reference import TargetObservation  # noqa: E402
from control.safety_brake import SafetyBrakeConfig  # noqa: E402
from control.stage6_pi_mcu_link import (  # noqa: E402
    InMemoryPacketTransport, MCUSafetyArbiter, MCUReferenceBlockReceiver,
    PiReferenceBlockProducer, Stage6PacketLink,
)
from control.trajectory_feedforward import SparseProjectionFactorizationCache  # noqa: E402
from sim.slope_estimation import (  # noqa: E402
    analytic_theta_eq, equilibrium_sum_torque_nm,
)


STAGE_DIR = ROOT / "models" / "minisegway" / "stage6"
CONFIG_PATH = STAGE_DIR / "config" / "stage6_2_safety_decoupling_config.json"
RESULT_DIR = STAGE_DIR / "results" / "safety_pi_mcu_decoupling"
RESULT_PATH = RESULT_DIR / "stage6_2_stress_results.json"


class TargetObservationWireReader:
    """Perception-to-Pi conversion has exactly the declared packet fields."""

    def __init__(self, sensor: stage6_support.SyntheticTargetSensor) -> None:
        self.sensor = sensor
        self.packet_count = 0

    def read(self) -> TargetObservation:
        source = self.sensor.read()
        packet = TargetObservationPacket(
            PROTOCOL_VERSION, source.sequence_id, source.capture_time_s,
            source.x_forward_m, source.y_left_m, source.valid, source.confidence,
        )
        decoded = TargetObservationPacket.from_bytes(packet.to_bytes())
        self.packet_count += 1
        return TargetObservation(
            decoded.capture_time_s, decoded.x_forward_m, decoded.y_left_m,
            version=decoded.protocol_version, sequence_id=decoded.sequence_id,
            valid=decoded.valid, confidence=decoded.confidence,
        )


def _plot_response(path: Path, scenario: dict, history: list[dict],
                   governor: FollowGovernor, source, arbiter: MCUSafetyArbiter,
                   d1_m: float, d2_m: float,
                   *, show_master_gt: bool = False) -> None:
    is_safety = "hazard_time_s" in scenario or "pi_crash_time_s" in scenario
    fig, axes = plt.subplots(4 if is_safety else 3, 1,
                             figsize=(12, 9 if is_safety else 8), sharex=True)
    time_s = np.asarray([float(row["t"]) for row in history])
    observations = governor.observation_history
    obs_time = [float(row["capture_time_s"]) for row in observations]

    axes[0].plot(time_s, [row["target_observation_x_forward_m"] for row in history],
                 label="relative distance", color="tab:blue")
    axes[0].axhline(d1_m, linestyle=":", color="tab:red", label="d1")
    axes[0].axhline(d2_m, linestyle=":", color="tab:green", label="d2")
    axes[0].plot(obs_time, [row["switch_distance_m"] for row in observations],
                 "--", color="tab:orange", label="dynamic switch")
    axes[0].set_ylabel("distance [m]")
    axes[0].legend(ncol=4, fontsize=8)

    axes[1].plot(obs_time, [row["master_velocity_hat_m_s"] for row in observations],
                 label="Master v estimated", color="tab:purple")
    if show_master_gt:
        axes[1].step(time_s, [row["master_velocity_GT_m_s"] for row in history],
                     where="post", label="Master v GT (post-hoc)",
                     color="tab:gray", linestyle="--")
    axes[1].step(obs_time, [row["latched_v_cmd_m_s"] for row in observations],
                 where="post", label="latched v_cmd", color="tab:red")
    intents = source.intent_history
    pending = [row for row in intents if row["full_locked"]
               and abs(float(row["latest_pending_raw_v_cmd_m_s"])
                       - float(row["accepted_linear_velocity_target_m_s"])) > 1e-12]
    if pending:
        axes[1].step([row["source_time_s"] for row in pending],
                     [row["latest_pending_raw_v_cmd_m_s"] for row in pending],
                     where="post", linestyle=":", color="tab:brown",
                     label="latest pending v_cmd")
    else:
        axes[1].plot([], [], linestyle=":", color="tab:brown",
                     label="pending v_cmd (none)")
    axes[1].plot(time_s, [row["rolling_v_ref_applied_m_s"] for row in history],
                 label="applied v_ref", color="tab:orange")
    axes[1].plot(time_s, [row["v_hat_m_s"] for row in history],
                 label="robot v_hat", color="tab:blue")
    axes[1].set_ylabel("velocity [m/s]")
    axes[1].legend(ncol=5, fontsize=8)

    axes[2].step(obs_time,
                 [int(row["governor_state"] == "CATCH_UP") for row in observations],
                 where="post", label="Governor catch=1")
    full_active = [
        int(any(float(plan["time_s"]) <= t
                and (plan["full_exit_time_s"] is None
                     or t < float(plan["full_exit_time_s"]))
                for plan in source.replan_events))
        for t in time_s
    ]
    axes[2].step(time_s, full_active, where="post", label="Pi FULL lifecycle=1")
    axes[2].step([row["source_time_s"] for row in intents],
                 [int(row["full_locked"] and abs(
                     float(row["latest_pending_raw_v_cmd_m_s"])
                     - float(row["accepted_linear_velocity_target_m_s"])) > 1e-12)
                  for row in intents], where="post", label="pending=1")
    axes[2].step(time_s,
                 [{"NORMAL": 0, "BRAKING": 1, "SAFE_HOLD": 2}[row["mcu_safety_state"]]
                  for row in history], where="post", label="MCU safety: 0/1/2")
    axes[2].set(ylabel="state", yticks=[0, 1, 2])
    axes[2].legend(ncol=4, fontsize=8)

    if is_safety:
        axes[3].plot(time_s,
                     [row["rolling_theta_ref_applied_rad"] for row in history],
                     label="theta_ref [rad]")
        axes[3].plot(time_s,
                     [row["rolling_u_ff_after_lifecycle_nm"] for row in history],
                     label="u_ff wire [Nm]")
        axes[3].set_ylabel("hidden reference")
        axes[3].legend(fontsize=8)
    for event in governor.events:
        axes[0].axvline(float(event["time_s"]), color="0.35", alpha=0.22)
        if event["event"] == "slowdown_wait":
            axes[2].axvline(float(event["time_s"]), color="tab:purple",
                            linestyle=":", alpha=0.6)
    for event in source.replan_events:
        axes[1].axvline(float(event["time_s"]), color="0.35",
                        linestyle=":", alpha=0.30)
    for event in arbiter.events:
        if event["event"] == "safety_takeover":
            for axis in axes:
                axis.axvline(float(event["time_s"]), color="tab:red",
                            linestyle="--", alpha=0.6)
    axes[-1].set_xlabel("time [s]")
    for axis in axes:
        axis.grid(True, alpha=0.22)
    fig.suptitle(scenario["name"])
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)


def run_scenario(config: dict, stage5_config: dict, common: tuple,
                 stage4_config: dict, stage4c_config: dict,
                 scenario: dict, *, result_dir: Path | None = None,
                 show_master_gt: bool = False) -> dict:
    output_dir = RESULT_DIR if result_dir is None else result_dir
    governor_config = FollowGovernorConfig(**config["governor"])
    motion_limits = common[3]["reference_limits"]
    dynamic_limits = common[2]
    if governor_config.a_brake_effective_m_s2 > float(
        motion_limits["max_acceleration_m_s2"]
    ):
        raise ValueError("governor brake effectiveness exceeds frozen acceleration")
    safety_config = config["safety"]
    brake_config = SafetyBrakeConfig(
        float(safety_config["safe_max_deceleration_m_s2"]),
        float(safety_config["safe_max_jerk_m_s3"]),
        float(safety_config["hidden_reference_handoff_time_s"]),
        float(common[3]["controller_dt_s"]),
    )
    if (brake_config.safe_max_deceleration_m_s2
            > float(motion_limits["max_acceleration_m_s2"])
            or brake_config.safe_max_jerk_m_s3
            > float(dynamic_limits["max_jerk_m_s3"])):
        raise ValueError("local safety brake exceeds frozen reference envelope")

    sensor = stage6_support.SyntheticTargetSensor(
        scenario, float(config["observation_frequency_hz"])
    )
    reader = TargetObservationWireReader(sensor)
    governor = FollowGovernor(governor_config)
    cache = SparseProjectionFactorizationCache()
    full_planner = stage5.make_full_planner(
        stage5_config, common,
        planning_dt_s=float(stage5_config["full_dynamic"]["planning_dt_s"]),
        backend="cached_kkt", cache=cache,
    )
    source = stage5.make_reference_source(
        stage5_config, common, reader, follower=governor,
        full_planner_override=full_planner,
        scheduler_gate_overrides=config["follow_scheduler_gate_policy"],
    )
    producer = PiReferenceBlockProducer(source)
    receiver = MCUReferenceBlockReceiver(
        dt_s=producer.dt_s, block_samples=producer.block_samples,
    )
    transport = InMemoryPacketTransport(
        latency_s=float(safety_config["transport_latency_s"])
    )
    nominal_pitch = float(common[7]["parameters"]["theta_eq_rad"])
    arbiter = MCUSafetyArbiter(
        receiver, brake_config,
        heartbeat_timeout_s=float(safety_config["heartbeat_timeout_s"]),
        starvation_margin_s=float(safety_config["reference_starvation_margin_s"]),
        nominal_pitch_rad=nominal_pitch,
        status_sink=lambda payload, time_s: transport.send(
            "pi", "safety_status", payload, time_s
        ),
    )
    link = Stage6PacketLink(
        transport=transport, producer=producer, governor=governor,
        arbiter=arbiter, crash_time_s=scenario.get("pi_crash_time_s"),
        hazard_time_s=scenario.get("hazard_time_s"),
        hazard_reason=scenario.get("hazard_reason", "OBSTACLE"),
    )
    q_adapter, runtime = stage5.make_runtime(
        common, stage5_config, stage4_config, stage4c_config
    )
    manifest = common[0]
    run = stage3b.run_case(
        {
            "name": scenario["name"], "duration_s": scenario["duration_s"],
            "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
            "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}],
        },
        float(manifest["yaw"]["K_psi_nm_per_rad"]),
        float(manifest["yaw"]["K_r_nm_per_rad_s"]),
        yaw_enabled=True, motor_mismatch_enabled=False,
        common=common, keep_history=True, payload_mode="empty",
        common_mode_augmentation=q_adapter,
        physics_step_callback=sensor.on_physics_step,
        simulation_setup_callback=sensor.setup,
        history_diagnostic_callback=lambda sim: {
            **sensor.diagnostic(sim),
            "mcu_safety_state": arbiter.state,
            "mcu_safety_reason": arbiter.reason,
            "reference_epoch": receiver.reference_epoch,
            "pi_safety_state": link.pi_safety_state,
            "mcu_next_ready": receiver.next_ready(),
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
    )

    history = run["history_50hz"]
    governor_rows = governor.observation_history
    obs_index = 0
    latest = None
    for row in history:
        sequence = int(row["target_observation_sequence_id"])
        while obs_index < len(governor_rows) and int(
            governor_rows[obs_index]["sequence_id"]
        ) <= sequence:
            latest = governor_rows[obs_index]
            obs_index += 1
        if latest is not None:
            row.update({
                "governor_state": latest["governor_state"],
                "governor_latched_v_cmd_m_s": latest["latched_v_cmd_m_s"],
                "master_velocity_hat_m_s": latest["master_velocity_hat_m_s"],
                "switch_distance_m": latest["switch_distance_m"],
            })

    name = str(scenario["name"])
    stage6_support.write_csv(output_dir / f"{name}_history.csv", history)
    stage6_support.write_csv(output_dir / f"{name}_governor_observations.csv",
                        governor_rows)
    stage6_support.write_csv(output_dir / f"{name}_governor_events.csv", governor.events)
    stage6_support.write_csv(output_dir / f"{name}_pi_intents.csv", source.intent_history)
    stage6_support.write_csv(output_dir / f"{name}_transport_events.csv", transport.events)
    stage6_support.write_csv(output_dir / f"{name}_mcu_events.csv",
                        arbiter.events + receiver.events)
    plot_path = output_dir / "plots" / f"{name}.png"
    _plot_response(plot_path, scenario, history, governor, source, arbiter,
                   governor_config.d1_m, governor_config.d2_m,
                   show_master_gt=show_master_gt)

    pending_rows = [row for row in source.intent_history if row["full_locked"]
                    and abs(float(row["latest_pending_raw_v_cmd_m_s"])
                            - float(row["accepted_linear_velocity_target_m_s"])) > 1e-12]
    pending_changes = [row for index, row in enumerate(pending_rows)
                       if index == 0 or float(row["latest_pending_raw_v_cmd_m_s"])
                       != float(pending_rows[index - 1]["latest_pending_raw_v_cmd_m_s"])]
    full_events = [event for event in source.replan_events
                   if event["planning_path"] == "FULL_DYNAMIC"]
    pending_resolution = []
    for index, row in enumerate(pending_changes):
        issued_time = float(row["source_time_s"])
        target = float(row["latest_pending_raw_v_cmd_m_s"])
        next_update_time = (
            float(pending_changes[index + 1]["source_time_s"])
            if index + 1 < len(pending_changes) else math.inf
        )
        matching = next((
            event for event in full_events
            if issued_time < float(event["time_s"]) <= next_update_time
            and math.isclose(float(event["target_velocity_m_s"]), target,
                             abs_tol=1e-9)
        ), None)
        pending_resolution.append({
            "pending_time_s": issued_time,
            "pending_target_m_s": target,
            "resolution": ("executed" if matching is not None else
                           "superseded" if math.isfinite(next_update_time)
                           else "unresolved"),
            "full_start_time_s": (None if matching is None
                                  else float(matching["time_s"])),
            "wait_to_full_s": (None if matching is None else
                               float(matching["time_s"]) - issued_time),
        })
    takeovers = [event for event in arbiter.events
                 if event["event"] == "safety_takeover"]
    hold_events = [event for event in arbiter.events
                   if event["event"] == "safe_hold"]
    accepted = [event for event in source.scheduler.events
                if event["event"] == "accepted"]
    result = {
        "scenario": scenario,
        "simulation": {
            "actual_duration_s": run["simulation_termination"]["actual_duration_s"],
            "terminated_early": run["simulation_termination"]["terminated_early"],
            "fell": run["longitudinal"]["fell"],
            "finite": run["finite"],
            "wheel_saturation_fraction": run["torque_allocation"]["per_wheel_saturation_fraction"],
            "sum_command_saturated_samples": sum(
                bool(row["sum_command_saturated"]) for row in history
            ),
            "false_slope_samples": sum(
                row.get("environment_mode") == "SLOPE" for row in history
            ),
            "min_relative_distance_m": min(
                float(row["target_observation_x_forward_m"]) for row in history
            ),
            "max_relative_distance_m": max(
                float(row["target_observation_x_forward_m"]) for row in history
            ),
            "final_relative_distance_m": (
                None if sensor.latest is None else sensor.latest.x_forward_m
            ),
        },
        "normal_follow": {
            "governor_events": governor.events,
            "retrigger_count": sum(event["event"] == "catch_up_retriggered"
                                   for event in governor.events),
            "accepted_commands": accepted,
            "full_plans": full_events,
            "full_exit_events": source.full_exit_events,
            "pending_target_changes": pending_changes,
            "pending_resolution": pending_resolution,
            "pending_sample_count": len(pending_rows),
            "processed_after_full_exit_count": sum(
                bool(event.get("processed_after_full_exit")) for event in accepted
            ),
            "max_abs_latched_v_cmd_m_s": max(
                abs(float(row["latched_v_cmd_m_s"])) for row in governor_rows
            ),
        },
        "safety": {
            "pi_safety_commands": link.packet_events,
            "takeovers": takeovers,
            "takeover_delay_s": (
                None if not takeovers or takeovers[0]["issued_time_s"] is None
                else takeovers[0]["time_s"] - takeovers[0]["issued_time_s"]
            ),
            "first_takeover_deltas": arbiter.first_takeover_deltas,
            "safe_hold_events": hold_events,
            "final_state": arbiter.state,
            "final_reference_epoch": receiver.reference_epoch,
            "unhandled_reference_underrun_count": receiver.unhandled_underrun_count,
            "reference_sequence_gap_count": sum(
                event["event"] == "reference_sequence_gap" for event in receiver.events
            ),
            "stale_reference_packet_count": sum(
                event["event"] == "stale_reference_packet" for event in receiver.events
            ),
            "invalid_epoch_rejection_count": sum(
                event["event"] == "invalid_epoch_rejected" for event in receiver.events
            ),
            "expected_injected_communication_fault": "pi_crash_time_s" in scenario,
        },
        "packets": {
            "target_observation_roundtrip_count": reader.packet_count,
            "robot_state_sent_count": link.robot_state_sequence_id,
            "heartbeat_sent_count": link.heartbeat_sequence_id,
            "safety_status_sent_count": arbiter.status_sequence_id,
            "reference_sent_count": producer.next_sequence_id,
            "transport_dropped_count": sum(
                event["event"] == "packet_dropped" for event in transport.events
            ),
        },
        "baseline_runtime": {
            "GT_runtime_dependency": run["GT_runtime_dependency"],
            "Q_disturbance_rejection_enabled": run["frozen_longitudinal"]["Q_disturbance_rejection_enabled"],
            "physics_dt_s": 0.001,
            "controller_dt_s": producer.dt_s,
        },
        "files": {
            "response_figure": plot_path.relative_to(ROOT).as_posix(),
            "history_csv": (output_dir / f"{name}_history.csv").relative_to(ROOT).as_posix(),
        },
    }
    return stage6_support.json_safe(result)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--scenario", help="run one configured stress case")
    args = parser.parse_args()
    config = stage5.load_json(args.config)
    stage5_config = stage5.load_json(stage5.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen_checks = stage5.load_common(
        stage5_config
    )
    if not all(frozen_checks.values()):
        raise RuntimeError(f"frozen baseline checks failed: {frozen_checks}")
    scenarios = config["scenarios"]
    if args.scenario:
        scenarios = [case for case in scenarios if case["name"] == args.scenario]
        if not scenarios:
            raise ValueError("unknown stress case")
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    results = []
    for scenario in scenarios:
        print(f"RUN {scenario['name']}", flush=True)
        results.append(run_scenario(config, stage5_config, common,
                                    stage4_config, stage4c_config, scenario))
        stage5.write_json(RESULT_PATH, stage6_support.json_safe({
            "stage": config["stage"], "status": "IN_PROGRESS",
            "frozen_baseline_checks": frozen_checks,
            "configuration": config, "scenarios": results,
        }))
    stage5.write_json(RESULT_PATH, stage6_support.json_safe({
        "stage": config["stage"], "status": "COMPLETED",
        "frozen_baseline_checks": frozen_checks,
        "configuration": config, "scenarios": results,
    }))
    print(f"RESULT {RESULT_PATH}", flush=True)


if __name__ == "__main__":
    main()
