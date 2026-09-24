"""Timestamp-aligned Pi radial KF; fair clean/noisy closed-loop comparisons."""

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
for path in (ROOT, ROOT/"scripts"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import run_stage3b_yaw_control as stage3b
import run_stage5_v1_5_pi_mcu_stream as stage5
import stage6_synthetic_support as stage6_support
import run_stage6_2_safety_decoupling as stage6_2
import run_stage6_4_2d_follow as stage6_4
from control.follow_governor import FollowGovernor, FollowGovernorConfig
from control.pi_mcu_packets import (PROTOCOL_VERSION, TargetObservationPacket,
    YawCommandPacket)
from control.radial_velocity_kf import (PiRadialObservationPipeline,
    PiRobotStateHistory, RadialVelocityKFConfig, RadialVelocityKalmanFilter)
from control.rolling_reference import TargetObservation
from control.safety_brake import SafetyBrakeConfig
from control.stage6_pi_mcu_link import (InMemoryPacketTransport,
    MCUSafetyArbiter, MCUReferenceBlockReceiver, PiReferenceBlockProducer)
from control.stage6_yaw_servo import BearingYawServo, MCULatestYawSource
from control.trajectory_feedforward import SparseProjectionFactorizationCache
from sim.slope_estimation import analytic_theta_eq, equilibrium_sum_torque_nm

STAGE_DIR = ROOT/"models"/"minisegway"/"stage6"
CONFIG_PATH = STAGE_DIR/"config"/"stage6_5_radial_kf_config.json"
RESULT_DIR = STAGE_DIR/"results"/"radial_kf_packet_alignment"


class NoisyTwoDTargetSensor(stage6_4.TwoDTargetSensor):
    def __init__(self, scenario, frequency_hz, *, noise_std_m, seed):
        super().__init__(scenario, frequency_hz)
        self.noise_std_m = float(noise_std_m)
        self.rng = np.random.default_rng(int(seed))

    def _capture(self, sim):
        super()._capture(sim)
        if self.noise_std_m <= 0.0:
            return
        old = self.latest
        noise = self.rng.normal(0.0, self.noise_std_m, size=2)
        self.latest = TargetObservation(old.capture_time_s,
            old.x_forward_m+float(noise[0]), old.y_left_m+float(noise[1]),
            version=old.version, sequence_id=old.sequence_id,
            valid=old.valid, confidence=old.confidence)


class KFPiLink(stage6_4.TwoDLink):
    """Simulation camera bridge sends bytes; Pi consumes only decoded packets."""

    def __init__(self, *, camera_reader, pipeline, raw_governor=None,
                 processing_delay_s, jitter_s, **kwargs):
        super().__init__(reader=pipeline, **kwargs)
        self.camera_reader = camera_reader
        self.pipeline = pipeline
        self.raw_governor = raw_governor
        self.processing_delay_s = float(processing_delay_s)
        self.jitter_s = float(jitter_s)
        self.last_camera_sequence_id = -1
        self.camera_bridge_events: list[dict] = []

    def _send_yaw(self, time_s: float) -> None:
        observation = self.camera_reader.read()
        if observation.sequence_id <= self.last_camera_sequence_id:
            return
        self.last_camera_sequence_id = observation.sequence_id
        packet = TargetObservationPacket(
            PROTOCOL_VERSION, observation.sequence_id,
            observation.capture_time_s, observation.x_forward_m,
            observation.y_left_m, observation.valid, observation.confidence)
        pattern = (0.0, 1.0, -0.5, 0.5)
        jitter = self.jitter_s*pattern[observation.sequence_id % len(pattern)]
        startup = observation.sequence_id == 0
        self.transport.send("pi", "observation", packet.to_bytes(), time_s,
            not_before_s=time_s+self.processing_delay_s+jitter,
            startup_prefill=startup)
        self.camera_bridge_events.append({"sequence_id": packet.sequence_id,
            "capture_time_s": packet.capture_time_s,
            "bridge_time_s": time_s,
            "planned_processing_delay_s": 0.0 if startup else
            self.processing_delay_s+jitter})

    def on_robot_state_packet(self, packet):
        self.pipeline.state_history.accept(packet)

    def on_pi_packet(self, channel: str, payload: bytes, time_s: float) -> None:
        if channel != "observation":
            raise ValueError(f"unknown Pi channel {channel}")
        packet = TargetObservationPacket.from_bytes(payload)
        observation = self.pipeline.accept_observation(packet, time_s)
        if observation is None:
            return
        self.last_observation = observation
        before = len(self.governor.events)
        intent = self.governor(observation)
        row = self.pipeline.rows[-1]
        row["governor_output_m_s"] = intent.linear_velocity_target_m_s
        row["governor_event"] = (
            self.governor.events[-1]["event"]
            if len(self.governor.events) > before else "NONE")
        if self.raw_governor is not None:
            self.raw_governor.set_robot_velocity_hat(row["aligned_v_robot_m_s"])
            raw_before = len(self.raw_governor.events)
            raw_intent = self.raw_governor(observation)
            row["raw_shadow_governor_output_m_s"] = (
                raw_intent.linear_velocity_target_m_s
            )
            row["raw_shadow_governor_event"] = (
                self.raw_governor.events[-1]["event"]
                if len(self.raw_governor.events) > raw_before else "NONE")
        beta, rate = self.servo.command(observation.x_forward_m,
                                        observation.y_left_m)
        yaw = YawCommandPacket(PROTOCOL_VERSION, self.yaw_sequence_id,
                               observation.capture_time_s, rate)
        self.yaw_sequence_id += 1
        self.transport.send("mcu", "yaw", yaw.to_bytes(), time_s)
        self.yaw_generation_events.append({"event": "yaw_generated",
            "time_s": time_s, "source_time_s": observation.capture_time_s,
            "sequence_id": yaw.sequence_id, "beta_rad": beta,
            "yaw_rate_target_rad_s": rate})


def _plot(path, rows, truth, d1, d2):
    accepted = [r for r in rows if r["alignment"] in ("EXACT", "INTERPOLATED")]
    t = [r["capture_time_s"] for r in accepted]
    gt = [truth[r["sequence_id"]]["master_radial_velocity_GT_m_s"]
          for r in accepted]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axes[0].plot(t, [r["d_meas_m"] for r in accepted], ".", alpha=0.4,
                 label="d measured")
    axes[0].plot(t, [r["kf_d_hat_m"] for r in accepted], label="d KF")
    axes[0].axhline(d1, ls=":", color="r", label="d1")
    axes[0].axhline(d2, ls=":", color="g", label="d2")
    axes[0].legend(ncol=4)
    axes[1].plot(t, gt, label="Master radial GT (post-hoc)")
    axes[1].plot(t, [r["kf_v_master_radial_hat_m_s"] for r in accepted],
                 label="KF")
    axes[1].plot(t, [r["raw_radial_velocity_for_diagnostic_only_m_s"]
                     for r in accepted], alpha=0.45, label="raw diagnostic")
    axes[1].set_ylabel("radial velocity [m/s]")
    axes[1].legend(ncol=3)
    axes[2].step(t, [r["governor_output_m_s"] for r in accepted],
                 where="post", label="KF Governor v_cmd")
    axes[2].step(t, [r["raw_shadow_governor_output_m_s"] for r in accepted],
                 where="post", alpha=0.5, label="raw shadow v_cmd")
    axes[2].set_ylabel("v_cmd [m/s]")
    axes[2].set_xlabel("capture time [s]")
    axes[2].legend()
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def _metrics(rows, truth, history, governor, raw_governor, receiver,
             state_history, link, run, stop_time_s=None):
    accepted = [r for r in rows if r["alignment"] in ("EXACT", "INTERPOLATED")]
    def error_stats(key):
        errors = [r[key]-truth[r["sequence_id"]]["master_radial_velocity_GT_m_s"]
                  for r in accepted if r[key] is not None]
        return {"samples": len(errors),
                "rmse_m_s": math.sqrt(sum(e*e for e in errors)/len(errors)),
                "peak_abs_error_m_s": max(abs(e) for e in errors)}
    def stop_delay(key):
        if stop_time_s is None:
            return None
        for index, row in enumerate(accepted):
            if not stop_time_s <= row["capture_time_s"] <= stop_time_s+3.9:
                continue
            group = accepted[index:index+3]
            if len(group) == 3 and all(r[key] is not None
                and abs(r[key]) <= 0.05 for r in group):
                return row["capture_time_s"]-stop_time_s
        return None
    alignment_miss = [r for r in rows if r["alignment"] == "STATE_HISTORY_MISS"]
    zero_command = (next((e for e in governor.events
        if stop_time_s is not None and stop_time_s <= e["time_s"] < stop_time_s+4.0
        and e["latched_velocity_m_s"] <= 0.05), None))
    zero_full = (next((e for e in link.producer.source.replan_events
        if stop_time_s is not None and e["planning_path"] == "FULL_DYNAMIC"
        and stop_time_s <= e["execution_start_time_s"] < stop_time_s+4.0
        and e["target_velocity_m_s"] <= 0.05), None))
    return {"kf_velocity": error_stats("kf_v_master_radial_hat_m_s"),
        "raw_velocity": error_stats("raw_radial_velocity_for_diagnostic_only_m_s"),
        "stop_response_delay_s": {
            "kf": stop_delay("kf_v_master_radial_hat_m_s"),
            "raw": stop_delay("raw_radial_velocity_for_diagnostic_only_m_s"),
            "governor_zero_command_capture_time": (None if zero_command is None
                else zero_command["time_s"]-stop_time_s),
            "full_zero_plan_execution_time": (None if zero_full is None
                else zero_full["execution_start_time_s"]-stop_time_s)},
        "governor_event_count": len(governor.events),
        "raw_shadow_governor_event_count": len(raw_governor.events),
        "governor_events": governor.events,
        "raw_shadow_governor_events": raw_governor.events,
        "alignment": {"accepted": len(accepted),
            "interpolated": sum(r["alignment"] == "INTERPOLATED" for r in rows),
            "history_miss": len(alignment_miss),
            "out_of_order": sum(r["alignment"] == "OUT_OF_ORDER" for r in rows),
            "max_packet_age_at_arrival_s": max(
                r["packet_arrival_time_s"]-r["capture_time_s"] for r in rows),
            "max_abs_aligned_time_error_s": max(abs(
                r["aligned_robot_state_time_s"]-r["capture_time_s"])
                for r in accepted),
            "max_abs_aligned_vs_latest_robot_v_m_s": max(abs(
                r["aligned_v_robot_m_s"]
                -r["latest_packet_v_robot_for_diagnostic_only_m_s"])
                for r in accepted),
            "state_packet_count": state_history.accepted_count,
            "state_buffer_size_at_end": len(state_history.samples),
            "state_packet_rejections": len(state_history.events)},
        "simulation": {"fell": run["longitudinal"]["fell"],
            "finite": run["finite"],
            "wheel_saturation_fraction": run["torque_allocation"]["per_wheel_saturation_fraction"],
            "sum_command_saturated_control_samples": sum(
                bool(r["sum_command_saturated"]) for r in history),
            "reference_underrun_count": receiver.unhandled_underrun_count,
            "GT_runtime_dependency": run["GT_runtime_dependency"],
            "min_v_cmd_m_s": min(r["latched_v_cmd_m_s"]
                                  for r in governor.observation_history)}}


def run_case(config, stage6_config, stage5_config, common,
             stage4_config, stage4c_config, case):
    scenarios = {s["name"]: s for s in stage6_config["scenarios"]}
    scenarios["stop_restart_turning"] = config["stop_restart_scenario"]
    scenario = scenarios[case["scenario"]]
    sensor = NoisyTwoDTargetSensor(scenario,
        stage6_config["observation_frequency_hz"],
        noise_std_m=case["noise_std_m"], seed=config["noise_seed"])
    camera_reader = stage6_2.TargetObservationWireReader(sensor)
    state_history = PiRobotStateHistory(
        retention_s=config["state_history_retention_s"])
    pipeline = PiRadialObservationPipeline(
        RadialVelocityKalmanFilter(RadialVelocityKFConfig(**config["kf"])),
        state_history)
    gov_config = FollowGovernorConfig(**stage6_config["governor"])
    governor = FollowGovernor(gov_config,
        radial_estimate_provider=pipeline.estimate_for)
    raw_governor = FollowGovernor(gov_config)
    cache = SparseProjectionFactorizationCache()
    planner = stage5.make_full_planner(stage5_config, common,
        planning_dt_s=float(stage5_config["full_dynamic"]["planning_dt_s"]),
        backend="cached_kkt", cache=cache)
    source = stage5.make_reference_source(stage5_config, common, pipeline,
        follower=governor, full_planner_override=planner,
        scheduler_gate_overrides=stage6_config["follow_scheduler_gate_policy"])
    producer = PiReferenceBlockProducer(source)
    receiver = MCUReferenceBlockReceiver(dt_s=producer.dt_s,
        block_samples=producer.block_samples)
    transport = InMemoryPacketTransport(
        stage6_config["safety"]["transport_latency_s"])
    yaw_config = stage6_config["yaw"]
    yaw_source = MCULatestYawSource(dt_s=producer.dt_s,
        timeout_s=yaw_config["freshness_timeout_s"],
        decay_rate_rad_s2=yaw_config["decay_rate_rad_s2"])
    safety = stage6_config["safety"]
    arbiter = MCUSafetyArbiter(receiver, SafetyBrakeConfig(
        safety["safe_max_deceleration_m_s2"], safety["safe_max_jerk_m_s3"],
        safety["hidden_reference_handoff_time_s"], producer.dt_s),
        heartbeat_timeout_s=safety["heartbeat_timeout_s"],
        starvation_margin_s=safety["reference_starvation_margin_s"],
        nominal_pitch_rad=float(common[7]["parameters"]["theta_eq_rad"]),
        status_sink=lambda payload, time_s: transport.send(
            "pi", "safety_status", payload, time_s), yaw_source=yaw_source)
    servo = BearingYawServo(gain_s_inv=yaw_config["gain_s_inv"],
        max_rate_rad_s=yaw_config["max_rate_rad_s"],
        deadband_rad=yaw_config["deadband_rad"])
    link = KFPiLink(transport=transport, producer=producer, governor=governor,
        arbiter=arbiter, crash_time_s=None, hazard_time_s=None,
        camera_reader=camera_reader, pipeline=pipeline,
        raw_governor=raw_governor,
        processing_delay_s=case["processing_delay_s"],
        jitter_s=case["jitter_s"], servo=servo, scenario=scenario,
        stage5_config=stage5_config, common=common, full_planner=planner,
        governor_config=gov_config,
        scheduler_policy=stage6_config["follow_scheduler_gate_policy"])
    q_adapter, runtime = stage5.make_runtime(
        common, stage5_config, stage4_config, stage4c_config)
    manifest = common[0]
    run = stage3b.run_case({"name": case["name"],
        "duration_s": scenario["duration_s"],
        "linear_velocity_schedule": [{"time_s": 0.0, "command": 0.0}],
        "yaw_rate_schedule": [{"time_s": 0.0, "command": 0.0}]},
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
            "reference_epoch": receiver.reference_epoch,
            "pi_latest_observation_sequence_id": pipeline.last_sequence_id,
        },
        equilibrium_reference_callback=lambda context: analytic_theta_eq(
            runtime.alpha_control_rad, runtime.plant),
        equilibrium_input_callback=lambda context: equilibrium_sum_torque_nm(
            runtime.alpha_control_rad,
            float(context["estimate"].velocity_hat_m_s),
            float(context["estimate"].theta_dot_hat_rad_s), runtime.plant),
        control_observer=runtime, reference_source=arbiter,
        mcu_local_state_callback=link.mcu_local_state,
        pre_reference_tick_callback=link.before_control_tick)
    truth = {int(r["sequence_id"]): r for r in sensor.evaluation_history}
    name = case["name"]
    stage6_support.write_csv(RESULT_DIR/f"{name}_estimator.csv", pipeline.rows)
    stage6_support.write_csv(RESULT_DIR/f"{name}_history.csv", run["history_50hz"])
    stage6_support.write_csv(RESULT_DIR/f"{name}_governor.csv", governor.observation_history)
    stage6_support.write_csv(RESULT_DIR/f"{name}_raw_shadow_governor.csv",
                        raw_governor.observation_history)
    stage6_support.write_csv(RESULT_DIR/f"{name}_camera_bridge.csv",
                        link.camera_bridge_events)
    stage6_support.write_csv(RESULT_DIR/f"{name}_transport.csv", transport.events)
    stage6_support.write_csv(RESULT_DIR/f"{name}_reference_plans.csv", source.replan_events)
    figure = RESULT_DIR/"plots"/f"{name}.png"
    _plot(figure, pipeline.rows, truth, gov_config.d1_m, gov_config.d2_m)
    result = _metrics(pipeline.rows, truth, run["history_50hz"], governor,
        raw_governor, receiver, state_history, link, run,
        stop_time_s=(8.0 if case["scenario"] == "stop_restart_turning" else None))
    window_start, window_end = ((8.5, 11.5)
        if case["scenario"] == "stop_restart_turning" else (4.0, 7.0))
    window_rows = [r for r in pipeline.rows
        if r["alignment"] in ("EXACT", "INTERPOLATED")
        and window_start <= r["capture_time_s"] <= window_end
        and r["raw_radial_velocity_for_diagnostic_only_m_s"] is not None]
    def window_rmse(key):
        errors = [r[key]-truth[r["sequence_id"]]["master_radial_velocity_GT_m_s"]
                  for r in window_rows]
        return math.sqrt(sum(x*x for x in errors)/len(errors))
    turning_rows = [r for r in run["history_50hz"]
        if window_start <= r["t"] <= window_end]
    result["turning_window"] = {
        "window_capture_s": [window_start, window_end],
        "master_mode": ("stationary" if case["scenario"] == "stop_restart_turning"
                        else "constant_speed"),
        "sample_count": len(window_rows),
        "kf_radial_velocity_rmse_m_s": window_rmse("kf_v_master_radial_hat_m_s"),
        "raw_radial_velocity_rmse_m_s": window_rmse(
            "raw_radial_velocity_for_diagnostic_only_m_s"),
        "max_abs_robot_yaw_rate_hat_rad_s": max(abs(r["r_hat_rad_s"])
            for r in turning_rows),
    }
    result.update({"case": case, "figure": figure.relative_to(ROOT).as_posix(),
                   "state_packets_sent": link.robot_state_sequence_id,
                   "observation_packets_sent": len(link.camera_bridge_events),
                   "yaw_packets_sent": link.yaw_sequence_id,
                   "reference_blocks_sent": producer.next_sequence_id})
    return stage6_support.json_safe(result)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case")
    args = parser.parse_args()
    config = stage5.load_json(CONFIG_PATH)
    stage6_config = stage5.load_json(stage6_4.CONFIG_PATH)
    stage5_config = stage5.load_json(stage5.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen = stage5.load_common(stage5_config)
    if not all(frozen.values()):
        raise RuntimeError(f"frozen baseline check failed: {frozen}")
    cases = config["cases"]
    if args.case:
        cases = [case for case in cases if case["name"] == args.case]
        if not cases:
            raise ValueError("unknown case")
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    results = []
    for case in cases:
        print(f"RUN {case['name']}", flush=True)
        results.append(run_case(config, stage6_config, stage5_config, common,
            stage4_config, stage4c_config, case))
        (RESULT_DIR/"stage6_5_results.json").write_text(json.dumps(
            {"stage": config["stage"], "status": "IN_PROGRESS",
             "frozen_checks": frozen, "cases": results},
            indent=2, ensure_ascii=False), encoding="utf-8")
    by_name = {row["case"]["name"]: row for row in results}
    for noisy_name, clean_name in (("noisy_turning", "noise_free_regression"),
                                   ("stop_restart_noisy", "stop_restart_clean")):
        if noisy_name not in by_name or clean_name not in by_name:
            continue
        noisy, clean = by_name[noisy_name], by_name[clean_name]
        noisy["decision_noise_comparison"] = {
            "clean_case": clean_name,
            "kf_excess_events_vs_clean": max(0,
                noisy["governor_event_count"]-clean["governor_event_count"]),
            "raw_shadow_excess_events_vs_clean": max(0,
                noisy["raw_shadow_governor_event_count"]
                -clean["raw_shadow_governor_event_count"]),
            "interpretation": "event-count proxy; changed closed-loop trajectory may also shift events",
        }
    (RESULT_DIR/"stage6_5_results.json").write_text(json.dumps(
        {"stage": config["stage"], "status": "COMPLETED",
         "frozen_checks": frozen, "cases": results},
        indent=2, ensure_ascii=False), encoding="utf-8")


if __name__ == "__main__":
    main()
