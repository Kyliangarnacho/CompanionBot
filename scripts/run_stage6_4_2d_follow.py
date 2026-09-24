"""Three deterministic 2D follow and split-authority closed-loop stress runs."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
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

import run_stage3b_yaw_control as stage3b
import run_stage5_v1_5_pi_mcu_stream as stage5
import stage6_synthetic_support as stage6_support
import run_stage6_2_safety_decoupling as stage6_2
from control.follow_governor import FollowGovernor, FollowGovernorConfig
from control.pi_mcu_packets import PROTOCOL_VERSION, YawCommandPacket
from control.safety_brake import SafetyBrakeConfig
from control.stage6_pi_mcu_link import (InMemoryPacketTransport,
    MCUSafetyArbiter, MCUReferenceBlockReceiver, PiReferenceBlockProducer,
    Stage6PacketLink)
from control.stage6_yaw_servo import BearingYawServo, MCULatestYawSource
from control.trajectory_feedforward import SparseProjectionFactorizationCache
from sim.slope_estimation import analytic_theta_eq, equilibrium_sum_torque_nm

STAGE_DIR = ROOT / "models" / "minisegway" / "stage6"
CONFIG_PATH = STAGE_DIR / "config" / "stage6_4_2d_follow_config.json"
RESULT_DIR = STAGE_DIR / "results" / "2d_follow_split_authority"


def _interpolate(schedule: list[dict], time_s: float) -> float:
    if time_s <= schedule[0]["time_s"]:
        return float(schedule[0]["value_rad"])
    for left, right in zip(schedule, schedule[1:]):
        if time_s <= right["time_s"]:
            alpha = ((time_s-left["time_s"])
                     / (right["time_s"]-left["time_s"]))
            return ((1-alpha)*left["value_rad"]+alpha*right["value_rad"])
    return float(schedule[-1]["value_rad"])


class TwoDTargetSensor(stage6_support.SyntheticTargetSensor):
    """World trajectory stays inside the synthetic sensor; Pi sees only x/y packets."""

    def _capture(self, sim) -> None:
        super()._capture(sim)
        assert self.latest is not None
        from control.rolling_reference import TargetObservation
        # With body forward=-Y, body left=+X under the right-handed +Z yaw sign.
        old = self.latest
        self.latest = TargetObservation(
            old.capture_time_s, old.x_forward_m, -old.y_left_m,
            version=old.version, sequence_id=old.sequence_id,
            valid=old.valid, confidence=old.confidence,
        )
        assert self.latest_evaluation is not None
        self.latest_evaluation["y_left_GT_m"] = self.latest.y_left_m
        self.latest_evaluation["master_x_world_GT_m"] = float(self.master_position_xy_m[0])
        self.latest_evaluation["master_y_world_GT_m"] = float(self.master_position_xy_m[1])
        robot_position, _ = self._robot_pose(sim)
        relative = self.master_position_xy_m-robot_position[:2]
        direction = relative/max(float(np.linalg.norm(relative)), 1e-9)
        speed = stage6_support.schedule_value(self.scenario["master_speed_schedule"],
                                          old.capture_time_s)
        heading = self.master_heading_rad+_interpolate(
            self.scenario["heading_offset_schedule"], old.capture_time_s)
        self.latest_evaluation["master_radial_velocity_GT_m_s"] = float(
            speed*np.dot(direction, np.array([math.cos(heading), math.sin(heading)])))
        self.evaluation_history[-1].update(self.latest_evaluation)

    def on_physics_step(self, sim) -> None:
        assert self.master_position_xy_m is not None
        time_s = float(sim.data.time)
        dt_s = time_s-self.last_physics_time_s
        midpoint = self.last_physics_time_s+0.5*dt_s
        speed = stage6_support.schedule_value(self.scenario["master_speed_schedule"], midpoint)
        angle = self.master_heading_rad+_interpolate(
            self.scenario["heading_offset_schedule"], midpoint)
        self.master_position_xy_m += dt_s*speed*np.array([math.cos(angle), math.sin(angle)])
        self.last_physics_time_s = time_s
        while time_s+1e-12 >= self.next_capture_time_s:
            self._capture(sim)
            self.next_capture_time_s += self.period_s

    def diagnostic(self, sim) -> dict:
        values = super().diagnostic(sim)
        if self.latest is not None:
            values["target_observation_y_left_m"] = self.latest.y_left_m
        return values


class TwoDLink(Stage6PacketLink):
    def __init__(self, *, reader, servo, scenario, stage5_config,
                 common, full_planner, governor_config, scheduler_policy,
                 **kwargs) -> None:
        super().__init__(**kwargs)
        self.reader = reader
        self.servo = servo
        self.scenario = scenario
        self.stage5_config = stage5_config
        self.common = common
        self.full_planner = full_planner
        self.governor_config = governor_config
        self.scheduler_policy = scheduler_policy
        self.yaw_sequence_id = 0
        self.last_observation_sequence_id = -1
        self.yaw_generation_events: list[dict] = []
        self.all_producers = [self.producer]
        self.old_epoch_packet = None
        self.old_epoch_injected = False
        self.resume_sent = False
        self.last_observation = None

    def _send_yaw(self, time_s: float) -> None:
        observation = self.reader.read()
        if observation.sequence_id <= self.last_observation_sequence_id:
            return
        self.last_observation_sequence_id = observation.sequence_id
        self.last_observation = observation
        self.governor(observation)
        if not observation.valid:
            return
        beta, rate = self.servo.command(observation.x_forward_m,
                                        observation.y_left_m)
        if self.scenario.get("yaw_gap_start_s", math.inf) <= time_s < self.scenario.get(
                "yaw_gap_end_s", math.inf):
            self.yaw_generation_events.append({"event": "yaw_generation_paused",
                                               "time_s": time_s,
                                               "source_time_s": observation.capture_time_s})
            return
        packet = YawCommandPacket(PROTOCOL_VERSION, self.yaw_sequence_id,
                                  observation.capture_time_s, rate)
        self.yaw_sequence_id += 1
        self.transport.send("mcu", "yaw", packet.to_bytes(), time_s)
        self.yaw_generation_events.append({"event": "yaw_generated",
            "time_s": time_s, "source_time_s": observation.capture_time_s,
            "sequence_id": packet.sequence_id, "beta_rad": beta,
            "yaw_rate_target_rad_s": rate})

    def _fresh_resume(self, time_s: float) -> None:
        status = self.latest_safety_status
        robot = self.latest_robot_state
        if status is None or robot is None or self.resume_sent:
            return
        if status.state != "LONGITUDINAL_STOP" or robot.safety_state != status.state:
            return
        if robot.sample_time_s < status.timestamp_s:
            return
        if self.scenario["name"] == "turnaround_pass_by":
            if self.governor.distance_m <= self.governor_config.d1_m:
                return
        elif time_s < self.scenario.get("resume_time_s", math.inf):
            return
        source = stage5.make_reference_source(
            self.stage5_config, self.common, self.reader,
            follower=self.governor, full_planner_override=self.full_planner,
            scheduler_gate_overrides=self.scheduler_policy)
        source.lifecycle.reset(float(robot.applied_p_ref_m), 0.0)
        producer = PiReferenceBlockProducer(source,
            reference_epoch=status.reference_epoch)
        block_ticks = producer.block_samples
        start_tick = (math.floor(time_s/producer.dt_s/block_ticks)+2)*block_ticks
        first_block, _ = producer.produce(start_tick, time_s)
        self.request_resume(time_s=time_s, hazard_cleared=True,
            fresh_producer=producer, first_block=first_block,
            authority="LONGITUDINAL")
        self.all_producers.append(producer)
        self.resume_sent = True

    def before_control_tick(self, time_s: float) -> None:
        self._send_yaw(time_s)
        if (self.scenario["name"] == "turnaround_pass_by"
                and not self.hazard_sent and self.last_observation is not None
                and time_s >= 6.0 and self.governor.distance_m
                <= self.governor_config.d2_m
                and self.governor.distance_rate_hat_m_s < 0):
            self.hazard_sent = True
            self.old_epoch_packet = self.arbiter.receiver.active
            self.send_brake(time_s, "MASTER_CLOSURE", "LONGITUDINAL_STOP")
        if (self.scenario["name"] == "asynchronous_safety"
                and self.hazard_time_s is not None and not self.hazard_sent
                and time_s >= self.hazard_time_s-1e-12):
            self.old_epoch_packet = self.arbiter.receiver.active
        super().before_control_tick(time_s)
        if (self.old_epoch_packet is not None and not self.old_epoch_injected
                and self.arbiter.receiver.reference_epoch > 0
                and time_s >= self.scenario.get("old_epoch_inject_s", math.inf)):
            self.transport.send("mcu", "reference",
                                self.old_epoch_packet.to_bytes(), time_s)
            self.old_epoch_injected = True
        if not self.resume_sent:
            self._fresh_resume(time_s)


def _plot(path: Path, scenario: dict, history: list[dict], governor,
          link: TwoDLink) -> None:
    t = np.array([row["t"] for row in history], dtype=float)
    obs = governor.observation_history
    ot = [row["capture_time_s"] for row in obs]
    fig, axes = plt.subplots(5,
                             1, figsize=(12, 11), sharex=True)
    axes[0].plot(ot, [row["distance_m"] for row in obs], label="d")
    axes[0].axhline(governor.config.d1_m, ls=":", c="r", label="d1")
    axes[0].axhline(governor.config.d2_m, ls=":", c="g", label="d2")
    axes[0].legend(ncol=3)
    axes[0].set_ylabel("distance [m]")
    if scenario["name"] == "curved_lateral":
        inset = axes[0].inset_axes([0.67, 0.13, 0.23, 0.51])
        inset.plot([row["master_x_world_GT_m"] for row in history],
                   [row["master_y_world_GT_m"] for row in history],
                   color="tab:purple")
        inset.set_title("Master XY (post-hoc)", fontsize=8)
        inset.set_aspect("equal", adjustable="datalim")
        inset.tick_params(labelsize=6)
    axes[1].plot(ot, [math.degrees(row["beta_rad"]) for row in obs], label="beta [deg]")
    radial_axis = axes[1].twinx()
    radial_axis.plot(ot, [row["master_radial_velocity_hat_m_s"] for row in obs],
                     color="tab:orange", alpha=0.65, label="radial velocity")
    radial_axis.set_ylabel("radial [m/s]")
    axes[1].legend()
    axes[2].step(ot, [row["latched_v_cmd_m_s"] for row in obs], where="post", label="v_cmd")
    axes[2].plot(t, [row["rolling_v_ref_applied_m_s"] for row in history], label="v_ref")
    axes[2].plot(t, [row["v_hat_m_s"] for row in history], label="v_hat")
    axes[2].legend(ncol=3)
    axes[2].set_ylabel("velocity [m/s]")
    axes[3].plot(t, [row["yaw_rate_cmd_rad_s"] for row in history], label="w_ref")
    axes[3].plot(t, [row["r_hat_rad_s"] for row in history], label="w_hat")
    axes[3].legend()
    axes[3].set_ylabel("yaw rate [rad/s]")
    if scenario["name"] == "curved_lateral":
        full_plans = [p for producer in link.all_producers
                      for p in producer.source.replan_events
                      if p["planning_path"] == "FULL_DYNAMIC"]
        axes[4].step(t, [int(any(p["execution_start_time_s"] <= time <
            (p["full_exit_time_s"] or scenario["duration_s"]) for p in full_plans))
            for time in t], where="post", label="FULL active")
        axes[4].legend()
    else:
        code = {"NORMAL": 0, "BRAKING": 1, "LONGITUDINAL_STOP": 2,
                "SAFE_HOLD": 3}
        axes[4].step(t, [code[row["mcu_safety_state"]] for row in history],
                     where="post", label="safety state")
        axes[4].step(t, [row["reference_epoch"] for row in history],
                     where="post", label="epoch")
        axes[4].step(t, [int(row["mcu_yaw_source"] == "LATEST_PI_YAW")
                            for row in history], where="post", label="yaw Pi source")
        axes[4].legend(ncol=3)
        if scenario["name"] == "asynchronous_safety":
            yaw_received = [e["time_s"] for e in link.arbiter.yaw_source.events
                            if e["event"] == "yaw_received"]
            ref_received = [e["time_s"] for e in link.arbiter.receiver.events
                            if e["event"] == "block_received"]
            axes[4].scatter(yaw_received, [4.0]*len(yaw_received), s=2,
                            label="yaw receive")
            axes[4].scatter(ref_received, [4.3]*len(ref_received), s=2,
                            label="reference receive")
            age_axis = axes[4].twinx()
            age_axis.plot(t, [row["yaw_receive_age_s"] for row in history],
                          color="tab:gray", alpha=0.5, label="yaw age")
            age_axis.axhline(link.arbiter.yaw_source.timeout_ticks*link.producer.dt_s,
                             color="tab:red", ls=":", alpha=0.5)
            age_axis.set_ylabel("yaw age [s]")
            axes[4].legend(ncol=4, fontsize=7)
    for ax in axes:
        ax.grid(alpha=0.25)
    axes[-1].set_xlabel("time [s]")
    fig.suptitle(scenario["name"])
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)


def run_scenario(config, stage5_config, common, stage4_config,
                 stage4c_config, scenario):
    sensor = TwoDTargetSensor(scenario, config["observation_frequency_hz"])
    reader = stage6_2.TargetObservationWireReader(sensor)
    governor_config = FollowGovernorConfig(**config["governor"])
    governor = FollowGovernor(governor_config)
    cache = SparseProjectionFactorizationCache()
    planner = stage5.make_full_planner(stage5_config, common,
        planning_dt_s=float(stage5_config["full_dynamic"]["planning_dt_s"]),
        backend="cached_kkt", cache=cache)
    source = stage5.make_reference_source(stage5_config, common, reader,
        follower=governor, full_planner_override=planner,
        scheduler_gate_overrides=config["follow_scheduler_gate_policy"])
    producer = PiReferenceBlockProducer(source)
    receiver = MCUReferenceBlockReceiver(dt_s=producer.dt_s,
                                         block_samples=producer.block_samples)
    transport = InMemoryPacketTransport(config["safety"]["transport_latency_s"])
    yaw_source = MCULatestYawSource(dt_s=producer.dt_s,
        timeout_s=config["yaw"]["freshness_timeout_s"],
        decay_rate_rad_s2=config["yaw"]["decay_rate_rad_s2"])
    safety = config["safety"]
    arbiter = MCUSafetyArbiter(receiver, SafetyBrakeConfig(
        safety["safe_max_deceleration_m_s2"], safety["safe_max_jerk_m_s3"],
        safety["hidden_reference_handoff_time_s"], producer.dt_s),
        heartbeat_timeout_s=safety["heartbeat_timeout_s"],
        starvation_margin_s=safety["reference_starvation_margin_s"],
        nominal_pitch_rad=float(common[7]["parameters"]["theta_eq_rad"]),
        status_sink=lambda payload, time_s: transport.send(
            "pi", "safety_status", payload, time_s), yaw_source=yaw_source)
    servo = BearingYawServo(gain_s_inv=config["yaw"]["gain_s_inv"],
        max_rate_rad_s=config["yaw"]["max_rate_rad_s"],
        deadband_rad=config["yaw"]["deadband_rad"])
    link = TwoDLink(transport=transport, producer=producer, governor=governor,
        arbiter=arbiter, crash_time_s=None,
        hazard_time_s=scenario.get("hazard_time_s"),
        hazard_action="LONGITUDINAL_STOP", hazard_reason="LONGITUDINAL_HAZARD",
        reader=reader, servo=servo, scenario=scenario,
        stage5_config=stage5_config, common=common, full_planner=planner,
        governor_config=governor_config,
        scheduler_policy=config["follow_scheduler_gate_policy"])
    q_adapter, runtime = stage5.make_runtime(
        common, stage5_config, stage4_config, stage4c_config)
    manifest = common[0]
    run = stage3b.run_case({"name": scenario["name"],
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
            "mcu_safety_action": arbiter.safety_action,
            "mcu_long_source": ("NORMAL_PI_BLOCK" if arbiter.state == "NORMAL"
                                else "LOCAL_SAFETY_BRAKE" if arbiter.state == "BRAKING"
                                else "SAFE_HOLD_ZERO"),
            "mcu_yaw_source": yaw_source.source,
            "yaw_receive_tick": yaw_source.receive_tick,
            "yaw_receive_age_s": (None if yaw_source.receive_tick is None else
                (int(round(float(sim.data.time)/producer.dt_s))
                 -yaw_source.receive_tick)*producer.dt_s),
            "reference_epoch": receiver.reference_epoch,
            "pi_safety_state": link.pi_safety_state,
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
    history = run["history_50hz"]
    name = scenario["name"]
    stage6_support.write_csv(RESULT_DIR/f"{name}_history.csv", history)
    stage6_support.write_csv(RESULT_DIR/f"{name}_governor.csv", governor.observation_history)
    stage6_support.write_csv(RESULT_DIR/f"{name}_yaw_generated.csv", link.yaw_generation_events)
    stage6_support.write_csv(RESULT_DIR/f"{name}_yaw_mcu.csv", yaw_source.events)
    stage6_support.write_csv(RESULT_DIR/f"{name}_mcu.csv", arbiter.events+receiver.events)
    stage6_support.write_csv(RESULT_DIR/f"{name}_transport.csv", transport.events)
    for index, item in enumerate(link.all_producers):
        stage6_support.write_csv(RESULT_DIR/f"{name}_pi_intents_epoch{index}.csv",
                            item.source.intent_history)
        stage6_support.write_csv(RESULT_DIR/f"{name}_full_plans_epoch{index}.csv",
                            item.source.replan_events)
    figure = RESULT_DIR/"plots"/f"{name}.png"
    _plot(figure, scenario, history, governor, link)
    yaw_applied = [event for event in yaw_source.events if event["event"] == "yaw_applied"]
    takes = [event for event in arbiter.events if event["event"] == "safety_takeover"]
    d_values = [row["distance_m"] for row in governor.observation_history]
    def stats(values):
        values = [float(x) for x in values]
        return ({"count": len(values), "min_s": min(values),
                 "max_s": max(values), "mean_s": sum(values)/len(values)}
                if values else {"count": 0})
    yaw_latencies = [e["time_s"]-e["source_time_s"] for e in yaw_applied]
    yaw_receive_times = [e["time_s"] for e in yaw_source.events
                         if e["event"] == "yaw_received"]
    block_receive = [e for e in receiver.events if e["event"] == "block_received"]
    lead_times = [e["start_control_tick"]*producer.dt_s-e["time_s"]
                  for e in block_receive]
    all_plans = [e for p in link.all_producers for e in p.source.replan_events
                 if e["planning_path"] == "FULL_DYNAMIC"]
    plan_latencies = [e["execution_start_time_s"]-e["planning_request_time_s"]
                      for e in all_plans]
    yaw_while_full = sum(any(
        p["execution_start_time_s"] <= e["time_s"]
        < (p["full_exit_time_s"] if p["full_exit_time_s"] is not None
           else next((take["time_s"] for take in takes
                      if take["time_s"] > p["execution_start_time_s"]),
                     scenario["duration_s"]))
        for p in all_plans) for e in yaw_applied)
    gt_by_seq = {int(e["sequence_id"]): e for e in sensor.evaluation_history}
    radial_errors = [e["master_radial_velocity_hat_m_s"]
                     -gt_by_seq[int(e["sequence_id"])]["master_radial_velocity_GT_m_s"]
                     for e in governor.observation_history
                     if int(e["sequence_id"]) in gt_by_seq]
    safe_yaw_rows = [row for row in history
                     if row["mcu_safety_state"] in ("BRAKING", "LONGITUDINAL_STOP")
                     and abs(row["yaw_rate_cmd_rad_s"]) > 1e-3]
    near_rows = [row for row in history if math.hypot(
        row["target_observation_x_forward_m"],
        row["target_observation_y_left_m"]) <= min(d_values)+0.05]
    sent = defaultdict(deque)
    transport_latencies = defaultdict(list)
    for event in transport.events:
        key = (event["direction"], event["channel"])
        if event["event"] == "packet_sent":
            sent[key].append(event["time_s"])
        elif event["event"] == "packet_received" and sent[key]:
            transport_latencies[event["channel"]].append(
                event["time_s"]-sent[key].popleft())
    resume_jumps = [e for e in arbiter.events
                    if e["event"] == "resume_reference_continuity"]
    return stage6_support.json_safe({
        "scenario": name,
        "simulation": {"duration_s": run["simulation_termination"]["actual_duration_s"],
            "fell": run["longitudinal"]["fell"], "finite": run["finite"],
            "wheel_saturation_fraction": run["torque_allocation"]["per_wheel_saturation_fraction"],
            "min_distance_m": min(d_values), "final_distance_m": d_values[-1],
            "max_abs_beta_deg": max(abs(math.degrees(row["beta_rad"]))
                                     for row in governor.observation_history),
            "min_v_cmd_m_s": min(row["latched_v_cmd_m_s"] for row in governor.observation_history),
            "max_v_cmd_m_s": max(row["latched_v_cmd_m_s"] for row in governor.observation_history),
            "sum_command_saturated_control_samples": sum(bool(row["sum_command_saturated"])
                for row in history),
            "closest_point_v_zero_yaw_nonzero_control_samples": sum(
                abs(row["rolling_v_ref_applied_m_s"]) <= 0.03
                and abs(row["yaw_rate_cmd_rad_s"]) >= 0.02 for row in near_rows),
            "radial_velocity_error_posthoc_rmse_m_s": math.sqrt(
                sum(x*x for x in radial_errors)/len(radial_errors))},
        "split_authority": {"yaw_generated_count": len([e for e in link.yaw_generation_events
            if e["event"] == "yaw_generated"]),
            "yaw_applied_count": len(yaw_applied),
            "yaw_updates_during_full_count": yaw_while_full,
            "yaw_source_to_apply_latency": stats(yaw_latencies),
            "observation_to_yaw_generation_latency": stats(
                e["time_s"]-e["source_time_s"] for e in link.yaw_generation_events
                if e["event"] == "yaw_generated"),
            "wire_transport_latency": {channel: stats(values)
                                       for channel, values in transport_latencies.items()
                                       if channel in ("yaw", "reference", "safety")},
            "yaw_receive_interval_jitter_from_50ms": stats(
                abs((b-a)-0.05) for a,b in zip(yaw_receive_times,yaw_receive_times[1:])
                if b-a < 0.1),
            "max_yaw_receive_age_s": yaw_source.max_age_s,
            "yaw_stale_event_count": sum(e["event"] == "yaw_source_changed" and
                e["source"] == "LOCAL_YAW_TO_ZERO" for e in yaw_source.events),
            "safety_takeovers": takes,
            "safety_to_takeover_latency_s": [e["time_s"]-e["issued_time_s"]
                for e in takes if e["issued_time_s"] is not None],
            "resume_reference_jumps": resume_jumps,
            "yaw_nonzero_while_longitudinal_safety_control_samples": len(safe_yaw_rows),
            "safety_first_tick_reference_deltas": arbiter.first_takeover_deltas,
            "resume_count": sum(e["event"] == "normal_resumed" for e in arbiter.events),
            "old_epoch_rejected_count": sum(e["event"] == "invalid_epoch_rejected"
                for e in receiver.events),
            "reference_underrun_count": receiver.unhandled_underrun_count,
            "final_epoch": receiver.reference_epoch,
            "final_safety_state": arbiter.state},
        "plans": {"full_count": sum(len([e for e in p.source.replan_events
            if e["planning_path"] == "FULL_DYNAMIC"]) for p in link.all_producers),
            "plan_to_execution_latency": stats(plan_latencies),
            "reference_receive_lead_time": stats(lead_times),
            "reference_receive_interval_jitter_from_50ms": stats(
                abs((b["time_s"]-a["time_s"])-0.05)
                for a,b in zip(block_receive,block_receive[1:])
                if b["time_s"]-a["time_s"] < 0.1),
            "block_generated_count": sum(len(p.events) for p in link.all_producers),
            "block_received_count": sum(e["event"] == "block_received"
                for e in receiver.events)},
        "baseline": {"GT_runtime_dependency": run["GT_runtime_dependency"],
            "controller_dt_s": producer.dt_s, "physics_dt_s": 0.001},
        "files": {"figure": figure.relative_to(ROOT).as_posix()}})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario")
    args = parser.parse_args()
    config = stage5.load_json(CONFIG_PATH)
    stage5_config = stage5.load_json(stage5.CONFIG_PATH)
    common, stage4_config, stage4c_config, frozen = stage5.load_common(stage5_config)
    if not all(frozen.values()):
        raise RuntimeError(f"frozen baseline checks failed: {frozen}")
    scenarios = config["scenarios"]
    if args.scenario:
        scenarios = [s for s in scenarios if s["name"] == args.scenario]
        if not scenarios:
            raise ValueError("unknown scenario")
    RESULT_DIR.mkdir(parents=True, exist_ok=True)
    results = []
    for scenario in scenarios:
        print(f"RUN {scenario['name']}", flush=True)
        results.append(run_scenario(config, stage5_config, common,
                                    stage4_config, stage4c_config, scenario))
        (RESULT_DIR/"stage6_4_2d_results.json").write_text(json.dumps(
            {"stage": config["stage"], "status": "IN_PROGRESS",
             "frozen_checks": frozen, "scenarios": results},
            ensure_ascii=False, indent=2), encoding="utf-8")
    (RESULT_DIR/"stage6_4_2d_results.json").write_text(json.dumps(
        {"stage": config["stage"], "status": "COMPLETED",
         "frozen_checks": frozen, "scenarios": results},
        ensure_ascii=False, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
