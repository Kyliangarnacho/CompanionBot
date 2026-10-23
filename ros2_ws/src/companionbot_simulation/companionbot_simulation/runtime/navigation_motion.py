"""Simulation composition of Nav2 intents and the unchanged Pi/MCU protocol.

Physics/control scheduling remains owned by the original runner. No Nav2
callback runs control, changes state, clears MCU safety, or writes an actuator.
"""
import json
import math

from companionbot_contracts.paths import repository_root
from companionbot_contracts.packets import PROTOCOL_VERSION, YawCommandPacket
from companionbot_control.runtime.mcu import MCUSafetyArbiter, MCUReferenceBlockReceiver
from companionbot_control.runtime.yaw_reference import MCULatestYawSource
from companionbot_control.core.safety_brake import SafetyBrakeConfig
from companionbot_follow.runtime.navigation_motion import NavigationIntentReader
from companionbot_follow.runtime.reference_producer import PiReferenceBlockProducer
from companionbot_simulation.runtime.packet_link import InMemoryPacketTransport, Stage6PacketLink


class NavigationPacketLink(Stage6PacketLink):
    def __init__(self, *, adapter, reader, source_factory, **kwargs):
        super().__init__(**kwargs)
        self.adapter, self.reader, self.source_factory = adapter, reader, source_factory
        self.yaw_sequence = 0
        self._last_resume_generation = -1
        self._last_navigation_stop = None
        self.all_producers = [self.producer]

    def on_robot_state_packet(self, packet):
        pass  # Nav2 consumes odometry; no synthetic follow governor is active.

    def after_pi_packets(self, time_s):
        self.reader.time_s = time_s
        # Existing SafetyStatus is transition-driven, not a startup heartbeat.
        # NORMAL is explicitly reported by the decoded 20Hz MCU RobotState;
        # stopped/fault reasons come from the matching transition status.
        robot_status = self.latest_robot_state
        safety_status = self.latest_safety_status
        if (safety_status is not None and safety_status.state != "NORMAL"
                and (robot_status is None or safety_status.timestamp_s >= robot_status.sample_time_s)):
            self.adapter.update_mcu_status(safety_status.state, safety_status.reason)
        elif (robot_status is not None and time_s-robot_status.sample_time_s <= .1
                and robot_status.safety_state == "NORMAL"):
            self.adapter.update_mcu_status("NORMAL", "normal_from_robot_state_packet")
        elif self.latest_safety_status is not None:
            self.adapter.update_mcu_status(self.latest_safety_status.state, self.latest_safety_status.reason)
        intent = self.adapter.sample(time_s)
        stop = self.adapter.take_stop()
        if stop is not None:
            self.send_brake(time_s, stop[1], "HARD_STOP")
            self._last_navigation_stop = (time_s, stop[1])
        tick = round(time_s / self.producer.dt_s)
        if tick % self.producer.block_samples == 0:
            packet = YawCommandPacket(PROTOCOL_VERSION, self.yaw_sequence, time_s,
                                      intent.yaw_rate_target_rad_s)
            self.yaw_sequence += 1
            self.transport.send("mcu", "yaw", packet.to_bytes(), time_s)
        # A new, explicit high-level lease may request software reference rearm
        # only after the MCU reports SAFE_HOLD. Physical arm/fault gates remain
        # entirely MCU-owned. Fresh stopped status and matching reference epoch
        # are checked again by Stage6PacketLink.request_resume / MCU receiver.
        snapshot = self.adapter.snapshot(include_events=False)
        status, robot = self.latest_safety_status, self.latest_robot_state
        # The frozen MCU ignores a new safety command while already SAFE_HOLD;
        # a previously armed future resume can still acknowledge NORMAL later.
        # Keep the revoked task's stop authoritative across that acknowledgement
        # without altering the MCU, producing more blocks or clearing its gate.
        if (not snapshot["authorized"] and self._last_navigation_stop is not None
                and status is not None and status.state == "NORMAL"
                and status.timestamp_s >= self._last_navigation_stop[0]):
            self.send_brake(time_s, self._last_navigation_stop[1], "HARD_STOP")
            self._last_navigation_stop = (time_s, self._last_navigation_stop[1])
        # A lease alone is not a motion command. Planning failure / an already
        # satisfied goal need no MCU rearm and must not leave a pending resume.
        moving_intent = (abs(intent.linear_velocity_target_m_s) > 1e-9
                         or abs(intent.yaw_rate_target_rad_s) > 1e-9)
        if (not snapshot["authorized"] or not self.reference_production_stopped
                or not moving_intent
                or status is None or robot is None or status.state != "SAFE_HOLD"
                or robot.safety_state != status.state or robot.sample_time_s < status.timestamp_s
                or snapshot["authority_epoch"] <= self._last_resume_generation):
            return
        # Never implicitly recover a MCU-originated heartbeat/starvation/fault.
        # Only a safety stop requested by this adapter belongs to this rearm path.
        local_reasons = {event["reason"] for event in self.packet_events
                         if event["event"] == "pi_safety_command"}
        if status.reason not in local_reasons:
            return
        if robot.applied_p_ref_m is None:
            return
        source = self.source_factory()
        source.lifecycle.reset(float(robot.applied_p_ref_m), 0.)
        producer = PiReferenceBlockProducer(source, reference_epoch=status.reference_epoch)
        block_ticks = producer.block_samples
        start_tick = (math.floor(time_s / producer.dt_s / block_ticks) + 2) * block_ticks
        block, _ = producer.produce(start_tick, time_s)
        self.request_resume(time_s=time_s, hazard_cleared=True, fresh_producer=producer,
                            first_block=block, authority="HARD")
        self.all_producers.append(producer)
        self._last_resume_generation = snapshot["authority_epoch"]


def build_navigation_reference(common, stage5, config, adapter):
    """Return existing MCU arbiter, serialized link and explicit nav adaptation.

    Borrow the existing small-command LIGHTWEIGHT Ruckig branch. A forward-only
    0..0.25 m/s envelope cannot cross the frozen 0.35 m/s FULL threshold, so
    retargets remain interruptible without changing the FULL lifecycle contract.
    Existing jerk / acceleration / block / MCU safety parameters are untouched.
    """
    if adapter.max_velocity_m_s >= config["scheduler"]["T_full_delta_v_m_s"]:
        raise ValueError("navigation_envelope_requires_review_of_full_planner_lock")
    reader = NavigationIntentReader(adapter)
    policy = dict(require_stable_candidate=False, minimum_delta_enabled=False,
                  follow_force_full_dynamic=False, follow_defer_until_full_exit=False)

    def source_factory():
        return stage5.make_reference_source(config, common, reader, follower=reader.intent,
                                            scheduler_gate_overrides=policy)

    producer = PiReferenceBlockProducer(source_factory())
    path = repository_root() / "models/minisegway/stage6/config/stage6_4_2d_follow_config.json"
    baseline = json.loads(path.read_text())
    safety, yaw = baseline["safety"], baseline["yaw"]
    transport = InMemoryPacketTransport(safety["transport_latency_s"])
    receiver = MCUReferenceBlockReceiver(dt_s=producer.dt_s, block_samples=producer.block_samples)
    yaw_source = MCULatestYawSource(dt_s=producer.dt_s, timeout_s=yaw["freshness_timeout_s"],
                                   decay_rate_rad_s2=yaw["decay_rate_rad_s2"])
    arbiter = MCUSafetyArbiter(receiver, SafetyBrakeConfig(safety["safe_max_deceleration_m_s2"],
        safety["safe_max_jerk_m_s3"], safety["hidden_reference_handoff_time_s"], producer.dt_s),
        heartbeat_timeout_s=safety["heartbeat_timeout_s"],
        starvation_margin_s=safety["reference_starvation_margin_s"],
        nominal_pitch_rad=float(common[7]["parameters"]["theta_eq_rad"]), yaw_source=yaw_source,
        status_sink=lambda payload, t: transport.send("pi", "safety_status", payload, t))
    link = NavigationPacketLink(adapter=adapter, reader=reader, source_factory=source_factory,
        transport=transport, producer=producer, governor=None, arbiter=arbiter,
        crash_time_s=None, hazard_time_s=None)
    return arbiter, link, dict(command_owner="Nav2 task lease", scheduler_policy=policy,
        planner="existing_LIGHTWEIGHT_Ruckig", physical_output_gate="unchanged_MCU",
        safety_config_source=str(path), map_id=adapter.map_id, map_version=adapter.map_version,
        max_velocity_m_s=adapter.max_velocity_m_s, max_yaw_rate_rad_s=adapter.max_yaw_rate_rad_s,
        velocity_smoother=False)
