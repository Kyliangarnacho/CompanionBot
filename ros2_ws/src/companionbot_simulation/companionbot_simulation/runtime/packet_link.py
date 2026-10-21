"""Simulation-only byte transport and Pi/MCU scheduling composition."""
from __future__ import annotations
from dataclasses import dataclass, replace
import math
import time
from typing import Callable
import numpy as np
from companionbot_contracts.packets import (PROTOCOL_VERSION, HeartbeatPacket, ReferenceBlockPacket, ResumeReferenceCommandPacket, RobotStatePacket, SafetyCommandPacket, SafetyStatusPacket, YawCommandPacket)
from companionbot_control.runtime.mcu import MCUSafetyArbiter, MCUReferenceBlockReceiver
from companionbot_follow.runtime.reference_producer import PiReferenceBlockProducer

class InMemoryPacketTransport:
    """Moves bytes with latency/loss; makes no planner or safety decisions."""

    def __init__(self, latency_s: float = 0.0) -> None:
        if not math.isfinite(latency_s) or latency_s < 0.0:
            raise ValueError("transport latency must be finite and nonnegative")
        self.latency_s = float(latency_s)
        self.in_flight: list[tuple[float, str, str, bytes]] = []
        self.drop_after: dict[tuple[str, str], float] = {}
        self.events: list[dict] = []
        self.max_reference_in_flight = 2

    def send(self, direction: str, channel: str, payload: bytes,
             time_s: float, *, not_before_s: float | None = None,
             startup_prefill: bool = False) -> None:
        if direction not in ("pi", "mcu"):
            raise ValueError("transport direction must name receiver")
        if time_s >= self.drop_after.get((direction, channel), math.inf):
            self.events.append({"event": "packet_dropped", "direction": direction,
                                "channel": channel, "time_s": float(time_s)})
            return
        if (channel == "reference" and sum(
            recipient == direction and queued_channel == "reference"
            for _, recipient, queued_channel, _ in self.in_flight
        ) >= self.max_reference_in_flight):
            self.events.append({"event": "packet_dropped", "direction": direction,
                                "channel": channel, "time_s": float(time_s),
                                "reason": "reference_transport_capacity"})
            return
        ready = (float(time_s) if startup_prefill else
                 max(float(time_s), float(not_before_s or time_s)) + self.latency_s)
        self.in_flight.append((ready, direction, channel, bytes(payload)))
        self.events.append({"event": "packet_sent", "direction": direction,
                            "channel": channel, "time_s": float(time_s),
                            "available_time_s": ready})

    def deliver(self, direction: str, time_s: float) -> list[tuple[str, bytes]]:
        received = []
        waiting = []
        for ready, recipient, channel, payload in self.in_flight:
            if recipient == direction and ready <= time_s + 1e-12:
                received.append((channel, bytes(payload)))
                self.events.append({"event": "packet_received", "direction": direction,
                                    "channel": channel, "time_s": float(time_s)})
            else:
                waiting.append((ready, recipient, channel, payload))
        self.in_flight = waiting
        return received

class Stage6PacketLink:
    """Simulation clock wires Pi and MCU endpoints through serialized packets."""

    def __init__(self, *, transport: InMemoryPacketTransport,
                 producer: PiReferenceBlockProducer, governor,
                 arbiter: MCUSafetyArbiter, crash_time_s: float | None,
                 hazard_time_s: float | None, hazard_reason: str = "OBSTACLE",
                 hazard_action: str = "BRAKE") -> None:
        self.transport = transport
        self.producer = producer
        self.governor = governor
        self.arbiter = arbiter
        self.crash_time_s = crash_time_s
        self.hazard_time_s = hazard_time_s
        self.hazard_reason = hazard_reason
        self.hazard_action = hazard_action
        self.hazard_sent = False
        self.heartbeat_sequence_id = 0
        self.safety_sequence_id = 0
        self.robot_state_sequence_id = 0
        self.pi_safety_state = "NORMAL"
        self.latest_safety_status: SafetyStatusPacket | None = None
        self.reference_production_stopped = False
        self.resume_sequence_id = 0
        self.resume_followup_start_tick: int | None = None
        self.latest_robot_state: RobotStatePacket | None = None
        self.packet_events: list[dict] = []
        if crash_time_s is not None:
            self.transport.drop_after[("pi", "robot_state")] = crash_time_s
            self.transport.drop_after[("pi", "safety_status")] = crash_time_s

    def mcu_local_state(self, time_s: float, estimate, yaw_estimate) -> None:
        self.arbiter.set_local_state(
            v_hat_m_s=estimate.velocity_hat_m_s,
            pitch_hat_rad=estimate.theta_hat_rad,
            pitch_rate_hat_rad_s=estimate.theta_dot_hat_rad_s,
        )
        tick = int(round(time_s / self.producer.dt_s))
        if tick % self.producer.block_samples == 0:
            packet = RobotStatePacket(
                PROTOCOL_VERSION, self.robot_state_sequence_id, time_s,
                float(estimate.velocity_hat_m_s),
                float(yaw_estimate.psi_control_rad),
                float(yaw_estimate.r_hat_rad_s), self.arbiter.state,
                (0.0 if self.arbiter.last_applied is None else
                 float(self.arbiter.last_applied.reference_state[0])),
            )
            self.robot_state_sequence_id += 1
            self.transport.send("pi", "robot_state", packet.to_bytes(), time_s)

    def _send_block(self, start_tick: int, time_s: float,
                    *, observe_command: bool, startup: bool = False) -> None:
        packet, compute_time_s = self.producer.produce(
            start_tick, time_s, observe_command=observe_command
        )
        self.transport.send(
            "mcu", "reference", packet.to_bytes(), time_s,
            not_before_s=(time_s if startup else time_s + compute_time_s),
            startup_prefill=startup,
        )

    def send_brake(self, time_s: float, reason: str,
                   action: str = "BRAKE") -> None:
        self.reference_production_stopped = True
        self.resume_followup_start_tick = None
        packet = SafetyCommandPacket(
            PROTOCOL_VERSION, self.safety_sequence_id, time_s, action, reason,
        )
        self.safety_sequence_id += 1
        self.transport.send("mcu", "safety", packet.to_bytes(), time_s)
        self.packet_events.append({"event": "pi_safety_command",
                                   "time_s": time_s, "reason": reason,
                                   "action": action})

    def request_resume(self, *, time_s: float, hazard_cleared: bool,
                       fresh_producer: PiReferenceBlockProducer,
                       first_block: ReferenceBlockPacket,
                       authority: str = "HARD") -> None:
        status = self.latest_safety_status
        robot = self.latest_robot_state
        required_state = ("LONGITUDINAL_STOP" if authority == "LONGITUDINAL"
                          else "SAFE_HOLD")
        if not hazard_cleared or status is None or status.state != required_state:
            raise ValueError("Pi requires explicit clearance and matching safety feedback")
        if (robot is None or robot.safety_state != required_state
                or robot.sample_time_s < status.timestamp_s
                or time_s - robot.sample_time_s
                > 2.0 * fresh_producer.block_samples * fresh_producer.dt_s):
            raise ValueError("Pi requires a fresh stopped RobotStatePacket")
        if (first_block.reference_epoch != status.reference_epoch
                or first_block.reference_epoch != fresh_producer.reference_epoch
                or first_block.sequence_id + 1 != fresh_producer.next_sequence_id
                or first_block.sample_count != fresh_producer.block_samples
                or not math.isclose(first_block.dt_s, fresh_producer.dt_s,
                                    abs_tol=1e-12)
                or first_block.start_control_tick
                % fresh_producer.block_samples != 0
                or first_block.start_control_tick * first_block.dt_s
                <= time_s + 1e-12
                or first_block.generated_time_s < robot.sample_time_s):
            raise ValueError("fresh resume block/producer does not match current epoch")
        if (robot.applied_p_ref_m is not None
                and abs(first_block.sample(0)[0] - robot.applied_p_ref_m) > 0.05):
            raise ValueError("resume position reference jumps from local hold")
        self.producer = fresh_producer
        self.resume_followup_start_tick = (
            first_block.start_control_tick + fresh_producer.block_samples
        )
        packet = ResumeReferenceCommandPacket(
            PROTOCOL_VERSION, self.resume_sequence_id, time_s,
            first_block.reference_epoch, authority,
        )
        self.resume_sequence_id += 1
        self.transport.send("mcu", "resume", packet.to_bytes(), time_s)
        self.transport.send("mcu", "reference", first_block.to_bytes(), time_s)
        self.packet_events.append({"event": "pi_resume_requested",
                                   "time_s": time_s,
                                   "reference_epoch": first_block.reference_epoch,
                                   "authority": authority,
                                   "first_block_sequence_id": first_block.sequence_id})

    def on_robot_state_packet(self, packet: RobotStatePacket) -> None:
        """Legacy follow path uses the latest packet velocity directly."""
        self.governor.set_robot_velocity_hat(packet.v_hat_m_s)

    def on_pi_packet(self, channel: str, payload: bytes, time_s: float) -> None:
        raise ValueError(f"unknown Pi channel {channel}")

    def after_pi_packets(self, time_s: float) -> None:
        """Optional Pi work after all arrived packets are decoded."""

    def before_control_tick(self, time_s: float) -> None:
        tick = int(round(time_s / self.producer.dt_s))
        pi_alive = (self.crash_time_s is None
                    or time_s < self.crash_time_s - 1e-12)
        if pi_alive:
            for channel, payload in self.transport.deliver("pi", time_s):
                if channel == "robot_state":
                    packet = RobotStatePacket.from_bytes(payload)
                    self.latest_robot_state = packet
                    self.on_robot_state_packet(packet)
                elif channel == "safety_status":
                    status = SafetyStatusPacket.from_bytes(payload)
                    self.latest_safety_status = status
                    self.pi_safety_state = status.state
                    if status.state != "NORMAL":
                        self.reference_production_stopped = True
                    elif self.resume_sequence_id > 0:
                        self.reference_production_stopped = False
                else:
                    self.on_pi_packet(channel, payload, time_s)
            self.after_pi_packets(time_s)
            sent_resume_followup = False
            if (not self.reference_production_stopped
                    and self.pi_safety_state == "NORMAL"
                    and self.resume_followup_start_tick is not None):
                self._send_block(self.resume_followup_start_tick, time_s,
                                 observe_command=True)
                self.resume_followup_start_tick = None
                sent_resume_followup = True
            if tick % self.producer.block_samples == 0:
                heartbeat = HeartbeatPacket(
                    PROTOCOL_VERSION, self.heartbeat_sequence_id, time_s
                )
                self.heartbeat_sequence_id += 1
                self.transport.send("mcu", "heartbeat", heartbeat.to_bytes(), time_s)
            if (self.hazard_time_s is not None and not self.hazard_sent
                    and time_s >= self.hazard_time_s - 1e-12):
                self.hazard_sent = True
                self.send_brake(time_s, self.hazard_reason, self.hazard_action)
            if (not sent_resume_followup
                    and not self.reference_production_stopped
                    and self.pi_safety_state == "NORMAL"):
                if tick == 0:
                    self._send_block(0, time_s, observe_command=True, startup=True)
                    self._send_block(self.producer.block_samples, time_s,
                                     observe_command=False, startup=True)
                elif tick % self.producer.block_samples == 0:
                    self._send_block(tick + self.producer.block_samples,
                                     time_s, observe_command=True)
        for channel, payload in self.transport.deliver("mcu", time_s):
            self.arbiter.receive(channel, payload, time_s)

