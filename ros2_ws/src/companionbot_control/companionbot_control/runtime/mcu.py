"""Frozen MCU reference consumer and local safety arbiter; no Pi planner."""
from __future__ import annotations
from dataclasses import dataclass, replace
import math
import time
from typing import Callable
import numpy as np
from companionbot_contracts.packets import (PROTOCOL_VERSION, HeartbeatPacket, ReferenceBlockPacket, ResumeReferenceCommandPacket, RobotStatePacket, SafetyCommandPacket, SafetyStatusPacket, YawCommandPacket)
from companionbot_control.core.safety_brake import SafetyBrakeConfig, SafetyBrakePrimitive, SafetyReference
from .yaw_reference import MCULatestYawSource

@dataclass(frozen=True)
class WireReferenceCommand:
    reference_state: np.ndarray
    reference_acceleration_m_s2: float
    u_ff_raw_nm: float
    u_ff_after_lifecycle_nm: float
    linear_velocity_target_m_s: float
    yaw_rate_target_rad_s: float
    feedforward_phase: str
    velocity_phase: str
    fade_alpha: float
    block_id: int
    block_sample_index: int
    block_start_time_s: float
    intent_source_time_s: float
    replanned: bool = False
    raw_linear_velocity_target_m_s: float | None = None
    scheduler_mode: str = "WIRE"
    accepted_velocity_changed: bool = False
    planning_path: str = "WIRE_NORMAL"
    candidate_delta_v_m_s: float = 0.0
    candidate_target_m_s: float | None = None
    candidate_stable: bool = False
    latest_pending_raw_v_cmd_m_s: float | None = None
    pending_command_active: bool = False

class MCUReferenceBlockReceiver:
    """Consumes only decoded ReferenceBlockPacket values and float32 samples."""

    def __init__(self, *, dt_s: float, block_samples: int,
                 reference_epoch: int = 0) -> None:
        self.dt_s = float(dt_s)
        self.block_samples = int(block_samples)
        self.reference_epoch = reference_epoch
        self.active: ReferenceBlockPacket | None = None
        self.next: ReferenceBlockPacket | None = None
        self.paused = False
        self.last_sequence_id: int | None = None
        self.last_tick: int | None = None
        self.events: list[dict] = []
        self.unhandled_underrun_count = 0

    def receive(self, payload: bytes, time_s: float) -> None:
        packet = ReferenceBlockPacket.from_bytes(payload)
        if packet.reference_epoch != self.reference_epoch:
            self.events.append({"event": "invalid_epoch_rejected",
                                "time_s": time_s, "sequence_id": packet.sequence_id,
                                "reference_epoch": packet.reference_epoch})
            return
        if (packet.sample_count != self.block_samples
                or not math.isclose(packet.dt_s, self.dt_s, abs_tol=1e-12)):
            raise ValueError("MCU reference packet geometry mismatch")
        if packet.start_control_tick * self.dt_s < time_s - 1e-12 or (
            self.paused and packet.start_control_tick * self.dt_s
            <= time_s + 1e-12
        ):
            self.events.append({"event": "stale_reference_packet",
                                "time_s": time_s, "sequence_id": packet.sequence_id})
            return
        if self.paused:
            if self.next is not None:
                self.events.append({"event": "reference_capacity_rejected",
                                    "time_s": time_s, "sequence_id": packet.sequence_id})
                return
            self.next = packet
        elif self.active is None:
            self.active = packet
        elif (self.next is None and packet.start_control_tick
              == self.active.start_control_tick + self.block_samples):
            self.next = packet
        else:
            self.events.append({"event": "reference_capacity_rejected",
                                "time_s": time_s, "sequence_id": packet.sequence_id})
            return
        if self.last_sequence_id is not None and packet.sequence_id != self.last_sequence_id + 1:
            self.events.append({"event": "reference_sequence_gap",
                                "time_s": time_s, "expected": self.last_sequence_id + 1,
                                "received": packet.sequence_id})
        self.last_sequence_id = packet.sequence_id
        self.events.append({"event": "block_received", "time_s": time_s,
                            "sequence_id": packet.sequence_id,
                            "start_control_tick": packet.start_control_tick})

    def ensure_active(self, tick: int) -> None:
        if self.active is not None and tick == (
            self.active.start_control_tick + self.block_samples
        ):
            self.active = self.next if (
                self.next is not None and self.next.start_control_tick == tick
            ) else None
            self.next = None
            if self.active is not None:
                self.events.append({"event": "active_swap", "time_s": tick * self.dt_s,
                                    "sequence_id": self.active.sequence_id})
        if self.active is None:
            self.unhandled_underrun_count += 1
            raise RuntimeError(f"unhandled MCU reference underrun at tick {tick}")

    def next_ready(self) -> bool:
        return (self.active is not None
                and self.next is not None
                and self.active.start_control_tick + self.block_samples
                == self.next.start_control_tick)

    def invalidate_current_epoch(self) -> int:
        invalidated = self.reference_epoch
        self.reference_epoch += 1
        self.active = None
        self.next = None
        self.last_sequence_id = None
        self.last_tick = None
        self.paused = True
        return invalidated

    def resume_block_ready(self, tick: int) -> bool:
        return (self.paused and self.active is None and self.next is not None
                and self.next.reference_epoch == self.reference_epoch
                and self.next.start_control_tick > tick)

    def activate_resume(self, tick: int) -> None:
        if (not self.paused or self.next is None
                or self.next.start_control_tick != tick):
            raise RuntimeError("resume block is not ready at this control tick")
        self.active = self.next
        self.next = None
        self.paused = False
        self.events.append({"event": "resume_block_activated",
                            "time_s": tick * self.dt_s,
                            "sequence_id": self.active.sequence_id,
                            "reference_epoch": self.reference_epoch})

    def remaining_ticks(self, tick: int) -> int:
        if self.active is None:
            return 0
        return self.active.start_control_tick + self.block_samples - tick

    def command(self, tick: int) -> WireReferenceCommand:
        self.ensure_active(tick)
        assert self.active is not None
        index = tick - self.active.start_control_tick
        if not 0 <= index < self.block_samples:
            self.unhandled_underrun_count += 1
            raise RuntimeError(f"MCU reference sample missing at tick {tick}")
        p, v, a, theta, theta_dot, u_ff = self.active.sample(index)
        self.last_tick = tick
        return WireReferenceCommand(
            np.asarray([p, v, theta, theta_dot], dtype=float), a, u_ff, u_ff,
            v, self.active.yaw_rate_target_rad_s, "WIRE", "WIRE", 1.0,
            self.active.sequence_id, index,
            self.active.start_control_tick * self.dt_s,
            self.active.generated_time_s,
        )

    def peek_next(self) -> tuple[np.ndarray, float]:
        if self.active is None or self.last_tick is None:
            return np.zeros(4, dtype=float), 0.0
        tick = self.last_tick + 1
        active_end = self.active.start_control_tick + self.block_samples
        packet = self.active if tick < active_end else self.next
        if packet is None:
            self.unhandled_underrun_count += 1
            raise RuntimeError(f"MCU reference lookahead missing at tick {tick}")
        p, v, a, theta, theta_dot, _ = packet.sample(
            tick - packet.start_control_tick
        )
        return np.asarray([p, v, theta, theta_dot], dtype=float), a

class MCUSafetyArbiter:
    """Only wire packets and local estimator scalars can affect MCU authority."""

    def __init__(self, receiver: MCUReferenceBlockReceiver,
                 brake_config: SafetyBrakeConfig, *, heartbeat_timeout_s: float,
                 starvation_margin_s: float, nominal_pitch_rad: float,
                 status_sink: Callable[[bytes, float], None],
                 yaw_source: MCULatestYawSource | None = None) -> None:
        self.receiver = receiver
        self.brake_config = brake_config
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.starvation_margin_ticks = math.ceil(starvation_margin_s / receiver.dt_s)
        self.nominal_pitch_rad = float(nominal_pitch_rad)
        self.status_sink = status_sink
        self.yaw_source = yaw_source
        self.safety_action = "NONE"
        self.pending_resume_authority: str | None = None
        self.state = "NORMAL"
        self.reason = "NONE"
        self.last_heartbeat_time_s: float | None = None
        self.last_heartbeat_sequence_id: int | None = None
        self.pending_safety: SafetyCommandPacket | None = None
        self.last_safety_sequence_id: int | None = None
        self.last_resume_sequence_id: int | None = None
        self.pending_resume_epoch: int | None = None
        self.resume_start_tick: int | None = None
        self.invalidated_epoch: int | None = None
        self.health = "PI_OK"
        self.local_v_hat_m_s = 0.0
        self.local_pitch_hat_rad = nominal_pitch_rad
        self.local_pitch_rate_hat_rad_s = 0.0
        self.last_applied: WireReferenceCommand | None = None
        self.brake: SafetyBrakePrimitive | None = None
        self.events: list[dict] = []
        self.status_sequence_id = 0
        self.first_takeover_deltas: dict | None = None

    def set_local_state(self, *, v_hat_m_s: float, pitch_hat_rad: float,
                        pitch_rate_hat_rad_s: float) -> None:
        self.local_v_hat_m_s = float(v_hat_m_s)
        self.local_pitch_hat_rad = float(pitch_hat_rad)
        self.local_pitch_rate_hat_rad_s = float(pitch_rate_hat_rad_s)

    def receive(self, channel: str, payload: bytes, time_s: float) -> None:
        if channel == "reference":
            self.receiver.receive(payload, time_s)
        elif channel == "yaw":
            if self.yaw_source is None:
                raise ValueError("yaw channel needs an MCU yaw source")
            self.yaw_source.receive(payload, int(round(time_s / self.receiver.dt_s)))
        elif channel == "heartbeat":
            packet = HeartbeatPacket.from_bytes(payload)
            if (self.last_heartbeat_sequence_id is not None
                    and packet.sequence_id <= self.last_heartbeat_sequence_id):
                return
            self.last_heartbeat_sequence_id = packet.sequence_id
            self.last_heartbeat_time_s = time_s
            if self.health == "PI_LOST":
                self.health = "PI_OK"
                self.events.append({"event": "pi_heartbeat_restored",
                                    "time_s": time_s})
                self._publish_status(time_s)
        elif channel == "safety":
            packet = SafetyCommandPacket.from_bytes(payload)
            if (self.last_safety_sequence_id is not None
                    and packet.sequence_id <= self.last_safety_sequence_id):
                return
            self.last_safety_sequence_id = packet.sequence_id
            self.events.append({"event": "safety_received", "time_s": time_s,
                                "issued_time_s": packet.issued_time_s,
                                "action": packet.action, "sequence_id": packet.sequence_id})
            if self.state != "NORMAL":
                if (packet.action in ("HARD_STOP", "BRAKE")
                        and self.safety_action == "LONGITUDINAL_STOP"):
                    self.safety_action = "HARD_STOP"
                    self.reason = packet.reason
                    self.pending_resume_epoch = None
                    self.resume_start_tick = None
                    if self.yaw_source is not None:
                        self.yaw_source.hard_stop(
                            int(round(time_s / self.receiver.dt_s)))
                    if self.state == "LONGITUDINAL_STOP":
                        self.state = "SAFE_HOLD"
                    self.events.append({"event": "hard_stop_escalated",
                                        "time_s": time_s, "reason": packet.reason})
                    self._publish_status(time_s)
                    return
                self.events.append({"event": "safety_command_ignored_while_safe",
                                    "time_s": time_s, "state": self.state,
                                    "sequence_id": packet.sequence_id})
                return
            self.pending_safety = packet
        elif channel == "resume":
            packet = ResumeReferenceCommandPacket.from_bytes(payload)
            if (self.last_resume_sequence_id is not None
                    and packet.sequence_id <= self.last_resume_sequence_id):
                return
            self.last_resume_sequence_id = packet.sequence_id
            expected_state = ("LONGITUDINAL_STOP" if packet.authority == "LONGITUDINAL"
                              else "SAFE_HOLD")
            if (self.state != expected_state
                    or packet.reference_epoch != self.receiver.reference_epoch
                    or (packet.authority == "HARD" and self.yaw_source is not None
                        and not self.yaw_source.fresh_for_rearm(
                            int(round(time_s / self.receiver.dt_s))))):
                self.events.append({"event": "resume_rejected", "time_s": time_s,
                                    "reference_epoch": packet.reference_epoch,
                                    "state": self.state})
                return
            self.pending_resume_epoch = packet.reference_epoch
            self.pending_resume_authority = packet.authority
            self.events.append({"event": "resume_requested", "time_s": time_s,
                                "reference_epoch": packet.reference_epoch})
        else:
            raise ValueError(f"unknown MCU channel {channel}")

    def _publish_status(self, time_s: float) -> None:
        packet = SafetyStatusPacket(
            PROTOCOL_VERSION, self.status_sequence_id, time_s, self.state,
            self.reason, self.receiver.reference_epoch,
            self.invalidated_epoch, self.health,
        )
        self.status_sequence_id += 1
        self.status_sink(packet.to_bytes(), time_s)

    def _trigger(self, time_s: float, reason: str,
                 issued_time_s: float | None = None,
                 action: str = "HARD_STOP") -> None:
        if self.state != "NORMAL":
            return
        before = self.last_applied
        initial = (SafetyReference(
            float(before.reference_state[0]), float(before.reference_state[1]),
            float(before.reference_acceleration_m_s2),
            float(before.reference_state[2]), float(before.reference_state[3]),
            float(before.u_ff_after_lifecycle_nm),
        ) if before is not None else SafetyReference(0, 0, 0, 0, 0, 0))
        self.brake = SafetyBrakePrimitive(self.brake_config, initial)
        old_epoch = self.receiver.reference_epoch
        active_sequence_id = (None if self.receiver.active is None
                              else self.receiver.active.sequence_id)
        next_cached = self.receiver.next_ready()
        buffer_remaining_s = self.receiver.remaining_ticks(
            int(round(time_s / self.receiver.dt_s))
        ) * self.receiver.dt_s
        self.invalidated_epoch = self.receiver.invalidate_current_epoch()
        self.pending_resume_epoch = None
        self.pending_resume_authority = None
        self.resume_start_tick = None
        self.state = "BRAKING"
        self.safety_action = "LONGITUDINAL_STOP" if action == "LONGITUDINAL_STOP" else "HARD_STOP"
        if self.safety_action == "HARD_STOP" and self.yaw_source is not None:
            self.yaw_source.hard_stop(int(round(time_s / self.receiver.dt_s)))
        self.reason = reason
        self.events.append({"event": "safety_takeover", "time_s": time_s,
                            "issued_time_s": issued_time_s,
                            "reason": reason, "action": self.safety_action,
                            "invalidated_reference_epoch": old_epoch,
                            "new_reference_epoch": self.receiver.reference_epoch,
                            "active_sequence_id": active_sequence_id,
                            "next_cached": next_cached,
                            "buffer_remaining_s": buffer_remaining_s,
                            "heartbeat_age_s": (None if self.last_heartbeat_time_s is None
                                                else time_s - self.last_heartbeat_time_s)})
        self._publish_status(time_s)

    def command(self, time_s: float) -> WireReferenceCommand:
        tick = int(round(time_s / self.receiver.dt_s))
        if (self.state in ("SAFE_HOLD", "LONGITUDINAL_STOP")
                and self.last_heartbeat_time_s is not None
                and time_s - self.last_heartbeat_time_s
                > self.heartbeat_timeout_s + 1e-12
                and self.health != "PI_LOST"):
            self.health = "PI_LOST"
            self.events.append({"event": "pi_lost_in_safe_hold", "time_s": time_s})
            if self.state == "LONGITUDINAL_STOP":
                self.state = "SAFE_HOLD"
                self.safety_action = "HARD_STOP"
                if self.yaw_source is not None:
                    self.yaw_source.hard_stop(tick)
            self._publish_status(time_s)
        if self.state in ("SAFE_HOLD", "LONGITUDINAL_STOP"):
            if (self.pending_resume_epoch == self.receiver.reference_epoch
                    and self.resume_start_tick is None
                    and self.receiver.resume_block_ready(tick)
                    and (self.pending_resume_authority != "HARD"
                         or self.yaw_source is None
                         or self.yaw_source.fresh_for_rearm(tick))):
                assert self.receiver.next is not None
                self.resume_start_tick = self.receiver.next.start_control_tick
                self.events.append({"event": "resume_armed", "time_s": time_s,
                                    "start_control_tick": self.resume_start_tick})
            if (self.resume_start_tick == tick
                    and (self.pending_resume_authority != "HARD"
                         or self.yaw_source is None
                         or self.yaw_source.fresh_for_rearm(tick))):
                assert self.receiver.next is not None
                assert self.last_applied is not None
                resume_p, resume_v, *_ = self.receiver.next.sample(0)
                self.events.append({"event": "resume_reference_continuity",
                                    "time_s": time_s,
                                    "p_ref_jump_m": resume_p-float(
                                        self.last_applied.reference_state[0]),
                                    "v_ref_jump_m_s": resume_v-float(
                                        self.last_applied.reference_state[1])})
                self.receiver.activate_resume(tick)
                self.state = "NORMAL"
                self.reason = "NONE"
                self.safety_action = "NONE"
                self.pending_resume_epoch = None
                self.pending_resume_authority = None
                self.resume_start_tick = None
                self.events.append({"event": "normal_resumed", "time_s": time_s,
                                    "reference_epoch": self.receiver.reference_epoch})
                self._publish_status(time_s)
            elif (self.receiver.next is not None
                  and self.receiver.next.start_control_tick <= tick):
                self.events.append({"event": "expired_resume_block_dropped",
                                    "time_s": time_s,
                                    "sequence_id": self.receiver.next.sequence_id})
                self.receiver.next = None
                self.resume_start_tick = None
        if self.state == "NORMAL":
            if self.pending_safety is not None:
                packet = self.pending_safety
                self.pending_safety = None
                self._trigger(time_s, packet.reason, packet.issued_time_s,
                              packet.action)
            elif (self.last_heartbeat_time_s is not None
                  and time_s - self.last_heartbeat_time_s
                  > self.heartbeat_timeout_s + 1e-12):
                self._trigger(time_s, "HEARTBEAT_TIMEOUT")
            else:
                self.receiver.ensure_active(tick)
                if (self.receiver.remaining_ticks(tick) <= self.starvation_margin_ticks
                        and not self.receiver.next_ready()):
                    self._trigger(time_s, "REFERENCE_STARVATION_IMMINENT")
        if self.state == "NORMAL":
            command = self.receiver.command(tick)
            if self.yaw_source is not None:
                command = replace(command, yaw_rate_target_rad_s=
                                  self.yaw_source.sample(tick, hard_stopped=False))
            self.last_applied = command
            return command

        assert self.brake is not None
        if self.state in ("SAFE_HOLD", "LONGITUDINAL_STOP"):
            assert self.last_applied is not None
            safe = SafetyReference(float(self.last_applied.reference_state[0]),
                                   0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            safe = self.brake.step()
        if (self.state == "BRAKING" and self.brake.hidden_handoff_done
                and self.brake.velocity_settled
                and abs(self.local_v_hat_m_s) <= 0.03
                and abs(self.local_pitch_hat_rad - self.nominal_pitch_rad) <= 0.08
                and abs(self.local_pitch_rate_hat_rad_s) <= 0.20):
            self.state = ("LONGITUDINAL_STOP" if self.safety_action == "LONGITUDINAL_STOP"
                          else "SAFE_HOLD")
            safe = SafetyReference(safe.p_m, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.events.append({"event": "safe_hold", "time_s": time_s,
                                "reason": self.reason, "state": self.state})
            self._publish_status(time_s)
        if self.state in ("SAFE_HOLD", "LONGITUDINAL_STOP"):
            safe = SafetyReference(safe.p_m, 0.0, 0.0, 0.0, 0.0, 0.0)
        yaw_target = (0.0 if self.yaw_source is None else self.yaw_source.sample(
            tick, hard_stopped=self.safety_action == "HARD_STOP"))
        command = WireReferenceCommand(
            np.asarray([safe.p_m, safe.v_m_s, safe.theta_rad,
                        safe.theta_dot_rad_s], dtype=float),
            safe.a_m_s2, safe.u_ff_nm, safe.u_ff_nm,
            safe.v_m_s, yaw_target, "SAFETY", self.state, 1.0,
            -1, self.brake.step_index - 1, time_s,
            time_s, planning_path="SAFETY",
        )
        if self.first_takeover_deltas is None:
            before = self.last_applied
            if before is not None:
                self.first_takeover_deltas = {
                    "v_ref_m_s": safe.v_m_s - float(before.reference_state[1]),
                    "a_ref_m_s2": safe.a_m_s2 - before.reference_acceleration_m_s2,
                    "theta_ref_rad": safe.theta_rad - float(before.reference_state[2]),
                    "theta_dot_ref_rad_s": safe.theta_dot_rad_s
                    - float(before.reference_state[3]),
                    "u_ff_nm": safe.u_ff_nm - before.u_ff_after_lifecycle_nm,
                }
        self.last_applied = command
        return command

    @property
    def next_reference_state(self) -> np.ndarray:
        if self.state != "NORMAL":
            assert self.last_applied is not None
            return self.last_applied.reference_state.copy()
        return self.receiver.peek_next()[0]

    @property
    def next_reference_acceleration_m_s2(self) -> float:
        if self.state != "NORMAL":
            assert self.last_applied is not None
            return self.last_applied.reference_acceleration_m_s2
        return self.receiver.peek_next()[1]

