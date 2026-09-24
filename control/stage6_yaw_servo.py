"""Thin bearing servo and MCU-local latest-value yaw reference."""

from __future__ import annotations

import math

from .pi_mcu_packets import YawCommandPacket


class BearingYawServo:
    def __init__(self, *, gain_s_inv: float, max_rate_rad_s: float,
                 deadband_rad: float = 0.0) -> None:
        if gain_s_inv <= 0 or max_rate_rad_s <= 0 or deadband_rad < 0:
            raise ValueError("invalid bearing servo parameters")
        self.gain_s_inv = float(gain_s_inv)
        self.max_rate_rad_s = float(max_rate_rad_s)
        self.deadband_rad = float(deadband_rad)

    def command(self, x_forward_m: float, y_left_m: float) -> tuple[float, float]:
        beta = math.atan2(y_left_m, x_forward_m)
        target = 0.0 if abs(beta) <= self.deadband_rad else self.gain_s_inv * beta
        return beta, max(-self.max_rate_rad_s,
                         min(target, self.max_rate_rad_s))


class MCULatestYawSource:
    def __init__(self, *, dt_s: float, timeout_s: float,
                 decay_rate_rad_s2: float) -> None:
        if min(dt_s, timeout_s, decay_rate_rad_s2) <= 0:
            raise ValueError("invalid yaw freshness parameters")
        self.dt_s = float(dt_s)
        self.timeout_ticks = math.ceil(timeout_s / dt_s)
        self.decay_step = float(decay_rate_rad_s2) * dt_s
        self.latest: YawCommandPacket | None = None
        self.receive_tick: int | None = None
        self.last_sequence_id: int | None = None
        self.output_rad_s = 0.0
        self.source = "LOCAL_YAW_TO_ZERO"
        self.events: list[dict] = []
        self.hard_rearm_after_tick: int | None = None
        self.max_age_s = 0.0

    def receive(self, payload: bytes, tick: int) -> None:
        packet = YawCommandPacket.from_bytes(payload)
        if self.last_sequence_id is not None and packet.sequence_id <= self.last_sequence_id:
            self.events.append({"event": "yaw_out_of_order_rejected", "time_s": tick*self.dt_s,
                                "sequence_id": packet.sequence_id})
            return
        self.latest = packet
        self.last_sequence_id = packet.sequence_id
        self.receive_tick = tick
        self.events.append({"event": "yaw_received", "time_s": tick*self.dt_s,
                            "sequence_id": packet.sequence_id,
                            "source_time_s": packet.source_time_s,
                            "receive_tick": tick})

    def hard_stop(self, tick: int) -> None:
        self.hard_rearm_after_tick = tick
        self.events.append({"event": "yaw_hard_disarmed", "time_s": tick*self.dt_s})

    def fresh_for_rearm(self, tick: int) -> bool:
        return (self.receive_tick is not None
                and self.hard_rearm_after_tick is not None
                and self.receive_tick > self.hard_rearm_after_tick
                and tick - self.receive_tick <= self.timeout_ticks)

    def sample(self, tick: int, *, hard_stopped: bool) -> float:
        fresh = (self.latest is not None and self.receive_tick is not None
                 and tick - self.receive_tick <= self.timeout_ticks)
        if self.receive_tick is not None:
            self.max_age_s = max(self.max_age_s,
                                 (tick-self.receive_tick)*self.dt_s)
        if hard_stopped or not fresh:
            target = 0.0
            source = "LOCAL_YAW_TO_ZERO"
        else:
            target = self.latest.yaw_rate_target_rad_s
            source = "LATEST_PI_YAW"
        if source == "LOCAL_YAW_TO_ZERO":
            delta = max(-self.decay_step, min(-self.output_rad_s, self.decay_step))
            self.output_rad_s += delta
        else:
            self.output_rad_s = target
        if source != self.source:
            self.events.append({"event": "yaw_source_changed", "time_s": tick*self.dt_s,
                                "source": source, "sequence_id": self.last_sequence_id})
        self.source = source
        if fresh and source == "LATEST_PI_YAW" and tick == self.receive_tick:
            self.events.append({"event": "yaw_applied", "time_s": tick*self.dt_s,
                                "apply_tick": tick,
                                "sequence_id": self.latest.sequence_id,
                                "source_time_s": self.latest.source_time_s,
                                "receive_time_s": self.receive_tick*self.dt_s})
        return self.output_rad_s
