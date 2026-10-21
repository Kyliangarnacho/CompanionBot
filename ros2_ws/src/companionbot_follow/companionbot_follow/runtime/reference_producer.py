"""Pi reference producer; consumes an existing FULL/reference source."""
from __future__ import annotations
from dataclasses import dataclass, replace
import math
import time
from typing import Callable
import numpy as np
from companionbot_contracts.packets import (PROTOCOL_VERSION, HeartbeatPacket, ReferenceBlockPacket, ResumeReferenceCommandPacket, RobotStatePacket, SafetyCommandPacket, SafetyStatusPacket, YawCommandPacket)

class PiReferenceBlockProducer:
    """Pi-only adapter around the existing Stage 5 FULL source."""

    def __init__(self, source, *, reference_epoch: int = 0) -> None:
        self.source = source
        self.reference_epoch = reference_epoch
        self.dt_s = float(source.dt_s)
        self.block_samples = int(source.block_samples)
        self.next_sequence_id = 0
        self.events: list[dict] = []

    def produce(self, start_tick: int, generated_time_s: float,
                *, observe_command: bool = True) -> tuple[ReferenceBlockPacket, float]:
        before = time.perf_counter()
        start_time_s = start_tick * self.dt_s
        commands = [
            self.source.command(
                start_time_s + index * self.dt_s,
                observe_command=(observe_command if index == 0 else True),
                planning_request_time_s=(generated_time_s if index == 0 else None),
            )
            for index in range(self.block_samples)
        ]
        compute_time_s = time.perf_counter() - before
        rows = [
            (float(command.reference_state[0]),
             float(command.reference_state[1]),
             float(command.reference_acceleration_m_s2),
             float(command.reference_state[2]),
             float(command.reference_state[3]),
             float(command.u_ff_after_lifecycle_nm))
            for command in commands
        ]
        packet = ReferenceBlockPacket.from_rows(
            reference_epoch=self.reference_epoch,
            sequence_id=self.next_sequence_id,
            start_control_tick=start_tick,
            generated_time_s=generated_time_s,
            dt_s=self.dt_s,
            yaw_rate_target_rad_s=float(commands[0].yaw_rate_target_rad_s),
            rows=rows,
        )
        self.next_sequence_id += 1
        self.events.append({
            "event": "block_generated", "sequence_id": packet.sequence_id,
            "start_control_tick": start_tick, "generated_time_s": generated_time_s,
            "compute_time_s": compute_time_s,
        })
        return packet, compute_time_s

