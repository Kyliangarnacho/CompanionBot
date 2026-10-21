"""Stage 6 packet contracts. Every endpoint receives decoded bytes, not objects."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
import struct


PROTOCOL_VERSION = 1
_REFERENCE_HEADER = struct.Struct("<IIIIddIf")
_REFERENCE_SAMPLE = struct.Struct("<6f")


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class _JsonPacket:
    def to_bytes(self) -> bytes:
        return json.dumps(
            {"packet_type": type(self).__name__, **asdict(self)},
            allow_nan=False, separators=(",", ":"),
        ).encode("utf-8")

    @classmethod
    def from_bytes(cls, payload: bytes):
        data = json.loads(payload.decode("utf-8"))
        if data.pop("packet_type", None) != cls.__name__:
            raise ValueError(f"expected {cls.__name__} packet")
        return cls(**data)


@dataclass(frozen=True)
class TargetObservationPacket(_JsonPacket):
    protocol_version: int
    sequence_id: int
    capture_time_s: float
    x_forward_m: float
    y_left_m: float
    valid: bool
    confidence: float

    def __post_init__(self) -> None:
        if self.protocol_version != PROTOCOL_VERSION or self.sequence_id < 0:
            raise ValueError("invalid target observation packet version/sequence")
        if not _finite(self.capture_time_s, self.x_forward_m,
                       self.y_left_m, self.confidence):
            raise ValueError("non-finite target observation packet")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("target confidence outside [0, 1]")


@dataclass(frozen=True)
class RobotStatePacket(_JsonPacket):
    protocol_version: int
    sequence_id: int
    sample_time_s: float
    v_hat_m_s: float
    yaw_hat_rad: float
    yaw_rate_hat_rad_s: float
    safety_state: str
    applied_p_ref_m: float | None = None

    def __post_init__(self) -> None:
        if self.protocol_version != PROTOCOL_VERSION or self.sequence_id < 0:
            raise ValueError("invalid robot state packet version/sequence")
        if (not _finite(self.sample_time_s, self.v_hat_m_s,
                        self.yaw_hat_rad, self.yaw_rate_hat_rad_s)
                or (self.applied_p_ref_m is not None
                    and not _finite(self.applied_p_ref_m))):
            raise ValueError("non-finite robot state packet")
        if self.safety_state not in (
            "NORMAL", "BRAKING", "LONGITUDINAL_STOP", "SAFE_HOLD"
        ):
            raise ValueError("invalid robot safety state")


@dataclass(frozen=True)
class ReferenceBlockPacket:
    """Immutable float32 wire block: p, v, a, theta, theta_dot, applied u_ff."""

    protocol_version: int
    reference_epoch: int
    sequence_id: int
    start_control_tick: int
    generated_time_s: float
    dt_s: float
    sample_count: int
    yaw_rate_target_rad_s: float
    sample_bytes: bytes

    @property
    def longitudinal_epoch(self) -> int:
        return self.reference_epoch

    def __post_init__(self) -> None:
        if self.protocol_version != PROTOCOL_VERSION:
            raise ValueError("invalid reference packet version")
        if min(self.reference_epoch, self.sequence_id,
               self.start_control_tick) < 0 or self.sample_count < 1:
            raise ValueError("invalid reference packet index/count")
        if not _finite(self.generated_time_s, self.dt_s,
                       self.yaw_rate_target_rad_s) or self.dt_s <= 0.0:
            raise ValueError("invalid reference packet timing/yaw")
        if len(self.sample_bytes) != self.sample_count * _REFERENCE_SAMPLE.size:
            raise ValueError("reference packet sample byte count mismatch")
        if any(
            not _finite(*_REFERENCE_SAMPLE.unpack_from(
                self.sample_bytes, index * _REFERENCE_SAMPLE.size
            ))
            for index in range(self.sample_count)
        ):
            raise ValueError("non-finite reference packet sample")

    @classmethod
    def from_rows(cls, *, reference_epoch: int, sequence_id: int,
                  start_control_tick: int, generated_time_s: float,
                  dt_s: float, yaw_rate_target_rad_s: float,
                  rows: list[tuple[float, float, float, float, float, float]]):
        if not rows or any(not _finite(*row) for row in rows):
            raise ValueError("reference rows must be finite and nonempty")
        return cls(
            PROTOCOL_VERSION, reference_epoch, sequence_id, start_control_tick,
            generated_time_s, dt_s, len(rows), yaw_rate_target_rad_s,
            b"".join(_REFERENCE_SAMPLE.pack(*row) for row in rows),
        )

    def to_bytes(self) -> bytes:
        return _REFERENCE_HEADER.pack(
            self.protocol_version, self.reference_epoch, self.sequence_id,
            self.start_control_tick, self.generated_time_s, self.dt_s,
            self.sample_count, self.yaw_rate_target_rad_s,
        ) + self.sample_bytes

    @classmethod
    def from_bytes(cls, payload: bytes):
        if len(payload) < _REFERENCE_HEADER.size:
            raise ValueError("truncated reference packet")
        return cls(*_REFERENCE_HEADER.unpack_from(payload),
                   bytes(payload[_REFERENCE_HEADER.size:]))

    def sample(self, index: int) -> tuple[float, float, float, float, float, float]:
        if not 0 <= index < self.sample_count:
            raise IndexError("reference sample index out of range")
        return _REFERENCE_SAMPLE.unpack_from(
            self.sample_bytes, index * _REFERENCE_SAMPLE.size
        )


@dataclass(frozen=True)
class SafetyCommandPacket(_JsonPacket):
    protocol_version: int
    sequence_id: int
    issued_time_s: float
    action: str
    reason: str

    def __post_init__(self) -> None:
        if self.protocol_version != PROTOCOL_VERSION or self.sequence_id < 0:
            raise ValueError("invalid safety command version/sequence")
        if (not _finite(self.issued_time_s)
                or self.action not in (
                    "BRAKE", "LONGITUDINAL_STOP", "HARD_STOP"
                )):
            raise ValueError("unsupported safety command")
        if not self.reason:
            raise ValueError("safety command reason is required")


@dataclass(frozen=True)
class HeartbeatPacket(_JsonPacket):
    protocol_version: int
    sequence_id: int
    timestamp_s: float

    def __post_init__(self) -> None:
        if (self.protocol_version != PROTOCOL_VERSION or self.sequence_id < 0
                or not _finite(self.timestamp_s)):
            raise ValueError("invalid heartbeat packet")


@dataclass(frozen=True)
class SafetyStatusPacket(_JsonPacket):
    protocol_version: int
    sequence_id: int
    timestamp_s: float
    state: str
    reason: str
    reference_epoch: int
    invalidated_epoch: int | None = None
    health: str = "PI_OK"

    def __post_init__(self) -> None:
        if (self.protocol_version != PROTOCOL_VERSION or self.sequence_id < 0
                or self.reference_epoch < 0 or not _finite(self.timestamp_s)):
            raise ValueError("invalid safety status packet")
        if self.state not in (
            "NORMAL", "BRAKING", "LONGITUDINAL_STOP", "SAFE_HOLD"
        ):
            raise ValueError("invalid safety status state")
        if self.invalidated_epoch is not None and (
            self.invalidated_epoch < 0
            or self.invalidated_epoch >= self.reference_epoch
        ):
            raise ValueError("invalidated epoch must precede current epoch")
        if self.health not in ("PI_OK", "PI_LOST"):
            raise ValueError("invalid safety health")

    @property
    def invalidated_longitudinal_epoch(self) -> int | None:
        return self.invalidated_epoch


@dataclass(frozen=True)
class ResumeReferenceCommandPacket(_JsonPacket):
    protocol_version: int
    sequence_id: int
    issued_time_s: float
    reference_epoch: int
    authority: str = "HARD"

    def __post_init__(self) -> None:
        if (self.protocol_version != PROTOCOL_VERSION or self.sequence_id < 0
                or self.reference_epoch < 1 or not _finite(self.issued_time_s)):
            raise ValueError("invalid resume reference command")
        if self.authority not in ("LONGITUDINAL", "HARD"):
            raise ValueError("invalid resume authority")


@dataclass(frozen=True)
class YawCommandPacket(_JsonPacket):
    protocol_version: int
    sequence_id: int
    source_time_s: float
    yaw_rate_target_rad_s: float

    def __post_init__(self) -> None:
        if (self.protocol_version != PROTOCOL_VERSION or self.sequence_id < 0
                or not _finite(self.source_time_s,
                               self.yaw_rate_target_rad_s)):
            raise ValueError("invalid yaw command packet")
