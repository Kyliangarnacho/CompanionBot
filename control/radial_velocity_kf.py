"""Pi-side radial target estimation from timestamped observation/state packets."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .pi_mcu_packets import RobotStatePacket, TargetObservationPacket
from .rolling_reference import TargetObservation


@dataclass(frozen=True)
class RadialVelocityKFConfig:
    initial_distance_std: float
    initial_velocity_std: float
    measurement_distance_std: float
    master_accel_std: float

    def __post_init__(self) -> None:
        values = (self.initial_distance_std, self.initial_velocity_std,
                  self.measurement_distance_std, self.master_accel_std)
        if any(not math.isfinite(v) or v <= 0.0 for v in values):
            raise ValueError("KF standard deviations must be finite and positive")


@dataclass(frozen=True)
class RadialEstimate:
    sequence_id: int
    capture_time_s: float
    distance_hat_m: float
    master_radial_velocity_hat_m_s: float
    aligned_robot_velocity_m_s: float
    beta_rad: float
    innovation_m: float


class RadialVelocityKalmanFilter:
    """Linear [distance, Master radial velocity] filter with known ego input."""

    def __init__(self, config: RadialVelocityKFConfig) -> None:
        self.config = config
        self.x: np.ndarray | None = None
        self.P: np.ndarray | None = None
        self.last_capture_time_s: float | None = None
        self.innovation_m = 0.0

    def update(self, *, capture_time_s: float, distance_meas_m: float,
               robot_radial_velocity_m_s: float) -> tuple[float, float, float]:
        t = float(capture_time_s)
        z = float(distance_meas_m)
        u = float(robot_radial_velocity_m_s)
        if not all(math.isfinite(x) for x in (t, z, u)) or z < 0.0:
            raise ValueError("KF input must be finite and distance nonnegative")
        if self.x is None:
            self.x = np.array([z, 0.0], dtype=float)
            self.P = np.diag([
                self.config.initial_distance_std**2,
                self.config.initial_velocity_std**2,
            ])
            self.last_capture_time_s = t
            self.innovation_m = 0.0
            return z, 0.0, 0.0
        assert self.P is not None and self.last_capture_time_s is not None
        dt = t-self.last_capture_time_s
        if dt <= 0.0:
            raise ValueError("KF capture timestamps must strictly increase")
        A = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        G = np.array([0.5*dt*dt, dt], dtype=float)
        x_prior = A@self.x + np.array([-dt*u, 0.0])
        P_prior = A@self.P@A.T + self.config.master_accel_std**2*np.outer(G, G)
        R = self.config.measurement_distance_std**2
        innovation = z-x_prior[0]
        gain = P_prior[:, 0]/(P_prior[0, 0]+R)
        self.x = x_prior+gain*innovation
        KH = np.outer(gain, np.array([1.0, 0.0]))
        I_KH = np.eye(2)-KH
        self.P = I_KH@P_prior@I_KH.T+R*np.outer(gain, gain)
        self.P = 0.5*(self.P+self.P.T)
        self.last_capture_time_s = t
        self.innovation_m = float(innovation)
        return float(self.x[0]), float(self.x[1]), float(innovation)


class PiRobotStateHistory:
    """Only decoded MCU RobotStatePacket values enter this interpolation buffer."""

    def __init__(self, *, retention_s: float = 2.0) -> None:
        if retention_s <= 0:
            raise ValueError("state retention must be positive")
        self.retention_s = float(retention_s)
        self.samples: deque[RobotStatePacket] = deque()
        self.events: list[dict] = []
        self.accepted_count = 0

    def accept(self, packet: RobotStatePacket) -> None:
        if self.samples and (packet.sequence_id <= self.samples[-1].sequence_id
                             or packet.sample_time_s <= self.samples[-1].sample_time_s):
            self.events.append({"event": "state_out_of_order_rejected",
                                "sequence_id": packet.sequence_id})
            return
        self.samples.append(packet)
        self.accepted_count += 1
        cutoff = packet.sample_time_s-self.retention_s
        while len(self.samples) > 2 and self.samples[1].sample_time_s < cutoff:
            self.samples.popleft()

    def velocity_at(self, capture_time_s: float) -> tuple[
            float, float, int, int, float, float, float] | None:
        t = float(capture_time_s)
        if not self.samples or t < self.samples[0].sample_time_s-1e-9:
            return None
        for left, right in zip(self.samples, list(self.samples)[1:]):
            if left.sample_time_s-1e-9 <= t <= right.sample_time_s+1e-9:
                alpha = max(0.0, min(1.0, (t-left.sample_time_s)
                                      /(right.sample_time_s-left.sample_time_s)))
                return ((1-alpha)*left.v_hat_m_s+alpha*right.v_hat_m_s,
                        t, left.sequence_id, right.sequence_id,
                        left.sample_time_s, right.sample_time_s, alpha)
        if abs(t-self.samples[-1].sample_time_s) <= 1e-9:
            last = self.samples[-1]
            return (last.v_hat_m_s, t, last.sequence_id, last.sequence_id,
                    t, t, 0.0)
        return None


class PiRadialObservationPipeline:
    """Packet-only observation alignment and KF; stores no simulator objects."""

    def __init__(self, kf: RadialVelocityKalmanFilter,
                 state_history: PiRobotStateHistory) -> None:
        self.kf = kf
        self.state_history = state_history
        self.latest_observation: TargetObservation | None = None
        self.estimates: dict[int, RadialEstimate] = {}
        self.rows: list[dict] = []
        self.last_raw_distance_m: float | None = None
        self.last_raw_time_s: float | None = None
        self.last_sequence_id = -1

    def accept_observation(self, packet: TargetObservationPacket,
                           arrival_time_s: float) -> TargetObservation | None:
        d_meas = math.hypot(packet.x_forward_m, packet.y_left_m)
        beta = math.atan2(packet.y_left_m, packet.x_forward_m)
        row = {"sequence_id": packet.sequence_id,
               "capture_time_s": packet.capture_time_s,
               "packet_arrival_time_s": float(arrival_time_s),
               "d_meas_m": d_meas, "beta_rad": beta,
               "confidence": packet.confidence,
               "aligned_robot_state_time_s": None,
               "aligned_v_robot_m_s": None,
               "kf_d_hat_m": None,
               "kf_v_master_radial_hat_m_s": None,
               "raw_radial_velocity_for_diagnostic_only_m_s": None,
               "kf_innovation_m": None, "governor_output_m_s": None,
               "alignment": "PENDING"}
        self.rows.append(row)
        if not packet.valid:
            row["alignment"] = "INVALID_OBSERVATION"
            return None
        if (packet.sequence_id <= self.last_sequence_id
                or (self.kf.last_capture_time_s is not None
                    and packet.capture_time_s <= self.kf.last_capture_time_s)):
            row["alignment"] = "OUT_OF_ORDER"
            return None
        aligned = self.state_history.velocity_at(packet.capture_time_s)
        if aligned is None:
            row["alignment"] = "STATE_HISTORY_MISS"
            return None
        (v_robot, aligned_time, left_seq, right_seq,
         left_time, right_time, alpha) = aligned
        u = v_robot*math.cos(beta)
        raw = None
        if self.last_raw_time_s is not None:
            raw = (d_meas-self.last_raw_distance_m)/(
                packet.capture_time_s-self.last_raw_time_s)+u
        d_hat, v_hat, innovation = self.kf.update(
            capture_time_s=packet.capture_time_s,
            distance_meas_m=d_meas, robot_radial_velocity_m_s=u)
        estimate = RadialEstimate(packet.sequence_id, packet.capture_time_s,
                                  d_hat, v_hat, v_robot, beta, innovation)
        self.estimates[packet.sequence_id] = estimate
        self.last_sequence_id = packet.sequence_id
        self.last_raw_distance_m = d_meas
        self.last_raw_time_s = packet.capture_time_s
        observation = TargetObservation(packet.capture_time_s,
            packet.x_forward_m, packet.y_left_m, version=packet.protocol_version,
            sequence_id=packet.sequence_id, valid=packet.valid,
            confidence=packet.confidence)
        self.latest_observation = observation
        row.update({"alignment": "INTERPOLATED" if left_seq != right_seq else "EXACT",
                    "aligned_robot_state_time_s": aligned_time,
                    "state_left_sequence_id": left_seq,
                    "state_right_sequence_id": right_seq,
                    "state_left_time_s": left_time,
                    "state_right_time_s": right_time,
                    "state_interp_alpha": alpha,
                    "aligned_v_robot_m_s": v_robot,
                    "latest_packet_v_robot_for_diagnostic_only_m_s":
                        self.state_history.samples[-1].v_hat_m_s,
                    "kf_d_hat_m": d_hat,
                    "kf_v_master_radial_hat_m_s": v_hat,
                    "raw_radial_velocity_for_diagnostic_only_m_s": raw,
                    "kf_innovation_m": innovation})
        return observation

    def estimate_for(self, observation: TargetObservation) -> RadialEstimate:
        return self.estimates[observation.sequence_id]

    def read(self) -> TargetObservation:
        if self.latest_observation is None:
            raise RuntimeError("no aligned observation available")
        return self.latest_observation
