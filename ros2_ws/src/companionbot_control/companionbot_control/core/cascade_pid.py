"""Deterministic cascade PID baseline for longitudinal MiniSegway balance."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class CascadePIDConfig:
    controller_frequency_hz: float
    position_kp_rad_per_m: float
    position_ki_rad_per_m_s: float
    velocity_kd_rad_per_m_s: float
    position_integral_limit_m_s: float
    theta_reference_limit_rad: float
    pitch_kp_nm_per_rad: float
    pitch_ki_nm_per_rad_s: float
    pitch_kd_nm_per_rad_s: float
    pitch_integral_limit_rad_s: float
    sum_torque_limit_nm: float

    @property
    def controller_dt_s(self) -> float:
        return 1.0 / self.controller_frequency_hz


@dataclass(frozen=True)
class CascadePIDCommand:
    theta_reference_rad: float
    raw_theta_reference_rad: float
    raw_sum_torque_nm: float
    limited_sum_torque_nm: float
    left_torque_nm: float
    right_torque_nm: float
    torque_saturated: bool
    theta_reference_saturated: bool
    position_integral_m_s: float
    pitch_integral_rad_s: float
    outer_antiwindup_blocked: bool
    inner_antiwindup_blocked: bool


def load_config(path: str | Path) -> CascadePIDConfig:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    gains = raw["gains"]
    limits = raw["limits"]
    return CascadePIDConfig(
        controller_frequency_hz=float(raw["controller_frequency_hz"]),
        position_kp_rad_per_m=float(gains["position_kp_rad_per_m"]),
        position_ki_rad_per_m_s=float(gains["position_ki_rad_per_m_s"]),
        velocity_kd_rad_per_m_s=float(gains["velocity_kd_rad_per_m_s"]),
        position_integral_limit_m_s=float(limits["position_integral_m_s"]),
        theta_reference_limit_rad=np.deg2rad(float(limits["theta_reference_deg"])),
        pitch_kp_nm_per_rad=float(gains["pitch_kp_nm_per_rad"]),
        pitch_ki_nm_per_rad_s=float(gains["pitch_ki_nm_per_rad_s"]),
        pitch_kd_nm_per_rad_s=float(gains["pitch_kd_nm_per_rad_s"]),
        pitch_integral_limit_rad_s=float(limits["pitch_integral_rad_s"]),
        sum_torque_limit_nm=float(limits["sum_torque_nm"]),
    )


def _conditional_integral(
    old_integral: float,
    error: float,
    dt: float,
    integral_limit: float,
    proportional_derivative_term: float,
    integral_gain: float,
    output_limit: float,
) -> tuple[float, float, float, bool]:
    """Integrate unless saturation and the proposed integral drives farther into it."""

    candidate = float(np.clip(old_integral + error * dt, -integral_limit, integral_limit))
    candidate_raw = proportional_derivative_term + integral_gain * candidate
    candidate_limited = float(np.clip(candidate_raw, -output_limit, output_limit))
    drives_farther = (
        not np.isclose(candidate_raw, candidate_limited, rtol=0.0, atol=1e-12)
        and np.sign(integral_gain * error) == np.sign(candidate_raw)
    )
    if drives_farther:
        raw = proportional_derivative_term + integral_gain * old_integral
        return old_integral, raw, float(np.clip(raw, -output_limit, output_limit)), True
    return candidate, candidate_raw, candidate_limited, False


class CascadePID:
    """Outer position/velocity loop plus inner pitch/pitch-rate loop.

    The reduced-model pitch convention is positive nose-down. Positive summed
    wheel torque creates negative pitch acceleration, so the inner feedback
    gains enter with positive signs.
    """

    def __init__(self, config: CascadePIDConfig):
        self.config = config
        self.position_integral_m_s = 0.0
        self.pitch_integral_rad_s = 0.0

    def reset(self) -> None:
        self.position_integral_m_s = 0.0
        self.pitch_integral_rad_s = 0.0

    def command(self, state: NDArray[np.float64]) -> CascadePIDCommand:
        position_m, velocity_m_s, pitch_error_rad, pitch_rate_rad_s = np.asarray(state, dtype=float)
        cfg = self.config

        position_error_m = -float(position_m)
        outer_pd = (
            cfg.position_kp_rad_per_m * position_error_m
            - cfg.velocity_kd_rad_per_m_s * float(velocity_m_s)
        )
        (
            self.position_integral_m_s,
            raw_theta_reference,
            theta_reference,
            outer_blocked,
        ) = _conditional_integral(
            self.position_integral_m_s,
            position_error_m,
            cfg.controller_dt_s,
            cfg.position_integral_limit_m_s,
            outer_pd,
            cfg.position_ki_rad_per_m_s,
            cfg.theta_reference_limit_rad,
        )

        pitch_tracking_error = float(pitch_error_rad) - theta_reference
        inner_pd = (
            cfg.pitch_kp_nm_per_rad * pitch_tracking_error
            + cfg.pitch_kd_nm_per_rad_s * float(pitch_rate_rad_s)
        )
        (
            self.pitch_integral_rad_s,
            raw_sum_torque,
            limited_sum_torque,
            inner_blocked,
        ) = _conditional_integral(
            self.pitch_integral_rad_s,
            pitch_tracking_error,
            cfg.controller_dt_s,
            cfg.pitch_integral_limit_rad_s,
            inner_pd,
            cfg.pitch_ki_nm_per_rad_s,
            cfg.sum_torque_limit_nm,
        )

        wheel_torque = limited_sum_torque / 2.0
        return CascadePIDCommand(
            theta_reference_rad=theta_reference,
            raw_theta_reference_rad=raw_theta_reference,
            raw_sum_torque_nm=raw_sum_torque,
            limited_sum_torque_nm=limited_sum_torque,
            left_torque_nm=wheel_torque,
            right_torque_nm=wheel_torque,
            torque_saturated=not np.isclose(raw_sum_torque, limited_sum_torque, rtol=0.0, atol=1e-12),
            theta_reference_saturated=not np.isclose(
                raw_theta_reference, theta_reference, rtol=0.0, atol=1e-12
            ),
            position_integral_m_s=self.position_integral_m_s,
            pitch_integral_rad_s=self.pitch_integral_rad_s,
            outer_antiwindup_blocked=outer_blocked,
            inner_antiwindup_blocked=inner_blocked,
        )
