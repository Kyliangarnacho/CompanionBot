"""Minimal relative-yaw estimation, PD control, and torque allocation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def wrap_to_pi(angle_rad: float) -> float:
    """Wrap one finite angle to ``[-pi, pi)``."""

    angle = float(angle_rad)
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class RelativeYawEstimate:
    psi_measurement_rad: float
    psi_control_rad: float
    r_hat_rad_s: float
    sample_time_s: float
    measurement_age_s: float
    integrated_new_sample: bool


class GyroRelativeYawEstimator:
    """Integrate startup-bias-corrected gyro-z at packet timestamps only."""

    _TIME_TOLERANCE_S = 1e-12

    def __init__(self) -> None:
        self._initialized = False
        self._psi_measurement_rad = 0.0
        self._r_hat_rad_s = 0.0
        self._last_sample_time_s = 0.0

    def reset(
        self,
        r_hat_rad_s: float,
        sample_time_s: float,
        measurement_age_s: float,
        *,
        allow_extrapolation: bool,
    ) -> RelativeYawEstimate:
        self._validate(r_hat_rad_s, sample_time_s, measurement_age_s)
        self._initialized = True
        self._psi_measurement_rad = 0.0
        self._r_hat_rad_s = float(r_hat_rad_s)
        self._last_sample_time_s = float(sample_time_s)
        return self._estimate(
            float(measurement_age_s), allow_extrapolation, False
        )

    def update(
        self,
        r_hat_rad_s: float,
        sample_time_s: float,
        measurement_age_s: float,
        *,
        valid_new_sample: bool,
        allow_extrapolation: bool,
    ) -> RelativeYawEstimate:
        if not self._initialized:
            raise RuntimeError("yaw estimator must be reset first")
        self._validate(r_hat_rad_s, sample_time_s, measurement_age_s)
        sample_time = float(sample_time_s)
        integrated = False
        if valid_new_sample:
            delta_t = sample_time - self._last_sample_time_s
            if delta_t < -self._TIME_TOLERANCE_S:
                raise ValueError("IMU sample timestamp moved backwards")
            if delta_t > self._TIME_TOLERANCE_S:
                self._r_hat_rad_s = float(r_hat_rad_s)
                self._psi_measurement_rad += self._r_hat_rad_s * delta_t
                self._last_sample_time_s = sample_time
                integrated = True
        return self._estimate(
            float(measurement_age_s),
            allow_extrapolation and valid_new_sample,
            integrated,
        )

    @staticmethod
    def _validate(r_hat_rad_s: float, sample_time_s: float, age_s: float) -> None:
        values = (float(r_hat_rad_s), float(sample_time_s), float(age_s))
        if not all(math.isfinite(value) for value in values) or age_s < 0.0:
            raise ValueError("yaw IMU inputs must be finite and age non-negative")

    def _estimate(
        self, age_s: float, allow_extrapolation: bool, integrated: bool
    ) -> RelativeYawEstimate:
        extrapolation_age = age_s if allow_extrapolation else 0.0
        return RelativeYawEstimate(
            psi_measurement_rad=self._psi_measurement_rad,
            psi_control_rad=(
                self._psi_measurement_rad
                + self._r_hat_rad_s * extrapolation_age
            ),
            r_hat_rad_s=self._r_hat_rad_s,
            sample_time_s=self._last_sample_time_s,
            measurement_age_s=age_s,
            integrated_new_sample=integrated,
        )


@dataclass(frozen=True)
class YawPDCommand:
    e_psi_rad: float
    e_r_rad_s: float
    u_diff_request_nm: float


class ParallelYawPD:
    def __init__(self, K_psi_nm_per_rad: float, K_r_nm_per_rad_s: float) -> None:
        self.K_psi = float(K_psi_nm_per_rad)
        self.K_r = float(K_r_nm_per_rad_s)
        if not all(math.isfinite(value) and value >= 0.0 for value in (
            self.K_psi, self.K_r
        )):
            raise ValueError("yaw PD gains must be finite and non-negative")

    def command(
        self,
        psi_ref_rad: float,
        r_ref_rad_s: float,
        psi_hat_rad: float,
        r_hat_rad_s: float,
    ) -> YawPDCommand:
        e_psi = wrap_to_pi(float(psi_ref_rad) - float(psi_hat_rad))
        e_r = float(r_ref_rad_s) - float(r_hat_rad_s)
        u_diff = self.K_psi * e_psi + self.K_r * e_r
        if not np.isfinite([e_psi, e_r, u_diff]).all():
            raise ValueError("yaw PD command must be finite")
        return YawPDCommand(e_psi, e_r, u_diff)


@dataclass(frozen=True)
class WheelTorqueAllocation:
    u_sum_nm: float
    u_diff_request_nm: float
    u_diff_available_nm: float
    u_diff_used_nm: float
    left_torque_nm: float
    right_torque_nm: float
    differential_clipped: bool
    final_guard_clipped: bool


def allocate_longitudinal_priority(
    u_sum_nm: float,
    u_diff_request_nm: float,
    per_wheel_limit_nm: float,
) -> WheelTorqueAllocation:
    """Preserve legal common-mode torque, then allocate remaining yaw authority."""

    u_sum = float(u_sum_nm)
    u_diff_request = float(u_diff_request_nm)
    limit = float(per_wheel_limit_nm)
    if not np.isfinite([u_sum, u_diff_request, limit]).all() or limit <= 0.0:
        raise ValueError("allocator inputs must be finite and limit positive")
    if abs(u_sum) > 2.0 * limit + 1e-12:
        raise ValueError("u_sum must be software-limited before yaw allocation")
    available = max(0.0, 2.0 * limit - abs(u_sum))
    used = float(np.clip(u_diff_request, -available, available))
    # MuJoCo wheel hinges share +X axes.  The plant sign check shows that
    # left-positive/right-negative torque produces positive world +Z yaw.
    left = 0.5 * (u_sum + used)
    right = 0.5 * (u_sum - used)
    guarded_left = float(np.clip(left, -limit, limit))
    guarded_right = float(np.clip(right, -limit, limit))
    guard_clipped = not (
        math.isclose(left, guarded_left, rel_tol=0.0, abs_tol=1e-12)
        and math.isclose(right, guarded_right, rel_tol=0.0, abs_tol=1e-12)
    )
    return WheelTorqueAllocation(
        u_sum_nm=u_sum,
        u_diff_request_nm=u_diff_request,
        u_diff_available_nm=available,
        u_diff_used_nm=used,
        left_torque_nm=guarded_left,
        right_torque_nm=guarded_right,
        differential_clipped=not math.isclose(
            used, u_diff_request, rel_tol=0.0, abs_tol=1e-12
        ),
        final_guard_clipped=guard_clipped,
    )
