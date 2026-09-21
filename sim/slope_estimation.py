"""Physics-based road-slope observers for the CompanionBot TWIP.

The production longitudinal state estimator remains authoritative.  This
module adds only a decoupled scalar slope state, following the architecture of
Parravicini, Corno, and Savaresi rather than replacing the vehicle observer.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping

import numpy as np


def _finite(name: str, value: float) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


@dataclass(frozen=True)
class SlopePlantParameters:
    body_mass_kg: float
    gravitational_mass_kg: float
    equivalent_translation_mass_kg: float
    body_com_length_m: float
    wheel_radius_m: float
    theta_flat_rad: float
    wheel_joint_damping_nm_s_rad: float = 0.002
    wheel_joint_frictionloss_nm: float = 0.002
    rolling_resistance_sum_nm: float = 0.0
    gravity_m_s2: float = 9.81

    @classmethod
    def from_reduced(
        cls,
        reduced: Mapping,
        *,
        rolling_resistance_sum_nm: float = 0.0,
    ) -> "SlopePlantParameters":
        p = reduced["parameters"]
        body_mass = float(p["body_mass_kg"])
        wheel_mass = float(p["single_wheel_rotating_mass_kg"])
        return cls(
            body_mass_kg=body_mass,
            gravitational_mass_kg=body_mass + 2.0 * wheel_mass,
            equivalent_translation_mass_kg=float(
                p["equivalent_translation_mass_kg"]
            ),
            body_com_length_m=float(p["body_com_length_m"]),
            wheel_radius_m=float(p["wheel_radius_m"]),
            theta_flat_rad=float(p["theta_eq_rad"]),
            rolling_resistance_sum_nm=float(rolling_resistance_sum_nm),
        )

    @property
    def pitch_grade_ratio(self) -> float:
        return (
            self.wheel_radius_m * self.gravitational_mass_kg
            / (self.body_mass_kg * self.body_com_length_m)
        )

    def hinge_loss_sum_nm(self, velocity_m_s: float, pitch_rate_rad_s: float) -> float:
        relative_rate = (
            float(velocity_m_s) / self.wheel_radius_m
            - float(pitch_rate_rad_s)
        )
        if abs(relative_rate) < 1e-9:
            coulomb = 0.0
        else:
            coulomb = math.copysign(self.wheel_joint_frictionloss_nm, relative_rate)
        return 2.0 * (
            self.wheel_joint_damping_nm_s_rad * relative_rate + coulomb
        )

    def rolling_loss_sum_nm(self, velocity_m_s: float) -> float:
        if abs(float(velocity_m_s)) < 1e-9:
            return 0.0
        return math.copysign(self.rolling_resistance_sum_nm, float(velocity_m_s))


def analytic_theta_eq(alpha_rad: float, plant: SlopePlantParameters) -> float:
    """Validated Stage 4A quasi-static slope-to-pitch mapping."""

    argument = plant.pitch_grade_ratio * math.sin(_finite("alpha_rad", alpha_rad))
    return plant.theta_flat_rad + math.asin(float(np.clip(argument, -1.0, 1.0)))


def inverse_analytic_alpha(theta_world_rad: float, plant: SlopePlantParameters) -> float:
    """Principal inverse of :func:`analytic_theta_eq` in the Stage 4 range."""

    argument = math.sin(
        _finite("theta_world_rad", theta_world_rad) - plant.theta_flat_rad
    ) / plant.pitch_grade_ratio
    return math.asin(float(np.clip(argument, -1.0, 1.0)))


def equilibrium_sum_torque_nm(
    alpha_rad: float,
    velocity_m_s: float,
    pitch_rate_rad_s: float,
    plant: SlopePlantParameters,
) -> float:
    """Constant-speed input from the complete adapted longitudinal balance.

    The first term is the gravity force at the wheel radius.  The remaining
    terms are losses already present in the MuJoCo plant, not extra physics.
    """

    grade = (
        plant.wheel_radius_m
        * plant.gravitational_mass_kg
        * plant.gravity_m_s2
        * math.sin(_finite("alpha_rad", alpha_rad))
    )
    return (
        grade
        + plant.hinge_loss_sum_nm(velocity_m_s, pitch_rate_rad_s)
        + plant.rolling_loss_sum_nm(velocity_m_s)
    )


@dataclass(frozen=True)
class StaticInverseConfig:
    maximum_abs_pitch_rate_rad_s: float = 0.10
    maximum_abs_reference_acceleration_m_s2: float = 0.10
    minimum_abs_velocity_m_s: float = 0.08
    filter_time_constant_s: float = 0.20
    maximum_abs_alpha_deg: float = 18.0


class StaticInverseSlopeEstimator:
    """Quasi-static pitch-equilibrium inverse with fixed, simple gating."""

    def __init__(
        self,
        plant: SlopePlantParameters,
        config: StaticInverseConfig,
        initial_alpha_rad: float = 0.0,
    ) -> None:
        self.plant = plant
        self.config = config
        self.alpha_hat_rad = _finite("initial_alpha_rad", initial_alpha_rad)
        self.updated = False
        self.status = "INITIAL_HOLD"

    def update(
        self,
        *,
        theta_world_rad: float,
        pitch_rate_rad_s: float,
        velocity_m_s: float,
        reference_acceleration_m_s2: float,
        saturated: bool,
        normal_traction: bool,
        dt_s: float,
    ) -> float:
        valid = (
            abs(pitch_rate_rad_s) <= self.config.maximum_abs_pitch_rate_rad_s
            and abs(reference_acceleration_m_s2)
            <= self.config.maximum_abs_reference_acceleration_m_s2
            and abs(velocity_m_s) >= self.config.minimum_abs_velocity_m_s
            and not saturated
            and normal_traction
        )
        self.updated = bool(valid)
        if not valid:
            self.status = "HOLD_INVALID_DYNAMICS"
            return self.alpha_hat_rad
        raw = inverse_analytic_alpha(theta_world_rad, self.plant)
        weight = float(dt_s) / (self.config.filter_time_constant_s + float(dt_s))
        self.alpha_hat_rad += weight * (raw - self.alpha_hat_rad)
        limit = math.radians(self.config.maximum_abs_alpha_deg)
        self.alpha_hat_rad = float(np.clip(self.alpha_hat_rad, -limit, limit))
        self.status = "UPDATED"
        return self.alpha_hat_rad


@dataclass(frozen=True)
class TwipSlopeEkfConfig:
    update_period_s: float = 0.01
    derivative_filter_time_constant_s: float = 0.08
    alpha_process_std_deg_sqrt_s: float = 2.0
    force_measurement_std_n: float = 0.22
    steady_pitch_measurement_std_deg: float = 0.35
    initial_std_deg: float = 5.0
    maximum_abs_alpha_deg: float = 18.0
    maximum_abs_pitch_rate_rad_s: float = 3.0
    minimum_abs_velocity_m_s: float = 0.05
    convergence_std_deg: float = 1.0
    static_gate: StaticInverseConfig = field(default_factory=StaticInverseConfig)


@dataclass(frozen=True)
class SlopeEstimate:
    alpha_hat_rad: float
    alpha_variance_rad2: float
    innovation: float
    innovation_variance: float
    normalized_innovation: float
    converged: bool
    updated: bool
    status: str
    force_innovation_n: float
    force_nis: float
    static_pitch_innovation_rad: float


class TwipSlopeEkf:
    """Scalar, decoupled EKF using the adapted nonlinear TWIP force balance."""

    def __init__(
        self,
        plant: SlopePlantParameters,
        config: TwipSlopeEkfConfig,
        initial_alpha_rad: float = 0.0,
    ) -> None:
        self.plant = plant
        self.config = config
        self.alpha_hat_rad = _finite("initial_alpha_rad", initial_alpha_rad)
        self.variance_rad2 = math.radians(config.initial_std_deg) ** 2
        self._elapsed_s = 0.0
        self._previous_velocity: float | None = None
        self._previous_pitch_rate: float | None = None
        self._filtered_acceleration = 0.0
        self._filtered_pitch_acceleration = 0.0
        self._filtered_torque = 0.0
        self.last = SlopeEstimate(
            self.alpha_hat_rad,
            self.variance_rad2,
            0.0,
            math.inf,
            0.0,
            False,
            False,
            "INITIAL_HOLD",
            0.0,
            0.0,
            0.0,
        )

    def _sequential_update(self, innovation: float, jacobian: float, noise: float) -> tuple[float, float]:
        innovation_variance = jacobian * jacobian * self.variance_rad2 + noise
        gain = self.variance_rad2 * jacobian / innovation_variance
        self.alpha_hat_rad += gain * innovation
        self.variance_rad2 = max(
            (1.0 - gain * jacobian) * self.variance_rad2, 1e-12
        )
        return innovation_variance, innovation / math.sqrt(innovation_variance)

    def update(
        self,
        *,
        velocity_m_s: float,
        theta_world_rad: float,
        pitch_rate_rad_s: float,
        actual_sum_torque_nm: float,
        reference_acceleration_m_s2: float,
        saturated: bool,
        normal_traction: bool,
        dt_s: float,
    ) -> SlopeEstimate:
        dt = _finite("dt_s", dt_s)
        self._elapsed_s += dt
        velocity = _finite("velocity_m_s", velocity_m_s)
        pitch_rate = _finite("pitch_rate_rad_s", pitch_rate_rad_s)
        torque = _finite("actual_sum_torque_nm", actual_sum_torque_nm)
        if self._previous_velocity is None:
            self._previous_velocity = velocity
            self._previous_pitch_rate = pitch_rate
            return self.last

        derivative_dt = max(dt, 1e-9)
        acceleration = (velocity - self._previous_velocity) / derivative_dt
        pitch_acceleration = (
            pitch_rate - float(self._previous_pitch_rate)
        ) / derivative_dt
        self._previous_velocity = velocity
        self._previous_pitch_rate = pitch_rate
        weight = dt / (self.config.derivative_filter_time_constant_s + dt)
        self._filtered_acceleration += weight * (
            acceleration - self._filtered_acceleration
        )
        self._filtered_pitch_acceleration += weight * (
            pitch_acceleration - self._filtered_pitch_acceleration
        )
        self._filtered_torque += weight * (torque - self._filtered_torque)

        if self._elapsed_s + 1e-12 < self.config.update_period_s:
            return self.last
        update_dt = self._elapsed_s
        self._elapsed_s = 0.0
        q_rate = math.radians(self.config.alpha_process_std_deg_sqrt_s) ** 2
        self.variance_rad2 += q_rate * update_dt

        dynamic_valid = (
            normal_traction
            and not saturated
            and abs(velocity) >= self.config.minimum_abs_velocity_m_s
            and abs(pitch_rate) <= self.config.maximum_abs_pitch_rate_rad_s
        )
        force_innovation = 0.0
        force_nis = 0.0
        innovation_variance = math.inf
        normalized = 0.0
        updated = False
        status = "HOLD_INVALID_DYNAMICS"
        alpha = self.alpha_hat_rad
        beta = _finite("theta_world_rad", theta_world_rad) - self.plant.theta_flat_rad
        mbl = self.plant.body_mass_kg * self.plant.body_com_length_m
        if dynamic_valid:
            hinge = self.plant.hinge_loss_sum_nm(velocity, pitch_rate)
            rolling = self.plant.rolling_loss_sum_nm(velocity)
            phase = alpha + beta
            residual = (
                self.plant.equivalent_translation_mass_kg
                * self._filtered_acceleration
                - mbl * pitch_rate * pitch_rate * math.sin(phase)
                + mbl * self._filtered_pitch_acceleration * math.cos(phase)
                - (self._filtered_torque - hinge - rolling)
                / self.plant.wheel_radius_m
                + self.plant.gravitational_mass_kg
                * self.plant.gravity_m_s2
                * math.sin(alpha)
            )
            jacobian = (
                -mbl * pitch_rate * pitch_rate * math.cos(phase)
                - mbl * self._filtered_pitch_acceleration * math.sin(phase)
                + self.plant.gravitational_mass_kg
                * self.plant.gravity_m_s2
                * math.cos(alpha)
            )
            force_innovation = -residual
            innovation_variance, normalized = self._sequential_update(
                force_innovation,
                jacobian,
                self.config.force_measurement_std_n**2,
            )
            force_nis = normalized * normalized
            updated = True
            status = "UPDATED_FORCE"

        static_valid = (
            abs(pitch_rate) <= self.config.static_gate.maximum_abs_pitch_rate_rad_s
            and abs(reference_acceleration_m_s2)
            <= self.config.static_gate.maximum_abs_reference_acceleration_m_s2
            and abs(velocity) >= self.config.static_gate.minimum_abs_velocity_m_s
            and normal_traction
            and not saturated
        )
        pitch_innovation = 0.0
        if static_valid:
            alpha = self.alpha_hat_rad
            argument = self.plant.pitch_grade_ratio * math.sin(alpha)
            clipped = float(np.clip(argument, -0.999999, 0.999999))
            predicted_pitch = self.plant.theta_flat_rad + math.asin(clipped)
            pitch_innovation = float(theta_world_rad) - predicted_pitch
            jacobian = (
                self.plant.pitch_grade_ratio * math.cos(alpha)
                / math.sqrt(1.0 - clipped * clipped)
            )
            innovation_variance, normalized = self._sequential_update(
                pitch_innovation,
                jacobian,
                math.radians(self.config.steady_pitch_measurement_std_deg) ** 2,
            )
            updated = True
            status = "UPDATED_FORCE_AND_PITCH" if dynamic_valid else "UPDATED_PITCH"

        limit = math.radians(self.config.maximum_abs_alpha_deg)
        self.alpha_hat_rad = float(np.clip(self.alpha_hat_rad, -limit, limit))
        converged = (
            updated
            and math.degrees(math.sqrt(self.variance_rad2))
            <= self.config.convergence_std_deg
        )
        self.last = SlopeEstimate(
            self.alpha_hat_rad,
            self.variance_rad2,
            float(pitch_innovation if static_valid else force_innovation),
            float(innovation_variance),
            float(normalized),
            bool(converged),
            bool(updated),
            status,
            float(force_innovation),
            float(force_nis),
            float(pitch_innovation),
        )
        return self.last
