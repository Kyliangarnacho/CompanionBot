"""Minimal covariance-aware longitudinal wheel-slip observer.

The production odometry velocity is wheel-derived, so it cannot be used as an
independent body-speed reference.  This module keeps a deliberately small
``[v_body, d_slip]`` augmentation: TWIP force balance predicts body velocity,
while the encoder observation is ``v_wheel = v_body + d_slip + noise``.
Ground-truth body velocity, friction, and contact state are never inputs.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .slope_estimation import SlopePlantParameters


@dataclass(frozen=True)
class SlipObserverConfig:
    wheel_velocity_std_m_s: float = 0.018
    body_acceleration_process_std_m_s2: float = 0.55
    slip_random_walk_std_m_s_sqrt_s: float = 0.45
    slip_decay_time_constant_s: float = 0.35
    derivative_filter_time_constant_s: float = 0.06
    imu_acceleration_filter_time_constant_s: float = 0.035
    imu_bias_learning_time_constant_s: float = 0.25
    maximum_abs_model_acceleration_m_s2: float = 5.0
    initial_body_velocity_std_m_s: float = 0.025
    initial_slip_velocity_std_m_s: float = 0.015
    enter_chi: float = 9.0
    exit_chi: float = 2.71
    enter_persistence_s: float = 0.08
    exit_persistence_s: float = 0.24
    warmup_s: float = 0.50
    minimum_active_speed_m_s: float = 0.08

    def __post_init__(self) -> None:
        positive = (
            self.wheel_velocity_std_m_s,
            self.body_acceleration_process_std_m_s2,
            self.slip_random_walk_std_m_s_sqrt_s,
            self.slip_decay_time_constant_s,
            self.derivative_filter_time_constant_s,
            self.imu_acceleration_filter_time_constant_s,
            self.imu_bias_learning_time_constant_s,
            self.maximum_abs_model_acceleration_m_s2,
            self.initial_body_velocity_std_m_s,
            self.initial_slip_velocity_std_m_s,
            self.enter_chi,
            self.exit_chi,
            self.enter_persistence_s,
            self.exit_persistence_s,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError("slip observer parameters must be finite and positive")
        if self.exit_chi >= self.enter_chi:
            raise ValueError("exit_chi must be lower than enter_chi")
        if self.warmup_s < 0.0 or self.minimum_active_speed_m_s < 0.0:
            raise ValueError("warmup and minimum active speed must be non-negative")


@dataclass(frozen=True)
class SlipEstimate:
    body_velocity_hat_m_s: float
    slip_velocity_hat_m_s: float
    wheel_velocity_m_s: float
    wheel_residual_m_s: float
    innovation_variance_m2_s2: float
    slip_score: float
    slip_active: bool
    confidence: float
    model_acceleration_m_s2: float
    status: str


class LongitudinalSlipObserver:
    """Two-state longitudinal disturbance observer with one boolean latch."""

    def __init__(
        self,
        plant: SlopePlantParameters,
        config: SlipObserverConfig = SlipObserverConfig(),
    ) -> None:
        self.plant = plant
        self.config = config
        self._x = np.zeros(2, dtype=float)
        self._P = np.diag([
            config.initial_body_velocity_std_m_s**2,
            config.initial_slip_velocity_std_m_s**2,
        ])
        self._initialized = False
        self._elapsed_s = 0.0
        self._enter_elapsed_s = 0.0
        self._exit_elapsed_s = 0.0
        self._slip_active = False
        self._previous_pitch_rate_rad_s = 0.0
        self._filtered_pitch_acceleration_rad_s2 = 0.0
        self._filtered_sum_torque_nm = 0.0
        self._filtered_imu_acceleration_m_s2 = 0.0
        self._imu_acceleration_bias_m_s2 = 0.0
        self.last = SlipEstimate(
            0.0, 0.0, 0.0, 0.0, math.inf, 0.0, False, 0.0, 0.0,
            "INITIAL_HOLD",
        )

    @property
    def slip_active(self) -> bool:
        return self._slip_active

    def reset(self, wheel_velocity_m_s: float = 0.0) -> SlipEstimate:
        velocity = float(wheel_velocity_m_s)
        if not math.isfinite(velocity):
            raise ValueError("wheel velocity must be finite")
        self._x[:] = [velocity, 0.0]
        self._P[:] = np.diag([
            self.config.initial_body_velocity_std_m_s**2,
            self.config.initial_slip_velocity_std_m_s**2,
        ])
        self._initialized = True
        self._elapsed_s = 0.0
        self._enter_elapsed_s = 0.0
        self._exit_elapsed_s = 0.0
        self._slip_active = False
        self._previous_pitch_rate_rad_s = 0.0
        self._filtered_pitch_acceleration_rad_s2 = 0.0
        self._filtered_sum_torque_nm = 0.0
        self._filtered_imu_acceleration_m_s2 = 0.0
        self._imu_acceleration_bias_m_s2 = 0.0
        self.last = SlipEstimate(
            velocity, 0.0, velocity, 0.0, math.inf, 0.0, False, 0.0, 0.0,
            "WARMUP",
        )
        return self.last

    def _model_acceleration(
        self,
        *,
        theta_world_rad: float,
        pitch_rate_rad_s: float,
        actual_sum_torque_nm: float,
        alpha_hat_rad: float,
        dt_s: float,
    ) -> float:
        weight = dt_s / (self.config.derivative_filter_time_constant_s + dt_s)
        raw_pitch_acceleration = (
            pitch_rate_rad_s - self._previous_pitch_rate_rad_s
        ) / dt_s
        self._previous_pitch_rate_rad_s = pitch_rate_rad_s
        self._filtered_pitch_acceleration_rad_s2 += weight * (
            raw_pitch_acceleration - self._filtered_pitch_acceleration_rad_s2
        )
        self._filtered_sum_torque_nm += weight * (
            actual_sum_torque_nm - self._filtered_sum_torque_nm
        )
        beta = theta_world_rad - self.plant.theta_flat_rad
        phase = alpha_hat_rad + beta
        velocity = float(self._x[0])
        hinge = self.plant.hinge_loss_sum_nm(velocity, pitch_rate_rad_s)
        rolling = self.plant.rolling_loss_sum_nm(velocity)
        mbl = self.plant.body_mass_kg * self.plant.body_com_length_m
        acceleration = (
            mbl * pitch_rate_rad_s**2 * math.sin(phase)
            - mbl * self._filtered_pitch_acceleration_rad_s2 * math.cos(phase)
            + (self._filtered_sum_torque_nm - hinge - rolling)
            / self.plant.wheel_radius_m
            - self.plant.gravitational_mass_kg
            * self.plant.gravity_m_s2
            * math.sin(alpha_hat_rad)
        ) / self.plant.equivalent_translation_mass_kg
        return float(np.clip(
            acceleration,
            -self.config.maximum_abs_model_acceleration_m_s2,
            self.config.maximum_abs_model_acceleration_m_s2,
        ))

    def update(
        self,
        *,
        wheel_velocity_m_s: float,
        theta_world_rad: float,
        pitch_rate_rad_s: float,
        actual_sum_torque_nm: float,
        alpha_hat_rad: float,
        accelerometer_m_s2,
        saturated: bool,
        dt_s: float,
    ) -> SlipEstimate:
        values = (
            wheel_velocity_m_s, theta_world_rad, pitch_rate_rad_s,
            actual_sum_torque_nm, alpha_hat_rad, dt_s,
        )
        if any(not math.isfinite(float(value)) for value in values) or dt_s <= 0.0:
            raise ValueError("slip observer inputs must be finite and dt_s positive")
        accelerometer = np.asarray(accelerometer_m_s2, dtype=float)
        if accelerometer.shape != (3,) or not np.isfinite(accelerometer).all():
            raise ValueError("accelerometer_m_s2 must be a finite length-3 vector")
        if not self._initialized:
            self.reset(wheel_velocity_m_s)

        dt = float(dt_s)
        self._elapsed_s += dt
        model_acceleration = self._model_acceleration(
            theta_world_rad=float(theta_world_rad),
            pitch_rate_rad_s=float(pitch_rate_rad_s),
            actual_sum_torque_nm=float(actual_sum_torque_nm),
            alpha_hat_rad=float(alpha_hat_rad),
            dt_s=dt,
        )
        # Positive travel is chassis -Y.  Project specific force onto the
        # horizontal-forward direction expressed in chassis coordinates;
        # gravity cancels algebraically, leaving longitudinal acceleration.
        theta = float(theta_world_rad)
        raw_imu_acceleration = (
            -float(accelerometer[1]) * math.cos(theta)
            + float(accelerometer[2]) * math.sin(theta)
        )
        if (
            self._elapsed_s <= self.config.warmup_s
            and abs(float(wheel_velocity_m_s)) < self.config.minimum_active_speed_m_s
            and abs(float(pitch_rate_rad_s)) < 0.25
        ):
            bias_weight = dt / (self.config.imu_bias_learning_time_constant_s + dt)
            self._imu_acceleration_bias_m_s2 += bias_weight * (
                raw_imu_acceleration - self._imu_acceleration_bias_m_s2
            )
        unbiased_imu_acceleration = (
            raw_imu_acceleration - self._imu_acceleration_bias_m_s2
        )
        accel_weight = dt / (
            self.config.imu_acceleration_filter_time_constant_s + dt
        )
        self._filtered_imu_acceleration_m_s2 += accel_weight * (
            unbiased_imu_acceleration - self._filtered_imu_acceleration_m_s2
        )
        acceleration = float(np.clip(
            self._filtered_imu_acceleration_m_s2,
            -self.config.maximum_abs_model_acceleration_m_s2,
            self.config.maximum_abs_model_acceleration_m_s2,
        ))

        slip_decay = math.exp(-dt / self.config.slip_decay_time_constant_s)
        F = np.asarray([[1.0, 0.0], [0.0, slip_decay]], dtype=float)
        self._x = F @ self._x
        self._x[0] += acceleration * dt
        q_body = (self.config.body_acceleration_process_std_m_s2 * dt) ** 2
        q_slip = self.config.slip_random_walk_std_m_s_sqrt_s**2 * dt
        self._P = F @ self._P @ F.T + np.diag([q_body, q_slip])

        wheel = float(wheel_velocity_m_s)
        # This is intentionally wheel speed minus the independent body model,
        # not wheel speed minus the wheel-derived production v_hat.
        wheel_residual = wheel - float(self._x[0])
        score_variance = (
            float(self._P[0, 0]) + self.config.wheel_velocity_std_m_s**2
        )
        score = wheel_residual * wheel_residual / max(score_variance, 1e-12)

        H = np.asarray([1.0, 1.0], dtype=float)
        innovation = wheel - float(H @ self._x)
        innovation_variance = float(
            H @ self._P @ H + self.config.wheel_velocity_std_m_s**2
        )
        gain = self._P @ H / max(innovation_variance, 1e-12)
        # Once slip is latched, do not let wheel speed drag the independent
        # body state toward the bad measurement; only the disturbance adapts.
        if self._slip_active:
            gain[0] = 0.0
        self._x += gain * innovation
        identity = np.eye(2)
        KH = np.outer(gain, H)
        measurement_variance = self.config.wheel_velocity_std_m_s**2
        self._P = (
            (identity - KH) @ self._P @ (identity - KH).T
            + np.outer(gain, gain) * measurement_variance
        )
        self._P = 0.5 * (self._P + self._P.T)
        # Normal-traction pseudo-measurement d_slip=0 resolves the otherwise
        # unobservable split between a slowly drifting integrated IMU velocity
        # and wheel disturbance.  It is disabled as soon as the raw NIS crosses
        # the enter threshold, so a genuine wheel-speed jump is not absorbed.
        if not self._slip_active and score <= self.config.enter_chi:
            zero_slip_variance = self.config.initial_slip_velocity_std_m_s**2
            zero_innovation_variance = float(self._P[1, 1] + zero_slip_variance)
            zero_gain = self._P[:, 1] / max(zero_innovation_variance, 1e-12)
            self._x += zero_gain * (-float(self._x[1]))
            K0H0 = np.outer(zero_gain, np.asarray([0.0, 1.0]))
            self._P = (
                (identity - K0H0) @ self._P @ (identity - K0H0).T
                + np.outer(zero_gain, zero_gain) * zero_slip_variance
            )
            self._P = 0.5 * (self._P + self._P.T)

        active_motion = max(abs(wheel), abs(float(self._x[0]))) >= (
            self.config.minimum_active_speed_m_s
        )
        eligible = self._elapsed_s >= self.config.warmup_s and active_motion
        if not self._slip_active:
            self._enter_elapsed_s = (
                self._enter_elapsed_s + dt
                if eligible and not saturated and score > self.config.enter_chi
                else 0.0
            )
            if self._enter_elapsed_s + 1e-12 >= self.config.enter_persistence_s:
                self._slip_active = True
                self._exit_elapsed_s = 0.0
        else:
            self._exit_elapsed_s = (
                self._exit_elapsed_s + dt
                if score < self.config.exit_chi else 0.0
            )
            if self._exit_elapsed_s + 1e-12 >= self.config.exit_persistence_s:
                self._slip_active = False
                self._enter_elapsed_s = 0.0

        confidence = 1.0 / (1.0 + math.sqrt(max(score_variance, 0.0)) / 0.05)
        if self._elapsed_s < self.config.warmup_s:
            status = "WARMUP"
        elif not active_motion:
            status = "HOLD_LOW_SPEED"
        elif self._slip_active:
            status = "SLIP_ACTIVE"
        else:
            status = "NORMAL"
        self.last = SlipEstimate(
            body_velocity_hat_m_s=float(self._x[0]),
            slip_velocity_hat_m_s=float(self._x[1]),
            wheel_velocity_m_s=wheel,
            wheel_residual_m_s=float(wheel_residual),
            innovation_variance_m2_s2=score_variance,
            slip_score=float(score),
            slip_active=self._slip_active,
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            model_acceleration_m_s2=model_acceleration,
            status=status,
        )
        return self.last
