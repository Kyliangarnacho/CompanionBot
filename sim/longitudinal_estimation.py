"""Minimal longitudinal attitude and encoder estimators for sensor bring-up."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from .minisegway import QuadratureEncoderProfile


DEFAULT_ESTIMATOR_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "minisegway"
    / "longitudinal_estimator_config.json"
)


def _wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class ComplementaryPitchConfig:
    sample_period_s: float
    time_constant_s: float

    @property
    def cutoff_frequency_hz(self) -> float:
        return 1.0 / (2.0 * math.pi * self.time_constant_s)

    @property
    def gyro_weight(self) -> float:
        return self.time_constant_s / (self.time_constant_s + self.sample_period_s)


@dataclass(frozen=True)
class LongitudinalEstimatorConfig:
    pitch: ComplementaryPitchConfig
    encoder_pll_bandwidth_rad_s: float
    encoder_pll_snap_to_zero: bool


def load_longitudinal_estimator_config(
    path: str | Path = DEFAULT_ESTIMATOR_CONFIG_PATH,
) -> LongitudinalEstimatorConfig:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    sample_period_s = float(raw["sample_period_s"])
    time_constant_s = float(raw["imu_complementary_filter"]["time_constant_s"])
    pll_raw = raw["encoder_velocity"]["pll"]
    pll_selected = float(pll_raw["selected_bandwidth_rad_s"])
    pll_snap_to_zero = bool(pll_raw["snap_to_zero"])
    if sample_period_s <= 0.0 or time_constant_s <= 0.0:
        raise ValueError("estimator sample period and complementary-filter time constant must be positive")
    if pll_selected <= 0.0:
        raise ValueError("encoder PLL bandwidth must be positive")
    if sample_period_s * (2.0 * pll_selected) >= 1.0:
        raise ValueError("selected encoder PLL violates dt*pll_kp < 1")
    return LongitudinalEstimatorConfig(
        pitch=ComplementaryPitchConfig(sample_period_s, time_constant_s),
        encoder_pll_bandwidth_rad_s=pll_selected,
        encoder_pll_snap_to_zero=pll_snap_to_zero,
    )


@dataclass(frozen=True)
class PitchEstimate:
    theta_hat_rad: float
    theta_dot_hat_rad_s: float
    theta_acc_rad: float


@dataclass(frozen=True)
class LongitudinalAccelerationCompensationConfig:
    """Opt-in experiment; the frozen complementary filter remains the default.

    50 ms smooths the existing PLL velocity derivative (3.18 Hz cutoff).
    At 0.25 m/s^2 measured acceleration, accel correction is halved; the
    smooth, symmetric gain returns to the frozen value as acceleration decays.
    These constants are fixed before the single experiment, not swept.
    """

    acceleration_time_constant_s: float = 0.05
    adaptive_acceleration_scale_m_s2: float = 0.25

    def __post_init__(self) -> None:
        if any(
            not math.isfinite(value) or value <= 0.0
            for value in (
                self.acceleration_time_constant_s,
                self.adaptive_acceleration_scale_m_s2,
            )
        ):
            raise ValueError("acceleration compensation constants must be finite and positive")


class ComplementaryPitchEstimator:
    """Fuse chassis-frame accelerometer tilt with pitch-axis gyro integration."""

    def __init__(
        self,
        config: ComplementaryPitchConfig,
        acceleration_compensation: LongitudinalAccelerationCompensationConfig | None = None,
    ):
        self.config = config
        self.acceleration_compensation = acceleration_compensation
        self._initialized = False
        self._theta_hat_rad = 0.0
        self.raw_theta_acc_rad = 0.0
        self.acceleration_used_m_s2 = 0.0
        self.accel_correction_gain_factor = 1.0
        self.compensated_accelerometer_m_s2 = np.zeros(3, dtype=float)

    @staticmethod
    def accel_pitch_rad(accelerometer_m_s2: NDArray[np.float64]) -> float:
        accel = np.asarray(accelerometer_m_s2, dtype=float)
        if accel.shape != (3,) or not np.isfinite(accel).all():
            raise ValueError("accelerometer sample must be a finite length-3 vector")
        return math.atan2(float(accel[1]), float(accel[2]))

    def reset(
        self,
        accelerometer_m_s2: NDArray[np.float64],
        gyro_rad_s: NDArray[np.float64] | None = None,
    ) -> PitchEstimate:
        theta_acc = self.accel_pitch_rad(accelerometer_m_s2)
        gyro_x = 0.0 if gyro_rad_s is None else self._gyro_x(gyro_rad_s)
        self._theta_hat_rad = theta_acc
        self._initialized = True
        self.raw_theta_acc_rad = theta_acc
        self.acceleration_used_m_s2 = 0.0
        self.accel_correction_gain_factor = 1.0
        self.compensated_accelerometer_m_s2 = np.asarray(
            accelerometer_m_s2, dtype=float
        ).copy()
        return PitchEstimate(self._theta_hat_rad, gyro_x, theta_acc)

    @staticmethod
    def _gyro_x(gyro_rad_s: NDArray[np.float64]) -> float:
        gyro = np.asarray(gyro_rad_s, dtype=float)
        if gyro.shape != (3,) or not np.isfinite(gyro).all():
            raise ValueError("gyro sample must be a finite length-3 vector")
        return float(gyro[0])

    def update(
        self,
        accelerometer_m_s2: NDArray[np.float64],
        gyro_rad_s: NDArray[np.float64],
        *,
        longitudinal_acceleration_m_s2: float = 0.0,
    ) -> PitchEstimate:
        if not self._initialized:
            return self.reset(accelerometer_m_s2, gyro_rad_s)
        theta_acc = self.accel_pitch_rad(accelerometer_m_s2)
        gyro_x = self._gyro_x(gyro_rad_s)
        theta_gyro = self._theta_hat_rad + gyro_x * self.config.sample_period_s
        self.raw_theta_acc_rad = theta_acc
        self.compensated_accelerometer_m_s2 = np.asarray(
            accelerometer_m_s2, dtype=float
        ).copy()
        if self.acceleration_compensation is not None:
            acceleration = float(longitudinal_acceleration_m_s2)
            if not math.isfinite(acceleration):
                raise ValueError("longitudinal acceleration estimate must be finite")
            # Positive longitudinal motion is world -Y; positive pitch is Rx.
            # Its translational specific-force contribution in chassis axes is
            # [0, -a*cos(theta), +a*sin(theta)]. Subtract using gyro prediction,
            # never GT, the planner's acceleration, or the nominal lean target.
            self.compensated_accelerometer_m_s2[1] += acceleration * math.cos(theta_gyro)
            self.compensated_accelerometer_m_s2[2] -= acceleration * math.sin(theta_gyro)
            theta_acc = self.accel_pitch_rad(self.compensated_accelerometer_m_s2)
            self.acceleration_used_m_s2 = acceleration
            scale = self.acceleration_compensation.adaptive_acceleration_scale_m_s2
            self.accel_correction_gain_factor = 1.0 / (1.0 + (acceleration / scale) ** 2)
        accel_innovation = _wrap_angle(theta_acc - theta_gyro)
        self._theta_hat_rad = _wrap_angle(
            theta_gyro
            + (1.0 - self.config.gyro_weight)
            * self.accel_correction_gain_factor
            * accel_innovation
        )
        return PitchEstimate(self._theta_hat_rad, gyro_x, theta_acc)


@dataclass(frozen=True)
class EncoderOdometryEstimate:
    position_hat_m: float
    velocity_hat_m_s: float
    wheel_pll_position_counts: NDArray[np.float64]
    wheel_pll_velocity_counts_s: NDArray[np.float64]
    wheel_relative_angle_hat_rad: NDArray[np.float64]
    wheel_relative_velocity_hat_rad_s: NDArray[np.float64]


class IncrementalEncoderPLL:
    """ODrive-style linear incremental-count position/velocity tracking PLL."""

    def __init__(
        self,
        sample_period_s: float,
        bandwidth_rad_s: float,
        snap_to_zero: bool = True,
    ):
        if sample_period_s <= 0.0 or bandwidth_rad_s <= 0.0:
            raise ValueError("PLL sample period and bandwidth must be positive")
        self.sample_period_s = float(sample_period_s)
        self.bandwidth_rad_s = float(bandwidth_rad_s)
        self.pll_kp = 2.0 * self.bandwidth_rad_s
        self.pll_ki = 0.25 * self.pll_kp**2
        if self.sample_period_s * self.pll_kp >= 1.0:
            raise ValueError("unstable encoder PLL gain: dt*pll_kp must be < 1")
        self.snap_to_zero = bool(snap_to_zero)
        self.snap_to_zero_threshold_counts_s = (
            0.5 * self.sample_period_s * self.pll_ki
        )
        self.position_estimate_counts = 0.0
        self.velocity_estimate_counts_s = 0.0
        self._initialized = False

    def reset(self, encoder_count: int) -> None:
        self.position_estimate_counts = float(encoder_count)
        self.velocity_estimate_counts_s = 0.0
        self._initialized = True

    def update(self, encoder_count: int) -> float:
        if not self._initialized:
            self.reset(encoder_count)
            return self.velocity_estimate_counts_s

        dt = self.sample_period_s
        self.position_estimate_counts += dt * self.velocity_estimate_counts_s
        discrete_position_error = float(
            int(encoder_count) - math.floor(self.position_estimate_counts)
        )
        self.position_estimate_counts += dt * self.pll_kp * discrete_position_error
        self.velocity_estimate_counts_s += (
            dt * self.pll_ki * discrete_position_error
        )
        if (
            self.snap_to_zero
            and abs(self.velocity_estimate_counts_s)
            < self.snap_to_zero_threshold_counts_s
        ):
            self.velocity_estimate_counts_s = 0.0
        return self.velocity_estimate_counts_s


class EncoderLongitudinalEstimator:
    """Quantized wheel-count odometry with IMU pitch geometry correction."""

    def __init__(
        self,
        encoder_profile: QuadratureEncoderProfile,
        wheel_radius_m: float,
        sample_period_s: float,
        pll_bandwidth_rad_s: float,
        pll_snap_to_zero: bool,
    ):
        if wheel_radius_m <= 0.0 or sample_period_s <= 0.0:
            raise ValueError("wheel radius and sample period must be positive")
        self.encoder_profile = encoder_profile
        self.wheel_radius_m = float(wheel_radius_m)
        self.sample_period_s = float(sample_period_s)
        self._count_signs = np.asarray(
            [encoder_profile.left_count_sign, encoder_profile.right_count_sign], dtype=float
        )
        self._initialized = False
        self._previous_pll_position_counts = np.zeros(2, dtype=float)
        self._previous_theta_hat_rad = 0.0
        self._position_hat_m = 0.0
        self._wheel_plls = (
            IncrementalEncoderPLL(
                self.sample_period_s, pll_bandwidth_rad_s, pll_snap_to_zero
            ),
            IncrementalEncoderPLL(
                self.sample_period_s, pll_bandwidth_rad_s, pll_snap_to_zero
            ),
        )

    def _counts_to_relative_angle(self, counts: NDArray) -> NDArray[np.float64]:
        return (
            2.0
            * math.pi
            * np.asarray(counts, dtype=float)
            / (self._count_signs * self.encoder_profile.output_decoded_counts_per_rev)
        )

    def reset(
        self, encoder_counts: NDArray[np.int64], theta_hat_rad: float
    ) -> EncoderOdometryEstimate:
        counts = np.asarray(encoder_counts, dtype=np.int64)
        if counts.shape != (2,):
            raise ValueError("encoder counts must be a length-2 vector")
        if not math.isfinite(theta_hat_rad):
            raise ValueError("pitch estimate must be finite")
        self._previous_theta_hat_rad = float(theta_hat_rad)
        self._position_hat_m = 0.0
        for pll, count in zip(self._wheel_plls, counts):
            pll.reset(int(count))
        wheel_pll_position_counts = np.asarray(
            [pll.position_estimate_counts for pll in self._wheel_plls], dtype=float
        )
        self._previous_pll_position_counts = wheel_pll_position_counts.copy()
        wheel_relative_angle_hat_rad = self._counts_to_relative_angle(
            wheel_pll_position_counts
        )
        self._initialized = True
        return EncoderOdometryEstimate(
            0.0,
            0.0,
            wheel_pll_position_counts,
            np.zeros(2, dtype=float),
            wheel_relative_angle_hat_rad,
            np.zeros(2, dtype=float),
        )

    def update(
        self,
        encoder_counts: NDArray[np.int64],
        theta_hat_rad: float,
        theta_dot_hat_rad_s: float,
    ) -> EncoderOdometryEstimate:
        if not self._initialized:
            return self.reset(encoder_counts, theta_hat_rad)
        counts = np.asarray(encoder_counts, dtype=np.int64)
        if counts.shape != (2,):
            raise ValueError("encoder counts must be a length-2 vector")
        if not math.isfinite(theta_hat_rad) or not math.isfinite(theta_dot_hat_rad_s):
            raise ValueError("pitch and pitch-rate estimates must be finite")

        wheel_pll_velocity_counts_s = np.asarray(
            [
                pll.update(int(count))
                for pll, count in zip(self._wheel_plls, counts)
            ],
            dtype=float,
        )
        wheel_pll_position_counts = np.asarray(
            [pll.position_estimate_counts for pll in self._wheel_plls], dtype=float
        )
        wheel_relative_angle_hat_rad = self._counts_to_relative_angle(
            wheel_pll_position_counts
        )
        wheel_relative_velocity_hat_rad_s = self._counts_to_relative_angle(
            wheel_pll_velocity_counts_s
        )

        delta_phi_hat = self._counts_to_relative_angle(
            wheel_pll_position_counts - self._previous_pll_position_counts
        )
        delta_theta = _wrap_angle(float(theta_hat_rad) - self._previous_theta_hat_rad)
        self._position_hat_m += self.wheel_radius_m * (
            float(np.mean(delta_phi_hat)) + delta_theta
        )
        self._previous_pll_position_counts = wheel_pll_position_counts.copy()
        self._previous_theta_hat_rad = float(theta_hat_rad)
        velocity_hat = self.wheel_radius_m * (
            float(np.mean(wheel_relative_velocity_hat_rad_s))
            + theta_dot_hat_rad_s
        )
        return EncoderOdometryEstimate(
            self._position_hat_m,
            velocity_hat,
            wheel_pll_position_counts,
            wheel_pll_velocity_counts_s,
            wheel_relative_angle_hat_rad,
            wheel_relative_velocity_hat_rad_s,
        )


@dataclass(frozen=True)
class LongitudinalEstimate:
    """Estimator output aligned to the current controller horizon.

    The complementary filter itself remains on the IMU packet timestamp.  Its
    delayed attitude is retained explicitly for diagnostics, while the legacy
    ``theta_hat`` names below now mean the short-horizon extrapolated values
    used together with the current encoder PLL state.
    """

    theta_hat_rad: float
    theta_dot_hat_rad_s: float
    theta_acc_rad: float
    theta_measurement_time_rad: float
    theta_dot_measurement_time_rad_s: float
    imu_measurement_age_s: float
    imu_extrapolation_age_s: float
    imu_kinematic_extrapolation_applied: bool
    position_hat_m: float
    velocity_hat_m_s: float
    wheel_pll_position_counts: NDArray[np.float64]
    wheel_pll_velocity_counts_s: NDArray[np.float64]
    wheel_relative_angle_hat_rad: NDArray[np.float64]
    wheel_relative_velocity_hat_rad_s: NDArray[np.float64]

    def plant_state(self, theta_eq_rad: float) -> NDArray[np.float64]:
        """Return sensorized physical plant state at the controller horizon."""

        if not math.isfinite(theta_eq_rad):
            raise ValueError("pitch equilibrium reference must be finite")
        return np.asarray(
            [
                self.position_hat_m,
                self.velocity_hat_m_s,
                _wrap_angle(self.theta_hat_rad - theta_eq_rad),
                self.theta_dot_hat_rad_s,
            ],
            dtype=float,
        )

    def tracking_error_state(
        self,
        theta_eq_rad: float,
        position_reference_m: float,
        velocity_reference_m_s: float,
    ) -> NDArray[np.float64]:
        """Return ``x_plant - [p_ref, v_ref, 0, 0]`` for feedback only."""

        if not math.isfinite(position_reference_m) or not math.isfinite(
            velocity_reference_m_s
        ):
            raise ValueError("tracking references must be finite")
        error = self.plant_state(theta_eq_rad)
        error[:2] -= [position_reference_m, velocity_reference_m_s]
        return error

    def controller_state(
        self, theta_eq_rad: float, position_reference_m: float = 0.0
    ) -> NDArray[np.float64]:
        """Return the legacy zero-velocity tracking state.

        Frozen stationary callers retain their existing behavior.  Commanded
        motion must use :meth:`plant_state` for model prediction and
        :meth:`tracking_error_state` for LQR feedback.
        """

        return self.tracking_error_state(
            theta_eq_rad,
            position_reference_m,
            0.0,
        )


class LongitudinalEstimator:
    """Compose the IMU attitude and encoder odometry baselines."""

    def __init__(
        self,
        config: LongitudinalEstimatorConfig,
        encoder_profile: QuadratureEncoderProfile,
        wheel_radius_m: float,
        pll_bandwidth_rad_s: float | None = None,
        *,
        acceleration_compensation: LongitudinalAccelerationCompensationConfig | None = None,
    ):
        pll_bandwidth = (
            config.encoder_pll_bandwidth_rad_s
            if pll_bandwidth_rad_s is None
            else float(pll_bandwidth_rad_s)
        )
        self.pitch = ComplementaryPitchEstimator(config.pitch, acceleration_compensation)
        self.acceleration_compensation = acceleration_compensation
        self._previous_velocity_hat_m_s = 0.0
        self._longitudinal_acceleration_lpf_m_s2 = 0.0
        self.odometry = EncoderLongitudinalEstimator(
            encoder_profile,
            wheel_radius_m,
            config.pitch.sample_period_s,
            pll_bandwidth,
            config.encoder_pll_snap_to_zero,
        )

    def reset(
        self,
        accelerometer_m_s2: NDArray[np.float64],
        gyro_rad_s: NDArray[np.float64],
        encoder_counts: NDArray[np.int64],
        *,
        measurement_age_s: float = 0.0,
        allow_kinematic_extrapolation: bool = True,
    ) -> LongitudinalEstimate:
        pitch_measurement_time = self.pitch.reset(accelerometer_m_s2, gyro_rad_s)
        theta_control_time, extrapolation_age = self._pitch_at_control_time(
            pitch_measurement_time,
            measurement_age_s,
            allow_kinematic_extrapolation,
        )
        odometry = self.odometry.reset(encoder_counts, theta_control_time)
        self._previous_velocity_hat_m_s = odometry.velocity_hat_m_s
        self._longitudinal_acceleration_lpf_m_s2 = 0.0
        return self._combine(
            pitch_measurement_time,
            theta_control_time,
            measurement_age_s,
            extrapolation_age,
            odometry,
        )

    def update(
        self,
        accelerometer_m_s2: NDArray[np.float64],
        gyro_rad_s: NDArray[np.float64],
        encoder_counts: NDArray[np.int64],
        *,
        measurement_age_s: float = 0.0,
        allow_kinematic_extrapolation: bool = True,
    ) -> LongitudinalEstimate:
        pitch_measurement_time = self.pitch.update(
            accelerometer_m_s2,
            gyro_rad_s,
            longitudinal_acceleration_m_s2=self._longitudinal_acceleration_lpf_m_s2,
        )
        theta_control_time, extrapolation_age = self._pitch_at_control_time(
            pitch_measurement_time,
            measurement_age_s,
            allow_kinematic_extrapolation,
        )
        odometry = self.odometry.update(
            encoder_counts,
            theta_control_time,
            pitch_measurement_time.theta_dot_hat_rad_s,
        )
        if self.acceleration_compensation is not None:
            dt_s = self.pitch.config.sample_period_s
            derivative = (
                odometry.velocity_hat_m_s - self._previous_velocity_hat_m_s
            ) / dt_s
            weight = dt_s / (
                self.acceleration_compensation.acceleration_time_constant_s + dt_s
            )
            self._longitudinal_acceleration_lpf_m_s2 += weight * (
                derivative - self._longitudinal_acceleration_lpf_m_s2
            )
            self._previous_velocity_hat_m_s = odometry.velocity_hat_m_s
            # Causal one-control-step delay: preserve IMU -> odometry ordering
            # and the frozen PLL updates; do not create a fusion/odom algebraic loop.
        return self._combine(
            pitch_measurement_time,
            theta_control_time,
            measurement_age_s,
            extrapolation_age,
            odometry,
        )

    def acceleration_compensation_log_fields(self) -> dict:
        """Sensor-only experiment diagnostics at the just-consumed IMU sample."""

        return {
            "odometry_acceleration_used_m_s2": self.pitch.acceleration_used_m_s2,
            "odometry_acceleration_lpf_next_m_s2": self._longitudinal_acceleration_lpf_m_s2,
            "theta_acc_raw_rad": self.pitch.raw_theta_acc_rad,
            "theta_acc_compensated_rad": self.pitch.accel_pitch_rad(
                self.pitch.compensated_accelerometer_m_s2
            ),
            "compensated_accelerometer_m_s2": self.pitch.compensated_accelerometer_m_s2.tolist(),
            "accel_correction_gain_factor": self.pitch.accel_correction_gain_factor,
            "acceleration_estimate_causal_delay_s": self.pitch.config.sample_period_s,
        }

    @staticmethod
    def _pitch_at_control_time(
        pitch_measurement_time: PitchEstimate,
        measurement_age_s: float,
        allow_kinematic_extrapolation: bool,
    ) -> tuple[float, float]:
        """Propagate delayed pitch to now with a guarded constant-rate model."""

        age = float(measurement_age_s)
        if not math.isfinite(age) or age < 0.0:
            raise ValueError("IMU measurement age must be finite and non-negative")
        extrapolation_age = age if allow_kinematic_extrapolation else 0.0
        theta_control_time = _wrap_angle(
            pitch_measurement_time.theta_hat_rad
            + pitch_measurement_time.theta_dot_hat_rad_s * extrapolation_age
        )
        return theta_control_time, extrapolation_age

    @staticmethod
    def _combine(
        pitch_measurement_time: PitchEstimate,
        theta_control_time_rad: float,
        measurement_age_s: float,
        extrapolation_age_s: float,
        odometry: EncoderOdometryEstimate,
    ) -> LongitudinalEstimate:
        return LongitudinalEstimate(
            theta_hat_rad=theta_control_time_rad,
            theta_dot_hat_rad_s=pitch_measurement_time.theta_dot_hat_rad_s,
            theta_acc_rad=pitch_measurement_time.theta_acc_rad,
            theta_measurement_time_rad=pitch_measurement_time.theta_hat_rad,
            theta_dot_measurement_time_rad_s=(
                pitch_measurement_time.theta_dot_hat_rad_s
            ),
            imu_measurement_age_s=float(measurement_age_s),
            imu_extrapolation_age_s=float(extrapolation_age_s),
            imu_kinematic_extrapolation_applied=bool(extrapolation_age_s > 0.0),
            position_hat_m=odometry.position_hat_m,
            velocity_hat_m_s=odometry.velocity_hat_m_s,
            wheel_pll_position_counts=odometry.wheel_pll_position_counts,
            wheel_pll_velocity_counts_s=odometry.wheel_pll_velocity_counts_s,
            wheel_relative_angle_hat_rad=odometry.wheel_relative_angle_hat_rad,
            wheel_relative_velocity_hat_rad_s=(
                odometry.wheel_relative_velocity_hat_rad_s
            ),
        )
