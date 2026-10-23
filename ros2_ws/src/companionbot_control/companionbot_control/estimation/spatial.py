"""Passive spatial projection. Never consumed by the frozen balancing controller.

Pitch and path distance remain owned by LongitudinalEstimator. Navigation yaw
is distinct from the frozen controller's integral of body gyro-z. No GT, ROS,
MuJoCo, command/reference or equilibrium input is accepted here.
"""
from dataclasses import dataclass
import math

import numpy as np

from companionbot_control.core.yaw_control import GyroRelativeYawEstimator, wrap_to_pi
from .longitudinal_estimation import ComplementaryPitchEstimator


@dataclass(frozen=True)
class SpatialAttitudeEstimate:
    roll_rad: float
    yaw_rad: float
    roll_rate_rad_s: float
    yaw_rate_rad_s: float


class SpatialAttitude:
    """Reuse baseline complementary roll and timestamp-aware yaw integration.

Euler Rz(yaw) Ry(pitch) Rx(roll). Existing physical pitch is authoritative;
roll is the missing DOF, estimated from the ROS-axis accelerometer/gyro. This
is a small-tilt, planar-support baseline, not a general six-DOF inertial filter.
"""
    def __init__(self, pitch_config):
        self.roll_filter = ComplementaryPitchEstimator(pitch_config)
        self.yaw_filter = GyroRelativeYawEstimator()
        self._roll = None
        self._last_time = None

    def update(self, accel_ros, gyro_ros, pitch_at_sample, sample_time_s, age_s):
        accel, gyro = np.asarray(accel_ros, dtype=float), np.asarray(gyro_ros, dtype=float)
        if (accel.shape != (3,) or gyro.shape != (3,) or not np.isfinite(accel).all()
                or not np.isfinite(gyro).all()
                or not all(math.isfinite(x) for x in (pitch_at_sample, sample_time_s, age_s))
                or age_s < 0 or age_s > .005 + 1e-12 or abs(pitch_at_sample) >= math.pi/4):
            raise ValueError("spatial_attitude_invalid_or_outside_tilt_envelope")
        if self._last_time is not None and not math.isclose(
                sample_time_s - self._last_time, self.roll_filter.config.sample_period_s, abs_tol=1e-9):
            raise ValueError("imu_packet_sample_gap_or_reversal")
        roll = math.atan2(accel[1], accel[2]) if self._roll is None else self._roll
        roll_rate = gyro[0] + math.tan(pitch_at_sample) * (gyro[1]*math.sin(roll) + gyro[2]*math.cos(roll))
        # ComplementaryPitchEstimator's scalar tilt kernel uses accel[1:3] and
        # gyro[0], exactly the roll observation in ROS coordinates.
        estimated_roll = self.roll_filter.update(accel, np.array([roll_rate, 0., 0.]))
        self._roll = estimated_roll.theta_hat_rad
        if abs(self._roll) >= math.pi/6:
            raise ValueError("spatial_roll_outside_planar_envelope")
        yaw_rate = (gyro[1]*math.sin(self._roll) + gyro[2]*math.cos(self._roll)) / math.cos(pitch_at_sample)
        if self._last_time is None:
            yaw = self.yaw_filter.reset(yaw_rate, sample_time_s, age_s, allow_extrapolation=True)
        else:
            yaw = self.yaw_filter.update(yaw_rate, sample_time_s, age_s,
                                         valid_new_sample=True, allow_extrapolation=True)
        self._last_time = sample_time_s
        return SpatialAttitudeEstimate(self._roll + roll_rate*age_s, yaw.psi_control_rad,
                                       float(roll_rate), float(yaw_rate))


@dataclass(frozen=True)
class SpatialSample:
    epoch: str
    sequence: int
    time_s: float
    distance_m: float
    velocity_m_s: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    roll_rate_rad_s: float
    pitch_rate_rad_s: float
    yaw_rate_rad_s: float
    wheel_angles_rad: tuple[float, float]
    wheel_velocities_rad_s: tuple[float, float]
    imu_sample_time_s: float
    imu_available_time_s: float
    imu_age_s: float
    imu_valid: bool = True
    imu_stale: bool = False
    encoder_valid: bool = True

    def validate(self):
        numbers = [self.time_s, self.distance_m, self.velocity_m_s, self.roll_rad, self.pitch_rad,
                   self.yaw_rad, self.roll_rate_rad_s, self.pitch_rate_rad_s, self.yaw_rate_rad_s,
                   *self.wheel_angles_rad, *self.wheel_velocities_rad_s,
                   self.imu_sample_time_s, self.imu_available_time_s, self.imu_age_s]
        if not self.epoch or not isinstance(self.sequence, int) or isinstance(self.sequence, bool) or self.sequence < 1:
            raise ValueError("invalid_source_identity")
        if len(self.wheel_angles_rad) != 2 or len(self.wheel_velocities_rad_s) != 2 or not np.isfinite(numbers).all():
            raise ValueError("nonfinite_or_malformed_state")
        if not self.imu_valid or self.imu_stale or not self.encoder_valid:
            raise ValueError("sensor_invalid_or_stale")
        if (self.time_s < 0 or self.imu_sample_time_s > self.imu_available_time_s + 1e-9
                or self.imu_available_time_s > self.time_s + 1e-9 or self.imu_age_s < -1e-9
                or self.imu_age_s > .005 + 1e-9
                or not math.isclose(self.time_s - self.imu_sample_time_s, self.imu_age_s, abs_tol=1e-9)):
            raise ValueError("inconsistent_sensor_source_time")
        if abs(self.pitch_rad) >= math.pi/4 or abs(self.roll_rad) >= math.pi/6:
            raise ValueError("outside_planar_tilt_envelope")


@dataclass(frozen=True)
class PlanarEstimate:
    x_m: float
    y_m: float
    yaw_rad: float
    covariance: np.ndarray


class PlanarOdometry:
    """Integrate existing pitch-corrected path increments, with explicit drift.

One epoch per instance; origin is the first accepted sample. Assumed process
uncertainties are declared, not learned from acceptance GT or claimed calibrated.
Any invalid stream latches a fault; restarting the owner explicitly starts a new
odom epoch. No hidden reconnect/reset or extrapolated motion after sensor loss.
"""
    def __init__(self, velocity_std=.03, yaw_rate_std=.02, position_rw=.005, yaw_rw=.002, max_gap_s=.12):
        if not np.isfinite([velocity_std, yaw_rate_std, position_rw, yaw_rw, max_gap_s]).all() or min(
                velocity_std, yaw_rate_std, position_rw, yaw_rw, max_gap_s) <= 0:
            raise ValueError("invalid_odometry_uncertainty")
        self.velocity_std, self.yaw_rate_std = velocity_std, yaw_rate_std
        self.position_rw, self.yaw_rw, self.max_gap_s = position_rw, yaw_rw, max_gap_s
        self.previous = None
        self.x = self.y = self.yaw = 0.
        self.covariance = np.zeros((3, 3))
        self.fault = ""

    def update(self, sample):
        if self.fault:
            raise ValueError("odometry_fault_latched:" + self.fault)
        try:
            sample.validate()
            if self.previous is not None:
                old = self.previous
                dt = sample.time_s - old.time_s
                if old.epoch != sample.epoch:
                    raise ValueError("source_epoch_changed_restart_required")
                if sample.sequence <= old.sequence or dt <= 0 or dt > self.max_gap_s:
                    raise ValueError("sequence_time_reversal_or_gap")
                ds = sample.distance_m - old.distance_m
                dyaw = wrap_to_pi(sample.yaw_rad - old.yaw_rad)
                middle = self.yaw + dyaw/2
                travel = ds * float(np.sinc(dyaw/(2*math.pi)))
                self.x += travel * math.cos(middle)
                self.y += travel * math.sin(middle)
                # Midpoint Jacobians for conservative process-noise propagation.
                c, s = math.cos(middle), math.sin(middle)
                f = np.array([[1, 0, -travel*s], [0, 1, travel*c], [0, 0, 1]])
                j = np.array([[c, -.5*travel*s], [s, .5*travel*c], [0, 1]])
                noise = np.diag([self.velocity_std**2*dt**2 + self.position_rw**2*dt,
                                 self.yaw_rate_std**2*dt**2 + self.yaw_rw**2*dt])
                self.covariance = f @ self.covariance @ f.T + j @ noise @ j.T
                self.yaw += dyaw
            if (not np.isfinite([self.x, self.y, self.yaw]).all()
                    or not np.isfinite(self.covariance).all()):
                raise ValueError("nonfinite_derived_odometry")
            self.previous = sample
        except ValueError as exc:
            self.fault = str(exc)
            raise
        return PlanarEstimate(self.x, self.y, self.yaw, self.covariance.copy())
