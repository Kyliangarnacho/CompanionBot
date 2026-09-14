"""Timestamped RotorS/PX4-style virtual IMU hardware pipeline."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np
from numpy.typing import NDArray


DEFAULT_IMU_HARDWARE_CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "models"
    / "minisegway"
    / "imu_hardware_config.json"
)
_TIME_TOLERANCE_S = 1e-12


@dataclass(frozen=True)
class ImuAxisNoiseParameters:
    noise_density: float
    random_walk: float
    bias_correlation_time_s: float
    turn_on_bias_sigma: float
    full_scale: float


@dataclass(frozen=True)
class VirtualImuHardwareConfig:
    profile_name: str
    sample_period_s: float
    availability_latency_s: float
    stale_threshold_s: float
    startup_gyro_calibration_duration_s: float
    rng_algorithm: str
    default_seed: int
    gyroscope: ImuAxisNoiseParameters
    accelerometer: ImuAxisNoiseParameters

    @property
    def startup_gyro_calibration_sample_count(self) -> int:
        return round(
            self.startup_gyro_calibration_duration_s / self.sample_period_s
        )


@dataclass(frozen=True)
class ImuHardwareSample:
    """One generated stochastic sample before it is wrapped in a packet."""

    ideal_accelerometer_m_s2: NDArray[np.float64]
    ideal_gyro_rad_s: NDArray[np.float64]
    preclip_accelerometer_m_s2: NDArray[np.float64]
    preclip_gyro_rad_s: NDArray[np.float64]
    noisy_accelerometer_m_s2: NDArray[np.float64]
    noisy_gyro_rad_s: NDArray[np.float64]
    accelerometer_saturated: NDArray[np.bool_]
    gyro_saturated: NDArray[np.bool_]
    accelerometer_turn_on_bias_m_s2: NDArray[np.float64]
    gyro_turn_on_bias_rad_s: NDArray[np.float64]
    accelerometer_bias_drift_m_s2: NDArray[np.float64]
    gyro_bias_drift_rad_s: NDArray[np.float64]
    accelerometer_white_noise_m_s2: NDArray[np.float64]
    gyro_white_noise_rad_s: NDArray[np.float64]


@dataclass(frozen=True)
class ImuPacket:
    """Immutable sensor packet generated exactly once per 1 kHz sensor tick."""

    sample: ImuHardwareSample
    sample_time_s: float
    available_time_s: float
    valid: bool

    @property
    def accelerometer_m_s2(self) -> NDArray[np.float64]:
        return self.sample.noisy_accelerometer_m_s2

    @property
    def gyroscope_rad_s(self) -> NDArray[np.float64]:
        return self.sample.noisy_gyro_rad_s


@dataclass(frozen=True)
class ImuReadout:
    """Latest available guarded IMU measurement delivered to estimation."""

    accelerometer_m_s2: NDArray[np.float64]
    gyroscope_rad_s: NDArray[np.float64]
    packet: ImuPacket
    measurement_age_s: float
    gyro_bias_estimate_rad_s: NDArray[np.float64]
    calibration_complete: bool
    imu_invalid: bool
    imu_stale: bool

    def measurement(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        return self.accelerometer_m_s2.copy(), self.gyroscope_rad_s.copy()

    def raw_log_fields(self) -> dict:
        sample = self.packet.sample
        return {
            "imu_accelerometer_ideal_raw_m_s2": (
                sample.ideal_accelerometer_m_s2.tolist()
            ),
            "imu_gyro_ideal_raw_rad_s": sample.ideal_gyro_rad_s.tolist(),
            "imu_accelerometer_noisy_raw_m_s2": (
                sample.noisy_accelerometer_m_s2.tolist()
            ),
            "imu_gyro_noisy_raw_rad_s": sample.noisy_gyro_rad_s.tolist(),
            "imu_gyro_corrected_rad_s": self.gyroscope_rad_s.tolist(),
            "imu_sample_time_s": self.packet.sample_time_s,
            "imu_available_time_s": self.packet.available_time_s,
            "imu_measurement_age_s": self.measurement_age_s,
            "imu_gyro_bias_estimate_rad_s": (
                self.gyro_bias_estimate_rad_s.tolist()
            ),
            "imu_calibration_complete": self.calibration_complete,
            "imu_gyro_saturated": sample.gyro_saturated.tolist(),
            "imu_accelerometer_saturated": (
                sample.accelerometer_saturated.tolist()
            ),
            "imu_valid": self.packet.valid and not self.imu_invalid,
            "imu_stale": self.imu_stale,
        }


def _axis_parameters(
    raw: dict, expected_units: dict[str, str]
) -> ImuAxisNoiseParameters:
    for key, expected_unit in expected_units.items():
        actual_unit = str(raw[key]["unit"])
        if actual_unit != expected_unit:
            raise ValueError(
                f"unexpected unit for {key}: expected {expected_unit}, got {actual_unit}"
            )
    parameters = ImuAxisNoiseParameters(
        noise_density=float(raw["noise_density"]["value"]),
        random_walk=float(raw["random_walk"]["value"]),
        bias_correlation_time_s=float(raw["bias_correlation_time"]["value"]),
        turn_on_bias_sigma=float(raw["turn_on_bias_sigma"]["value"]),
        full_scale=float(raw["full_scale"]["value"]),
    )
    if (
        parameters.noise_density < 0.0
        or parameters.random_walk < 0.0
        or parameters.bias_correlation_time_s <= 0.0
        or parameters.turn_on_bias_sigma < 0.0
        or parameters.full_scale <= 0.0
    ):
        raise ValueError(
            "IMU noise sigmas must be non-negative; tau/full-scale must be positive"
        )
    return parameters


def load_imu_hardware_config(
    path: str | Path = DEFAULT_IMU_HARDWARE_CONFIG_PATH,
) -> VirtualImuHardwareConfig:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    active_profile = str(raw["active_profile"])
    profile = raw["profile"]
    if active_profile != str(profile["name"]):
        raise ValueError("active IMU profile must match the single configured profile")
    sample_period_s = float(raw["sample_period_s"])
    latency_s = float(raw["availability_latency_s"])
    stale_threshold_s = float(raw["stale_threshold_s"])
    calibration_duration_s = float(raw["startup_gyro_calibration_duration_s"])
    rng_algorithm = str(raw["rng_algorithm"])
    default_seed = int(raw["default_seed"])
    if sample_period_s <= 0.0:
        raise ValueError("IMU sample period must be positive")
    if latency_s < 0.0 or stale_threshold_s <= 0.0:
        raise ValueError("IMU latency must be non-negative and stale threshold positive")
    if calibration_duration_s <= 0.0:
        raise ValueError("gyro calibration duration must be positive")
    calibration_samples = calibration_duration_s / sample_period_s
    if not math.isclose(
        calibration_samples,
        round(calibration_samples),
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("gyro calibration duration must contain an integer sample count")
    if default_seed < 0:
        raise ValueError("IMU RNG seed must be non-negative")
    if rng_algorithm != "numpy.random.PCG64":
        raise ValueError(f"unsupported IMU RNG algorithm: {rng_algorithm}")
    return VirtualImuHardwareConfig(
        profile_name=active_profile,
        sample_period_s=sample_period_s,
        availability_latency_s=latency_s,
        stale_threshold_s=stale_threshold_s,
        startup_gyro_calibration_duration_s=calibration_duration_s,
        rng_algorithm=rng_algorithm,
        default_seed=default_seed,
        gyroscope=_axis_parameters(
            profile["gyroscope"],
            {
                "noise_density": "rad/s/sqrt(Hz)",
                "random_walk": "rad/s^2/sqrt(Hz)",
                "bias_correlation_time": "s",
                "turn_on_bias_sigma": "rad/s",
                "full_scale": "rad/s",
            },
        ),
        accelerometer=_axis_parameters(
            profile["accelerometer"],
            {
                "noise_density": "m/s^2/sqrt(Hz)",
                "random_walk": "m/s^3/sqrt(Hz)",
                "bias_correlation_time": "s",
                "turn_on_bias_sigma": "m/s^2",
                "full_scale": "m/s^2",
            },
        ),
    )


class VirtualImuHardware:
    """Three-axis RotorS noise process and full-scale clipping."""

    def __init__(
        self, config: VirtualImuHardwareConfig, seed: int | None = None
    ):
        self.config = config
        self.seed = config.default_seed if seed is None else int(seed)
        if self.seed < 0:
            raise ValueError("IMU RNG seed must be non-negative")
        self._rng = np.random.Generator(np.random.PCG64(self.seed))
        self.reset_count = 0
        self.sample_count = 0
        self.last_sample: ImuHardwareSample | None = None
        self.reset(seed=self.seed)

    @staticmethod
    def _discrete_coefficients(
        parameters: ImuAxisNoiseParameters, dt: float
    ) -> tuple[float, float, float]:
        sigma_white = parameters.noise_density / math.sqrt(dt)
        tau = parameters.bias_correlation_time_s
        phi = math.exp(-dt / tau)
        sigma_bias_discrete = math.sqrt(
            -parameters.random_walk**2
            * tau
            / 2.0
            * (math.exp(-2.0 * dt / tau) - 1.0)
        )
        return sigma_white, phi, sigma_bias_discrete

    def reset(self, seed: int | None = None) -> None:
        """Clear drift and sample each axis' turn-on bias exactly once."""

        if seed is not None:
            self.seed = int(seed)
            if self.seed < 0:
                raise ValueError("IMU RNG seed must be non-negative")
            self._rng = np.random.Generator(np.random.PCG64(self.seed))
        gyro_turn_on = np.empty(3, dtype=float)
        accel_turn_on = np.empty(3, dtype=float)
        for axis in range(3):
            gyro_turn_on[axis] = (
                self.config.gyroscope.turn_on_bias_sigma
                * float(self._rng.standard_normal())
            )
            accel_turn_on[axis] = (
                self.config.accelerometer.turn_on_bias_sigma
                * float(self._rng.standard_normal())
            )
        self.gyro_turn_on_bias_rad_s = gyro_turn_on
        self.accelerometer_turn_on_bias_m_s2 = accel_turn_on
        self.gyro_bias_drift_rad_s = np.zeros(3, dtype=float)
        self.accelerometer_bias_drift_m_s2 = np.zeros(3, dtype=float)
        self.sample_count = 0
        self.reset_count += 1
        self.last_sample = None

    @staticmethod
    def _vector(name: str, value: NDArray[np.float64]) -> NDArray[np.float64]:
        vector = np.asarray(value, dtype=float)
        if vector.shape != (3,):
            raise ValueError(f"{name} must be a length-3 vector")
        return vector.copy()

    def sample(
        self,
        ideal_accelerometer_m_s2: NDArray[np.float64],
        ideal_gyro_rad_s: NDArray[np.float64],
    ) -> ImuHardwareSample:
        ideal_accel = self._vector("ideal accelerometer", ideal_accelerometer_m_s2)
        ideal_gyro = self._vector("ideal gyro", ideal_gyro_rad_s)
        dt = self.config.sample_period_s
        gyro_white_sigma, gyro_phi, gyro_bias_sigma = self._discrete_coefficients(
            self.config.gyroscope, dt
        )
        accel_white_sigma, accel_phi, accel_bias_sigma = (
            self._discrete_coefficients(self.config.accelerometer, dt)
        )

        gyro_white = np.empty(3, dtype=float)
        accel_white = np.empty(3, dtype=float)
        for axis in range(3):
            self.gyro_bias_drift_rad_s[axis] = (
                gyro_phi * self.gyro_bias_drift_rad_s[axis]
                + gyro_bias_sigma * float(self._rng.standard_normal())
            )
            gyro_white[axis] = gyro_white_sigma * float(
                self._rng.standard_normal()
            )
        for axis in range(3):
            self.accelerometer_bias_drift_m_s2[axis] = (
                accel_phi * self.accelerometer_bias_drift_m_s2[axis]
                + accel_bias_sigma * float(self._rng.standard_normal())
            )
            accel_white[axis] = accel_white_sigma * float(
                self._rng.standard_normal()
            )

        preclip_gyro = (
            ideal_gyro
            + self.gyro_turn_on_bias_rad_s
            + self.gyro_bias_drift_rad_s
            + gyro_white
        )
        preclip_accel = (
            ideal_accel
            + self.accelerometer_turn_on_bias_m_s2
            + self.accelerometer_bias_drift_m_s2
            + accel_white
        )
        gyro_saturated = np.abs(preclip_gyro) > self.config.gyroscope.full_scale
        accel_saturated = (
            np.abs(preclip_accel) > self.config.accelerometer.full_scale
        )
        noisy_gyro = np.clip(
            preclip_gyro,
            -self.config.gyroscope.full_scale,
            self.config.gyroscope.full_scale,
        )
        noisy_accel = np.clip(
            preclip_accel,
            -self.config.accelerometer.full_scale,
            self.config.accelerometer.full_scale,
        )
        sample = ImuHardwareSample(
            ideal_accelerometer_m_s2=ideal_accel,
            ideal_gyro_rad_s=ideal_gyro,
            preclip_accelerometer_m_s2=preclip_accel.copy(),
            preclip_gyro_rad_s=preclip_gyro.copy(),
            noisy_accelerometer_m_s2=noisy_accel.copy(),
            noisy_gyro_rad_s=noisy_gyro.copy(),
            accelerometer_saturated=accel_saturated.copy(),
            gyro_saturated=gyro_saturated.copy(),
            accelerometer_turn_on_bias_m_s2=(
                self.accelerometer_turn_on_bias_m_s2.copy()
            ),
            gyro_turn_on_bias_rad_s=self.gyro_turn_on_bias_rad_s.copy(),
            accelerometer_bias_drift_m_s2=(
                self.accelerometer_bias_drift_m_s2.copy()
            ),
            gyro_bias_drift_rad_s=self.gyro_bias_drift_rad_s.copy(),
            accelerometer_white_noise_m_s2=accel_white.copy(),
            gyro_white_noise_rad_s=gyro_white.copy(),
        )
        self.sample_count += 1
        self.last_sample = sample
        return sample


class VirtualImuSensor:
    """Sensor-clock packet generation, calibration, availability, and guards."""

    def __init__(
        self, config: VirtualImuHardwareConfig, seed: int | None = None
    ):
        self.config = config
        self.hardware = VirtualImuHardware(config, seed)
        queue_length = max(
            8,
            math.ceil(
                (config.availability_latency_s + config.stale_threshold_s)
                / config.sample_period_s
            )
            + 4,
        )
        self._packets: deque[ImuPacket] = deque(maxlen=queue_length)
        self.reset_count = 0
        self.reset(seed=self.hardware.seed)

    @property
    def seed(self) -> int:
        return self.hardware.seed

    def reset(self, seed: int | None = None) -> None:
        self.hardware.reset(seed=seed)
        self._packets.clear()
        self._last_generated_time_s: float | None = None
        self._last_generated_packet: ImuPacket | None = None
        self._last_valid_packet: ImuPacket | None = None
        self._last_read_time_s: float | None = None
        self._last_readout: ImuReadout | None = None
        self.gyro_bias_estimate_rad_s = np.zeros(3, dtype=float)
        self.calibration_complete = False
        self.calibration_valid_sample_count = 0
        self.generated_packet_count = 0
        self.runtime_packet_count = 0
        self.runtime_gyro_saturation_packet_count = 0
        self.runtime_accelerometer_saturation_packet_count = 0
        self.runtime_gyro_saturation_axis_count = 0
        self.runtime_accelerometer_saturation_axis_count = 0
        self.invalid_read_count = 0
        self.stale_read_count = 0
        self.read_count = 0
        self._measurement_ages_s: list[float] = []
        self.reset_count += 1

    def generate_packet(
        self,
        ideal_accelerometer_m_s2: NDArray[np.float64],
        ideal_gyro_rad_s: NDArray[np.float64],
        sample_time_s: float,
    ) -> ImuPacket:
        """Generate or return the one cached packet for this sensor tick."""

        sample_time = float(sample_time_s)
        if not math.isfinite(sample_time):
            raise ValueError("IMU sample time must be finite")
        if self._last_generated_time_s is not None:
            if math.isclose(
                sample_time,
                self._last_generated_time_s,
                rel_tol=0.0,
                abs_tol=_TIME_TOLERANCE_S,
            ):
                assert self._last_generated_packet is not None
                return self._last_generated_packet
            if sample_time < self._last_generated_time_s:
                raise ValueError("IMU sample times must be strictly increasing")

        sample = self.hardware.sample(
            ideal_accelerometer_m_s2, ideal_gyro_rad_s
        )
        available_time = sample_time + self.config.availability_latency_s
        valid = bool(
            math.isfinite(available_time)
            and np.isfinite(sample.noisy_accelerometer_m_s2).all()
            and np.isfinite(sample.noisy_gyro_rad_s).all()
        )
        packet = ImuPacket(sample, sample_time, available_time, valid)
        self._packets.append(packet)
        self._last_generated_time_s = sample_time
        self._last_generated_packet = packet
        self.generated_packet_count += 1
        if sample_time >= 0.0:
            self.runtime_packet_count += 1
            self.runtime_gyro_saturation_packet_count += int(
                np.any(sample.gyro_saturated)
            )
            self.runtime_accelerometer_saturation_packet_count += int(
                np.any(sample.accelerometer_saturated)
            )
            self.runtime_gyro_saturation_axis_count += int(
                np.count_nonzero(sample.gyro_saturated)
            )
            self.runtime_accelerometer_saturation_axis_count += int(
                np.count_nonzero(sample.accelerometer_saturated)
            )
        return packet

    def calibrate_stationary(
        self,
        ideal_accelerometer_m_s2: NDArray[np.float64],
        ideal_gyro_rad_s: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Estimate zero-rate gyro bias solely from noisy raw packets."""

        if self.generated_packet_count != 0:
            raise RuntimeError("stationary calibration must immediately follow reset")
        count = self.config.startup_gyro_calibration_sample_count
        duration = self.config.startup_gyro_calibration_duration_s
        gyro_samples = []
        for index in range(count):
            sample_time = -duration + index * self.config.sample_period_s
            packet = self.generate_packet(
                ideal_accelerometer_m_s2,
                ideal_gyro_rad_s,
                sample_time,
            )
            if packet.valid:
                gyro_samples.append(packet.gyroscope_rad_s.copy())
        if len(gyro_samples) != count:
            raise RuntimeError("stationary gyro calibration received invalid packets")
        self.gyro_bias_estimate_rad_s = np.mean(
            np.asarray(gyro_samples, dtype=float), axis=0
        )
        self.calibration_valid_sample_count = len(gyro_samples)
        self.calibration_complete = True
        return self.gyro_bias_estimate_rad_s.copy()

    @staticmethod
    def _packet_finite(packet: ImuPacket) -> bool:
        return bool(
            packet.valid
            and math.isfinite(packet.sample_time_s)
            and math.isfinite(packet.available_time_s)
            and packet.available_time_s + _TIME_TOLERANCE_S
            >= packet.sample_time_s
            and np.isfinite(packet.accelerometer_m_s2).all()
            and np.isfinite(packet.gyroscope_rad_s).all()
        )

    def read(self, current_time_s: float) -> ImuReadout:
        """Read the latest packet whose availability time has arrived."""

        current_time = float(current_time_s)
        if not math.isfinite(current_time):
            raise ValueError("IMU consumer time must be finite")
        if not self.calibration_complete:
            raise RuntimeError("gyro calibration must complete before IMU use")
        if (
            self._last_read_time_s is not None
            and math.isclose(
                current_time,
                self._last_read_time_s,
                rel_tol=0.0,
                abs_tol=_TIME_TOLERANCE_S,
            )
        ):
            assert self._last_readout is not None
            return self._last_readout

        candidate = None
        for packet in reversed(self._packets):
            if packet.available_time_s <= current_time + _TIME_TOLERANCE_S:
                candidate = packet
                break
        invalid = candidate is None or not self._packet_finite(candidate)
        candidate_age = (
            float("inf")
            if candidate is None
            else current_time - candidate.sample_time_s
        )
        stale = bool(candidate is not None and candidate_age > self.config.stale_threshold_s)
        if not invalid and not stale:
            assert candidate is not None
            self._last_valid_packet = candidate
        source = candidate if not invalid and not stale else self._last_valid_packet
        if source is None:
            self.invalid_read_count += int(invalid)
            self.stale_read_count += int(stale)
            raise RuntimeError("no valid previous IMU measurement is available")

        measurement_age = current_time - source.sample_time_s
        readout = ImuReadout(
            accelerometer_m_s2=source.accelerometer_m_s2.copy(),
            gyroscope_rad_s=(
                source.gyroscope_rad_s - self.gyro_bias_estimate_rad_s
            ),
            packet=source,
            measurement_age_s=measurement_age,
            gyro_bias_estimate_rad_s=self.gyro_bias_estimate_rad_s.copy(),
            calibration_complete=self.calibration_complete,
            imu_invalid=invalid,
            imu_stale=stale,
        )
        self.invalid_read_count += int(invalid)
        self.stale_read_count += int(stale)
        self.read_count += 1
        self._measurement_ages_s.append(measurement_age)
        self._last_read_time_s = current_time
        self._last_readout = readout
        return readout

    @property
    def last_readout(self) -> ImuReadout | None:
        return self._last_readout

    def diagnostics(self, include_hidden_truth: bool = False) -> dict:
        ages = np.asarray(self._measurement_ages_s, dtype=float)
        result = {
            "profile": self.config.profile_name,
            "rng_seed": self.seed,
            "sensor_sample_period_s": self.config.sample_period_s,
            "availability_latency_s": self.config.availability_latency_s,
            "stale_threshold_s": self.config.stale_threshold_s,
            "generated_packet_count_including_calibration": (
                self.generated_packet_count
            ),
            "runtime_packet_count": self.runtime_packet_count,
            "calibration_duration_s": (
                self.config.startup_gyro_calibration_duration_s
            ),
            "calibration_expected_sample_count": (
                self.config.startup_gyro_calibration_sample_count
            ),
            "calibration_valid_sample_count": self.calibration_valid_sample_count,
            "calibration_complete": self.calibration_complete,
            "gyro_bias_estimate_rad_s": self.gyro_bias_estimate_rad_s.tolist(),
            "runtime_gyro_saturation_packet_count": (
                self.runtime_gyro_saturation_packet_count
            ),
            "runtime_gyro_saturation_packet_fraction": (
                self.runtime_gyro_saturation_packet_count
                / max(self.runtime_packet_count, 1)
            ),
            "runtime_accelerometer_saturation_packet_count": (
                self.runtime_accelerometer_saturation_packet_count
            ),
            "runtime_accelerometer_saturation_packet_fraction": (
                self.runtime_accelerometer_saturation_packet_count
                / max(self.runtime_packet_count, 1)
            ),
            "runtime_gyro_saturation_axis_count": (
                self.runtime_gyro_saturation_axis_count
            ),
            "runtime_accelerometer_saturation_axis_count": (
                self.runtime_accelerometer_saturation_axis_count
            ),
            "consumer_read_count": self.read_count,
            "invalid_read_count": self.invalid_read_count,
            "stale_read_count": self.stale_read_count,
            "measurement_age_typical_s": (
                float(np.median(ages)) if ages.size else None
            ),
            "measurement_age_max_s": float(np.max(ages)) if ages.size else None,
        }
        if include_hidden_truth:
            result["posthoc_hidden_truth_only"] = {
                "gyro_turn_on_bias_rad_s": (
                    self.hardware.gyro_turn_on_bias_rad_s.tolist()
                ),
                "accelerometer_turn_on_bias_m_s2": (
                    self.hardware.accelerometer_turn_on_bias_m_s2.tolist()
                ),
                "gyro_bias_drift_final_rad_s": (
                    self.hardware.gyro_bias_drift_rad_s.tolist()
                ),
                "accelerometer_bias_drift_final_m_s2": (
                    self.hardware.accelerometer_bias_drift_m_s2.tolist()
                ),
                "control_boundary": "never exposed by imu_measurement(); post-hoc diagnostics only",
            }
        return result
