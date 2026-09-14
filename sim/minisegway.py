"""Minimal fixed-step interface for the passive MiniSegway MuJoCo plant."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from .virtual_imu import (
    DEFAULT_IMU_HARDWARE_CONFIG_PATH,
    ImuHardwareSample,
    ImuReadout,
    VirtualImuSensor,
    load_imu_hardware_config,
)


DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "minisegway" / "mini_segway.xml"
DEFAULT_ENCODER_PROFILE_PATH = (
    Path(__file__).resolve().parents[1] / "models" / "minisegway" / "encoder_profile.json"
)


@dataclass(frozen=True)
class QuadratureEncoderProfile:
    name: str
    motor_shaft_decoded_counts_per_rev: int
    exact_gearbox_ratio: float
    output_decoded_counts_per_rev: float
    quantization: str
    left_count_sign: int
    right_count_sign: int

    def counts_from_angle_delta(self, angle_delta_rad: NDArray[np.float64]) -> NDArray[np.int64]:
        """Quantize unwrapped output-shaft angle relative to the captured zero."""

        angles = np.asarray(angle_delta_rad, dtype=float)
        if angles.shape != (2,) or not np.isfinite(angles).all():
            raise ValueError("left/right encoder angle deltas must be a finite length-2 vector")
        signs = np.asarray([self.left_count_sign, self.right_count_sign], dtype=float)
        scaled_counts = angles * signs * self.output_decoded_counts_per_rev / (2.0 * math.pi)
        return np.copysign(np.floor(np.abs(scaled_counts) + 0.5), scaled_counts).astype(np.int64)


def load_encoder_profile(
    path: str | Path = DEFAULT_ENCODER_PROFILE_PATH,
) -> QuadratureEncoderProfile:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    profile = QuadratureEncoderProfile(**raw)
    expected_output_resolution = (
        profile.motor_shaft_decoded_counts_per_rev * profile.exact_gearbox_ratio
    )
    if profile.motor_shaft_decoded_counts_per_rev <= 0 or profile.exact_gearbox_ratio <= 0.0:
        raise ValueError("encoder resolution and gearbox ratio must be positive")
    if not math.isclose(
        profile.output_decoded_counts_per_rev,
        expected_output_resolution,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("output encoder resolution must equal motor resolution times gearbox ratio")
    if profile.quantization != "round_to_nearest_away_from_zero":
        raise ValueError(f"unsupported encoder quantization: {profile.quantization}")
    if profile.left_count_sign not in (-1, 1) or profile.right_count_sign not in (-1, 1):
        raise ValueError("left/right encoder count signs must be +1 or -1")
    return profile


@dataclass(frozen=True)
class SimSnapshot:
    step_index: int
    time_s: float
    qpos: NDArray[np.float64]
    qvel: NDArray[np.float64]
    applied_ctrl_nm: NDArray[np.float64]
    wheel_encoder_counts: NDArray[np.int64]


class MiniSegwaySim:
    """One call to :meth:`step` advances exactly one MuJoCo physics timestep."""

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        encoder_profile_path: str | Path = DEFAULT_ENCODER_PROFILE_PATH,
        imu_hardware_config_path: str | Path = DEFAULT_IMU_HARDWARE_CONFIG_PATH,
        imu_seed: int | None = None,
    ):
        self.model_path = Path(model_path).resolve()
        self.encoder_profile_path = Path(encoder_profile_path).resolve()
        self.encoder_profile = load_encoder_profile(self.encoder_profile_path)
        self.imu_hardware_config_path = Path(imu_hardware_config_path).resolve()
        self.imu_hardware_config = load_imu_hardware_config(
            self.imu_hardware_config_path
        )
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._left_actuator = self.model.actuator("left_wheel_torque").id
        self._right_actuator = self.model.actuator("right_wheel_torque").id
        root_joint = self.model.joint("root").id
        self._root_qpos = int(self.model.jnt_qposadr[root_joint])
        self._root_dof = int(self.model.jnt_dofadr[root_joint])
        self._wheel_angle_sensor_adrs = np.asarray(
            [
                self.model.sensor_adr[self.model.sensor("left_wheel_angle").id],
                self.model.sensor_adr[self.model.sensor("right_wheel_angle").id],
            ],
            dtype=int,
        )
        self._accelerometer_sensor = self.model.sensor("imu_accelerometer").id
        self._gyro_sensor = self.model.sensor("imu_gyro").id
        if not math.isclose(
            self.imu_hardware_config.sample_period_s,
            self.physics_dt,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError(
                "V1 IMU sample period must equal the MuJoCo physics timestep"
            )
        self.step_index = 0
        self._imu_sensor: VirtualImuSensor | None = None
        self.reset()
        self._imu_sensor = VirtualImuSensor(
            self.imu_hardware_config,
            seed=imu_seed,
        )

    @property
    def physics_dt(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def imu_sample_period_s(self) -> float:
        return self.imu_hardware_config.sample_period_s

    @property
    def imu_seed(self) -> int:
        if self._imu_sensor is None:
            return self.imu_hardware_config.default_seed
        return self._imu_sensor.seed

    @property
    def imu_profile_name(self) -> str:
        return self.imu_hardware_config.profile_name

    def reset(
        self,
        pitch_rad: float | None = None,
        imu_seed: int | None = None,
    ) -> SimSnapshot:
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        if pitch_rad is not None:
            half_angle = pitch_rad / 2.0
            self.data.qpos[self._root_qpos + 3 : self._root_qpos + 7] = [
                math.cos(half_angle),
                math.sin(half_angle),
                0.0,
                0.0,
            ]
        mujoco.mj_forward(self.model, self.data)
        self._encoder_zero_angles_rad = self._raw_wheel_angles_rad()
        if self._imu_sensor is not None:
            self._imu_sensor.reset(seed=imu_seed)
        self.step_index = 0
        return self.snapshot()

    def step(self, left_torque_nm: float, right_torque_nm: float) -> SimSnapshot:
        torques = np.asarray([left_torque_nm, right_torque_nm], dtype=float)
        if not np.isfinite(torques).all():
            raise ValueError("wheel torque commands must be finite")
        self.data.ctrl[self._left_actuator] = torques[0]
        self.data.ctrl[self._right_actuator] = torques[1]
        mujoco.mj_step(self.model, self.data)
        self.step_index += 1
        self._generate_imu_sensor_packet()
        return self.snapshot()

    def step_hold(self, left_torque_nm: float, right_torque_nm: float, physics_steps: int) -> SimSnapshot:
        if physics_steps < 1:
            raise ValueError("physics_steps must be at least one")
        snapshot = self.snapshot()
        for _ in range(physics_steps):
            snapshot = self.step(left_torque_nm, right_torque_nm)
        return snapshot

    def snapshot(self) -> SimSnapshot:
        return SimSnapshot(
            step_index=self.step_index,
            time_s=float(self.data.time),
            qpos=self.data.qpos.copy(),
            qvel=self.data.qvel.copy(),
            applied_ctrl_nm=self.data.actuator_force[[self._left_actuator, self._right_actuator]].copy(),
            wheel_encoder_counts=self.wheel_encoder_counts(),
        )

    def _raw_wheel_angles_rad(self) -> NDArray[np.float64]:
        """Read MuJoCo joint-angle sensors used only inside the virtual encoder."""

        return self.data.sensordata[self._wheel_angle_sensor_adrs].copy()

    def wheel_encoder_counts(self) -> NDArray[np.int64]:
        """Return MCU-style signed integer counts accumulated since the last reset."""

        angle_delta = self._raw_wheel_angles_rad() - self._encoder_zero_angles_rad
        return self.encoder_profile.counts_from_angle_delta(angle_delta)

    def ideal_imu_measurement(
        self,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Return ideal MuJoCo chassis-frame IMU vectors for diagnostics only."""

        return (
            np.asarray(self.data.sensor(self._accelerometer_sensor).data, dtype=float).copy(),
            np.asarray(self.data.sensor(self._gyro_sensor).data, dtype=float).copy(),
        )

    def _generate_imu_sensor_packet(self) -> None:
        if self._imu_sensor is None:
            return
        # Refresh native derived sensors at this exact post-step timestamp.
        mujoco.mj_forward(self.model, self.data)
        ideal_accelerometer, ideal_gyro = self.ideal_imu_measurement()
        self._imu_sensor.generate_packet(
            ideal_accelerometer,
            ideal_gyro,
            float(self.data.time),
        )

    def calibrate_imu_stationary(self) -> NDArray[np.float64]:
        """Run the 0.5 s pre-run held zero-rate gyro calibration phase."""

        if self._imu_sensor is None:
            raise RuntimeError("virtual IMU sensor is not initialized")
        if self.step_index != 0 or not math.isclose(
            float(self.data.time), 0.0, rel_tol=0.0, abs_tol=1e-12
        ):
            raise RuntimeError("stationary IMU calibration must run immediately after reset")
        self.data.qacc[:] = 0.0
        mujoco.mj_sensorAcc(self.model, self.data)
        ideal_accelerometer, ideal_gyro = self.ideal_imu_measurement()
        return self._imu_sensor.calibrate_stationary(
            ideal_accelerometer, ideal_gyro
        )

    def imu_measurement(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Return noisy virtual-hardware IMU vectors for estimator consumption."""

        if self._imu_sensor is None:
            raise RuntimeError("virtual IMU sensor is not initialized")
        return self._imu_sensor.read(float(self.data.time)).measurement()

    def imu_estimator_input(
        self,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], float, bool]:
        """Return measurement, actual packet age, and extrapolation permission.

        Timing metadata is part of the virtual hardware interface.  Truth and
        generated noise components remain outside the estimator input.  A stale
        or invalid readout reuses the existing guarded measurement but disables
        further kinematic extrapolation.
        """

        if self._imu_sensor is None:
            raise RuntimeError("virtual IMU sensor is not initialized")
        readout = self._imu_sensor.read(float(self.data.time))
        accelerometer, gyro = readout.measurement()
        extrapolation_allowed = not (readout.imu_invalid or readout.imu_stale)
        return (
            accelerometer,
            gyro,
            float(readout.measurement_age_s),
            extrapolation_allowed,
        )

    @property
    def last_imu_hardware_sample(self) -> ImuHardwareSample | None:
        """Expose the consumed packet's raw sample only to diagnostic loggers."""

        readout = self.last_imu_readout
        return None if readout is None else readout.packet.sample

    @property
    def last_imu_readout(self) -> ImuReadout | None:
        return None if self._imu_sensor is None else self._imu_sensor.last_readout

    def imu_raw_log_fields(self) -> dict[str, list[float]]:
        """Return ideal/noisy raw fields from the most recent hardware tick."""

        readout = self.last_imu_readout
        if readout is None:
            raise RuntimeError("no virtual IMU packet has been consumed")
        return readout.raw_log_fields()

    def imu_diagnostics(self, include_hidden_truth: bool = False) -> dict:
        """Return sensor diagnostics; hidden biases require explicit post-hoc opt-in."""

        if self._imu_sensor is None:
            raise RuntimeError("virtual IMU sensor is not initialized")
        return self._imu_sensor.diagnostics(include_hidden_truth)

    def longitudinal_state(self, theta_eq_rad: float, position_reference_m: float = 0.0) -> NDArray[np.float64]:
        """Return `[p, p_dot, theta-theta_eq, theta_dot]` in the reduced-model convention."""

        qpos = self.data.qpos
        qvel = self.data.qvel
        w, x, y, z = qpos[self._root_qpos + 3 : self._root_qpos + 7]
        theta = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        theta_error = (theta - theta_eq_rad + math.pi) % (2.0 * math.pi) - math.pi
        return np.array(
            [
                -qpos[self._root_qpos + 1] - position_reference_m,
                -qvel[self._root_dof + 1],
                theta_error,
                qvel[self._root_dof + 3],
            ],
            dtype=float,
        )
