"""Minimal fixed-step interface for the passive MiniSegway MuJoCo plant."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray


DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "minisegway" / "mini_segway.xml"


@dataclass(frozen=True)
class SimSnapshot:
    step_index: int
    time_s: float
    qpos: NDArray[np.float64]
    qvel: NDArray[np.float64]
    applied_ctrl_nm: NDArray[np.float64]


class MiniSegwaySim:
    """One call to :meth:`step` advances exactly one MuJoCo physics timestep."""

    def __init__(self, model_path: str | Path = DEFAULT_MODEL_PATH):
        self.model_path = Path(model_path).resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        self._left_actuator = self.model.actuator("left_wheel_torque").id
        self._right_actuator = self.model.actuator("right_wheel_torque").id
        root_joint = self.model.joint("root").id
        self._root_qpos = int(self.model.jnt_qposadr[root_joint])
        self._root_dof = int(self.model.jnt_dofadr[root_joint])
        self.step_index = 0
        self.reset()

    @property
    def physics_dt(self) -> float:
        return float(self.model.opt.timestep)

    def reset(self, pitch_rad: float | None = None) -> SimSnapshot:
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
        )

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
