"""Runtime rigid-payload mass-property updates for the MiniSegway plant."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class PayloadConfig:
    target_body: str
    add_time_s: float
    mass_kg: float
    position_body_m: NDArray[np.float64]
    full_size_m: NDArray[np.float64]


@dataclass(frozen=True)
class RigidMassProperties:
    mass_kg: float
    com_body_m: NDArray[np.float64]
    inertia_com_body_kg_m2: NDArray[np.float64]


def load_payload_config(path: str | Path) -> PayloadConfig:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw["shape"]["type"] != "box":
        raise ValueError("Dynamic payload V1 supports only an axis-aligned box inertia shape")
    config = PayloadConfig(
        target_body=str(raw["target_body"]),
        add_time_s=float(raw["add_time_s"]),
        mass_kg=float(raw["mass_kg"]),
        position_body_m=np.asarray(raw["position_body_m"], dtype=float),
        full_size_m=np.asarray(raw["shape"]["full_size_m"], dtype=float),
    )
    if config.add_time_s < 0.0 or config.mass_kg <= 0.0:
        raise ValueError("payload add time must be nonnegative and mass must be positive")
    if config.position_body_m.shape != (3,) or config.full_size_m.shape != (3,):
        raise ValueError("payload position and box size must each have three components")
    if not np.isfinite(np.r_[config.position_body_m, config.full_size_m]).all():
        raise ValueError("payload geometry must be finite")
    if np.any(config.full_size_m <= 0.0):
        raise ValueError("payload box dimensions must be positive")
    return config


def box_inertia_com(mass_kg: float, full_size_m: NDArray[np.float64]) -> NDArray[np.float64]:
    x, y, z = np.asarray(full_size_m, dtype=float)
    return mass_kg / 12.0 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])


def _parallel_axis(mass_kg: float, offset_m: NDArray[np.float64]) -> NDArray[np.float64]:
    return mass_kg * (float(offset_m @ offset_m) * np.eye(3) - np.outer(offset_m, offset_m))


def combine_rigid_mass_properties(
    base: RigidMassProperties,
    added: RigidMassProperties,
) -> RigidMassProperties:
    total_mass = base.mass_kg + added.mass_kg
    combined_com = (base.mass_kg * base.com_body_m + added.mass_kg * added.com_body_m) / total_mass
    base_offset = base.com_body_m - combined_com
    added_offset = added.com_body_m - combined_com
    combined_inertia = (
        base.inertia_com_body_kg_m2
        + _parallel_axis(base.mass_kg, base_offset)
        + added.inertia_com_body_kg_m2
        + _parallel_axis(added.mass_kg, added_offset)
    )
    return RigidMassProperties(total_mass, combined_com, combined_inertia)


def read_body_mass_properties(model: mujoco.MjModel, body_id: int) -> RigidMassProperties:
    rotation_flat = np.zeros(9, dtype=float)
    mujoco.mju_quat2Mat(rotation_flat, model.body_iquat[body_id])
    rotation = rotation_flat.reshape(3, 3)
    inertia_body = rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T
    return RigidMassProperties(
        float(model.body_mass[body_id]),
        model.body_ipos[body_id].copy(),
        inertia_body,
    )


def write_body_mass_properties(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    properties: RigidMassProperties,
) -> None:
    inertia = 0.5 * (properties.inertia_com_body_kg_m2 + properties.inertia_com_body_kg_m2.T)
    principal, axes = np.linalg.eigh(inertia)
    if not np.isfinite(principal).all() or np.any(principal <= 0.0):
        raise ValueError("combined body inertia must be finite and positive definite")
    if np.linalg.det(axes) < 0.0:
        axes[:, -1] *= -1.0
    inertia_quaternion = np.zeros(4, dtype=float)
    mujoco.mju_mat2Quat(inertia_quaternion, axes.reshape(-1))

    # mj_setConst evaluates qpos0 and resets parts of mjData as a side effect.
    # Preserve the live simulation state so a payload event changes inertia,
    # not pose, velocity, time, or the held actuator command.
    time_before = float(data.time)
    qpos_before = data.qpos.copy()
    qvel_before = data.qvel.copy()
    act_before = data.act.copy()
    ctrl_before = data.ctrl.copy()
    model.body_mass[body_id] = properties.mass_kg
    model.body_ipos[body_id] = properties.com_body_m
    model.body_inertia[body_id] = principal
    model.body_iquat[body_id] = inertia_quaternion
    mujoco.mj_setConst(model, data)
    data.time = time_before
    data.qpos[:] = qpos_before
    data.qvel[:] = qvel_before
    data.act[:] = act_before
    data.ctrl[:] = ctrl_before
    mujoco.mj_forward(model, data)


class DynamicPayload:
    """One-shot, reversible runtime payload attached rigidly to one body."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: PayloadConfig):
        self.model = model
        self.data = data
        self.config = config
        self.body_id = model.body(config.target_body).id
        self.nominal = read_body_mass_properties(model, self.body_id)
        payload = RigidMassProperties(
            config.mass_kg,
            config.position_body_m.copy(),
            box_inertia_com(config.mass_kg, config.full_size_m),
        )
        self.loaded = combine_rigid_mass_properties(self.nominal, payload)
        self.is_loaded = False

    def apply(self) -> None:
        if self.is_loaded:
            return
        write_body_mass_properties(self.model, self.data, self.body_id, self.loaded)
        self.is_loaded = True

    def remove(self) -> None:
        if not self.is_loaded:
            return
        write_body_mass_properties(self.model, self.data, self.body_id, self.nominal)
        self.is_loaded = False

    def update_for_time(self, time_s: float) -> bool:
        """Apply the configured payload once time reaches the event; return True on transition."""

        if not self.is_loaded and time_s + 1e-12 >= self.config.add_time_s:
            self.apply()
            return True
        return False
