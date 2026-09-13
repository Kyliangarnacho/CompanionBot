"""Compile and exercise the passive MiniSegway plant without a controller."""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "models" / "minisegway" / "mini_segway.xml"
REPORT_PATH = ROOT / "models" / "minisegway" / "smoke_report.json"


model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
left_hinge_id = model.joint("left_wheel_hinge").id
right_hinge_id = model.joint("right_wheel_hinge").id
wheel_dof_ids = [model.jnt_dofadr[left_hinge_id], model.jnt_dofadr[right_hinge_id]]

# Verify direct torque semantics before the dynamic contact test.
data.ctrl[:] = [0.12, -0.23]
mujoco.mj_forward(model, data)
direct_torque_force = data.actuator_force.copy()
direct_torque_qfrc = data.qfrc_actuator[wheel_dof_ids].copy()
data.ctrl[:] = [1.0, -1.0]
mujoco.mj_forward(model, data)
limited_torque_force = data.actuator_force.copy()
mujoco.mj_resetDataKeyframe(model, data, 0)

# A small open-loop pitch perturbation is intentional: this is a plant/contact
# smoke test, not a balance benchmark.  With no controller the robot should fall.
root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
root_qpos = model.jnt_qposadr[root_id]
pitch = np.deg2rad(2.0)
data.qpos[root_qpos + 3 : root_qpos + 7] = [np.cos(pitch / 2), np.sin(pitch / 2), 0, 0]

contact_pairs: set[tuple[str, str]] = set()
max_contacts = 0
min_chassis_z = float("inf")
for _ in range(3000):
    mujoco.mj_step(model, data)
    max_contacts = max(max_contacts, data.ncon)
    min_chassis_z = min(min_chassis_z, float(data.xpos[model.body("chassis").id, 2]))
    for index in range(data.ncon):
        contact = data.contact[index]
        name_a = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
        name_b = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
        contact_pairs.add(tuple(sorted((name_a, name_b))))

all_finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
visual_contact = any("_visual" in name for pair in contact_pairs for name in pair)
left_wheel_contact = tuple(sorted(("floor", "left_wheel_collision"))) in contact_pairs
right_wheel_contact = tuple(sorted(("floor", "right_wheel_collision"))) in contact_pairs
left_geom_id = model.geom("left_wheel_collision").id
right_geom_id = model.geom("right_wheel_collision").id
wheel_sizes = [model.geom_size[left_geom_id, :2].tolist(), model.geom_size[right_geom_id, :2].tolist()]
hinge_axes = [model.jnt_axis[left_hinge_id].tolist(), model.jnt_axis[right_hinge_id].tolist()]

report = {
    "model": str(MODEL_PATH),
    "mujoco_version": mujoco.__version__,
    "duration_s": float(data.time),
    "nq": model.nq,
    "nv": model.nv,
    "njnt": model.njnt,
    "nbody": model.nbody,
    "ngeom": model.ngeom,
    "nu": model.nu,
    "total_mass_kg": float(model.body_subtreemass[model.body("chassis").id]),
    "direct_torque_test_nm": direct_torque_force.tolist(),
    "direct_joint_qfrc_test_nm": direct_torque_qfrc.tolist(),
    "hard_limit_test_nm": limited_torque_force.tolist(),
    "all_state_finite": all_finite,
    "max_contacts": max_contacts,
    "min_chassis_world_z_m": min_chassis_z,
    "final_chassis_world_z_m": float(data.xpos[model.body("chassis").id, 2]),
    "left_wheel_floor_contact": left_wheel_contact,
    "right_wheel_floor_contact": right_wheel_contact,
    "wheel_collision_radius_halfwidth_m": wheel_sizes,
    "wheel_hinge_axes": hinge_axes,
    "visual_geom_contact_detected": visual_contact,
    "contact_pairs": [list(pair) for pair in sorted(contact_pairs)],
}
REPORT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))

if not all_finite:
    raise SystemExit("FAIL: non-finite MuJoCo state")
if not (left_wheel_contact and right_wheel_contact):
    raise SystemExit("FAIL: both analytic wheel cylinders must contact the floor")
if visual_contact:
    raise SystemExit("FAIL: a visual-only mesh generated contact")
if not all(np.allclose(size, [0.042, 0.012]) for size in wheel_sizes):
    raise SystemExit("FAIL: wheel collision is not the required 84 x 24 mm cylinder")
if not all(np.allclose(axis, [1, 0, 0]) for axis in hinge_axes):
    raise SystemExit("FAIL: wheel hinge axis is not chassis X")
if not np.allclose(direct_torque_force, [0.12, -0.23]):
    raise SystemExit("FAIL: gear=1 control did not map directly to hinge torque")
if not np.allclose(direct_torque_qfrc, [0.12, -0.23]):
    raise SystemExit("FAIL: actuator force did not reach the hinge generalized force")
if not np.allclose(limited_torque_force, [0.63, -0.63]):
    raise SystemExit("FAIL: peak torque hard limit was not enforced")
