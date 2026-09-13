"""Validate the CAD-derived pitch equilibrium with MuJoCo gravity torque."""

from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = ROOT / "models" / "minisegway"
REDUCED_PATH = MODEL_DIR / "reduced_twip.json"
OUTPUT_PATH = MODEL_DIR / "equilibrium_validation.json"


reduced = json.loads(REDUCED_PATH.read_text(encoding="utf-8"))
plant = json.loads((MODEL_DIR / "plant_report.json").read_text(encoding="utf-8"))
chassis = plant["chassis_aggregate"]
center = chassis["center_m"]
inertia = chassis["inertia_about_com_kg_m2"]
full_inertia = [inertia[0][0], inertia[1][1], inertia[2][2], inertia[0][1], inertia[0][2], inertia[1][2]]

xml = f'''<mujoco model="pitch equilibrium check">
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="chassis">
      <joint name="pitch" type="hinge" axis="1 0 0"/>
      <inertial pos="{' '.join(map(str, center))}" mass="{chassis['mass_kg']}"
                fullinertia="{' '.join(map(str, full_inertia))}"/>
    </body>
  </worldbody>
</mujoco>'''
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)


def gravity_bias(angle: float) -> float:
    data.qpos[0] = angle
    data.qvel[0] = 0
    mujoco.mj_forward(model, data)
    return float(data.qfrc_bias[0])


theta_eq = reduced["parameters"]["theta_eq_rad"]
delta = math.radians(1.0)
report = {
    "theta_eq_rad": theta_eq,
    "theta_eq_deg": math.degrees(theta_eq),
    "gravity_bias_at_theta_eq_nm": gravity_bias(theta_eq),
    "gravity_bias_at_theta_eq_minus_1deg_nm": gravity_bias(theta_eq - delta),
    "gravity_bias_at_theta_eq_plus_1deg_nm": gravity_bias(theta_eq + delta),
    "meaning": "qfrc_bias changes sign at theta_eq; with zero applied torque, positive pitch error accelerates further positive"
}
OUTPUT_PATH.write_text(json.dumps(report, indent=2), encoding="utf-8")
print(json.dumps(report, indent=2))

if abs(report["gravity_bias_at_theta_eq_nm"]) > 1e-10:
    raise SystemExit("FAIL: CAD-derived theta_eq is not the MuJoCo gravity-torque zero")
if not (
    report["gravity_bias_at_theta_eq_minus_1deg_nm"] > 0
    and report["gravity_bias_at_theta_eq_plus_1deg_nm"] < 0
):
    raise SystemExit("FAIL: pitch gravity torque signs do not match the reduced model")
