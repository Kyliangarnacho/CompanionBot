"""Build the local MiniSegway visual assets and an inertially explicit MuJoCo plant."""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path

import cadquery as cq
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = PROJECT_ROOT / "models" / "minisegway"
DEFAULT_CAD_DIR = Path(r"D:\project\CompanionBot_upstreams\Mini_Segway\docs\cad")


@dataclass
class MassElement:
    name: str
    mass: float
    center: np.ndarray
    inertia_com: np.ndarray
    provenance: str


def load_step(cad_dir: Path, filename: str) -> cq.Shape:
    return cq.Compound.makeCompound(cq.importers.importStep(str(cad_dir / filename)).vals())


def cad_element(name: str, shape: cq.Shape, mass: float, provenance: str) -> MassElement:
    volume_mm3 = shape.Volume()
    center_mm = shape.Center()
    inertia_per_mass_mm2 = np.asarray(shape.matrixOfInertia(shape), dtype=float) / volume_mm3
    return MassElement(
        name=name,
        mass=mass,
        center=np.array([center_mm.x, center_mm.y, center_mm.z]) * 1e-3,
        inertia_com=inertia_per_mass_mm2 * mass * 1e-6,
        provenance=provenance,
    )


def box_element(name: str, mass: float, center, full_size, provenance: str) -> MassElement:
    x, y, z = np.asarray(full_size, dtype=float)
    inertia = mass / 12.0 * np.diag([y * y + z * z, x * x + z * z, x * x + y * y])
    return MassElement(name, mass, np.asarray(center, dtype=float), inertia, provenance)


def aggregate(elements: list[MassElement], name: str, provenance: str) -> MassElement:
    mass = sum(element.mass for element in elements)
    center = sum(element.mass * element.center for element in elements) / mass
    inertia = np.zeros((3, 3))
    for element in elements:
        offset = element.center - center
        inertia += element.inertia_com + element.mass * (
            np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
        )
    return MassElement(name, mass, center, inertia, provenance)


def fmt(values, precision=10) -> str:
    return " ".join(f"{float(value):.{precision}g}" for value in values)


def inertial_xml(element: MassElement, indent: str = "      ") -> str:
    matrix = element.inertia_com
    full = [matrix[0, 0], matrix[1, 1], matrix[2, 2], matrix[0, 1], matrix[0, 2], matrix[1, 2]]
    return (
        f'{indent}<inertial pos="{fmt(element.center)}" mass="{element.mass:.10g}" '
        f'fullinertia="{fmt(full)}"/>'
    )


def serialise(element: MassElement) -> dict:
    return {
        "mass_kg": element.mass,
        "center_m": element.center.tolist(),
        "inertia_about_com_kg_m2": element.inertia_com.tolist(),
        "provenance": element.provenance,
    }


def export_stl(shape: cq.Shape, path: Path) -> None:
    cq.exporters.export(shape, str(path), tolerance=0.15, angularTolerance=0.12)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cad-dir", type=Path, default=DEFAULT_CAD_DIR)
    args = parser.parse_args()

    params = json.loads((MODEL_DIR / "plant_parameters.json").read_text(encoding="utf-8"))
    known = params["known"]
    provisional = params["provisional"]
    cad_dir = args.cad_dir.resolve()
    asset_dir = MODEL_DIR / "assets" / "upstream_local"
    asset_dir.mkdir(parents=True, exist_ok=True)

    main_frame = load_step(cad_dir, "NDT_FRAME_MAIN_V1.stp")
    wheel_base = load_step(cad_dir, "NDT_FRAME_WHEEL_BASE.stp")
    cover = load_step(cad_dir, "NDT_ROBOT_COVER.stp")

    axle_mm = known["motor_output_face_x_m"] * 1000.0
    motor_right = load_step(cad_dir, "25d-metal-gearmotor-34-47-encoder.step")
    motor_right = motor_right.rotate((0, 0, 0), (0, 1, 0), 90).translate((axle_mm, 0, 0))
    motor_left = load_step(cad_dir, "25d-metal-gearmotor-34-47-encoder.step")
    motor_left = motor_left.rotate((0, 0, 0), (0, 1, 0), -90).translate((-axle_mm, 0, 0))

    shield = load_step(cad_dir, "pololu-dual-mc33926-motor-driver-shield-for-arduino.step")
    shield_center = shield.Center()
    shield = shield.translate((-shield_center.x, -shield_center.y, -shield_center.z))
    shield = shield.rotate((0, 0, 0), (1, 0, 0), 90)
    shield = shield.translate(tuple(np.asarray(provisional["mc33926_center_m"]) * 1000.0))

    wheel = load_step(cad_dir, "Skate Wheel 84×24mm - Black.step")
    adapter_right = load_step(cad_dir, "pololu-aluminum-scooter-wheel-adapter-4mm.step")
    adapter_right = adapter_right.rotate((0, 0, 0), (0, 1, 0), -90).translate((-8.649, 0, 0))
    adapter_left = load_step(cad_dir, "pololu-aluminum-scooter-wheel-adapter-4mm.step")
    adapter_left = adapter_left.rotate((0, 0, 0), (0, 1, 0), 90).translate((8.649, 0, 0))

    visual_shapes = {
        "main_frame.stl": main_frame,
        "wheel_base.stl": wheel_base,
        "cover.stl": cover,
        "motor_left.stl": motor_left,
        "motor_right.stl": motor_right,
        "mc33926.stl": shield,
        "wheel.stl": wheel,
        "adapter_left.stl": adapter_left,
        "adapter_right.stl": adapter_right,
    }
    for filename, shape in visual_shapes.items():
        export_stl(shape, asset_dir / filename)

    density = provisional["printed_effective_density_kg_m3"]
    printed = []
    cad_audit = {}
    for name, shape in (("main_frame", main_frame), ("wheel_base", wheel_base), ("cover", cover)):
        volume_m3 = shape.Volume() * 1e-9
        element = cad_element(
            name,
            shape,
            volume_m3 * density,
            f"CAD volume/inertia shape; provisional effective density {density:g} kg/m^3",
        )
        printed.append(element)
        cad_audit[name] = {
            **serialise(element),
            "cad_volume_cm3": shape.Volume() / 1000.0,
            "effective_density_kg_m3": density,
        }

    chassis_elements = printed + [
        cad_element("motor_left", motor_left, known["motor_mass_kg"], "manufacturer mass; CAD centroid/inertia shape"),
        cad_element("motor_right", motor_right, known["motor_mass_kg"], "manufacturer mass; CAD centroid/inertia shape"),
    ]
    for index, center in enumerate(provisional["battery_centers_m"], start=1):
        chassis_elements.append(
            box_element(
                f"battery_{index}",
                known["battery_mass_kg"],
                center,
                known["battery_size_m"],
                "manufacturer mass/size; placement provisional from front/back photos",
            )
        )
    chassis_elements.append(
        cad_element("mc33926", shield, known["mc33926_mass_kg"], "given mass; CAD inertia shape; photo-derived provisional placement")
    )
    chassis_elements.append(
        box_element(
            "electronics_lump",
            provisional["electronics_lump_mass_kg"],
            provisional["electronics_lump_center_m"],
            provisional["electronics_lump_size_m"],
            "provisional mass, envelope and photo-derived placement",
        )
    )
    chassis = aggregate(chassis_elements, "chassis", "rigid aggregate excluding wheels/adapters")

    wheel_base_element = cad_element("wheel", wheel, known["wheel_mass_kg"], "manufacturer mass; CAD centroid/inertia shape")
    right_wheel = aggregate(
        [
            wheel_base_element,
            cad_element("adapter_right", adapter_right, known["adapter_mass_kg"], "manufacturer mass; CAD centroid/inertia shape"),
        ],
        "right_wheel_assembly",
        "rotating wheel plus adapter",
    )
    left_wheel = aggregate(
        [
            wheel_base_element,
            cad_element("adapter_left", adapter_left, known["adapter_mass_kg"], "manufacturer mass; CAD centroid/inertia shape"),
        ],
        "left_wheel_assembly",
        "rotating wheel plus adapter",
    )

    wheel_x = provisional["wheel_center_x_m"]
    whole = aggregate(
        [
            chassis,
            MassElement("left_wheel_world", left_wheel.mass, left_wheel.center + [-wheel_x, 0, 0], left_wheel.inertia_com, left_wheel.provenance),
            MassElement("right_wheel_world", right_wheel.mass, right_wheel.center + [wheel_x, 0, 0], right_wheel.inertia_com, right_wheel.provenance),
        ],
        "whole_robot",
        "chassis plus both rotating wheel assemblies",
    )

    friction = provisional["wheel_friction"]
    battery_visuals = []
    half_battery = np.asarray(known["battery_size_m"]) / 2.0
    for index, center in enumerate(provisional["battery_centers_m"], start=1):
        battery_visuals.append(
            f'      <geom name="battery_{index}_visual" class="visual" type="box" pos="{fmt(center)}" '
            f'size="{fmt(half_battery)}" rgba="0.15 0.25 0.75 1"/>'
        )

    model_xml = f'''<mujoco model="MiniSegway physics plant">
  <compiler angle="radian" meshdir="assets/upstream_local"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>
  <statistic center="0 0 0.05" extent="0.28"/>

  <default>
    <default class="visual">
      <geom contype="0" conaffinity="0" group="2"/>
    </default>
    <default class="collision">
      <geom group="3" rgba="0.85 0.35 0.12 0.25" solref="0.005 1" solimp="0.9 0.95 0.001"/>
    </default>
  </default>

  <asset>
    <texture name="ground_tex" type="2d" builtin="checker" rgb1="0.22 0.25 0.28" rgb2="0.12 0.14 0.16" width="256" height="256"/>
    <material name="ground_mat" texture="ground_tex" texrepeat="4 4" reflectance="0.15"/>
    <mesh name="main_frame_mesh" file="main_frame.stl" scale="0.001 0.001 0.001"/>
    <mesh name="wheel_base_mesh" file="wheel_base.stl" scale="0.001 0.001 0.001"/>
    <mesh name="cover_mesh" file="cover.stl" scale="0.001 0.001 0.001"/>
    <mesh name="motor_left_mesh" file="motor_left.stl" scale="0.001 0.001 0.001"/>
    <mesh name="motor_right_mesh" file="motor_right.stl" scale="0.001 0.001 0.001"/>
    <mesh name="mc33926_mesh" file="mc33926.stl" scale="0.001 0.001 0.001"/>
    <mesh name="wheel_mesh" file="wheel.stl" scale="0.001 0.001 0.001"/>
    <mesh name="adapter_left_mesh" file="adapter_left.stl" scale="0.001 0.001 0.001"/>
    <mesh name="adapter_right_mesh" file="adapter_right.stl" scale="0.001 0.001 0.001"/>
  </asset>

  <worldbody>
    <light pos="0 -0.5 0.8" dir="0 0.6 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="2 2 0.05" material="ground_mat" condim="3"/>
    <body name="chassis" pos="0 0 0.043">
      <freejoint name="root"/>
{inertial_xml(chassis)}

      <!-- Original CAD-derived visual meshes: no contact and no inferred mass. -->
      <geom name="main_frame_visual" class="visual" type="mesh" mesh="main_frame_mesh" rgba="0.12 0.42 0.72 1"/>
      <geom name="wheel_base_visual" class="visual" type="mesh" mesh="wheel_base_mesh" rgba="0.95 0.50 0.10 1"/>
      <geom name="cover_visual" class="visual" type="mesh" mesh="cover_mesh" rgba="0.18 0.72 0.32 0.72"/>
      <geom name="motor_left_visual" class="visual" type="mesh" mesh="motor_left_mesh" rgba="0.45 0.47 0.50 1"/>
      <geom name="motor_right_visual" class="visual" type="mesh" mesh="motor_right_mesh" rgba="0.45 0.47 0.50 1"/>
      <geom name="mc33926_visual" class="visual" type="mesh" mesh="mc33926_mesh" rgba="0.08 0.36 0.18 1"/>
{chr(10).join(battery_visuals)}
      <geom name="electronics_lump_visual" class="visual" type="box" pos="{fmt(provisional['electronics_lump_center_m'])}" size="{fmt(np.asarray(provisional['electronics_lump_size_m']) / 2.0)}" rgba="0.10 0.42 0.20 0.65"/>

      <!-- Deliberately simple chassis collision proxies. -->
      <geom name="wheel_base_collision" class="collision" type="box" pos="0 0.0005 0.0042" size="0.074 0.0201 0.0198"/>
      <geom name="lower_chassis_collision" class="collision" type="box" pos="0.001 -0.0015 0.038" size="0.0595 0.0325 0.038"/>
      <geom name="upper_chassis_collision" class="collision" type="capsule" fromto="-0.024 -0.0015 0.096 0.026 -0.0015 0.096" size="0.033"/>

      <body name="left_wheel" pos="-{wheel_x:.10g} 0 0">
        <joint name="left_wheel_hinge" type="hinge" axis="1 0 0" damping="0.002" frictionloss="0.002"/>
{inertial_xml(left_wheel, '        ')}
        <geom name="left_wheel_visual" class="visual" type="mesh" mesh="wheel_mesh" rgba="0.04 0.05 0.06 1"/>
        <geom name="left_adapter_visual" class="visual" type="mesh" mesh="adapter_left_mesh" rgba="0.12 0.48 0.82 1"/>
        <geom name="left_wheel_collision" class="collision" type="cylinder" size="{known['wheel_radius_m']:.10g} {known['wheel_width_m']/2:.10g}" quat="0.7071067812 0 0.7071067812 0" friction="{fmt(friction)}"/>
      </body>

      <body name="right_wheel" pos="{wheel_x:.10g} 0 0">
        <joint name="right_wheel_hinge" type="hinge" axis="1 0 0" damping="0.002" frictionloss="0.002"/>
{inertial_xml(right_wheel, '        ')}
        <geom name="right_wheel_visual" class="visual" type="mesh" mesh="wheel_mesh" rgba="0.04 0.05 0.06 1"/>
        <geom name="right_adapter_visual" class="visual" type="mesh" mesh="adapter_right_mesh" rgba="0.12 0.48 0.82 1"/>
        <geom name="right_wheel_collision" class="collision" type="cylinder" size="{known['wheel_radius_m']:.10g} {known['wheel_width_m']/2:.10g}" quat="0.7071067812 0 0.7071067812 0" friction="{fmt(friction)}"/>
      </body>
    </body>
  </worldbody>

  <!-- gear=1 maps ctrl directly to scalar hinge torque in N*m.  These are
       stall/peak-level hard limits, not continuous motor ratings. -->
  <actuator>
    <motor name="left_wheel_torque" joint="left_wheel_hinge" gear="1"
           ctrllimited="true" ctrlrange="-{known['wheel_torque_hard_peak_nm']:.10g} {known['wheel_torque_hard_peak_nm']:.10g}"
           forcelimited="true" forcerange="-{known['wheel_torque_hard_peak_nm']:.10g} {known['wheel_torque_hard_peak_nm']:.10g}"/>
    <motor name="right_wheel_torque" joint="right_wheel_hinge" gear="1"
           ctrllimited="true" ctrlrange="-{known['wheel_torque_hard_peak_nm']:.10g} {known['wheel_torque_hard_peak_nm']:.10g}"
           forcelimited="true" forcerange="-{known['wheel_torque_hard_peak_nm']:.10g} {known['wheel_torque_hard_peak_nm']:.10g}"/>
  </actuator>

  <keyframe>
    <key name="upright" qpos="0 0 0.043 1 0 0 0 0 0"/>
  </keyframe>
</mujoco>
'''
    (MODEL_DIR / "mini_segway.xml").write_text(model_xml, encoding="utf-8")

    report = {
        "parameters": params,
        "cad_parts": cad_audit,
        "chassis_components": [serialise(element) for element in chassis_elements],
        "chassis_aggregate": serialise(chassis),
        "left_wheel_aggregate": serialise(left_wheel),
        "right_wheel_aggregate": serialise(right_wheel),
        "whole_robot": serialise(whole),
        "whole_robot_pitch_inertia_about_com_kg_m2": float(whole.inertia_com[0, 0]),
        "single_wheel_rotating_mass_kg": right_wheel.mass,
        "single_wheel_axial_inertia_about_hinge_kg_m2": float(
            right_wheel.inertia_com[0, 0]
            + right_wheel.mass * (right_wheel.center[1] ** 2 + right_wheel.center[2] ** 2)
        ),
    }
    (MODEL_DIR / "plant_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")

    print(f"model={MODEL_DIR / 'mini_segway.xml'}")
    print(f"total_mass_kg={whole.mass:.9f}")
    print(f"whole_com_from_axle_m={fmt(whole.center)}")
    print(f"pitch_inertia_about_whole_com_kg_m2={whole.inertia_com[0, 0]:.10g}")
    print(f"single_wheel_mass_kg={right_wheel.mass:.9f}")
    print(f"single_wheel_axial_inertia_kg_m2={report['single_wheel_axial_inertia_about_hinge_kg_m2']:.10g}")


if __name__ == "__main__":
    main()
    # cadquery-ocp 7.8.1.1.post1 completes all writes but crashes with Windows
    # status 0xC0000005 during interpreter teardown on this host (even for a
    # bare `import cadquery`).  Exit only after main() returns successfully;
    # build exceptions still propagate normally.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)
