#!/usr/bin/env python3
"""Derive a conservative Nav2 disc from existing MuJoCo collision primitives.

No new robot geometry: MuJoCo supplies each primitive's bounding sphere. The
triangle inequality bounds every point about the axle for every body attitude,
so this also covers the existing spatial pitch/roll envelope without flattening
the physical body TF. Run with the project .venv interpreter.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]


def derive(source, margin_m=0.02):
    model = mujoco.MjModel.from_xml_path(str(source))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    axle = data.body("chassis").xpos
    bounds = []
    for index in range(model.ngeom):
        if model.geom_bodyid[index] == 0 or not (
                model.geom_contype[index] or model.geom_conaffinity[index]):
            continue
        center_distance = float(np.linalg.norm(data.geom_xpos[index] - axle))
        sphere_radius = float(model.geom_rbound[index])
        bounds.append({"geom": model.geom(index).name,
                       "center_distance_from_axle_m": center_distance,
                       "primitive_bounding_sphere_radius_m": sphere_radius,
                       "attitude_invariant_radius_bound_m": center_distance + sphere_radius})
    if not bounds:
        raise ValueError("no physical robot collision geometry")
    radius = max(item["attitude_invariant_radius_bound_m"] for item in bounds)
    return {"schema_version": 1, "source": str(source.relative_to(ROOT)),
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "method": "MuJoCo geom_rbound + distance from axle; attitude-invariant triangle inequality",
            "mujoco_version": mujoco.__version__, "physical_radius_bound_m": radius,
            "geometric_margin_m": margin_m, "robot_radius_m": radius + margin_m,
            "collision_geometry": bounds}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--output", type=Path,
                        default=ROOT/"ros2_ws/src/companionbot_bringup/config/nav2_geometry.json")
    args = parser.parse_args()
    result = derive(ROOT/"models/minisegway/mini_segway.xml")
    if args.check:
        actual = json.loads(args.output.read_text())
        if actual != result:
            raise SystemExit("derived navigation geometry differs; explicitly regenerate after review")
    else:
        args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"checked": args.check, "robot_radius_m": result["robot_radius_m"],
                      "source_sha256": result["source_sha256"]}))


if __name__ == "__main__":
    main()
