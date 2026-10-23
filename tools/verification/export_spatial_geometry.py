"""Export/check the ROS description's derived dimensions from the frozen MJCF.

No mesh is copied and no plant/control parameter is written. New virtual sensor
mounts have their own explicit simulation configuration, not a fake calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "models/minisegway/mini_segway.xml"
OUTPUT = ROOT / "ros2_ws/src/companionbot_description/config/minisegway.yaml"


def exported_geometry():
    tree = ET.parse(SOURCE)
    chassis = tree.find("worldbody/body[@name='chassis']")
    wheels = [chassis.find(f"body[@name='{side}_wheel']") for side in ("left", "right")]
    numbers = lambda text: [float(x) for x in text.split()]
    radius = numbers(wheels[0].find("geom[@class='collision']").get("size"))[0]
    assert radius == numbers(wheels[1].find("geom[@class='collision']").get("size"))[0]
    positions = [numbers(w.get("pos")) for w in wheels]
    assert positions[0][0] < 0 < positions[1][0]
    assert all(numbers(w.find("joint").get("axis")) == [1, 0, 0] for w in wheels)
    inertia = chassis.find("inertial")
    imu = chassis.find("site[@name='imu_site']")
    assert numbers(imu.get("quat")) == [1, 0, 0, 0]
    raw_com = numbers(inertia.get("pos"))
    raw_i = numbers(inertia.get("fullinertia"))
    # C = Rz(+pi/2): (x,y,z)_ROS=(-y,x,z)_MJ. I_ROS=C I_MJ C^T.
    geometry = {
        "schema_version": 1,
        "source": str(SOURCE.relative_to(ROOT)),
        "source_sha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        "mj_to_ros": [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
        "wheel_radius_m": radius,
        "wheel_width_m": 2 * numbers(wheels[0].find("geom[@class='collision']").get("size"))[1],
        "wheel_separation_m": positions[1][0] - positions[0][0],
        "ros_left_encoder_index": 1,
        "ros_right_encoder_index": 0,
        "imu_xyz_m": [0.0, 0.0, numbers(imu.get("pos"))[2]],
        "chassis_mass_kg": float(inertia.get("mass")),
        "chassis_com_m": [-raw_com[1], raw_com[0], raw_com[2]],
        "chassis_inertia": [raw_i[1], raw_i[0], raw_i[2], -raw_i[3], -raw_i[5], raw_i[4]],
        "wheel_mass_kg": float(wheels[0].find("inertial").get("mass")),
    }
    geometry["geometry_id"] = hashlib.sha256(json.dumps(geometry, sort_keys=True).encode()).hexdigest()
    return geometry


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--check", action="store_true")
    args = cli.parse_args()
    content = json.dumps(exported_geometry(), indent=2) + "\n"
    if args.check:
        if not OUTPUT.is_file() or OUTPUT.read_text() != content:
            raise RuntimeError("derived spatial dimensions differ; inspect the frozen source before exporting")
    else:
        OUTPUT.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT.write_text(content)
    print(str(OUTPUT.relative_to(ROOT)) + (" verified" if args.check else " exported"))


if __name__ == "__main__":
    main()
