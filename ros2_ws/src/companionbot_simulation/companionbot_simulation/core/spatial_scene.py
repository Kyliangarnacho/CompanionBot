"""Massless virtual lidar and additive indoor geometry, with no pose output.

Only this sensor simulator may read physical site poses for ray casting. The
estimator receives noisy ranges, not the sensor pose, hit ids or scene geometry.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from companionbot_contracts.spatial_geometry import load_spatial_geometry, load_sim_lidar


def prepare_indoor_model(source, destination, *, geometry=None, lidar=None):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination:
        raise ValueError("derived indoor scene must not overwrite the frozen plant")
    geometry = load_spatial_geometry() if geometry is None else geometry
    lidar = load_sim_lidar() if lidar is None else lidar
    tree = ET.parse(source)
    compiler = tree.find("compiler")
    compiler.set("meshdir", str((source.parent / compiler.get("meshdir")).resolve()))
    world = tree.find("worldbody")
    chassis = world.find("body[@name='chassis']")
    position = geometry.mj_to_ros.T @ np.asarray(lidar["xyz_m"])
    rotation = geometry.mj_to_ros.T @ Rotation.from_euler("xyz", lidar["rpy_rad"]).as_matrix()
    q = Rotation.from_matrix(rotation).as_quat()
    ET.SubElement(chassis, "site", name="spatial_lidar", pos=" ".join(map(str, position)),
                  quat=" ".join(map(str, [q[3], *q[:3]])), size="0.001", group="5")
    # World coordinates below use the ROS convention at upright startup. The
    # robot's original masses, joints, sensors, friction and actuators are kept.
    walls = [
        ("room_front", [6.05, 0., .5], [.05, 4.1, .5]),
        ("room_back", [-2.05, 0., .5], [.05, 4.1, .5]),
        ("room_left", [2., 4.05, .5], [4., .05, .5]),
        ("room_right", [2., -4.05, .5], [4., .05, .5]),
        ("room_box", [2., -2.5, .25], [.3, .4, .25]),
    ]
    for name, center, half_size in walls:
        native = geometry.mj_to_ros.T @ np.asarray(center)
        ET.SubElement(world, "geom", name=name, type="box", pos=" ".join(map(str, native)),
                      size=" ".join(map(str, [half_size[1], half_size[0], half_size[2]])),
                      group="4", rgba="0.6 0.65 0.7 1", contype="1", conaffinity="1")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="unicode")
    metadata = {"source": str(source), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                "derived_scene": str(destination), "geometry_id": geometry.geometry_id,
                "sensor": lidar, "walls_ros_startup": walls,
                "plant_parameter_changes": [], "new_sensor_mass_kg": 0.0}
    destination.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    return destination


@dataclass(frozen=True)
class ScanSample:
    time_s: float
    ranges_m: tuple[float, ...]
    angle_min: float
    angle_increment: float
    scan_time: float
    range_min: float
    range_max: float


class VirtualPlanarLidar:
    def __init__(self, model, config=None):
        self.config = load_sim_lidar() if config is None else config
        self.site = model.site("spatial_lidar").id
        self.angles = -math.pi + np.arange(self.config["rays"]) * (2*math.pi/self.config["rays"])
        self.vectors = np.column_stack((np.cos(self.angles), np.sin(self.angles), np.zeros_like(self.angles)))
        self.groups = np.zeros(6, dtype=np.uint8)
        self.groups[self.config["geometry_groups"]] = 1
        self.rng = np.random.Generator(np.random.PCG64(self.config["seed"]))

    def sample(self, model, data):
        rotation = data.site_xmat[self.site].reshape(3, 3)
        world_vectors = np.ascontiguousarray((self.vectors @ rotation.T).ravel())
        distances = np.empty(len(self.angles), dtype=np.float64)
        hits = np.empty(len(self.angles), dtype=np.int32)
        # Include opaque collision geometry (including self-occlusion and the
        # floor); transparent CAD visual meshes are not range targets.
        mujoco.mj_multiRay(model, data, data.site_xpos[self.site].copy(), world_vectors,
                          self.groups, 1, -1, hits, distances, len(self.angles), self.config["range_max_m"])
        hit = distances >= 0
        distances[hit] += self.rng.normal(0., self.config["range_noise_std_m"], int(np.sum(hit)))
        distances[~hit] = np.inf
        distances[hit & (distances < self.config["range_min_m"])] = -np.inf
        distances[distances > self.config["range_max_m"]] = np.inf
        return ScanSample(float(data.time), tuple(map(float, distances)), -math.pi,
                          2*math.pi/len(self.angles), self.config["period_s"],
                          self.config["range_min_m"], self.config["range_max_m"])
