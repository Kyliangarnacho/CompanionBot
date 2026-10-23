"""Add the audited, cached AWS Bookstore to the frozen robot plant.

Asset acquisition/conversion is explicit tools/assets/prepare_bookstore.py work.
Runtime never downloads, guesses a fallback world or changes robot parameters.
"""
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import load_spatial_geometry, load_sim_lidar


def prepare_retail_model(source, destination, *, asset_dir=None, geometry=None, lidar=None):
    source, destination = Path(source).resolve(), Path(destination).resolve()
    if source == destination:
        raise ValueError("retail_scene_must_not_overwrite_frozen_plant")
    cache = (Path(asset_dir) if asset_dir else repository_root()/".venv/assets/bookstore/derived").resolve()
    manifest_path = cache/"manifest.json"
    if not manifest_path.is_file():
        raise FileNotFoundError("Bookstore assets unavailable; run tools/assets/prepare_bookstore.py explicitly")
    manifest = json.loads(manifest_path.read_text())
    lock = repository_root()/"models/minisegway/retail_navigation/bookstore_world.lock.json"
    if manifest["lock_sha256"] != hashlib.sha256(lock.read_bytes()).hexdigest():
        raise ValueError("bookstore_asset_lock_mismatch")
    for name, expected in manifest["generated_sha256"].items():
        path = cache/name
        if path.parent != cache or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError("bookstore_derived_asset_hash_mismatch:"+name)
    geometry = load_spatial_geometry() if geometry is None else geometry
    lidar = load_sim_lidar() if lidar is None else lidar
    tree = ET.parse(source)
    compiler = tree.find("compiler")
    compiler.set("meshdir", str((source.parent/compiler.get("meshdir")).resolve()))
    world = tree.find("worldbody")
    chassis = world.find("body[@name='chassis']")
    position = geometry.mj_to_ros.T @ np.asarray(lidar["xyz_m"])
    rotation = geometry.mj_to_ros.T @ Rotation.from_euler("xyz", lidar["rpy_rad"]).as_matrix()
    q = Rotation.from_matrix(rotation).as_quat()
    ET.SubElement(chassis, "site", name="spatial_lidar", pos=" ".join(map(str, position)),
                  quat=" ".join(map(str, [q[3], *q[:3]])), size="0.001", group="5")
    assets = ET.parse(cache/"bookstore.xml").getroot()
    for asset in assets.find("asset"):
        tree.find("asset").append(copy.deepcopy(asset))
    q = Rotation.from_matrix(geometry.mj_to_ros.T.copy()).as_quat()
    retail = ET.SubElement(world, "body", name="retail_world", quat=" ".join(map(str, [q[3], *q[:3]])))
    for body in assets.find("worldbody"):
        retail.append(copy.deepcopy(body))
    # Rendering configuration only: collision groups remain separate from the
    # textured source geometry. Sensor ray masks select their own groups.
    statistic = tree.find("statistic")
    statistic.set("extent", "18")
    statistic.set("center", "0 0 0.8")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="unicode")
    destination.with_suffix(".json").write_text(json.dumps({
        "source": str(source), "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "geometry_id": geometry.geometry_id, "sensor": lidar, "world": "AWS Bookstore",
        "asset_manifest": str(manifest_path), "asset_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "upstream_commit": manifest["upstream_commit"], "plant_parameter_changes": [],
        "world_translation_m": manifest["world_translation_m"], "new_sensor_mass_kg": 0.,
        "spawn_ros_xy_yaw": [0., 0., 0.], "spawn_is_experiment_initial_condition": True,
        "runtime_gt_localization": False}, indent=2)+"\n")
    return destination
