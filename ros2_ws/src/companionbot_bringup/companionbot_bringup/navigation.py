"""Navigation launch checks; no motion, localization or planning implementation."""
import hashlib
import json
from pathlib import Path

import yaml

from companionbot_contracts.paths import repository_root


def navigation_radius(config):
    record = json.loads(Path(config).read_text())
    source = repository_root()/record["source"]
    if (record["schema_version"] != 1 or
            hashlib.sha256(source.read_bytes()).hexdigest() != record["source_sha256"] or
            record["robot_radius_m"] != record["physical_radius_bound_m"] + record["geometric_margin_m"] or
            not 0 < record["robot_radius_m"] < 1):
        raise ValueError("navigation geometry requires explicit regeneration from canonical plant")
    return record["robot_radius_m"]


def validate_map(filename):
    path = Path(filename)
    if not path.is_absolute() or not path.is_file():
        raise ValueError("localization requires an existing absolute map_yaml")
    mapping = yaml.safe_load(path.read_text())
    image = path.parent/mapping["image"]
    if not image.is_file() or not image.stat().st_size or float(mapping["resolution"]) <= 0:
        raise ValueError("map_yaml requires a nonempty map image and positive resolution")
    return path
