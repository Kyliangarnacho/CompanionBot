"""Canonical map/destination binding validation for launch and Agent consumers."""
import hashlib
import json
import math
from pathlib import Path

import yaml


def load_map_binding(filename, *, require_reachability=True):
    path = Path(filename).resolve()
    binding = json.loads(path.read_text())
    map_yaml = (path.parent/binding["map_yaml"]).resolve()
    image = (map_yaml.parent/yaml.safe_load(map_yaml.read_text())["image"]).resolve()
    if (binding["schema_version"] != 1 or not binding["map_id"] or not binding["map_version"]
            or hashlib.sha256(map_yaml.read_bytes()).hexdigest() != binding["map_sha256"]
            or hashlib.sha256(image.read_bytes()).hexdigest() != binding["image_sha256"]):
        raise ValueError("navigation_map_binding_hash_or_version_mismatch")
    for key, target in binding["destinations"].items():
        if (not key or require_reachability and not target.get("reachability_verified")
                or not all(math.isfinite(target[field]) for field in ("x_m", "y_m", "yaw_rad"))):
            raise ValueError("unverified_or_invalid_navigation_destination:"+key)
    return binding | {"map_yaml": str(map_yaml)}
