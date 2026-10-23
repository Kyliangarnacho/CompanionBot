"""Spatial geometry contract; derived from MJCF, with no simulator dependency."""
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

from .paths import repository_root

DESCRIPTION = repository_root() / "ros2_ws/src/companionbot_description"


@dataclass(frozen=True)
class SpatialGeometry:
    geometry_id: str
    wheel_radius_m: float
    wheel_separation_m: float
    ros_left_encoder_index: int
    ros_right_encoder_index: int
    mj_to_ros: np.ndarray

    def vector_to_ros(self, value):
        vector = np.asarray(value, dtype=float)
        if vector.shape != (3,) or not np.isfinite(vector).all():
            raise ValueError("native sensor vector must be finite length 3")
        return self.mj_to_ros @ vector


def load_spatial_geometry(path=None):
    raw = json.loads(Path(path or DESCRIPTION / "config/minisegway.yaml").read_text())
    content = {key: value for key, value in raw.items() if key != "geometry_id"}
    if raw["geometry_id"] != hashlib.sha256(json.dumps(content, sort_keys=True).encode()).hexdigest():
        raise ValueError("spatial geometry digest mismatch")
    matrix = np.asarray(raw["mj_to_ros"], dtype=float)
    if (raw["schema_version"] != 1 or matrix.shape != (3, 3)
            or not np.isfinite(matrix).all() or not np.allclose(matrix.T @ matrix, np.eye(3))
            or not np.isclose(np.linalg.det(matrix), 1.0)
            or not np.isfinite([raw["wheel_radius_m"], raw["wheel_separation_m"]]).all()
            or min(raw["wheel_radius_m"], raw["wheel_separation_m"]) <= 0
            or {raw["ros_left_encoder_index"], raw["ros_right_encoder_index"]} != {0, 1}):
        raise ValueError("invalid spatial geometry")
    matrix.setflags(write=False)
    return SpatialGeometry(raw["geometry_id"], raw["wheel_radius_m"], raw["wheel_separation_m"],
                           raw["ros_left_encoder_index"], raw["ros_right_encoder_index"], matrix)


def load_sim_lidar(path=None):
    raw = json.loads(Path(path or DESCRIPTION / "config/sim_lidar.yaml").read_text())
    if (raw["hardware_calibrated"] or raw["acquisition"] != "instantaneous_all_rays"
            or not isinstance(raw["rays"], int) or not 4 <= raw["rays"] <= 4096
            or not np.isfinite(raw["xyz_m"] + raw["rpy_rad"]).all()
            or len(raw["xyz_m"]) != 3 or len(raw["rpy_rad"]) != 3
            or not 0 < raw["period_s"] <= 1
            or not 0 < raw["range_min_m"] < raw["range_max_m"]
            or not 0 <= raw["range_noise_std_m"] < raw["range_min_m"]):
        raise ValueError("invalid simulation-only lidar configuration")
    return raw
