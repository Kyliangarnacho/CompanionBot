"""C++ spatial/source boundary parity. Requires the real native build."""
from dataclasses import asdict
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess

import numpy as np
import pytest

from companionbot_contracts.paths import repository_root
from companionbot_contracts.spatial_geometry import DESCRIPTION
from companionbot_control.estimation.spatial import PlanarOdometry, SpatialSample


def executable():
    selected = os.environ.get("COMPANIONBOT_SPATIAL_REPLAY")
    path = Path(selected) if selected else repository_root()/"ros2_ws/build/companionbot_control/spatial_replay"
    assert path.is_file(), "Build native spatial code with scripts/build_ros.sh before running parity"
    return path


def test_native_source_safety_and_fault_latch():
    selected = executable().with_name("spatial_native_test")
    result = subprocess.run([str(selected)], capture_output=True, text=True, check=True)
    assert "invariants passed" in result.stdout


def test_native_randomized_tick_pose_and_covariance_parity(tmp_path):
    rng = np.random.default_rng(20261023)
    inputs, expected = [], []
    odom = PlanarOdometry()
    distance = yaw = 0.
    for index in range(6000):
        t = (index+1)*.02
        distance += float(rng.normal(.003, .01))
        yaw += float(rng.normal(.001, .04))
        sample = SpatialSample("source", 10*(index+1), t, distance, .15,
            .02*math.sin(index*.1), .04*math.sin(index*.05), yaw, .001, .002, .05,
            (.01*index, -.02*index), (.5, -1.), t-.001, t, .001)
        inputs.append(asdict(sample))
        estimate = odom.update(sample)
        expected.append([estimate.x_m, estimate.y_m, estimate.yaw_rad, *estimate.covariance.ravel()])
    source = tmp_path/"input.jsonl"
    source.write_text(''.join(json.dumps(item)+'\n' for item in inputs))
    result = subprocess.run([str(executable()), "--replay", str(source)], check=True, capture_output=True, text=True)
    actual = []
    for line in result.stdout.splitlines():
        item = json.loads(line)
        actual.append([item["x_m"], item["y_m"], item["yaw_rad"], *np.asarray(item["covariance"]).ravel()])
    # Same equations; BLAS and scalar C++ matrix operation rounding may differ.
    np.testing.assert_allclose(actual, expected, rtol=0, atol=3e-12)


@pytest.mark.parametrize("extra", [
    [0., -0., .0001, .00001, 1e15, 1e16, 1e17, 1e-7, 1.2345678901234567],
    [1000000000000000128., 1.0000000000000001e18, 1.234e-8, 1.23e20, 1.234e-4],
    {"文字": "保留规范 / 😀", "array": [True, False, None, -1, 10**20]},
])
def test_native_retains_original_python_geometry_digest_normalization(tmp_path, extra):
    data = json.loads((DESCRIPTION/"config/minisegway.yaml").read_text())
    data.pop("geometry_id")
    data["audit_extra"] = extra
    canonical = json.dumps(data, sort_keys=True)
    data["geometry_id"] = hashlib.sha256(canonical.encode()).hexdigest()
    source = tmp_path/"geometry.json"
    source.write_text(json.dumps(data))
    result = subprocess.run([str(executable()), "--geometry", str(source)], check=True, capture_output=True, text=True)
    actual = json.loads(result.stdout)
    assert actual["canonical"] == canonical
    assert actual["geometry_id"] == data["geometry_id"]


def test_native_rejects_modified_geometry_without_changing_canonical_source(tmp_path):
    data = json.loads((DESCRIPTION/"config/minisegway.yaml").read_text())
    data["wheel_radius_m"] *= 2
    source = tmp_path/"geometry.json"
    source.write_text(json.dumps(data))
    result = subprocess.run([str(executable()), "--geometry", str(source)], capture_output=True, text=True)
    assert result.returncode != 0 and "digest mismatch" in result.stderr
