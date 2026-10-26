"""Compatibility import for the unique native spatial_state executable.

The Python odometry math remains a test oracle in estimation.spatial. Runtime
source validation, lifecycle, odometry and TF publication belong to rclcpp.
Rebuild the ROS overlay before using this entry point; no Python fallback.
"""
import os
from pathlib import Path
import sys


def main():
    from ament_index_python.packages import get_package_prefix
    executable = Path(get_package_prefix("companionbot_control"))/"lib/companionbot_control/spatial_state"
    if not executable.is_file() or executable.read_bytes()[:4] != b"\x7fELF":
        raise RuntimeError("native spatial_state executable unavailable; run scripts/build_ros.sh")
    os.execv(str(executable), [str(executable), *sys.argv[1:]])
