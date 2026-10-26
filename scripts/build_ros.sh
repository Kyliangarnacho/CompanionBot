#!/usr/bin/env bash
set -e
companionbot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$companionbot_root"
source /opt/ros/jazzy/setup.bash
.venv/bin/python -c 'import sys; print(sys.executable); print(sys.version)'
.venv/bin/python -m pip --version
# Invoke colcon through the same interpreter; also pin CMake's Python generators.
.venv/bin/python -m colcon --log-base ros2_ws/log build --base-paths ros2_ws/src \
    --build-base ros2_ws/build --install-base ros2_ws/install --symlink-install \
    --cmake-args "-DPython3_EXECUTABLE=$companionbot_root/.venv/bin/python" \
    "-DPYTHON_EXECUTABLE=$companionbot_root/.venv/bin/python" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
