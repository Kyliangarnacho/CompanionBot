#!/usr/bin/env bash
# Initialize official rosdep metadata inside .venv; no sudo/system edits.
set -e
companionbot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$companionbot_root"
source scripts/env.sh
companionbot_root="$COMPANIONBOT_ROOT"
python -c 'import sys; print(sys.executable); print(sys.version)'
python -m pip --version
export ROSDEP_SOURCE_PATH="$companionbot_root/.venv/rosdep/sources.list.d"
export ROS_HOME="$companionbot_root/.venv/ros_home"
mkdir -p "$ROSDEP_SOURCE_PATH"
curl --fail --location --retry 2 \
    https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list \
    -o "$ROSDEP_SOURCE_PATH/20-default.list"
python /usr/bin/rosdep update --rosdistro jazzy
python /usr/bin/rosdep check --from-paths ros2_ws/src --ignore-src
