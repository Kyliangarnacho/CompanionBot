# Source this file from bash; all project Python/ROS commands use the Linux .venv.
companionbot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [ ! -x "$companionbot_root/.venv/bin/python" ] || [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "Missing Linux .venv or ROS 2 Jazzy; see docs/RUNTIME_ARCHITECTURE.md" >&2
    return 1
fi
source /opt/ros/jazzy/setup.bash
source "$companionbot_root/.venv/bin/activate"
if [ -d "$companionbot_root/.venv/native/usr/lib/x86_64-linux-gnu" ]; then
    export LD_LIBRARY_PATH="$companionbot_root/.venv/native/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PATH="$companionbot_root/.venv/native/usr/bin:$PATH"
    export ALSA_PLUGIN_DIR="$companionbot_root/.venv/native/usr/lib/x86_64-linux-gnu/alsa-lib"
fi
if [ -n "${PULSE_SERVER:-}" ] && ! compgen -G '/dev/snd/pcm*' > /dev/null; then
    export ALSA_CONFIG_PATH="$companionbot_root/tools/environment/alsa_pulse.conf"
fi
if [ -f "$companionbot_root/ros2_ws/install/setup.bash" ]; then
    source "$companionbot_root/ros2_ws/install/setup.bash"
fi
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$companionbot_root/models/minisegway/runtime_architecture/results/ros_logs}"
export YOLO_CONFIG_DIR="$companionbot_root/.venv/ultralytics"
export MPLCONFIGDIR="$companionbot_root/.venv/matplotlib"
export PIP_CACHE_DIR="$companionbot_root/.venv/pip-cache"
export COMPANIONBOT_ROOT="$companionbot_root"
export COLCON_DEFAULTS_FILE="$companionbot_root/ros2_ws/colcon.defaults.yaml"
if [ -d "$companionbot_root/.venv/rosdep/sources.list.d" ]; then
    export ROSDEP_SOURCE_PATH="$companionbot_root/.venv/rosdep/sources.list.d"
    export ROS_HOME="$companionbot_root/.venv/ros_home"
fi
mkdir -p "$YOLO_CONFIG_DIR" "$MPLCONFIGDIR" "$PIP_CACHE_DIR" "$ROS_LOG_DIR"
unset companionbot_root
