#!/usr/bin/env bash
# Standard ROS image + SLAM Toolbox pose graph; run while mapping is active.
set -euo pipefail
if [ "$#" -ne 1 ] || [[ "$1" != /* ]]; then
    echo "Usage: bash scripts/save_slam_map.sh /absolute/path/map_basename" >&2
    exit 2
fi
companionbot_map_prefix="$1"
for companionbot_map_ext in yaml pgm posegraph data; do
    if [ -e "$companionbot_map_prefix.$companionbot_map_ext" ]; then
        echo "Refusing to overwrite $companionbot_map_prefix.$companionbot_map_ext" >&2
        exit 2
    fi
done
mkdir -p -- "$(dirname -- "$companionbot_map_prefix")"
ros2 run nav2_map_server map_saver_cli -f "$companionbot_map_prefix" --ros-args \
    -p use_sim_time:=true -p save_map_timeout:=10.0 \
    -p free_thresh_default:=0.25 -p occupied_thresh_default:=0.65
# Use the typed service from the same project interpreter to check the result;
# ros2 service call's process exit does not imply RESULT_SUCCESS.
"$COMPANIONBOT_ROOT/.venv/bin/python" - "$companionbot_map_prefix" <<'PY'
import sys
import rclpy
from slam_toolbox.srv import SerializePoseGraph
rclpy.init()
node = rclpy.create_node('save_slam_pose_graph')
try:
    client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
    if not client.wait_for_service(timeout_sec=10.):
        raise RuntimeError('SLAM Toolbox serialization service unavailable')
    request = SerializePoseGraph.Request(filename=sys.argv[1])
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.)
    if not future.done() or future.result().result != SerializePoseGraph.Response.RESULT_SUCCESS:
        raise RuntimeError('SLAM Toolbox pose graph save failed; image files may already exist')
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY
for companionbot_map_ext in yaml pgm posegraph data; do
    test -s "$companionbot_map_prefix.$companionbot_map_ext"
done
