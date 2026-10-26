"""Finite spatial sensor/TF diagnostic; never a second formal interaction Demo.

Intentionally starts no GUI. RViz is opened manually by the user if desired.
"""
import json
from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scipy.spatial.transform import Rotation
import xacro

from companionbot_contracts.paths import repository_root
from companionbot_bringup.processes import exit_with_runtime


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    camera = value("provisional_camera").lower()
    if camera not in ("true", "false"):
        raise ValueError("provisional_camera must be true or false")
    share = Path(get_package_share_directory("companionbot_description"))
    mappings = {"geometry_config": str(share/"config/minisegway.yaml"),
                "lidar_config": str(share/"config/sim_lidar.yaml"),
                "sim_lidar": "true", "provisional_camera": camera}
    visual_config = value('visual_companion_config')
    if visual_config and camera == 'true':
        raise ValueError('select_one_sim_camera_or_provisional_camera_mount')
    if camera == "true" or visual_config:
        # Reuse the existing provisional extrinsic as the one source of this
        # fact. RSP is the only mount authority in this diagnostic graph.
        if visual_config:
            from companionbot_simulation.core.visual_companion_scene import load_visual_camera
            visual = json.loads(Path(visual_config).read_text())
            mount = load_visual_camera(repository_root()/visual['camera_config'])['camera_extrinsic']
            mappings['provisional_camera'] = 'true'  # Existing xacro mount switch; exact sim values below.
        else:
            mount = json.loads((repository_root()/"models/minisegway/stage7/config/final_demo.json").read_text())["camera_extrinsic"]
        body_optical = Rotation.from_matrix(mount["rotation_body_camera"])
        camera_optical = Rotation.from_euler("xyz", [-1.5707963267948966, 0., -1.5707963267948966])
        mappings["camera_xyz"] = " ".join(map(str, mount["translation_body_camera_m"]))
        mappings["camera_rpy"] = " ".join(map(str, (body_optical*camera_optical.inv()).as_euler("xyz")))
    description = xacro.process_file(str(share/"urdf/companionbot.urdf.xacro"), mappings=mappings).toxml()
    namespace = value("namespace")
    config = Path(get_package_share_directory("companionbot_bringup"))/"config/spatial.yaml"
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
             namespace=namespace, output="screen", parameters=[{"robot_description": description,
                 "use_sim_time": True, "publish_frequency": 100.}],
             remappings=[("joint_states", "joint_states")]),
        Node(package="companionbot_control", executable="spatial_state", namespace=namespace, output="screen",
             parameters=[str(config), {"use_sim_time": True,
                 "geometry_config": str(share/"config/minisegway.yaml")}]),
        Node(package="companionbot_control", executable="tf_authority", namespace=namespace, output="screen",
             parameters=[{"use_sim_time": True, "map_authority": value("map_authority")}]),
        Node(package="companionbot_simulation", executable="spatial_sim", namespace=namespace, output="screen",
             additional_env={"OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1",
                 **({'MUJOCO_GL':'egl'} if visual_config else {})},
             parameters=[{"use_sim_time": True, "output_dir": value("output_dir"), "scenario": value("scenario"),
                 "geometry_config": mappings["geometry_config"], "lidar_config": mappings["lidar_config"],
                 "duration_s": float(value("duration_s")), "realtime_factor": float(value("realtime_factor")),
                 "viewer": value("viewer").lower() == "true",
                 "imu_dropout_at_s": float(value("imu_dropout_at_s")),
                 "record_truth": value("record_truth").lower() == "true",
                 "wait_for_scan_consumer": value("wait_for_scan_consumer").lower() == "true",
                 "finish_grace_s": float(value("finish_grace_s")),
                 "scan_dropout_start_s": float(value("scan_dropout_start_s")),
                 "scan_dropout_duration_s": float(value("scan_dropout_duration_s")),
                 "world": value("world"), "world_assets": value("world_assets"),
                 "route_file": value("route_file"),
                 "navigation_map_id": value("navigation_map_id"),
                 "navigation_map_version": value("navigation_map_version"),
                 "visual_companion_config": visual_config}]),
    ]
    return [*nodes, *(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=exit_with_runtime)) for node in nodes)]


def generate_launch_description():
    defaults = {"namespace": "companionbot", "scenario": "mixed", "duration_s": "14.0", "viewer": "false",
                "realtime_factor": "1.0", "provisional_camera": "false", "imu_dropout_at_s": "-1.0",
                "finish_grace_s": "0.25", "record_truth": "false", "map_authority": "",
                "wait_for_scan_consumer": "false",
                "scan_dropout_start_s": "-1.0", "scan_dropout_duration_s": "0.0",
                "world": "indoor", "world_assets": "", "route_file": "",
                "navigation_map_id": "", "navigation_map_version": "",
                "visual_companion_config": "",
                "output_dir": str(repository_root()/"models/minisegway/spatial_slam_foundation/results/sessions"/uuid4().hex)}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=start)])
