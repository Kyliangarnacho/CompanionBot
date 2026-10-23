"""Bookstore sensor/control feature composition, headless and explicitly modal.

Mapping and localization never start together. Mapping motion is an explicit
finite diagnostic route; localization movement requires the authorized adapter.
The formal user interaction entry remains demo_stage8_interaction.py.
"""
from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from companionbot_contracts.paths import repository_root
from companionbot_contracts.navigation_map import load_map_binding


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory("companionbot_bringup"))
    mode = value("mode")
    if mode not in ("mapping", "localization"):
        raise ValueError("mode must explicitly select mapping or localization")
    identity = {}
    if mode == "localization":
        if not value("navigation_bindings"):
            raise ValueError("retail localization requires an explicit hash-verified navigation_bindings file")
        binding = load_map_binding(value("navigation_bindings"), require_reachability=False)
        identity = {"map_yaml": binding["map_yaml"], "navigation_map_id": binding["map_id"],
                    "navigation_map_version": binding["map_version"]}
        for key, expected in identity.items():
            if value(key) and value(key) != expected:
                raise ValueError("navigation_binding_conflicts_with_launch_argument:"+key)
    scenario = value("mapping_scenario") if mode == "mapping" else "navigation"
    if mode == "mapping" and scenario not in ("balance", "mapping"):
        raise ValueError("retail mapping_scenario must be balance or mapping")
    if mode == "mapping" and scenario == "mapping" and not value("route_file"):
        raise ValueError("retail mapping motion requires an explicit route_file; legacy room route is not reused")
    sensor_keys = ("world_assets", "route_file", "duration_s", "realtime_factor", "output_dir", "viewer",
        "record_truth", "finish_grace_s", "scan_dropout_start_s", "scan_dropout_duration_s",
        "imu_dropout_at_s", "navigation_map_id", "navigation_map_version")
    spatial = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch/spatial_simulation.launch.py")),
        launch_arguments={**{k: identity.get(k, value(k)) for k in sensor_keys}, "world": "bookstore", "scenario": scenario,
            "map_authority": "/slam_toolbox" if mode == "mapping" else "/amcl",
            "wait_for_scan_consumer": "true"}.items())
    if mode == "mapping":
        localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch/slam.launch.py")),
            launch_arguments={"use_sim_time": "true", "slam_params_file": value("slam_params_file"),
                              "map_file_name": value("map_file_name")}.items())
    else:
        keys = ("map_yaml", "nav2_params_file", "set_initial_pose", "initial_pose_x", "initial_pose_y", "initial_pose_yaw")
        localization = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch/navigation.launch.py")),
            launch_arguments={**{k: identity.get(k, value(k)) for k in keys}, "use_sim_time": "true"}.items())
    return [localization, spatial]


def generate_launch_description():
    share = Path(get_package_share_directory("companionbot_bringup"))
    defaults = {"mode": "mapping", "world_assets": "", "route_file": "", "mapping_scenario": "balance", "viewer": "false",
        "duration_s": "82.0", "realtime_factor": "1.0", "record_truth": "false", "finish_grace_s": "-1.0",
        "imu_dropout_at_s": "-1.0", "scan_dropout_start_s": "-1.0", "scan_dropout_duration_s": "0.0",
        "map_yaml": "", "map_file_name": "", "navigation_map_id": "", "navigation_map_version": "",
        "navigation_bindings": "",
        "set_initial_pose": "false", "initial_pose_x": "0.0", "initial_pose_y": "0.0", "initial_pose_yaw": "0.0",
        "nav2_params_file": str(share/"config/nav2.yaml"), "slam_params_file": str(share/"config/slam.yaml"),
        "output_dir": str(repository_root()/"models/minisegway/retail_navigation/results/sessions"/uuid4().hex)}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=start)])
