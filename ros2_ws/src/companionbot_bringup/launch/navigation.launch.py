"""Stock Nav2 localization/planning/control; motion intent still needs authorization.

No sensor producer, actuator, SLAM, velocity smoother, recovery motion or GUI.
Map->odom must be owned exclusively by this launch's AMCL.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

from companionbot_bringup.navigation import navigation_radius, validate_map
from companionbot_bringup.processes import exit_with_runtime


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory("companionbot_bringup"))
    upstream_bt = Path(get_package_share_directory("nav2_bt_navigator"))/"behavior_trees"
    map_yaml = validate_map(value("map_yaml"))
    initial = value("set_initial_pose").lower()
    if initial not in ("true", "false") or value("use_sim_time").lower() not in ("true", "false"):
        raise ValueError("boolean launch values must be true or false")
    params = ParameterFile(RewrittenYaml(source_file=value("nav2_params_file"),
        param_rewrites={"use_sim_time": value("use_sim_time"),
                        "robot_radius": str(navigation_radius(share/"config/nav2_geometry.json"))},
        convert_types=True), allow_substs=True)
    common = [params, {"use_sim_time": value("use_sim_time").lower() == "true"}]
    nodes = [
        Node(package="nav2_map_server", executable="map_server", name="map_server", output="screen",
             parameters=[*common, {"yaml_filename": str(map_yaml)}]),
        Node(package="nav2_amcl", executable="amcl", name="amcl", output="screen",
             parameters=[*common, {"set_initial_pose": initial == "true",
                 "initial_pose.x": float(value("initial_pose_x")),
                 "initial_pose.y": float(value("initial_pose_y")),
                 "initial_pose.yaw": float(value("initial_pose_yaw"))}]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output="screen",
             parameters=common),
        Node(package="nav2_controller", executable="controller_server", name="controller_server", output="screen",
             parameters=common, remappings=[("cmd_vel", "/companionbot/navigation/cmd_vel")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output="screen",
             parameters=[*common, {
                 "default_nav_to_pose_bt_xml": str(upstream_bt/"navigate_w_replanning_only_if_path_becomes_invalid.xml")}]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="navigation_lifecycle", output="screen",
             parameters=[{"use_sim_time": value("use_sim_time").lower() == "true", "autostart": True,
                 "node_names": ["map_server", "amcl", "planner_server", "controller_server", "bt_navigator"],
                 "bond_timeout": 4.0, "attempt_respawn_reconnection": False}]),
    ]
    return [*nodes, *(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=exit_with_runtime)) for node in nodes)]


def generate_launch_description():
    share = Path(get_package_share_directory("companionbot_bringup"))
    defaults = {"map_yaml": "", "use_sim_time": "true", "set_initial_pose": "false",
        "initial_pose_x": "0.0", "initial_pose_y": "0.0", "initial_pose_yaw": "0.0",
        "nav2_params_file": str(share/"config/nav2.yaml")}
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=start)])
