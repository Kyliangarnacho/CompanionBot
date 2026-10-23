"""Frozen control/plant diagnostic; not a second formal user Demo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from companionbot_contracts.paths import repository_root
from uuid import uuid4
from companionbot_bringup.processes import exit_with_runtime


def generate_launch_description():
    node = Node(package="companionbot_simulation", executable="frozen_sim", output="screen",
                namespace=LaunchConfiguration("namespace"), parameters=[{
                    "duration_s": ParameterValue(LaunchConfiguration("duration_s"), value_type=float),
                    "realtime_factor": ParameterValue(LaunchConfiguration("realtime_factor"), value_type=float),
                    "output_dir": LaunchConfiguration("output_dir")}])
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="companionbot"),
        DeclareLaunchArgument("output_dir", default_value=str(repository_root() /
                              "models/minisegway/runtime_architecture/results/sessions" / uuid4().hex)),
        DeclareLaunchArgument("duration_s", default_value="8.0"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        node, RegisterEventHandler(OnProcessExit(target_action=node, on_exit=exit_with_runtime))])
