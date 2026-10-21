"""Frozen control/plant diagnostic; not a second formal user Demo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("duration_s", default_value="8.0"),
        DeclareLaunchArgument("realtime_factor", default_value="1.0"),
        Node(package="companionbot_simulation", executable="frozen_sim", output="screen", parameters=[{
            "duration_s": ParameterValue(LaunchConfiguration("duration_s"), value_type=float),
            "realtime_factor": ParameterValue(LaunchConfiguration("realtime_factor"), value_type=float)}])])
