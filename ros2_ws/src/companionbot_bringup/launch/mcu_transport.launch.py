"""Explicit MCU communication commissioning; not a second interaction Demo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from companionbot_bringup.processes import exit_with_runtime

def generate_launch_description():
    node=Node(package="companionbot_control",executable="mcu_bridge",namespace=LaunchConfiguration("namespace"),
        parameters=[PathJoinSubstitution([FindPackageShare("companionbot_bringup"),"config","mcu_bridge.yaml"]),
            {"device":LaunchConfiguration("device"),"allow_command_tx":ParameterValue(LaunchConfiguration("allow_command_tx"),value_type=bool)}],
        output="screen")
    return LaunchDescription([
        DeclareLaunchArgument("device",description="Explicit serial by-id path; no implicit port selection"),
        DeclareLaunchArgument("namespace",default_value="companionbot"),
        DeclareLaunchArgument("allow_command_tx",default_value="false"),
        RegisterEventHandler(OnProcessExit(target_action=node,on_exit=exit_with_runtime)),node])
