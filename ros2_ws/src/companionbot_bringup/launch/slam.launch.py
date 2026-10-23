"""SLAM feature launch on the existing spatial graph; never a motion owner."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, GroupAction, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

from companionbot_bringup.processes import exit_with_runtime


def start(context):
    upstream = Path(get_package_share_directory("slam_toolbox"))
    filename = LaunchConfiguration("map_file_name").perform(context)
    if filename and (not Path(filename).is_absolute() or
            any(not Path(filename+ext).is_file() or Path(filename+ext).stat().st_size == 0
                for ext in ('.posegraph', '.data'))):
        raise ValueError('map_file_name requires an absolute basename with nonempty .posegraph and .data')
    parameters = [SetParameter(name="map_file_name", value=filename),
                  SetParameter(name="map_start_at_dock", value=True)] if filename else []
    return [GroupAction([*parameters,
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(upstream/"launch/online_async_launch.py")),
            launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time"),
                              "slam_params_file": LaunchConfiguration("slam_params_file")}.items())])]


def generate_launch_description():
    share = Path(get_package_share_directory("companionbot_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("slam_params_file", default_value=str(share/"config/slam.yaml")),
        DeclareLaunchArgument("map_file_name", default_value=""),
        OpaqueFunction(function=start),
        RegisterEventHandler(OnProcessExit(on_exit=exit_with_runtime)),
    ])
