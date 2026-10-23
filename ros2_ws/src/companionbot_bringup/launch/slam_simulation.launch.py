"""Repeatable headless mapping experiment using existing spatial/control owners."""
import json
from pathlib import Path
from uuid import uuid4
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from companionbot_contracts.paths import repository_root


def generate_launch_description():
    share = Path(get_package_share_directory('companionbot_bringup'))
    route = json.loads((share/'config/mapping_route.json').read_text())
    defaults = {'scenario':'mapping', 'duration_s':str(route['duration_s']), 'realtime_factor':'1.0',
        'record_truth':'false', 'finish_grace_s':'-1.0', 'imu_dropout_at_s':'-1.0',
        'map_file_name':'',
        'scan_dropout_start_s':'-1.0', 'scan_dropout_duration_s':'0.0',
        'output_dir':str(repository_root()/'models/minisegway/slam_toolbox_integration/results/sessions'/uuid4().hex),
        'slam_params_file':str(share/'config/slam.yaml')}
    return LaunchDescription([
        *(DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/slam.launch.py')),
            launch_arguments={'use_sim_time':'true','slam_params_file':LaunchConfiguration('slam_params_file'),
                              'map_file_name':LaunchConfiguration('map_file_name')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/spatial_simulation.launch.py')),
            launch_arguments={**{k:LaunchConfiguration(k) for k in defaults if k not in ('slam_params_file','map_file_name')},
                              'map_authority':'/slam_toolbox', 'wait_for_scan_consumer':'true'}.items()),
    ])
