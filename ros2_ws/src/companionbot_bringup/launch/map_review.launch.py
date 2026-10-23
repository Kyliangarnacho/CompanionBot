"""Standard static OccupancyGrid review; no localization or map->odom TF."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'),
                'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='map_review_lifecycle',
            output='screen', parameters=[{'autostart':True, 'node_names':['map_server']}]),
    ])
