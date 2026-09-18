"""Generate a 2D map from the latest GPS PCD and start Panther Nav2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav_gps_mapping.pcd_to_occupancy import (
    newest_gps_map,
    read_xyz,
    save_traversability_products,
)


def generate_launch_description():
    share = Path(get_package_share_directory('uav_gps_mapping'))
    nav2_share = Path(get_package_share_directory('nav2_bringup'))
    map_directory = Path.home() / 'husarion_ws' / 'maps'
    source = newest_gps_map(map_directory)
    save_traversability_products(
        read_xyz(source),
        map_directory / 'latest_panther_traversability',
        resolution=0.10,
    )
    map_yaml = (
        map_directory / 'latest_panther_traversability_nav.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / 'launch' / 'bringup_launch.py')),
        launch_arguments={
            'map': str(map_yaml),
            'params_file': str(share / 'config' / 'panther_nav2.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'slam': 'False',
            'use_localization': 'True',
            'use_namespace': 'False',
            'use_composition': 'False',
        }.items(),
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / 'launch' / 'rviz_launch.py')),
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'rviz_config': str(
                nav2_share / 'rviz' / 'nav2_default_view.rviz'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'map_directory', default_value=str(map_directory)),
        nav2,
        Node(
            package='uav_gps_mapping',
            executable='nav_cmd_relay',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        rviz,
    ])
