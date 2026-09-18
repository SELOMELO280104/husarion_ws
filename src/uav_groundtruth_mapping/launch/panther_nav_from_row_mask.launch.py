"""Generate the orchard row mask and start Panther Nav2 on that mask."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav_gps_mapping.pcd_to_occupancy import read_xyz
from uav_groundtruth_mapping.row_corridor_mask import (
    newest_groundtruth_map,
    save_row_corridor_products,
)


def generate_launch_description():
    map_directory = Path.home() / 'husarion_ws' / 'maps'
    source = newest_groundtruth_map(map_directory)
    output_base = (
        map_directory / 'latest_panther_row_corridors')
    save_row_corridor_products(
        read_xyz(source),
        output_base,
        resolution=0.10,
        source_name=str(source),
    )

    gps_share = Path(
        get_package_share_directory('uav_gps_mapping'))
    nav2_share = Path(
        get_package_share_directory('nav2_bringup'))
    map_yaml = output_base.with_name(
        f'{output_base.name}_nav.yaml')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / 'launch' / 'bringup_launch.py')),
        launch_arguments={
            'map': str(map_yaml),
            'params_file': str(
                gps_share / 'config' / 'panther_nav2.yaml'),
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
        nav2,
        Node(
            package='uav_gps_mapping',
            executable='nav_cmd_relay',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        rviz,
    ])
