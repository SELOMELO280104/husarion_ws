"""Generate Panther row-corridor products from a ground-truth PCD."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


def _generator(context):
    arguments = [
        '--map-directory',
        LaunchConfiguration('map_directory').perform(context),
        '--output-base',
        LaunchConfiguration('output_base').perform(context),
        '--resolution',
        LaunchConfiguration('resolution').perform(context),
        '--robot-radius',
        LaunchConfiguration('robot_radius').perform(context),
        '--safety-margin',
        LaunchConfiguration('safety_margin').perform(context),
    ]
    source = LaunchConfiguration('input').perform(context)
    if source:
        arguments.extend(['--input', source])
    return [
        ExecuteProcess(
            cmd=[
                str(
                    Path(get_package_prefix(
                        'uav_groundtruth_mapping'))
                    / 'lib'
                    / 'uav_groundtruth_mapping'
                    / 'row_corridor_mask'),
                *arguments,
            ],
            output='screen',
        ),
    ]


def generate_launch_description():
    map_directory = Path.home() / 'husarion_ws' / 'maps'
    return LaunchDescription([
        DeclareLaunchArgument('input', default_value=''),
        DeclareLaunchArgument(
            'map_directory',
            default_value=str(map_directory)),
        DeclareLaunchArgument(
            'output_base',
            default_value=str(
                map_directory / 'latest_panther_row_corridors')),
        DeclareLaunchArgument('resolution', default_value='0.10'),
        DeclareLaunchArgument('robot_radius', default_value='0.65'),
        DeclareLaunchArgument('safety_margin', default_value='0.10'),
        OpaqueFunction(function=_generator),
    ])
