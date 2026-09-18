"""Original-position tracking test with QR/ArUco visible after takeoff."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_file = Path(get_package_share_directory('uav_groundtruth_mapping')) \
        / 'launch' / 'complete_visual_follow_harvest.launch.py'
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)), launch_arguments={
            'panther_x': '-6.0',
            'panther_y': '-8.0',
            'panther_yaw': '0.0',
            'uav_x': '-6.0',
            'uav_y': '-9.0',
            'uav_z': '0.8',
            'uav_yaw': '0.0',
            'manual_uav_start': 'false',
            'uav_search_delay': '0.0',
            'enable_rl_search': 'false',
            'enable_rl_landing': 'false',
            'control_backend': 'gazebo_velocity',
        }.items())])
