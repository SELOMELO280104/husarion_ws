"""Camera tracking with the repository-style cascaded PI landing fallback."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    base = Path(get_package_share_directory('uav_groundtruth_mapping')) \
        / 'launch' / 'complete_visual_follow_harvest.launch.py'
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(base)), launch_arguments={
            'panther_x': '-6.0', 'panther_y': '-8.0',
            'uav_x': '-6.0', 'uav_y': '-9.0', 'uav_z': '0.8',
            'manual_uav_start': 'false', 'uav_search_delay': '0.0',
            'enable_rl_search': 'false',
            'enable_rl_landing': 'true',
            'rl_policy_path': '',
            'enable_visual_tracker': 'true',
            'control_backend': 'gazebo_velocity',
        }.items())])
