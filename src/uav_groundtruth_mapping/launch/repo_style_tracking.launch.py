"""Compatibility alias for the repository-style landing demonstration."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('uav_groundtruth_mapping'))
    landing = share / 'launch' / 'repo_style_landing.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(landing))),
    ])
