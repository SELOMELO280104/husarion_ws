"""One-command safe simulator plus RL search training."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = Path(get_package_share_directory('uav_groundtruth_mapping'))
    random_launch = share / 'launch' / 'random_safe_visual_follow_harvest.launch.py'
    training_launch = share / 'launch' / 'rl_search_training.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(random_launch)),
            launch_arguments={
                'enable_rl_search': 'true',
                'manual_uav_start': 'false',
                'uav_search_delay': '0.0',
                'use_rviz': 'false',
                'show_uav_camera': 'true',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(training_launch)),
            launch_arguments={
                'enabled': 'true',
                'table_path': '/home/robocare/husarion_ws/maps/uav_search_q_table.json',
                'training_minutes': '5.0',
            }.items()),
    ])
