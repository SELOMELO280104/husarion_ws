"""One-command Vicon-style ground-truth relative tracking test."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('uav_groundtruth_mapping'))
    base = share / 'launch' / 'complete_visual_follow_harvest.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base)), launch_arguments={
                'panther_x': '-6.0', 'panther_y': '-8.0',
                'uav_x': '-6.0', 'uav_y': '-9.0', 'uav_z': '0.8',
                'manual_uav_start': 'false', 'uav_search_delay': '0.0',
                'enable_visual_tracker': 'false',
                'enable_rl_search': 'false', 'enable_rl_landing': 'false',
            }.items()),
        Node(
            package='uav_ugv_control',
            executable='groundtruth_relative_tracker',
            name='groundtruth_relative_tracker',
            parameters=[{
                'uav_pose_topic': '/fmu/out/vehicle_local_position_v1',
                'ugv_pose_topic': '/panther/native/odometry',
                'desired_height': 10.0,
            }], output='screen'),
    ])
