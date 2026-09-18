"""Opt-in randomized, collision-checked wrapper for the complete launch."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from uav_groundtruth_mapping.safe_spawn import SpawnPose, choose_safe_spawn


def generate_launch_description():
    bounds = (-8.0, 36.0, -6.0, 24.0)
    obstacles = ()
    uav_xy = ((-2.7281, -4.3581), (34.3122, -2.6755), (33.8033, 21.0960))
    ugv_xy = ((33.9756, 13.4418), (33.7684, 5.1691), (-6.0275, 7.9315),
              (-6.4493, 15.5260), (20.4980, 22.1624), (18.5101, 10.0502),
              (17.5836, -1.6993))
    ugv = choose_safe_spawn(
        [SpawnPose(x, y, 0.0, 1.0) for x, y in ugv_xy], obstacles,
        bounds=bounds)
    uav = choose_safe_spawn(
        [SpawnPose(x, y, 8.0, 0.8) for x, y in uav_xy],
        obstacles, other_poses=(ugv,), bounds=bounds, min_altitude=2.0)
    launch_file = Path(get_package_share_directory('uav_groundtruth_mapping')) \
        / 'launch' / 'complete_visual_follow_harvest.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('enable_rl_search', default_value='false'),
        DeclareLaunchArgument('manual_uav_start', default_value='true'),
        DeclareLaunchArgument('uav_search_delay', default_value='30.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('show_uav_camera', default_value='true'),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_file)), launch_arguments={
            'panther_x': str(ugv.x), 'panther_y': str(ugv.y),
            'uav_x': str(uav.x), 'uav_y': str(uav.y), 'uav_z': str(uav.z),
            'enable_rl_search': LaunchConfiguration('enable_rl_search'),
            'manual_uav_start': LaunchConfiguration('manual_uav_start'),
            'uav_search_delay': LaunchConfiguration('uav_search_delay'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'show_uav_camera': LaunchConfiguration('show_uav_camera'),
        }.items())])
