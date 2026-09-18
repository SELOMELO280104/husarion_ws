"""Launch the safe RL search adapter and online trainer together."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('training_minutes', default_value='5.0'),
        DeclareLaunchArgument(
            'table_path',
            default_value='/home/robocare/husarion_ws/maps/uav_search_q_table.json'),
        Node(
            package='uav_groundtruth_mapping',
            executable='rl_search_ros_adapter',
            name='rl_search_ros_adapter',
            parameters=[{'enabled': LaunchConfiguration('enabled')}],
            output='screen'),
        Node(
            package='uav_groundtruth_mapping',
            executable='rl_search_training',
            name='rl_search_training',
            parameters=[{
                'enabled': LaunchConfiguration('enabled'),
                'table_path': LaunchConfiguration('table_path'),
                'training_minutes': LaunchConfiguration('training_minutes'),
            }],
            output='screen'),
        Node(
            package='uav_groundtruth_mapping',
            executable='rl_training_dashboard',
            name='rl_training_dashboard',
            output='screen'),
    ])
