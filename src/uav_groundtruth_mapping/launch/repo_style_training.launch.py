"""One-command online Double-Q training in the moving orchard simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('uav_groundtruth_mapping'))
    base = share / 'launch' / 'complete_visual_follow_harvest.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument(
            'policy_path',
            default_value=str(
                Path.home() / 'husarion_ws' / 'maps'
                / 'repo_landing_double_q.json')),
        DeclareLaunchArgument('max_episodes', default_value='50000'),
        DeclareLaunchArgument('auto_resume', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(base)),
            launch_arguments={
                'panther_x': '-6.0',
                'panther_y': '-8.0',
                'panther_yaw': '0.0',
                'uav_x': '-6.0',
                'uav_y': '-8.0',
                'uav_z': '4.0',
                'uav_yaw': '0.0',
                'manual_uav_start': 'false',
                'enable_visual_tracker': 'false',
                'enable_rl_search': 'false',
                'enable_rl_landing': 'false',
                'show_uav_camera': 'false',
                'autostart_mission': 'true',
                'initial_estop': 'false',
                'use_rviz': LaunchConfiguration('use_rviz'),
                'control_backend': 'gazebo_velocity',
            }.items(),
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='repo_training_uav_pose_bridge',
            arguments=[
                '/model/x500_flow_tag_0/pose'
                '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '--ros-args', '-r',
                '/model/x500_flow_tag_0/pose:=/repo_landing/uav_pose',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='uav_ugv_control',
            executable='repo_relative_state',
            name='repo_relative_state',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='uav_ugv_control',
            executable='repo_landing_trainer',
            name='repo_landing_trainer',
            parameters=[{
                'use_sim_time': True,
                'policy_path': LaunchConfiguration('policy_path'),
                'max_episodes': ParameterValue(
                    LaunchConfiguration('max_episodes'), value_type=int),
                'auto_resume': ParameterValue(
                    LaunchConfiguration('auto_resume'), value_type=bool),
            }],
            output='screen',
        ),
    ])
