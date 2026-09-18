"""One-command ROS 2 port of the moving-platform landing demonstration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('uav_groundtruth_mapping'))
    base = share / 'launch' / 'complete_visual_follow_harvest.launch.py'
    controller = LaunchConfiguration('controller')
    policy_path = LaunchConfiguration('policy_path')
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller',
            default_value='pi',
            description='pi for the cascaded controller, policy for Double-Q'),
        DeclareLaunchArgument(
            'policy_path',
            default_value=str(
                Path.home() / 'husarion_ws' / 'maps'
                / 'repo_landing_double_q.json')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('automatic_landing', default_value='true'),
        DeclareLaunchArgument('tracking_settle_time', default_value='5.0'),
        DeclareLaunchArgument('descent_speed', default_value='0.10'),
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
            name='repo_uav_groundtruth_pose_bridge',
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
            executable='repo_landing_controller',
            name='repo_landing_controller',
            condition=IfCondition(PythonExpression([
                "'", controller, "' == 'pi'",
            ])),
            parameters=[{
                'use_sim_time': True,
                'automatic_landing': ParameterValue(
                    LaunchConfiguration('automatic_landing'), value_type=bool),
                'tracking_settle_time': ParameterValue(
                    LaunchConfiguration('tracking_settle_time'),
                    value_type=float),
                'descent_speed': ParameterValue(
                    LaunchConfiguration('descent_speed'), value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='uav_ugv_control',
            executable='repo_policy_controller',
            name='repo_policy_controller',
            condition=IfCondition(PythonExpression([
                "'", controller, "' == 'policy'",
            ])),
            parameters=[{
                'use_sim_time': True,
                'policy_path': policy_path,
                'descent_speed': ParameterValue(
                    LaunchConfiguration('descent_speed'), value_type=float),
            }],
            output='screen',
        ),
    ])
