"""Spawn the sensor-equipped Panther with Gazebo's native skid-steer drive."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(
        get_package_share_directory('uav_groundtruth_mapping'))
    description_share = Path(
        get_package_share_directory('husarion_ugv_description'))
    components_share = Path(
        get_package_share_directory('husarion_components_description'))
    components_config = (
        description_share / 'config' / 'components.yaml')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            str(share / 'urdf' / 'panther_gps_denied.urdf.xacro'),
            ' use_sim:=true',
            ' namespace:=panther',
            ' components_config_path:=',
            str(components_config),
        ]),
        value_type=str,
    )
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='panther',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'frame_prefix': 'panther/',
        }],
        output='screen',
    )
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='panther',
        arguments=[
            '-name', 'panther',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', '0.2',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )
    component_bridges = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                components_share / 'launch'
                / 'gz_components.launch.py')),
        launch_arguments={
            'components_config_path': str(components_config),
            'namespace': 'panther',
        }.items(),
    )
    native_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='native_panther_bridge',
        arguments=[
            (
                '/panther/native/cmd_vel'
                '@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ),
            (
                '/panther/native/odometry'
                '@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ),
        ],
        output='screen',
    )
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='native_panther_imu_bridge',
        arguments=[
            (
                '/panther/imu/data_raw'
                '@sensor_msgs/msg/Imu[gz.msgs.IMU'
            ),
        ],
        output='screen',
    )
    drive = Node(
        package='uav_groundtruth_mapping',
        executable='native_panther_drive',
        parameters=[{
            'use_sim_time': True,
            'initial_estop': ParameterValue(
                LaunchConfiguration('initial_estop'),
                value_type=bool,
            ),
        }],
        output='screen',
    )
    sensor_fusion = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[
            str(share / 'config' / 'panther_sensor_fusion.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[
            ('odometry/filtered', '/panther/odometry/filtered'),
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-6.0'),
        DeclareLaunchArgument('y', default_value='-8.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('initial_estop', default_value='true'),
        state_publisher,
        spawn,
        component_bridges,
        native_bridge,
        imu_bridge,
        drive,
        sensor_fusion,
    ])
