"""Start the orchard and a complete GPS-denied Panther navigation stack."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(
        get_package_share_directory('uav_groundtruth_mapping'))
    worlds_share = Path(
        get_package_share_directory('husarion_gz_worlds'))
    nav2_share = Path(
        get_package_share_directory('nav2_bringup'))
    orchard_assets = (
        Path.home() / 'uav_ugv_ros2_jazzy'
        / 'external' / 'cpr_gazebo')
    maps = Path.home() / 'husarion_ws' / 'maps'

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    corridors_csv = LaunchConfiguration('corridors_csv')
    use_rviz = LaunchConfiguration('use_rviz')

    gazebo = ExecuteProcess(
        cmd=[
            'gz', 'sim', [world, '.sdf'],
            '-r', '-v', '3', '--force-version', '8',
        ],
        output='screen',
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gps_denied_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    boundary_walls = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_orchard_safety_walls',
        condition=IfCondition(
            LaunchConfiguration('enable_boundary_walls')),
        arguments=[
            '-world', world,
            '-file', str(
                share / 'sdf' / 'orchard_safety_walls.sdf'),
            '-name', 'orchard_safety_walls',
            '-allow_renaming', 'false',
        ],
        output='screen',
    )
    panther = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'native_panther_sim.launch.py')),
        launch_arguments={
            'x': LaunchConfiguration('spawn_x'),
            'y': LaunchConfiguration('spawn_y'),
            'yaw': LaunchConfiguration('spawn_yaw'),
            'initial_estop': LaunchConfiguration('initial_estop'),
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / 'launch' / 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': str(
                share / 'config' / 'gps_denied_nav2.yaml'),
            'use_sim_time': 'true',
            'autostart': 'true',
            'slam': 'False',
            'use_localization': 'True',
            'use_namespace': 'False',
            'use_composition': 'False',
        }.items(),
    )
    mission = Node(
        package='uav_groundtruth_mapping',
        executable='serpentine_mission_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'corridors_csv': corridors_csv,
            'map_yaml': map_yaml,
            'waypoint_spacing': ParameterValue(
                LaunchConfiguration('waypoint_spacing'),
                value_type=float,
            ),
            'headland_clearance': ParameterValue(
                LaunchConfiguration('headland_clearance'),
                value_type=float,
            ),
            'autostart': ParameterValue(
                LaunchConfiguration('autostart_mission'),
                value_type=bool,
            ),
            'initial_pose_x': ParameterValue(
                LaunchConfiguration('spawn_x'),
                value_type=float,
            ),
            'initial_pose_y': ParameterValue(
                LaunchConfiguration('spawn_y'),
                value_type=float,
            ),
            'initial_pose_yaw': ParameterValue(
                LaunchConfiguration('spawn_yaw'),
                value_type=float,
            ),
            'required_readiness_topic':
                LaunchConfiguration('required_readiness_topic'),
            'required_readiness_label':
                LaunchConfiguration('required_readiness_label'),
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='gps_denied_navigation_rviz',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d',
            str(nav2_share / 'rviz' / 'nav2_default_view.rviz'),
        ],
        remappings=[
            ('/scan', '/panther/main_lidar/scan'),
            ('/robot_description', '/panther/robot_description'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='orchard'),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'panther_gps_denied_{os.getpid()}',
            description=(
                'Unique Gazebo Transport partition. This prevents a server '
                'left by an interrupted older run from publishing stale '
                'clock, odometry, or sensor messages into this run.')),
        DeclareLaunchArgument(
            'rmw_implementation',
            default_value='rmw_cyclonedds_cpp',
            description=(
                'ROS middleware used by every process in this launch. '
                'Cyclone DDS avoids Fast DDS lifecycle-service response '
                'losses seen when the complete Gazebo/Nav2 stack starts.')),
        DeclareLaunchArgument(
            'map',
            default_value=str(
                maps / 'latest_panther_row_corridors_nav.yaml')),
        DeclareLaunchArgument(
            'corridors_csv',
            default_value=str(
                maps
                / 'latest_panther_row_corridors_centerlines.csv')),
        DeclareLaunchArgument('spawn_x', default_value='-6.0'),
        DeclareLaunchArgument('spawn_y', default_value='-8.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'initial_estop',
            default_value='true',
            description=(
                'Start with the simulated hardware E-stop engaged. Set false '
                'only for unattended simulation runs.')),
        DeclareLaunchArgument(
            'waypoint_spacing', default_value='1.0'),
        DeclareLaunchArgument(
            'headland_clearance',
            default_value='2.25',
            description=(
                'Distance beyond row endpoints used for safe three-leg '
                'headland turns.')),
        DeclareLaunchArgument(
            'enable_boundary_walls',
            default_value='true',
            description=(
                'Spawn four collision walls at the measured orchard mesh '
                'limits. The walls are visible to Gazebo LiDAR.')),
        DeclareLaunchArgument(
            'autostart_mission',
            default_value='false',
            description=(
                'Start only after localization, LiDAR, E-stop, and Nav2 '
                'are ready. False keeps explicit operator start.')),
        DeclareLaunchArgument(
            'required_readiness_topic',
            default_value='',
            description=(
                'Optional transient-local Bool topic that must be true before '
                'the Panther route can start or continue.')),
        DeclareLaunchArgument(
            'required_readiness_label',
            default_value='coordinated vehicle'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('boundary_delay', default_value='3.0'),
        DeclareLaunchArgument('panther_delay', default_value='6.0'),
        DeclareLaunchArgument('nav2_delay', default_value='16.0'),
        DeclareLaunchArgument('mission_delay', default_value='19.0'),
        DeclareLaunchArgument('rviz_delay', default_value='20.0'),
        SetEnvironmentVariable('GTK_PATH', ''),
        SetEnvironmentVariable('GIO_MODULE_DIR', ''),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            LaunchConfiguration('rmw_implementation')),
        SetEnvironmentVariable(
            'GZ_PARTITION', LaunchConfiguration('gz_partition')),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [
                str(worlds_share / 'worlds'), ':',
                str(worlds_share / 'models'), ':',
                str(orchard_assets), ':',
                EnvironmentVariable(
                    'GZ_SIM_RESOURCE_PATH', default_value=''),
            ]),
        gazebo,
        clock_bridge,
        TimerAction(
            period=LaunchConfiguration('boundary_delay'),
            actions=[boundary_walls],
        ),
        TimerAction(
            period=LaunchConfiguration('panther_delay'),
            actions=[panther],
        ),
        TimerAction(
            period=LaunchConfiguration('nav2_delay'),
            actions=[nav2],
        ),
        TimerAction(
            period=LaunchConfiguration('mission_delay'),
            actions=[mission],
        ),
        TimerAction(
            period=LaunchConfiguration('rviz_delay'),
            actions=[rviz],
        ),
    ])
