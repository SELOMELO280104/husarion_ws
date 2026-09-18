"""Start orchard, PX4, exact-pose mapper, and RViz in one launch."""

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


def generate_launch_description():
    share = Path(
        get_package_share_directory('uav_groundtruth_mapping'))
    worlds_share = Path(
        get_package_share_directory('husarion_gz_worlds'))
    husarion_share = Path(
        get_package_share_directory('husarion_ugv_gazebo'))
    orchard_assets = (
        Path.home() / 'uav_ugv_ros2_jazzy'
        / 'external' / 'cpr_gazebo')
    world = LaunchConfiguration('world')
    px4_dir = LaunchConfiguration('px4_dir')

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
        name='groundtruth_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    panther = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(husarion_share / 'launch' / 'simulate_robot.launch.py')),
        launch_arguments={
            'namespace': 'panther',
            'robot_model': 'panther',
            'x': '-6.0',
            'y': '-8.0',
            'z': '0.2',
            'yaw': '0.0',
            'add_world_transform': 'False',
            'disable_manager': 'False',
        }.items(),
    )
    px4 = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth_gps'],
        cwd=px4_dir,
        additional_env={
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': world,
            'PX4_GZ_MODEL_POSE': LaunchConfiguration('uav_pose'),
            'PX4_GZ_NO_FOLLOW': '1',
            'PX4_UXRCE_DDS_PORT': '9999',
            'CCACHE_DIR': '/tmp/uav_groundtruth_mapping_ccache',
            'PX4_PARAM_SYS_HAS_BARO': '0',
            'PX4_PARAM_EKF2_HGT_REF': '1',
            'PX4_PARAM_SENS_EN_BAROSIM': '1',
            'PX4_PARAM_SENS_EN_MAGSIM': '1',
            'PX4_PARAM_NAV_DLL_ACT': '0',
            'PX4_PARAM_NAV_RCL_ACT': '0',
            'PX4_PARAM_MPC_XY_VEL_MAX':
                LaunchConfiguration('survey_speed'),
            'PX4_PARAM_MPC_XY_CRUISE':
                LaunchConfiguration('survey_speed'),
        },
        output='screen',
    )
    sensor_systems = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', ['/world/', world, '/entity/system/add'],
            '--reqtype', 'gz.msgs.EntityPlugin_V',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req',
            (
                'entity: {id: 1}, plugins: ['
                '{filename: "gz-sim-air-pressure-system", '
                'name: "gz::sim::systems::AirPressure"}, '
                '{filename: "gz-sim-magnetometer-system", '
                'name: "gz::sim::systems::Magnetometer"}]'
            ),
        ],
        output='screen',
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                share / 'launch'
                / 'groundtruth_mapping.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'map_directory': LaunchConfiguration('map_directory'),
            'voxel_size': LaunchConfiguration('voxel_size'),
            'scan_delay': LaunchConfiguration('scan_delay'),
            'max_lidar_range': LaunchConfiguration('max_lidar_range'),
            'min_downward_angle_deg':
                LaunchConfiguration('min_downward_angle_deg'),
            'max_rgb_time_offset':
                LaunchConfiguration('max_rgb_time_offset'),
            'enable_statistical_filter':
                LaunchConfiguration('enable_statistical_filter'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='orchard'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=str(Path.home() / 'PX4-Autopilot')),
        DeclareLaunchArgument(
            'uav_pose', default_value='0,-5,0.2,0,0,0'),
        DeclareLaunchArgument(
            'survey_speed', default_value='1.0'),
        DeclareLaunchArgument(
            'voxel_size', default_value='0.10'),
        DeclareLaunchArgument(
            'scan_delay', default_value='0.10'),
        DeclareLaunchArgument(
            'max_lidar_range', default_value='18.0'),
        DeclareLaunchArgument(
            'min_downward_angle_deg', default_value='35.0'),
        DeclareLaunchArgument(
            'max_rgb_time_offset', default_value='0.06'),
        DeclareLaunchArgument(
            'enable_statistical_filter', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_panther',
            default_value='true',
            description='Spawn Panther during the UAV survey.'),
        DeclareLaunchArgument(
            'map_directory',
            default_value=str(Path.home() / 'husarion_ws' / 'maps')),
        DeclareLaunchArgument(
            'sensor_system_delay', default_value='6.0'),
        DeclareLaunchArgument('uav_delay', default_value='16.0'),
        DeclareLaunchArgument('mapping_delay', default_value='28.0'),
        DeclareLaunchArgument('ugv_delay', default_value='8.0'),
        SetEnvironmentVariable('GTK_PATH', ''),
        SetEnvironmentVariable('GIO_MODULE_DIR', ''),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            [
                str(worlds_share / 'worlds'), ':',
                str(worlds_share / 'models'), ':',
                str(orchard_assets), ':',
                px4_dir, '/Tools/simulation/gz/models:',
                EnvironmentVariable(
                    'GZ_SIM_RESOURCE_PATH', default_value=''),
            ]),
        gazebo,
        clock_bridge,
        TimerAction(
            period=LaunchConfiguration('ugv_delay'),
            condition=IfCondition(LaunchConfiguration('use_panther')),
            actions=[panther],
        ),
        TimerAction(
            period=LaunchConfiguration('sensor_system_delay'),
            actions=[sensor_systems],
        ),
        TimerAction(
            period=LaunchConfiguration('uav_delay'),
            actions=[px4],
        ),
        TimerAction(
            period=LaunchConfiguration('mapping_delay'),
            actions=[mapping],
        ),
    ])
