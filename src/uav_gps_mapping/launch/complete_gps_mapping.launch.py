"""Start orchard GUI, Panther, PX4 UAV, GPS mapper, and RViz in sequence."""

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
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('uav_gps_mapping'))
    worlds_share = Path(
        get_package_share_directory('husarion_gz_worlds'))
    orchard_assets = (
        Path.home() / 'uav_ugv_ros2_jazzy'
        / 'external' / 'cpr_gazebo')
    husarion_share = Path(
        get_package_share_directory('husarion_ugv_gazebo'))
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
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )
    panther = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(husarion_share / 'launch' / 'simulate_robot.launch.py')),
        launch_arguments={
            'namespace': 'panther',
            'robot_model': 'panther',
            'x': '-6.0', 'y': '-8.0', 'z': '0.2', 'yaw': '0.0',
            # Nav2 AMCL owns map -> panther/odom. Publishing a second static
            # world -> panther/odom transform would create a TF conflict.
            'add_world_transform': 'false',
            'disable_manager': PythonExpression([
                "'", LaunchConfiguration('disable_ugv_manager'),
                "'.lower() == 'true'",
            ]),
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
            'CCACHE_DIR': '/tmp/uav_gps_mapping_ccache',
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
            str(share / 'launch' / 'gps_mapping.launch.py')),
        launch_arguments={
            # The Nav2 RViz instance also displays the map and robot; avoid a
            # second RViz window from the GPS mapper.
            'use_rviz': 'false',
            'map_directory': LaunchConfiguration('map_directory'),
            'origin_x': '0.0',
            'origin_y': '-5.0',
            'origin_z': '0.2',
            'voxel_size': LaunchConfiguration('voxel_size'),
            'scan_delay': LaunchConfiguration('scan_delay'),
            'max_lidar_range': LaunchConfiguration('max_lidar_range'),
            'min_downward_angle_deg':
                LaunchConfiguration('min_downward_angle_deg'),
            'max_rgb_time_offset':
                LaunchConfiguration('max_rgb_time_offset'),
            'enable_icp': LaunchConfiguration('enable_icp'),
            'icp_translation_only':
                LaunchConfiguration('icp_translation_only'),
            'icp_max_correspondence_distance':
                LaunchConfiguration(
                    'icp_max_correspondence_distance'),
            'icp_max_translation':
                LaunchConfiguration('icp_max_translation'),
            'icp_max_rotation_deg':
                LaunchConfiguration('icp_max_rotation_deg'),
            'icp_min_height':
                LaunchConfiguration('icp_min_height'),
            'icp_max_height':
                LaunchConfiguration('icp_max_height'),
            'enable_statistical_filter':
                LaunchConfiguration('enable_statistical_filter'),
            'outlier_mean_k':
                LaunchConfiguration('outlier_mean_k'),
            'outlier_stddev_multiplier':
                LaunchConfiguration('outlier_stddev_multiplier'),
            'gz_rgb_topic': '/downward_camera/image',
            'ros_rgb_topic': '/uav/gps_mapping/rgb/image',
            'gz_camera_info_topic': '/downward_camera/camera_info',
            'ros_camera_info_topic':
                '/uav/gps_mapping/rgb/camera_info',
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(share / 'launch' / 'panther_nav_from_latest.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'map_directory': LaunchConfiguration('map_directory'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='orchard'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=str(Path.home() / 'PX4-Autopilot')),
        DeclareLaunchArgument(
            'uav_pose', default_value='0,-5,0.2,0,0,0'),
        DeclareLaunchArgument('survey_speed', default_value='1.5'),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.10',
            description='3D downsampling cell size in metres.'),
        DeclareLaunchArgument(
            'scan_delay',
            default_value='0.15',
            description='Seconds to buffer a scan for pose interpolation.'),
        DeclareLaunchArgument(
            'max_lidar_range',
            default_value='18.0',
            description='Reject longer simulated LiDAR returns.'),
        DeclareLaunchArgument(
            'min_downward_angle_deg',
            default_value='35.0',
            description='Reject shallow rays near the world boundary.'),
        DeclareLaunchArgument(
            'max_rgb_time_offset',
            default_value='0.06',
            description='Maximum LiDAR-to-RGB timestamp offset, seconds.'),
        DeclareLaunchArgument(
            'enable_icp',
            default_value='true',
            description='Refine GPS-seeded scans against the local map.'),
        DeclareLaunchArgument(
            'icp_translation_only',
            default_value='true',
            description='Trust PX4 attitude; let ICP correct position only.'),
        DeclareLaunchArgument(
            'icp_max_correspondence_distance',
            default_value='0.35',
            description='Maximum ICP match distance in metres.'),
        DeclareLaunchArgument(
            'icp_max_translation',
            default_value='0.20',
            description='Maximum correction away from PX4 GPS, metres.'),
        DeclareLaunchArgument(
            'icp_max_rotation_deg',
            default_value='0.50',
            description='Maximum correction away from PX4 attitude.'),
        DeclareLaunchArgument(
            'icp_min_height',
            default_value='0.35',
            description='Lowest rigid structure used by ICP, metres.'),
        DeclareLaunchArgument(
            'icp_max_height',
            default_value='1.30',
            description='Highest rigid structure used by ICP, metres.'),
        DeclareLaunchArgument(
            'enable_statistical_filter',
            default_value='true',
            description='Write a second PCD with isolated points removed.'),
        DeclareLaunchArgument(
            'outlier_mean_k',
            default_value='12',
            description='Neighbour count used by the outlier filter.'),
        DeclareLaunchArgument(
            'outlier_stddev_multiplier',
            default_value='1.5',
            description='Outlier rejection threshold in standard deviations.'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_panther',
            default_value='true',
            description='Spawn Panther and start its Nav2 stack.'),
        DeclareLaunchArgument(
            'disable_ugv_manager', default_value='false'),
        DeclareLaunchArgument(
            'map_directory',
            default_value=str(Path.home() / 'husarion_ws' / 'maps')),
        DeclareLaunchArgument('ugv_delay', default_value='8.0'),
        DeclareLaunchArgument(
            'sensor_system_delay', default_value='6.0'),
        DeclareLaunchArgument('uav_delay', default_value='16.0'),
        DeclareLaunchArgument('mapping_delay', default_value='28.0'),
        DeclareLaunchArgument('navigation_delay', default_value='18.0'),
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
            actions=[panther]),
        TimerAction(
            period=LaunchConfiguration('sensor_system_delay'),
            actions=[sensor_systems]),
        TimerAction(
            period=LaunchConfiguration('uav_delay'),
            actions=[px4]),
        TimerAction(
            period=LaunchConfiguration('mapping_delay'),
            actions=[mapping]),
        TimerAction(
            period=LaunchConfiguration('navigation_delay'),
            condition=IfCondition(LaunchConfiguration('use_panther')),
            actions=[navigation]),
    ])
