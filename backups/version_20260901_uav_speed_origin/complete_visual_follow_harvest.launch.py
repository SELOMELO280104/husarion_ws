"""Run Panther first, then search and follow it with a GPS-denied UAV."""

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
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the complete coordinated launch graph."""
    share = Path(
        get_package_share_directory('uav_groundtruth_mapping'))
    worlds_share = Path(
        get_package_share_directory('husarion_gz_worlds'))
    orchard_assets = (
        Path.home() / 'uav_ugv_ros2_jazzy'
        / 'external' / 'cpr_gazebo')
    world = LaunchConfiguration('world')
    px4_dir = LaunchConfiguration('px4_dir')
    gz_partition = LaunchConfiguration('gz_partition')
    gazebo_backend = IfCondition(PythonExpression([
        "'", LaunchConfiguration('control_backend'),
        "' == 'gazebo_velocity'",
    ]))
    px4_backend = IfCondition(PythonExpression([
        "'", LaunchConfiguration('control_backend'),
        "' == 'px4_acceleration'",
    ]))

    panther_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                share / 'launch'
                / 'complete_gps_denied_navigation.launch.py')),
        launch_arguments={
            'world': world,
            'gz_partition': gz_partition,
            'rmw_implementation':
                LaunchConfiguration('rmw_implementation'),
            'map': LaunchConfiguration('map'),
            'corridors_csv': LaunchConfiguration('corridors_csv'),
            'spawn_x': LaunchConfiguration('panther_x'),
            'spawn_y': LaunchConfiguration('panther_y'),
            'spawn_yaw': LaunchConfiguration('panther_yaw'),
            'initial_estop': LaunchConfiguration('initial_estop'),
            'waypoint_spacing': LaunchConfiguration('waypoint_spacing'),
            'headland_clearance':
                LaunchConfiguration('headland_clearance'),
            'autostart_mission':
                LaunchConfiguration('autostart_mission'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            # The Panther deliberately gets a head start. It must not be
            # gated by a tag lock that the UAV has not searched for yet.
            'required_readiness_topic': '',
            'required_readiness_label': 'independent UAV search',
        }.items(),
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
                '{filename: "libOpticalFlowSystem.so", '
                'name: "custom::OpticalFlowSystem"}]'
            ),
        ],
        condition=px4_backend,
        output='screen',
    )
    dds_agent = ExecuteProcess(
        cmd=[
            LaunchConfiguration('uxrce_agent'), 'udp4',
            '-p', LaunchConfiguration('uxrce_port'),
        ],
        condition=px4_backend,
        output='screen',
    )
    px4 = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4',
            ]),
        ],
        cwd=PathJoinSubstitution([
            px4_dir, 'build', 'px4_sitl_default',
            'src', 'modules', 'simulation', 'gz_bridge',
        ]),
        additional_env={
            'PX4_SIM_MODEL': 'gz_x500_flow_tag',
            'PX4_GZ_STANDALONE': '1',
            'PX4_GZ_WORLD': world,
            'PX4_GZ_MODEL_POSE': LaunchConfiguration('uav_pose'),
            'PX4_GZ_NO_FOLLOW': '1',
            'PX4_UXRCE_DDS_PORT': LaunchConfiguration('uxrce_port'),
            # Ignore any stale value restored from parameters.bson.  Every ROS
            # node in this launch uses ROS_DOMAIN_ID=0, so PX4 must create its
            # DDS participant in domain 0 as well.
            'PX4_PARAM_UXRCE_DDS_DOM_ID': '0',
            'CCACHE_DIR': '/tmp/uav_visual_follow_ccache',
            'PX4_PARAM_SYS_HAS_GPS': '0',
            'PX4_PARAM_SIM_GPS_USED': '0',
            'PX4_PARAM_EKF2_GPS_CTRL': '0',
            'PX4_PARAM_EKF2_OF_CTRL': '1',
            'PX4_PARAM_SYS_HAS_MAG': '0',
            'PX4_PARAM_SENS_EN_MAGSIM': '0',
            'PX4_PARAM_EKF2_MAG_TYPE': '5',
            'PX4_PARAM_COM_ARM_WO_GPS': '1',
            'PX4_PARAM_CBRK_SUPPLY_CHK': '894281',
            'PX4_PARAM_NAV_DLL_ACT': '0',
            'PX4_PARAM_NAV_RCL_ACT': '0',
            'PX4_PARAM_MPC_XY_VEL_MAX': '1.2',
            'PX4_PARAM_MPC_Z_VEL_MAX_UP': '1.0',
            'PX4_PARAM_MPC_Z_VEL_MAX_DN': '0.8',
        },
        condition=px4_backend,
        output='screen',
    )
    gazebo_uav = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_gps_denied_visual_uav',
        condition=gazebo_backend,
        arguments=[
            '-world', world,
            '-file', PathJoinSubstitution([
                px4_dir, 'Tools', 'simulation', 'gz', 'models',
                'x500_flow_tag', 'model.sdf',
            ]),
            '-name', 'x500_flow_tag_0',
            '-allow_renaming', 'false',
            '-x', LaunchConfiguration('uav_x'),
            '-y', LaunchConfiguration('uav_y'),
            '-z', LaunchConfiguration('uav_z'),
            '-Y', LaunchConfiguration('uav_yaw'),
        ],
        output='screen',
    )
    rgb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_tag_rgb_bridge',
        arguments=[
            '/uav/tag_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_tag_depth_bridge',
        arguments=[
            (
                '/uav/tag_camera/depth_image@sensor_msgs/msg/Image'
                '[gz.msgs.Image'
            ),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_tag_camera_info_bridge',
        arguments=[
            (
                '/uav/tag_camera/camera_info@sensor_msgs/msg/CameraInfo'
                '[gz.msgs.CameraInfo'
            ),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    velocity_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_visual_velocity_bridge',
        condition=gazebo_backend,
        arguments=[
            (
                '/model/x500_flow_tag_0/cmd_vel'
                '@geometry_msgs/msg/Twist]gz.msgs.Twist'
            ),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    follower = Node(
        package='uav_ugv_control',
        executable='visual_tag_follower',
        name='uav_visual_tag_follower',
        parameters=[{
            'use_sim_time': True,
            'marker_id': ParameterValue(
                LaunchConfiguration('marker_id'), value_type=int),
            'desired_depth': ParameterValue(
                LaunchConfiguration('follow_height'), value_type=float),
            'takeoff_altitude': ParameterValue(
                LaunchConfiguration('follow_height'), value_type=float),
            'max_horizontal_speed': ParameterValue(
                LaunchConfiguration('uav_max_speed'), value_type=float),
            'velocity_feedforward_gain': ParameterValue(
                LaunchConfiguration('velocity_feedforward_gain'),
                value_type=float),
            'target_velocity_filter_alpha': ParameterValue(
                LaunchConfiguration('target_velocity_filter_alpha'),
                value_type=float),
            'max_estimated_target_speed': ParameterValue(
                LaunchConfiguration('max_estimated_target_speed'),
                value_type=float),
            'marker_timeout': 3.0,
            'ground_depth_timeout': 5.0,
            'search_start_delay': ParameterValue(
                LaunchConfiguration('uav_search_delay'), value_type=float),
            'manual_start_required': ParameterValue(
                LaunchConfiguration('manual_uav_start'), value_type=bool),
            'search_origin_x': ParameterValue(
                LaunchConfiguration('uav_x'), value_type=float),
            'search_origin_y': ParameterValue(
                LaunchConfiguration('uav_y'), value_type=float),
            'ugv_origin_x': ParameterValue(
                LaunchConfiguration('panther_x'), value_type=float),
            'ugv_origin_y': ParameterValue(
                LaunchConfiguration('panther_y'), value_type=float),
            'ugv_nominal_speed': ParameterValue(
                LaunchConfiguration('ugv_nominal_speed'), value_type=float),
            'route_sweep_distance': ParameterValue(
                LaunchConfiguration('route_sweep_distance'),
                value_type=float),
            'route_sweep_period': ParameterValue(
                LaunchConfiguration('route_sweep_period'), value_type=float),
            'local_reacquire_timeout': ParameterValue(
                LaunchConfiguration('local_reacquire_timeout'),
                value_type=float),
            'fallback_search_spacing': ParameterValue(
                LaunchConfiguration('fallback_search_spacing'),
                value_type=float),
            'control_backend': LaunchConfiguration('control_backend'),
        }],
        output='screen',
    )
    camera_viewer = Node(
        package='uav_ugv_control',
        executable='uav_camera_viewer',
        name='uav_marker_camera_viewer',
        condition=IfCondition(LaunchConfiguration('show_uav_camera')),
        parameters=[{
            'image_topic': '/uav/tag_follower/debug_image',
            'status_topic': '/uav/tag_follower/status',
            'start_service': '/uav/tag_follower/start_locating',
            'always_on_top': True,
        }],
        output='screen',
    )

    maps = Path.home() / 'husarion_ws' / 'maps'
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='orchard'),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=str(Path.home() / 'PX4-Autopilot')),
        DeclareLaunchArgument(
            'gz_partition',
            default_value=f'coordinated_harvest_{os.getpid()}'),
        DeclareLaunchArgument(
            'rmw_implementation',
            default_value='rmw_cyclonedds_cpp'),
        DeclareLaunchArgument(
            'uxrce_agent',
            default_value=str(
                Path.home()
                / 'Micro-XRCE-DDS-Agent-2.4.3'
                / 'build_system'
                / 'MicroXRCEAgent'),
            description=(
                'Micro XRCE-DDS Agent 2.4.3, matching the PX4 v2.x '
                'client and ROS 2 Jazzy Fast DDS ABI.')),
        DeclareLaunchArgument('uxrce_port', default_value='9999'),
        DeclareLaunchArgument(
            'control_backend',
            default_value='gazebo_velocity',
            description=(
                'gazebo_velocity is the validated GPS-free RGB-D visual '
                'servo. px4_acceleration keeps the experimental PX4 '
                'acceleration backend available for later estimator work.')),
        DeclareLaunchArgument(
            'map',
            default_value=str(
                maps / 'latest_panther_row_corridors_nav.yaml')),
        DeclareLaunchArgument(
            'corridors_csv',
            default_value=str(
                maps
                / 'latest_panther_row_corridors_centerlines.csv')),
        DeclareLaunchArgument('panther_x', default_value='-6.0'),
        DeclareLaunchArgument('panther_y', default_value='-8.0'),
        DeclareLaunchArgument('panther_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'uav_pose',
            default_value='-6,-9,0.8,0,0,0',
            description=(
                'Ground spawn one metre behind Panther, clear of its '
                'collision footprint; optical-flow damping holds the UAV '
                'until the roof tag enters the camera frame during climb.')),
        DeclareLaunchArgument('uav_x', default_value='-6.0'),
        DeclareLaunchArgument('uav_y', default_value='-9.0'),
        DeclareLaunchArgument('uav_z', default_value='0.8'),
        DeclareLaunchArgument('uav_yaw', default_value='0.0'),
        DeclareLaunchArgument('follow_height', default_value='10.0'),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('uav_max_speed', default_value='1.0'),
        DeclareLaunchArgument(
            'velocity_feedforward_gain',
            default_value='0.75',
            description=(
                'Feed-forward fraction of the camera-relative Panther '
                'velocity estimate used by the UAV tracker.')),
        DeclareLaunchArgument(
            'target_velocity_filter_alpha',
            default_value='0.20',
            description=(
                'Low-pass update weight for the Panther velocity estimate.')),
        DeclareLaunchArgument(
            'max_estimated_target_speed',
            default_value='0.80',
            description=(
                'Maximum accepted Panther speed estimate in m/s.')),
        DeclareLaunchArgument(
            'uav_search_delay',
            default_value='30.0',
            description=(
                'Head start measured from the Panther RUNNING state. The UAV '
                'holds at launch until this time has elapsed.')),
        DeclareLaunchArgument(
            'manual_uav_start',
            default_value='true',
            description=(
                'Require the Start Locating button in the UAV camera window '
                'before the UAV may search or follow.')),
        DeclareLaunchArgument(
            'ugv_nominal_speed',
            default_value='0.32',
            description=(
                'Conservative Panther progress estimate used only to '
                'prioritize the camera search along the planned route.')),
        DeclareLaunchArgument(
            'route_sweep_distance',
            default_value='4.0',
            description=(
                'Longitudinal uncertainty searched behind and ahead of the '
                'predicted Panther route position, in metres.')),
        DeclareLaunchArgument(
            'route_sweep_period', default_value='12.0'),
        DeclareLaunchArgument(
            'local_reacquire_timeout',
            default_value='8.0',
            description=(
                'Search around the last visual sighting for this many '
                'seconds before returning to route interception.')),
        DeclareLaunchArgument(
            'fallback_search_spacing',
            default_value='3.0',
            description=(
                'Expanding-square spacing when no route prior is available.')),
        DeclareLaunchArgument('waypoint_spacing', default_value='1.0'),
        DeclareLaunchArgument(
            'headland_clearance', default_value='2.25'),
        DeclareLaunchArgument(
            'initial_estop',
            default_value='false',
            description=(
                'Set true when an operator must explicitly release E-stop.')),
        DeclareLaunchArgument(
            'autostart_mission',
            default_value='true',
            description=(
                'The route starts only after Panther/Nav2 and the UAV visual '
                'lock are all ready.')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'show_uav_camera',
            default_value='true',
            description=(
                'Open a live UAV camera window containing the marker box, '
                'decoded ID, confidence, centre and depth overlay.')),
        DeclareLaunchArgument(
            'sensor_system_delay', default_value='4.0'),
        DeclareLaunchArgument('dds_agent_delay', default_value='6.0'),
        DeclareLaunchArgument('px4_delay', default_value='10.0'),
        DeclareLaunchArgument(
            'camera_bridge_delay',
            default_value='9.0',
            description=(
                'Start camera and velocity bridges before UAV insertion so '
                'its velocity controller receives commands immediately.')),
        DeclareLaunchArgument(
            'follower_delay',
            default_value='9.2',
            description=(
                'Start the visual follower before UAV insertion; it publishes '
                'a bounded bootstrap command until depth becomes available.')),
        DeclareLaunchArgument(
            'camera_view_delay',
            default_value='2.0',
            description=(
                'Open the dedicated camera window early. It displays a '
                'waiting screen until annotated frames begin publishing.')),
        SetEnvironmentVariable('GTK_PATH', ''),
        SetEnvironmentVariable('GIO_MODULE_DIR', ''),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            LaunchConfiguration('rmw_implementation')),
        SetEnvironmentVariable('GZ_PARTITION', gz_partition),
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
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            [
                px4_dir,
                '/build/px4_sitl_default/src/modules/simulation/'
                'gz_plugins:',
                EnvironmentVariable(
                    'GZ_SIM_SYSTEM_PLUGIN_PATH', default_value=''),
            ]),
        panther_stack,
        TimerAction(
            period=LaunchConfiguration('sensor_system_delay'),
            actions=[sensor_systems],
        ),
        TimerAction(
            period=LaunchConfiguration('dds_agent_delay'),
            actions=[dds_agent],
        ),
        TimerAction(
            period=LaunchConfiguration('px4_delay'),
            actions=[px4, gazebo_uav],
        ),
        TimerAction(
            period=LaunchConfiguration('camera_bridge_delay'),
            actions=[
                rgb_bridge,
                depth_bridge,
                camera_info_bridge,
                velocity_bridge,
            ],
        ),
        TimerAction(
            period=LaunchConfiguration('follower_delay'),
            actions=[follower],
        ),
        TimerAction(
            period=LaunchConfiguration('camera_view_delay'),
            actions=[camera_viewer],
        ),
    ])
