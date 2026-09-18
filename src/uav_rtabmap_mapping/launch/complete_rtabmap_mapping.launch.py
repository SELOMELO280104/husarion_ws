"""Start only Orchard, PX4 UAV, RGB-D bridges and RTAB-Map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory('uav_rtabmap_mapping'))
    ros_gz_share = Path(get_package_share_directory('ros_gz_sim'))

    world = LaunchConfiguration('world')
    world_file = LaunchConfiguration('world_file')
    px4_dir = LaunchConfiguration('px4_dir')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(ros_gz_share / 'launch' / 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [world_file, ' -r -v 3'],
        }.items(),
    )
    clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='rtabmap_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
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
            'CCACHE_DIR': '/tmp/uav_rtabmap_ccache',
            'PX4_PARAM_SYS_HAS_BARO': '0',
            'PX4_PARAM_EKF2_HGT_REF': '1',
            'PX4_PARAM_SENS_EN_BAROSIM': '1',
            'PX4_PARAM_SENS_EN_MAGSIM': '1',
            'PX4_PARAM_NAV_DLL_ACT': '0',
            'PX4_PARAM_NAV_RCL_ACT': '0',
            'PX4_PARAM_MPC_XY_VEL_MAX': '1.0',
            'PX4_PARAM_MPC_XY_CRUISE': '1.0',
        },
        output='screen',
    )
    dds_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '9999'],
        output='screen',
    )
    prepare_output = ExecuteProcess(
        cmd=['mkdir', '-p', '/home/robocare/husarion_ws/maps/rtabmap'],
        output='screen',
    )
    prepare_world = Node(
        package='uav_rtabmap_mapping',
        executable='resolve_orchard_world.py',
        name='resolve_rtabmap_orchard_world',
        output='screen',
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / 'launch' / 'rtabmap_rgbd.launch.py')),
        launch_arguments={
            'database_path':
                '/home/robocare/husarion_ws/maps/rtabmap/uav_orchard.db',
            'use_rtabmap_viz': LaunchConfiguration('use_rtabmap_viz'),
        }.items(),
    )

    # When this launch is started from the Snap build of VS Code, Snap injects
    # GTK/locale paths from core20.  Gazebo and rtabmap_viz are host binaries
    # and loading core20's libpthread makes both GUI processes abort.
    snap_gui_variables = [
        'GTK_EXE_PREFIX',
        'GTK_PATH',
        'GTK_MODULES',
        'GTK_IM_MODULE_FILE',
        'GDK_PIXBUF_MODULE_FILE',
        'GDK_PIXBUF_MODULEDIR',
        'GSETTINGS_SCHEMA_DIR',
        'GIO_MODULE_DIR',
        'LOCPATH',
        'SNAP_LIBRARY_PATH',
    ]

    return LaunchDescription([
        *[UnsetEnvironmentVariable(name) for name in snap_gui_variables],
        SetEnvironmentVariable(
            'XDG_DATA_HOME', '/home/robocare/.local/share'),
        SetEnvironmentVariable(
            'XDG_DATA_DIRS',
            '/usr/local/share:/usr/share:/var/lib/snapd/desktop'),
        DeclareLaunchArgument('world', default_value='orchard'),
        DeclareLaunchArgument(
            'world_file',
            default_value='/tmp/uav_rtabmap_orchard.sdf'),
        DeclareLaunchArgument(
            'px4_dir', default_value='/home/robocare/PX4-Autopilot'),
        DeclareLaunchArgument(
            'uav_pose', default_value='0,-5,0.2,0,0,0'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='true'),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/home/robocare/husarion_ws/src/husarion_gz_worlds:'
            '/home/robocare/PX4-Autopilot/Tools/simulation/gz/models'),
        prepare_output,
        prepare_world,
        TimerAction(period=1.0, actions=[gazebo, clock]),
        TimerAction(period=2.0, actions=[dds_agent]),
        TimerAction(period=10.0, actions=[px4]),
        TimerAction(period=18.0, actions=[mapping]),
    ])
