"""Bridge the UAV RGB-D camera and run an isolated RTAB-Map pipeline."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rtabmap_share = Path(get_package_share_directory('rtabmap_launch'))

    rgb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_rtabmap_rgb_bridge',
        arguments=[
            '/downward_camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
        remappings=[
            ('/downward_camera/image', '/uav/rtabmap/rgb/image_raw')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_rtabmap_depth_bridge',
        arguments=[
            '/downward_camera/depth_image'
            '@sensor_msgs/msg/Image[gz.msgs.Image'],
        remappings=[
            ('/downward_camera/depth_image',
             '/uav/rtabmap/depth/image_raw')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_rtabmap_camera_info_bridge',
        arguments=[
            '/downward_camera/camera_info'
            '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
        remappings=[
            ('/downward_camera/camera_info',
             '/uav/rtabmap/rgb/camera_info')],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )
    pose_odometry = Node(
        package='uav_rtabmap_mapping',
        executable='gazebo_pose_odometry.py',
        name='uav_rtabmap_pose_odometry',
        parameters=[{
            'use_sim_time': True,
            'camera_frame': LaunchConfiguration('camera_frame'),
        }],
        output='screen',
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rtabmap_share / 'launch' / 'rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'frame_id': LaunchConfiguration('camera_frame'),
            'vo_frame_id': 'uav_rtabmap_odom',
            'publish_tf_odom': 'true',
            'rgb_topic': '/uav/rtabmap/rgb/image_raw',
            'depth_topic': '/uav/rtabmap/depth/image_raw',
            'camera_info_topic': '/uav/rtabmap/rgb/camera_info',
            'visual_odometry': 'false',
            'odom_topic': '/uav/rtabmap/odom',
            'rgbd_sync': 'true',
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            'approx_sync_max_interval': '0.08',
            'topic_queue_size': '30',
            'sync_queue_size': '30',
            'qos': '2',
            'odom_args': (
                '--Odom/Strategy 0 '
                '--Vis/MinInliers 12 '
                '--Vis/MaxFeatures 2000 '
                '--GFTT/MinDistance 5 '
                '--OdomF2M/MaxSize 3000'
            ),
            'database_path': LaunchConfiguration('database_path'),
            'rtabmap_viz': LaunchConfiguration('use_rtabmap_viz'),
            'rviz': 'false',
            'rtabmap_args': (
                '--delete_db_on_start '
                '--Reg/Strategy 0 '
                '--RGBD/NeighborLinkRefining false '
                '--RGBD/ProximityBySpace false '
                '--Grid/FromDepth true '
                '--Grid/3D true '
                '--Rtabmap/DetectionRate 5 '
                '--Mem/IncrementalMemory true'
            ),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_frame',
            default_value='x500_depth_gps_0/camera_link/downward_camera',
            description='Frame ID reported by the Gazebo RGB-D sensor.'),
        DeclareLaunchArgument(
            'database_path',
            default_value=(
                '/home/robocare/husarion_ws/maps/rtabmap/'
                'uav_orchard.db')),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='true'),
        rgb_bridge,
        depth_bridge,
        info_bridge,
        pose_odometry,
        rtabmap,
    ])
