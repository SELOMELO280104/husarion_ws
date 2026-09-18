"""Bridge Gazebo sensors and run the exact-pose RGB LiDAR mapper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    gz_lidar = LaunchConfiguration('gz_lidar_topic').perform(context)
    ros_lidar = LaunchConfiguration('ros_lidar_topic').perform(context)
    gz_pose = LaunchConfiguration('gz_pose_topic').perform(context)
    ros_pose = LaunchConfiguration('ros_pose_topic').perform(context)
    gz_rgb = LaunchConfiguration('gz_rgb_topic').perform(context)
    ros_rgb = LaunchConfiguration('ros_rgb_topic').perform(context)
    gz_info = LaunchConfiguration('gz_camera_info_topic').perform(context)
    ros_info = LaunchConfiguration('ros_camera_info_topic').perform(context)
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='groundtruth_lidar_bridge',
            arguments=[
                f'{gz_lidar}@sensor_msgs/msg/PointCloud2'
                '[gz.msgs.PointCloudPacked',
                '--ros-args', '-r', f'{gz_lidar}:={ros_lidar}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='groundtruth_pose_bridge',
            arguments=[
                f'{gz_pose}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                '--ros-args', '-r', f'{gz_pose}:={ros_pose}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='groundtruth_rgb_bridge',
            arguments=[
                f'{gz_rgb}@sensor_msgs/msg/Image[gz.msgs.Image',
                '--ros-args', '-r', f'{gz_rgb}:={ros_rgb}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='groundtruth_camera_info_bridge',
            arguments=[
                f'{gz_info}@sensor_msgs/msg/CameraInfo'
                '[gz.msgs.CameraInfo',
                '--ros-args', '-r', f'{gz_info}:={ros_info}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='uav_groundtruth_mapping',
            executable='groundtruth_pointcloud_mapper',
            parameters=[{
                'use_sim_time': True,
                'cloud_topic': ros_lidar,
                'pose_topic': ros_pose,
                'rgb_topic': ros_rgb,
                'camera_info_topic': ros_info,
                'output_directory':
                    LaunchConfiguration('map_directory'),
                'voxel_size': ParameterValue(
                    LaunchConfiguration('voxel_size'),
                    value_type=float),
                'scan_delay': ParameterValue(
                    LaunchConfiguration('scan_delay'),
                    value_type=float),
                'max_lidar_range': ParameterValue(
                    LaunchConfiguration('max_lidar_range'),
                    value_type=float),
                'min_downward_angle_deg': ParameterValue(
                    LaunchConfiguration('min_downward_angle_deg'),
                    value_type=float),
                'max_rgb_time_offset': ParameterValue(
                    LaunchConfiguration('max_rgb_time_offset'),
                    value_type=float),
                'enable_statistical_filter': ParameterValue(
                    LaunchConfiguration('enable_statistical_filter'),
                    value_type=bool),
                'outlier_mean_k': ParameterValue(
                    LaunchConfiguration('outlier_mean_k'),
                    value_type=int),
                'outlier_stddev_multiplier': ParameterValue(
                    LaunchConfiguration('outlier_stddev_multiplier'),
                    value_type=float),
            }],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='groundtruth_world_frame',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'world', '--child-frame-id', 'map',
            ],
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='groundtruth_mapping_rviz',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]


def generate_launch_description():
    share = Path(
        get_package_share_directory('uav_groundtruth_mapping'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_lidar_topic',
            default_value='/uav/gps_mapping/lidar/points'),
        DeclareLaunchArgument(
            'ros_lidar_topic',
            default_value='/uav/groundtruth/lidar/points'),
        DeclareLaunchArgument(
            'gz_pose_topic',
            default_value='/model/x500_depth_gps_0/pose',
            description=(
                'Gazebo PosePublisher topic for the PX4 spawned model.')),
        DeclareLaunchArgument(
            'ros_pose_topic',
            default_value='/uav/groundtruth/pose'),
        DeclareLaunchArgument(
            'gz_rgb_topic',
            default_value='/downward_camera/image'),
        DeclareLaunchArgument(
            'ros_rgb_topic',
            default_value='/uav/groundtruth/rgb/image'),
        DeclareLaunchArgument(
            'gz_camera_info_topic',
            default_value='/downward_camera/camera_info'),
        DeclareLaunchArgument(
            'ros_camera_info_topic',
            default_value='/uav/groundtruth/rgb/camera_info'),
        DeclareLaunchArgument(
            'map_directory',
            default_value=str(Path.home() / 'husarion_ws' / 'maps')),
        DeclareLaunchArgument('voxel_size', default_value='0.10'),
        DeclareLaunchArgument('scan_delay', default_value='0.10'),
        DeclareLaunchArgument('max_lidar_range', default_value='18.0'),
        DeclareLaunchArgument(
            'min_downward_angle_deg', default_value='35.0'),
        DeclareLaunchArgument(
            'max_rgb_time_offset', default_value='0.06'),
        DeclareLaunchArgument(
            'enable_statistical_filter', default_value='true'),
        DeclareLaunchArgument('outlier_mean_k', default_value='12'),
        DeclareLaunchArgument(
            'outlier_stddev_multiplier', default_value='1.5'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(
                share / 'rviz' / 'groundtruth_mapping.rviz')),
        OpaqueFunction(function=_nodes),
    ])
