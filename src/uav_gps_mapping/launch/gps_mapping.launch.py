"""Bridge the UAV LiDAR and run the PX4 GPS/INS mapper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    gz_topic = LaunchConfiguration('gz_lidar_topic').perform(context)
    ros_topic = LaunchConfiguration('ros_lidar_topic').perform(context)
    gz_rgb_topic = LaunchConfiguration('gz_rgb_topic').perform(context)
    ros_rgb_topic = LaunchConfiguration('ros_rgb_topic').perform(context)
    gz_info_topic = LaunchConfiguration('gz_camera_info_topic').perform(
        context)
    ros_info_topic = LaunchConfiguration('ros_camera_info_topic').perform(
        context)
    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gps_mapper_lidar_bridge',
            arguments=[
                f'{gz_topic}@sensor_msgs/msg/PointCloud2'
                '[gz.msgs.PointCloudPacked',
                '--ros-args', '-r', f'{gz_topic}:={ros_topic}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gps_mapper_rgb_bridge',
            arguments=[
                f'{gz_rgb_topic}@sensor_msgs/msg/Image[gz.msgs.Image',
                '--ros-args', '-r', f'{gz_rgb_topic}:={ros_rgb_topic}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gps_mapper_camera_info_bridge',
            arguments=[
                f'{gz_info_topic}@sensor_msgs/msg/CameraInfo'
                '[gz.msgs.CameraInfo',
                '--ros-args', '-r', f'{gz_info_topic}:={ros_info_topic}',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='uav_gps_mapping',
            executable='gps_pointcloud_mapper',
            name='gps_pointcloud_mapper',
            parameters=[{
                'use_sim_time': True,
                'cloud_topic': ros_topic,
                'rgb_topic': ros_rgb_topic,
                'camera_info_topic': ros_info_topic,
                'output_directory':
                    LaunchConfiguration('map_directory'),
                'origin_x': ParameterValue(
                    LaunchConfiguration('origin_x'), value_type=float),
                'origin_y': ParameterValue(
                    LaunchConfiguration('origin_y'), value_type=float),
                'origin_z': ParameterValue(
                    LaunchConfiguration('origin_z'), value_type=float),
                'voxel_size': ParameterValue(
                    LaunchConfiguration('voxel_size'), value_type=float),
                'scan_delay': ParameterValue(
                    LaunchConfiguration('scan_delay'), value_type=float),
                'max_lidar_range': ParameterValue(
                    LaunchConfiguration('max_lidar_range'),
                    value_type=float),
                'min_downward_angle_deg': ParameterValue(
                    LaunchConfiguration('min_downward_angle_deg'),
                    value_type=float),
                'max_rgb_time_offset': ParameterValue(
                    LaunchConfiguration('max_rgb_time_offset'),
                    value_type=float),
                'enable_icp': ParameterValue(
                    LaunchConfiguration('enable_icp'),
                    value_type=bool),
                'icp_translation_only': ParameterValue(
                    LaunchConfiguration('icp_translation_only'),
                    value_type=bool),
                'icp_max_correspondence_distance': ParameterValue(
                    LaunchConfiguration(
                        'icp_max_correspondence_distance'),
                    value_type=float),
                'icp_max_translation': ParameterValue(
                    LaunchConfiguration('icp_max_translation'),
                    value_type=float),
                'icp_max_rotation_deg': ParameterValue(
                    LaunchConfiguration('icp_max_rotation_deg'),
                    value_type=float),
                'icp_min_height': ParameterValue(
                    LaunchConfiguration('icp_min_height'),
                    value_type=float),
                'icp_max_height': ParameterValue(
                    LaunchConfiguration('icp_max_height'),
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
            name='gps_world_frame',
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
            name='gps_mapping_rviz',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('uav_gps_mapping'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'gz_lidar_topic',
            default_value='/uav/gps_mapping/lidar/points'),
        DeclareLaunchArgument(
            'ros_lidar_topic',
            default_value='/uav/gps_mapping/lidar/points'),
        DeclareLaunchArgument(
            'gz_rgb_topic', default_value='/downward_camera/image'),
        DeclareLaunchArgument(
            'ros_rgb_topic',
            default_value='/uav/gps_mapping/rgb/image'),
        DeclareLaunchArgument(
            'gz_camera_info_topic',
            default_value='/downward_camera/camera_info'),
        DeclareLaunchArgument(
            'ros_camera_info_topic',
            default_value='/uav/gps_mapping/rgb/camera_info'),
        DeclareLaunchArgument(
            'map_directory',
            default_value=str(Path.home() / 'husarion_ws' / 'maps')),
        DeclareLaunchArgument('origin_x', default_value='0.0'),
        DeclareLaunchArgument('origin_y', default_value='-5.0'),
        DeclareLaunchArgument('origin_z', default_value='0.2'),
        DeclareLaunchArgument('voxel_size', default_value='0.10'),
        DeclareLaunchArgument('scan_delay', default_value='0.15'),
        DeclareLaunchArgument('max_lidar_range', default_value='18.0'),
        DeclareLaunchArgument(
            'min_downward_angle_deg', default_value='35.0'),
        DeclareLaunchArgument(
            'max_rgb_time_offset', default_value='0.06'),
        DeclareLaunchArgument('enable_icp', default_value='true'),
        DeclareLaunchArgument(
            'icp_translation_only', default_value='true'),
        DeclareLaunchArgument(
            'icp_max_correspondence_distance',
            default_value='0.35'),
        DeclareLaunchArgument(
            'icp_max_translation', default_value='0.20'),
        DeclareLaunchArgument(
            'icp_max_rotation_deg', default_value='0.50'),
        DeclareLaunchArgument(
            'icp_min_height', default_value='0.35'),
        DeclareLaunchArgument(
            'icp_max_height', default_value='1.30'),
        DeclareLaunchArgument(
            'enable_statistical_filter', default_value='true'),
        DeclareLaunchArgument(
            'outlier_mean_k', default_value='12'),
        DeclareLaunchArgument(
            'outlier_stddev_multiplier', default_value='1.5'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=str(share / 'rviz' / 'gps_mapping.rviz')),
        OpaqueFunction(function=_nodes),
    ])
