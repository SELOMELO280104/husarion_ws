from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_frame',
            default_value='world',
            description='ROS/Gazebo world frame used by the Panther TF tree.',
        ),
        DeclareLaunchArgument(
            'ugv_frame',
            default_value='panther/base_link',
            description='Frame of the UGV that the X500 should follow.',
        ),
        DeclareLaunchArgument(
            'ground_truth_tf_topic',
            default_value='/tf_gt',
            description='Optional TFMessage topic with simulator ground truth transforms.',
        ),
        DeclareLaunchArgument(
            'vehicle_status_topic',
            default_value='/fmu/out/vehicle_status_v4',
            description='PX4 vehicle status topic.',
        ),
        DeclareLaunchArgument(
            'vehicle_local_position_topic',
            default_value='/fmu/out/vehicle_local_position_v1',
            description='PX4 local position topic.',
        ),
        DeclareLaunchArgument(
            'vehicle_command_ack_topic',
            default_value='/fmu/out/vehicle_command_ack_v1',
            description='PX4 vehicle command acknowledgement topic.',
        ),
        DeclareLaunchArgument(
            'follow_distance',
            default_value='4.0',
            description='Distance behind the UGV, in meters.',
        ),
        DeclareLaunchArgument(
            'lateral_offset',
            default_value='0.0',
            description='Left/right offset from the UGV path, in meters.',
        ),
        DeclareLaunchArgument(
            'altitude',
            default_value='5.0',
            description='Target altitude above the UGV, in meters.',
        ),
        DeclareLaunchArgument(
            'uav_origin_x',
            default_value='-11.0',
            description='X500 spawn/origin x in the ROS/Gazebo ENU world.',
        ),
        DeclareLaunchArgument(
            'uav_origin_y',
            default_value='5.0',
            description='X500 spawn/origin y in the ROS/Gazebo ENU world.',
        ),
        DeclareLaunchArgument(
            'uav_origin_z',
            default_value='0.2',
            description='X500 spawn/origin z in the ROS/Gazebo ENU world.',
        ),
        DeclareLaunchArgument(
            'auto_arm',
            default_value='True',
            choices=['True', 'true', 'False', 'false'],
            description='Automatically arm PX4 after setpoint streaming starts.',
        ),
        DeclareLaunchArgument(
            'auto_offboard',
            default_value='True',
            choices=['True', 'true', 'False', 'false'],
            description='Automatically request PX4 offboard mode.',
        ),
        DeclareLaunchArgument(
            'takeoff_hold_seconds',
            default_value='3.0',
            description='Seconds to hover at the takeoff altitude before following the UGV.',
        ),
        Node(
            package='uav_ugv_control',
            executable='x500_follow_ugv',
            name='x500_follow_ugv',
            output='screen',
            parameters=[{
                'world_frame': LaunchConfiguration('world_frame'),
                'ugv_frame': LaunchConfiguration('ugv_frame'),
                'ground_truth_tf_topic': LaunchConfiguration('ground_truth_tf_topic'),
                'vehicle_status_topic': LaunchConfiguration('vehicle_status_topic'),
                'vehicle_local_position_topic': LaunchConfiguration(
                    'vehicle_local_position_topic'
                ),
                'vehicle_command_ack_topic': LaunchConfiguration('vehicle_command_ack_topic'),
                'follow_distance': ParameterValue(
                    LaunchConfiguration('follow_distance'), value_type=float
                ),
                'lateral_offset': ParameterValue(
                    LaunchConfiguration('lateral_offset'), value_type=float
                ),
                'altitude': ParameterValue(
                    LaunchConfiguration('altitude'), value_type=float
                ),
                'uav_origin_x': ParameterValue(
                    LaunchConfiguration('uav_origin_x'), value_type=float
                ),
                'uav_origin_y': ParameterValue(
                    LaunchConfiguration('uav_origin_y'), value_type=float
                ),
                'uav_origin_z': ParameterValue(
                    LaunchConfiguration('uav_origin_z'), value_type=float
                ),
                'auto_arm': ParameterValue(
                    LaunchConfiguration('auto_arm'), value_type=bool
                ),
                'auto_offboard': ParameterValue(
                    LaunchConfiguration('auto_offboard'), value_type=bool
                ),
                'takeoff_hold_seconds': ParameterValue(
                    LaunchConfiguration('takeoff_hold_seconds'), value_type=float
                ),
            }],
        ),
    ])
