#!/usr/bin/env python3
"""Publish stable ENU camera odometry from PX4 VehicleOdometry."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster


def quaternion_to_matrix(w, x, y, z):
    return [
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ]


def multiply(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def matrix_to_quaternion(m):
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return (0.25*s, (m[2][1]-m[1][2])/s,
                (m[0][2]-m[2][0])/s, (m[1][0]-m[0][1])/s)
    i = max(range(3), key=lambda n: m[n][n])
    if i == 0:
        s = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        return ((m[2][1]-m[1][2])/s, 0.25*s,
                (m[0][1]+m[1][0])/s, (m[0][2]+m[2][0])/s)
    if i == 1:
        s = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][2]-m[2][0])/s, (m[0][1]+m[1][0])/s,
                0.25*s, (m[1][2]+m[2][1])/s)
    s = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[1][0]-m[0][1])/s, (m[0][2]+m[2][0])/s,
            (m[1][2]+m[2][1])/s, 0.25*s)


class Px4PoseOdometry(Node):
    def __init__(self):
        super().__init__('uav_rtabmap_pose_odometry')
        self.declare_parameter(
            'camera_frame',
            'x500_depth_gps_0/camera_link/downward_camera')
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publisher = self.create_publisher(
            Odometry, '/uav/rtabmap/odom', qos_profile_sensor_data)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.callback, qos_profile_sensor_data)
        self.status_subscription = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.status_callback, qos_profile_sensor_data)
        self.mission_active = False
        self.origin_ned = None
        self.published_once = False

    def status_callback(self, message):
        active = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION)
        if active and not self.mission_active:
            self.origin_ned = None
            self.published_once = False
            self.get_logger().info(
                'QGroundControl AUTO_MISSION started: RTAB odometry enabled.')
        elif not active and self.mission_active:
            self.get_logger().info(
                'AUTO_MISSION ended: RTAB odometry paused.')
        self.mission_active = active

    def callback(self, message):
        # Do not let PX4's normal preflight EKF initialization and resets create
        # a false trajectory or map nodes while the vehicle is on the ground.
        if not self.mission_active:
            return
        if not all(math.isfinite(v) for v in (*message.position, *message.q)):
            return
        if self.origin_ned is None:
            self.origin_ned = tuple(float(v) for v in message.position)

        # PX4: NED world, FRD body. ROS map: ENU world, FLU body.
        r_ned_frd = quaternion_to_matrix(*message.q)
        ned_to_enu = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
        flu_to_frd = [[1, 0, 0], [0, -1, 0], [0, 0, -1]]
        r_enu_flu = multiply(multiply(ned_to_enu, r_ned_frd), flu_to_frd)

        # Fixed camera_link extrinsic from the PX4 x500_depth_gps model:
        # 5 cm below the base, pitched +45 degrees about body Y.
        c = math.cos(math.pi / 4)
        s = math.sin(math.pi / 4)
        r_flu_camera = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
        r_enu_camera = multiply(r_enu_flu, r_flu_camera)
        qw, qx, qy, qz = matrix_to_quaternion(r_enu_camera)

        px = float(message.position[1]) - self.origin_ned[1]
        py = float(message.position[0]) - self.origin_ned[0]
        pz = -(float(message.position[2]) - self.origin_ned[2])
        offset = [
            r_enu_flu[row][2] * -0.05 for row in range(3)]

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = self.camera_frame
        odom.pose.pose.position.x = px + offset[0]
        odom.pose.pose.position.y = py + offset[1]
        odom.pose.pose.position.z = pz + offset[2]
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        for index in (0, 7, 14, 21, 28, 35):
            odom.pose.covariance[index] = 1e-3
        self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

        if not self.published_once:
            self.get_logger().info(
                'Publishing stable PX4 VehicleOdometry in ENU for RTAB-Map.')
            self.published_once = True


def main():
    rclpy.init()
    node = Px4PoseOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
