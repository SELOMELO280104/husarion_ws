"""Gazebo ground-truth equivalent of the repository's Vicon controller."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node


class GroundtruthRelativeTracker(Node):
    def __init__(self):
        super().__init__('groundtruth_relative_tracker')
        self.declare_parameter('uav_pose_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('ugv_pose_topic', '/panther/native/odometry')
        self.declare_parameter('cmd_vel_topic', '/model/x500_flow_tag_0/cmd_vel')
        self.declare_parameter('kp', 0.35)
        self.declare_parameter('kd', 0.10)
        self.declare_parameter('desired_height', 10.0)
        self.declare_parameter('max_speed', 1.0)
        self.uav = self.ugv = None
        self.pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.create_subscription(VehicleLocalPosition,
            str(self.get_parameter('uav_pose_topic').value), self._uav, 10)
        self.create_subscription(Odometry,
            str(self.get_parameter('ugv_pose_topic').value), self._ugv, 10)
        self.create_timer(0.05, self._control)

    def _uav(self, msg):
        # PX4 local position is NED: convert its down-positive z to altitude.
        self.uav = (msg.x, msg.y, -msg.z, msg.vx, msg.vy, -msg.vz)

    def _ugv(self, msg):
        self.ugv = msg

    def _control(self):
        command = Twist()
        if self.uav is None or self.ugv is None:
            self.pub.publish(command)
            return
        dx = self.ugv.pose.pose.position.x - self.uav[0]
        dy = self.ugv.pose.pose.position.y - self.uav[1]
        vx = self.ugv.twist.twist.linear.x - self.uav[3]
        vy = self.ugv.twist.twist.linear.y - self.uav[4]
        kp = float(self.get_parameter('kp').value)
        kd = float(self.get_parameter('kd').value)
        max_speed = float(self.get_parameter('max_speed').value)
        command.linear.x = kp * dx + kd * vx
        command.linear.y = kp * dy + kd * vy
        scale = max(1.0, math.hypot(command.linear.x, command.linear.y) / max_speed)
        command.linear.x /= scale
        command.linear.y /= scale
        command.linear.z = max(-0.6, min(0.6, float(self.get_parameter(
            'desired_height').value) - self.uav[2]))
        self.pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = GroundtruthRelativeTracker()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
