"""Repository-style cascaded relative-position controller for PX4/Gazebo.

The upstream project calls this a cascaded PI controller driven by Vicon
relative state. In simulation, PX4 local position and Panther odometry provide
the equivalent state. No camera, ArUco, GPS, or RL policy is used here.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node


class RepoStyleController(Node):
    def __init__(self):
        super().__init__('repo_style_cascaded_controller')
        self.declare_parameter('uav_topic', '/fmu/out/vehicle_local_position_v1')
        self.declare_parameter('ugv_topic', '/panther/native/odometry')
        self.declare_parameter('cmd_topic', '/model/x500_flow_tag_0/cmd_vel')
        self.declare_parameter('position_kp', 0.45)
        self.declare_parameter('velocity_kp', 0.25)
        self.declare_parameter('velocity_ki', 0.03)
        self.declare_parameter('desired_altitude', 10.0)
        self.declare_parameter('altitude_kp', 0.35)
        self.declare_parameter('max_horizontal_speed', 2.0)
        self.declare_parameter('max_vertical_speed', 0.8)
        self.uav = self.ugv = None
        self.integral_x = self.integral_y = 0.0
        self.last_time = self.get_clock().now()
        self.pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.create_subscription(VehicleLocalPosition,
            str(self.get_parameter('uav_topic').value), self._uav, 10)
        self.create_subscription(Odometry,
            str(self.get_parameter('ugv_topic').value), self._ugv, 10)
        self.create_timer(0.02, self._control)

    def _uav(self, msg):
        self.uav = (msg.x, msg.y, -msg.z, msg.vx, msg.vy, -msg.vz)

    def _ugv(self, msg):
        self.ugv = msg

    def _control(self):
        command = Twist()
        if self.uav is None or self.ugv is None:
            self.pub.publish(command)
            return
        now = self.get_clock().now()
        dt = min(0.1, max(0.001, (now - self.last_time).nanoseconds * 1e-9))
        self.last_time = now
        ex = self.ugv.pose.pose.position.x - self.uav[0]
        ey = self.ugv.pose.pose.position.y - self.uav[1]
        evx = self.ugv.twist.twist.linear.x - self.uav[3]
        evy = self.ugv.twist.twist.linear.y - self.uav[4]
        self.integral_x = max(-2.0, min(2.0, self.integral_x + ex * dt))
        self.integral_y = max(-2.0, min(2.0, self.integral_y + ey * dt))
        position_kp = float(self.get_parameter('position_kp').value)
        velocity_kp = float(self.get_parameter('velocity_kp').value)
        velocity_ki = float(self.get_parameter('velocity_ki').value)
        command.linear.x = position_kp * ex + velocity_kp * evx + velocity_ki * self.integral_x
        command.linear.y = position_kp * ey + velocity_kp * evy + velocity_ki * self.integral_y
        maximum = float(self.get_parameter('max_horizontal_speed').value)
        scale = max(1.0, math.hypot(command.linear.x, command.linear.y) / maximum)
        command.linear.x /= scale
        command.linear.y /= scale
        altitude_error = float(self.get_parameter('desired_altitude').value) - self.uav[2]
        command.linear.z = max(-float(self.get_parameter('max_vertical_speed').value),
                               min(float(self.get_parameter('max_vertical_speed').value),
                                   float(self.get_parameter('altitude_kp').value) * altitude_error))
        self.pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = RepoStyleController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
