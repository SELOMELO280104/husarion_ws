"""ROS 2 bridge between a search policy and the Gazebo UAV velocity topic."""

from __future__ import annotations

import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8


class RLSearchRosAdapter(Node):
    """Apply policy actions only while explicitly enabled; otherwise hover."""

    ACTIONS = ((0.0, 0.0, 0.0), (0.6, 0.0, 0.0), (-0.6, 0.0, 0.0),
               (0.0, 0.6, 0.0), (0.0, -0.6, 0.0), (0.0, 0.0, 0.4),
               (0.0, 0.0, -0.4))

    def __init__(self):
        super().__init__('rl_search_ros_adapter')
        self.declare_parameter('enabled', False)
        self.declare_parameter('cmd_vel_topic', '/model/x500_flow_tag_0/cmd_vel')
        self.enabled = bool(self.get_parameter('enabled').value)
        self.latest_action = 0
        self.pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.create_subscription(UInt8, '~/action', self._action, 10)
        self.create_subscription(Bool, '~/enable', self._enable, 10)
        self.create_subscription(String, '/uav/tag_follower/status',
                                 self._status, 10)
        self.create_timer(0.1, self._publish)

    def _action(self, message):
        self.latest_action = min(int(message.data), len(self.ACTIONS) - 1)

    def _enable(self, message):
        self.enabled = bool(message.data)
        if not self.enabled:
            self.latest_action = 0

    def _status(self, message):
        try:
            payload = json.loads(message.data)
            if payload.get('tag_visible') or payload.get('landing_state') not in (
                    None, 'DISABLED'):
                self.latest_action = 0
        except (TypeError, ValueError):
            self.latest_action = 0

    def _publish(self):
        command = Twist()
        if self.enabled:
            command.linear.x, command.linear.y, command.linear.z = \
                self.ACTIONS[self.latest_action]
        self.pub.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = RLSearchRosAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
