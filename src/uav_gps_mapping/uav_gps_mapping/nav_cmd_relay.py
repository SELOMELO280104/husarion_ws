"""Relay Nav2 velocity commands into Panther's autonomous mux input."""

from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


class NavCommandRelay(Node):
    """Forward only stamped Nav2 commands to the autonomous priority input."""

    def __init__(self):
        super().__init__('panther_nav_cmd_relay')
        self.publisher = self.create_publisher(
            TwistStamped, '/panther/autonomous/cmd_vel', 10)
        # Nav2's collision monitor publishes the final safety-filtered command.
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self._callback, 10)
        self.get_logger().info(
            'Relaying /cmd_vel to /panther/autonomous/cmd_vel.')

    def _callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'panther/base_link'
        stamped.twist = msg
        self.publisher.publish(stamped)


def main(args=None):
    rclpy.init(args=args)
    node = NavCommandRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
