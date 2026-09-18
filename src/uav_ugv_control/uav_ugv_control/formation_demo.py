import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class FormationDemo(Node):
    def __init__(self):
        super().__init__('formation_demo')

        self.declare_parameter('uav_cmd_topic', '/uav/cmd_vel')
        self.declare_parameter('ugv_cmd_topic', '/ugv/cmd_vel')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('ugv_forward_speed', 0.25)
        self.declare_parameter('uav_forward_speed', 0.35)
        self.declare_parameter('uav_altitude_rate', 0.08)
        self.declare_parameter('yaw_rate', 0.15)

        self.uav_cmd_topic = self.get_parameter('uav_cmd_topic').value
        self.ugv_cmd_topic = self.get_parameter('ugv_cmd_topic').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.ugv_forward_speed = float(self.get_parameter('ugv_forward_speed').value)
        self.uav_forward_speed = float(self.get_parameter('uav_forward_speed').value)
        self.uav_altitude_rate = float(self.get_parameter('uav_altitude_rate').value)
        self.yaw_rate = float(self.get_parameter('yaw_rate').value)

        self.uav_pub = self.create_publisher(Twist, self.uav_cmd_topic, 10)
        self.ugv_pub = self.create_publisher(Twist, self.ugv_cmd_topic, 10)
        self.start_time = self.get_clock().now()

        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self.publish_commands)

        self.get_logger().info(
            f'Publishing UAV commands on {self.uav_cmd_topic} and UGV '
            f'commands on {self.ugv_cmd_topic}'
        )

    def publish_commands(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        ugv_cmd = Twist()
        ugv_cmd.linear.x = self.ugv_forward_speed
        ugv_cmd.angular.z = self.yaw_rate * math.sin(elapsed / 4.0)

        uav_cmd = Twist()
        uav_cmd.linear.x = self.uav_forward_speed
        uav_cmd.linear.z = self.uav_altitude_rate * math.sin(elapsed / 3.0)
        uav_cmd.angular.z = self.yaw_rate * math.cos(elapsed / 4.0)

        self.ugv_pub.publish(ugv_cmd)
        self.uav_pub.publish(uav_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FormationDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
