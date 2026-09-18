"""Safety adapter for Gazebo's native Panther skid-steer drive."""

from __future__ import annotations

import copy
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class NativePantherDrive(Node):
    """Enforce E-stop/watchdog and expose native odometry to Nav2."""

    def __init__(self):
        super().__init__('native_panther_drive')
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('initial_estop', True)
        self.command_timeout = self.get_parameter(
            'command_timeout').get_parameter_value().double_value
        initial_estop = self.get_parameter(
            'initial_estop').get_parameter_value().bool_value

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.command_pub = self.create_publisher(
            Twist, '/panther/native/cmd_vel', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/panther/wheel/odometry', 20)
        self.imu_pub = self.create_publisher(
            Imu, '/panther/imu/data', qos_profile_sensor_data)
        self.estop_pub = self.create_publisher(
            Bool, '/panther/hardware/e_stop', latched_qos)
        self.create_subscription(
            Twist, '/cmd_vel', self._command_callback, 10)
        self.create_subscription(
            Odometry,
            '/panther/native/odometry',
            self._odometry_callback,
            20,
        )
        self.create_subscription(
            Imu,
            '/panther/imu/data_raw',
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_service(
            Trigger,
            '/panther/hardware/e_stop_reset',
            self._reset_estop,
        )
        self.create_service(
            Trigger,
            '/panther/hardware/e_stop_trigger',
            self._trigger_estop,
        )

        self.estop_active = initial_estop
        self.last_command = Twist()
        self.last_command_time = None
        # Retain the timer objects explicitly.  Apart from documenting their
        # lifetime, this prevents Python garbage collection from removing the
        # low-frequency E-stop publisher after startup on some rclpy builds.
        self.drive_timer = self.create_timer(0.05, self._drive_timer)
        self.estop_timer = self.create_timer(1.0, self._publish_estop)
        self._publish_estop()
        if self.estop_active:
            self.get_logger().warning(
                'Native drive ready with E-stop ACTIVE. Reset '
                '/panther/hardware/e_stop_reset before starting a mission.')
        else:
            self.get_logger().warning(
                'Native drive ready with initial E-stop disabled by launch '
                'configuration; collision monitoring remains active.')

    def _command_callback(self, message):
        self.last_command = copy.deepcopy(message)
        self.last_command_time = time.monotonic()

    def _drive_timer(self):
        fresh = (
            self.last_command_time is not None
            and time.monotonic() - self.last_command_time
            <= self.command_timeout
        )
        if self.estop_active or not fresh:
            self.command_pub.publish(Twist())
        else:
            self.command_pub.publish(self.last_command)

    def _publish_estop(self):
        message = Bool()
        message.data = self.estop_active
        self.estop_pub.publish(message)

    def _reset_estop(self, _, response):
        self.estop_active = False
        self._publish_estop()
        response.success = True
        response.message = 'Panther E-stop reset; native drive enabled'
        self.get_logger().info(response.message)
        return response

    def _trigger_estop(self, _, response):
        self.estop_active = True
        self.last_command = Twist()
        self.command_pub.publish(Twist())
        self._publish_estop()
        response.success = True
        response.message = 'Panther E-stop activated; drive stopped'
        self.get_logger().warning(response.message)
        return response

    def _odometry_callback(self, message):
        output = copy.deepcopy(message)
        # Use the active ROS simulation clock. A Gazebo server left behind by
        # an interrupted earlier run can otherwise inject one retained odom
        # sample with a future stamp; TF then rejects every current transform
        # as TF_OLD_DATA and the collision monitor correctly stops the robot.
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = 'panther/odom'
        output.child_frame_id = 'panther/base_link'
        self.odom_pub.publish(output)

    def _imu_callback(self, message):
        output = copy.deepcopy(message)
        # The upstream Panther model names this Gazebo sensor frame
        # ``imu_link`` while robot_state_publisher intentionally prefixes the
        # complete URDF tree with ``panther/``. Correct that single model
        # inconsistency so robot_localization can apply the IMU-to-base fixed
        # transform (including the sensor's 180 degree mounting yaw).
        output.header.frame_id = 'panther/imu_link'
        self.imu_pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = NativePantherDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        # ros_gz_bridge can disappear while the executor is taking its final
        # odometry sample during launch shutdown. Treat only that shutdown
        # race as clean; a runtime failure while ROS is live must still fail.
        if rclpy.ok():
            raise
    finally:
        if rclpy.ok():
            node.estop_active = True
            node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
