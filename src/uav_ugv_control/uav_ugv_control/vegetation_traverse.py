import math
import random
from typing import Optional

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

try:
    from tf2_ros import Buffer
    from tf2_ros import ConnectivityException
    from tf2_ros import ExtrapolationException
    from tf2_ros import LookupException
    from tf2_ros import TransformListener
except ImportError:
    Buffer = None
    TransformListener = None
    LookupException = ConnectivityException = ExtrapolationException = Exception


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class VegetationTraverse(Node):
    def __init__(self):
        super().__init__('vegetation_traverse')

        self.declare_parameter(
            'scan_topic', '/panther/main_lidar/scan')
        self.declare_parameter('cmd_topic', '/panther/manual/cmd_vel')
        self.declare_parameter('e_stop_reset_service', '/panther/hardware/e_stop_reset')
        self.declare_parameter('target_frame', 'panther/base_link')
        self.declare_parameter('use_base_frame_metrics', True)
        self.declare_parameter('rate_hz', 12.0)
        self.declare_parameter('front_angle', math.pi)
        self.declare_parameter('forward_speed', 0.24)
        self.declare_parameter('crawl_speed', 0.10)
        self.declare_parameter('reverse_speed', -0.18)
        self.declare_parameter('turn_speed', 0.48)
        self.declare_parameter('front_stop_distance', 1.05)
        self.declare_parameter('front_slow_distance', 1.8)
        self.declare_parameter('front_clear_distance', 1.55)
        self.declare_parameter('side_stop_distance', 0.55)
        self.declare_parameter('front_corridor_half_width', 0.46)
        self.declare_parameter('side_window_front_x', 1.4)
        self.declare_parameter('scan_stale_seconds', 1.0)
        self.declare_parameter('reverse_seconds', 1.1)
        self.declare_parameter('min_turn_seconds', 2.0)
        self.declare_parameter('steer_gain', 0.55)
        self.declare_parameter('max_steer', 0.45)
        self.declare_parameter('debug_log_seconds', 1.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.e_stop_reset_service = self.get_parameter('e_stop_reset_service').value
        self.target_frame = self.get_parameter('target_frame').value
        self.use_base_frame_metrics = bool(self.get_parameter('use_base_frame_metrics').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.front_angle = float(self.get_parameter('front_angle').value)
        self.forward_speed = float(self.get_parameter('forward_speed').value)
        self.crawl_speed = float(self.get_parameter('crawl_speed').value)
        self.reverse_speed = float(self.get_parameter('reverse_speed').value)
        self.turn_speed = float(self.get_parameter('turn_speed').value)
        self.front_stop_distance = float(self.get_parameter('front_stop_distance').value)
        self.front_slow_distance = float(self.get_parameter('front_slow_distance').value)
        self.front_clear_distance = float(self.get_parameter('front_clear_distance').value)
        self.side_stop_distance = float(self.get_parameter('side_stop_distance').value)
        self.front_corridor_half_width = float(
            self.get_parameter('front_corridor_half_width').value
        )
        self.side_window_front_x = float(self.get_parameter('side_window_front_x').value)
        self.scan_stale_seconds = float(self.get_parameter('scan_stale_seconds').value)
        self.reverse_seconds = float(self.get_parameter('reverse_seconds').value)
        self.min_turn_seconds = float(self.get_parameter('min_turn_seconds').value)
        self.steer_gain = float(self.get_parameter('steer_gain').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.debug_log_seconds = float(self.get_parameter('debug_log_seconds').value)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb, qos_profile_sensor_data
        )
        self.estop_client = self.create_client(Trigger, self.e_stop_reset_service)
        self.tf_buffer = None
        self.tf_listener = None
        if self.use_base_frame_metrics and Buffer is not None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_scan_time = None
        self.estop_reset_done = False
        self.estop_future = None
        self.mode = 'drive'
        self.turn_direction = 1
        self.reverse_started = self.get_clock().now()
        self.turn_started = self.get_clock().now()

        period = 1.0 / max(self.rate_hz, 0.1)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'Vegetation traversal using {self.scan_topic}; front scan angle '
            f'{self.front_angle:.3f} rad; publishing {self.cmd_topic}'
        )
        if self.tf_buffer is not None:
            self.get_logger().info(
                f'Collision checks use lidar points transformed into {self.target_frame}'
            )
        elif self.use_base_frame_metrics:
            self.get_logger().warn(
                'tf2_ros is unavailable, falling back to raw LaserScan angle checks'
            )

    def _scan_cb(self, msg):
        self.latest_scan = msg
        self.latest_scan_time = self.get_clock().now()

    def _scan_age(self):
        if self.latest_scan_time is None:
            return math.inf
        return (self.get_clock().now() - self.latest_scan_time).nanoseconds * 1e-9

    def _range_for_angle(self, scan, target_angle, half_width):
        best = math.inf
        for index, raw_distance in enumerate(scan.ranges):
            if not math.isfinite(raw_distance):
                continue
            if raw_distance < scan.range_min or raw_distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            if abs(normalize_angle(angle - target_angle)) <= half_width:
                best = min(best, raw_distance)

        if math.isfinite(best):
            return best
        return scan.range_max if math.isfinite(scan.range_max) else 10.0

    def _metrics_from_scan_angles(self, scan):
        front = self.front_angle
        return {
            'front': self._range_for_angle(scan, front, math.radians(24)),
            'front_left': self._range_for_angle(scan, front + math.radians(35), math.radians(26)),
            'front_right': self._range_for_angle(scan, front - math.radians(35), math.radians(26)),
            'left': self._range_for_angle(scan, front + math.radians(82), math.radians(38)),
            'right': self._range_for_angle(scan, front - math.radians(82), math.radians(38)),
            'source': 'scan_angle',
        }

    @staticmethod
    def _rotate_vector(quaternion, x, y, z):
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w

        uv_x = qy * z - qz * y
        uv_y = qz * x - qx * z
        uv_z = qx * y - qy * x

        uuv_x = qy * uv_z - qz * uv_y
        uuv_y = qz * uv_x - qx * uv_z
        uuv_z = qx * uv_y - qy * uv_x

        return (
            x + 2.0 * (qw * uv_x + uuv_x),
            y + 2.0 * (qw * uv_y + uuv_y),
            z + 2.0 * (qw * uv_z + uuv_z),
        )

    def _transform_laser_point(self, transform, x, y, z):
        rx, ry, rz = self._rotate_vector(transform.rotation, x, y, z)
        return (
            rx + transform.translation.x,
            ry + transform.translation.y,
            rz + transform.translation.z,
        )

    def _metrics_from_base_frame(self, scan):
        frame_id = scan.header.frame_id.lstrip('/')
        if not frame_id:
            self.get_logger().warn(
                'LaserScan has an empty frame_id; stopping until a valid lidar frame is available',
                throttle_duration_sec=2.0,
            )
            return None

        try:
            stamped_transform = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.target_frame} <- {frame_id}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

        transform = stamped_transform.transform
        range_max = scan.range_max if math.isfinite(scan.range_max) else 10.0
        metrics = {
            'front': range_max,
            'front_left': range_max,
            'front_right': range_max,
            'left': range_max,
            'right': range_max,
            'source': self.target_frame,
        }

        for index, raw_distance in enumerate(scan.ranges):
            if not math.isfinite(raw_distance):
                continue
            if raw_distance < scan.range_min or raw_distance > scan.range_max:
                continue

            angle = scan.angle_min + index * scan.angle_increment
            laser_x = raw_distance * math.cos(angle)
            laser_y = raw_distance * math.sin(angle)
            base_x, base_y, _ = self._transform_laser_point(
                transform, laser_x, laser_y, 0.0
            )

            if base_x >= 0.0 and abs(base_y) <= self.front_corridor_half_width:
                metrics['front'] = min(metrics['front'], base_x)

            if base_x >= 0.0 and 0.0 <= base_y <= self.front_corridor_half_width * 1.8:
                metrics['front_left'] = min(metrics['front_left'], base_x)

            if base_x >= 0.0 and -self.front_corridor_half_width * 1.8 <= base_y <= 0.0:
                metrics['front_right'] = min(metrics['front_right'], base_x)

            if -0.2 <= base_x <= self.side_window_front_x and base_y > 0.0:
                metrics['left'] = min(metrics['left'], base_y)

            if -0.2 <= base_x <= self.side_window_front_x and base_y < 0.0:
                metrics['right'] = min(metrics['right'], -base_y)

        return metrics

    def _metrics(self, scan):
        if self.tf_buffer is not None:
            return self._metrics_from_base_frame(scan)
        return self._metrics_from_scan_angles(scan)

    def _reset_estop_if_needed(self):
        if self.estop_reset_done:
            return True

        if self.estop_future is not None:
            if not self.estop_future.done():
                return False

            result = self.estop_future.result()
            if result is not None and result.success:
                self.estop_reset_done = True
                self.get_logger().info(f'E-stop reset: {result.message}')
                return True

            self.estop_future = None

        if not self.estop_client.service_is_ready():
            self.estop_client.wait_for_service(timeout_sec=0.0)
            self._publish_cmd(0.0, 0.0)
            return False

        self.estop_future = self.estop_client.call_async(Trigger.Request())
        self._publish_cmd(0.0, 0.0)
        return False

    def _choose_turn_direction(self, metrics):
        front_left = metrics['front_left']
        front_right = metrics['front_right']
        left = metrics['left']
        right = metrics['right']

        if front_left < front_right - 0.20:
            return -1
        if front_right < front_left - 0.20:
            return 1
        if left < self.side_stop_distance and right > left + 0.25:
            return -1
        if right < self.side_stop_distance and left > right + 0.25:
            return 1

        return random.choice([-1, 1])

    def _front_clearance(self, metrics):
        return metrics['front']

    def _log_metrics(self, metrics, front_clearance):
        if self.debug_log_seconds <= 0.0:
            return

        self.get_logger().info(
            'lidar clearance '
            f'source={metrics["source"]} mode={self.mode} '
            f'front={metrics["front"]:.2f}m '
            f'front_left={metrics["front_left"]:.2f}m '
            f'front_right={metrics["front_right"]:.2f}m '
            f'left={metrics["left"]:.2f}m '
            f'right={metrics["right"]:.2f}m '
            f'front_corridor={front_clearance:.2f}m '
            f'stop={self.front_stop_distance:.2f}m',
            throttle_duration_sec=self.debug_log_seconds,
        )

    def _start_reverse(self, metrics):
        self.mode = 'reverse'
        self.turn_direction = self._choose_turn_direction(metrics)
        self.reverse_started = self.get_clock().now()
        label = 'left' if self.turn_direction > 0 else 'right'
        self.get_logger().info(f'Obstacle close ahead, reversing before turning {label}')

    def _start_turn(self):
        self.mode = 'turn'
        self.turn_started = self.get_clock().now()
        label = 'left' if self.turn_direction > 0 else 'right'
        self.get_logger().info(f'Turning {label}')

    def _publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        # Leave the stamp at zero so Panther's twist mux evaluates the command
        # against its own controller clock. This avoids wall-time commands being
        # rejected as "future" messages while Gazebo sim time is starting.
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)

    def _tick(self):
        if not self._reset_estop_if_needed():
            return

        if self.latest_scan is None or self._scan_age() > self.scan_stale_seconds:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().warn(
                f'Waiting for fresh lidar scans on {self.scan_topic}',
                throttle_duration_sec=2.0,
            )
            return

        metrics = self._metrics(self.latest_scan)
        if metrics is None:
            self._publish_cmd(0.0, 0.0)
            return

        front_clearance = self._front_clearance(metrics)
        self._log_metrics(metrics, front_clearance)

        if self.mode == 'reverse':
            elapsed = (self.get_clock().now() - self.reverse_started).nanoseconds * 1e-9
            if elapsed >= self.reverse_seconds:
                self._start_turn()
            else:
                self._publish_cmd(self.reverse_speed, self.turn_direction * self.turn_speed * 0.65)
                return

        if self.mode == 'turn':
            elapsed = (self.get_clock().now() - self.turn_started).nanoseconds * 1e-9
            if elapsed >= self.min_turn_seconds and front_clearance > self.front_clear_distance:
                self.mode = 'drive'
            else:
                self._publish_cmd(0.0, self.turn_direction * self.turn_speed)
                return

        if front_clearance < self.front_stop_distance:
            self._start_reverse(metrics)
            self._publish_cmd(self.reverse_speed, self.turn_direction * self.turn_speed * 0.65)
            return

        front_balance = metrics['front_left'] - metrics['front_right']
        side_balance = metrics['left'] - metrics['right']
        angular_z = clamp(
            self.steer_gain * (0.65 * front_balance + 0.35 * side_balance),
            -self.max_steer,
            self.max_steer,
        )

        speed = self.forward_speed
        if front_clearance < self.front_slow_distance:
            speed = self.crawl_speed

        self._publish_cmd(speed, angular_z)


def main(args=None):
    rclpy.init(args=args)
    node = VegetationTraverse()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
