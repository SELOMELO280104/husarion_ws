import math
from typing import Optional

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time

try:
    from px4_msgs.msg import OffboardControlMode
    from px4_msgs.msg import TrajectorySetpoint
    from px4_msgs.msg import VehicleCommandAck
    from px4_msgs.msg import VehicleCommand
    from px4_msgs.msg import VehicleLocalPosition
    from px4_msgs.msg import VehicleStatus
except ImportError:
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommandAck = None
    VehicleCommand = None
    VehicleLocalPosition = None
    VehicleStatus = None

from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from tf2_ros import TransformListener


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def set_if_present(message, name, value):
    if hasattr(message, name):
        setattr(message, name, value)


class X500FollowUGV(Node):
    def __init__(self):
        super().__init__('x500_follow_ugv')

        if OffboardControlMode is None:
            self.get_logger().error(
                'px4_msgs is not available. Run ./scripts/import_px4_x500.sh, '
                'then rebuild the workspace with px4_msgs present.'
            )
            raise RuntimeError('px4_msgs is required for PX4 offboard control')

        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('ugv_frame', 'panther/base_link')
        self.declare_parameter('ground_truth_tf_topic', '/tf_gt')
        self.declare_parameter('vehicle_status_topic', '/fmu/out/vehicle_status_v4')
        self.declare_parameter(
            'vehicle_local_position_topic', '/fmu/out/vehicle_local_position_v1'
        )
        self.declare_parameter('vehicle_command_ack_topic', '/fmu/out/vehicle_command_ack_v1')
        self.declare_parameter('follow_distance', 4.0)
        self.declare_parameter('lateral_offset', 0.0)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('uav_origin_x', -11.0)
        self.declare_parameter('uav_origin_y', 5.0)
        self.declare_parameter('uav_origin_z', 0.2)
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_offboard', True)
        self.declare_parameter('startup_setpoints', 30)
        self.declare_parameter('takeoff_hold_seconds', 3.0)
        self.declare_parameter('command_retry_seconds', 3.0)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)
        self.declare_parameter('source_system', 1)
        self.declare_parameter('source_component', 1)

        self.world_frame = self.get_parameter('world_frame').value
        self.ugv_frame = self.get_parameter('ugv_frame').value
        self.ground_truth_tf_topic = self.get_parameter('ground_truth_tf_topic').value
        self.vehicle_status_topic = self.get_parameter('vehicle_status_topic').value
        self.vehicle_local_position_topic = self.get_parameter(
            'vehicle_local_position_topic'
        ).value
        self.vehicle_command_ack_topic = self.get_parameter('vehicle_command_ack_topic').value
        self.follow_distance = float(self.get_parameter('follow_distance').value)
        self.lateral_offset = float(self.get_parameter('lateral_offset').value)
        self.altitude = float(self.get_parameter('altitude').value)
        self.uav_origin_x = float(self.get_parameter('uav_origin_x').value)
        self.uav_origin_y = float(self.get_parameter('uav_origin_y').value)
        self.uav_origin_z = float(self.get_parameter('uav_origin_z').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.auto_offboard = bool(self.get_parameter('auto_offboard').value)
        self.startup_setpoints = int(self.get_parameter('startup_setpoints').value)
        self.takeoff_hold_seconds = float(self.get_parameter('takeoff_hold_seconds').value)
        self.command_retry_seconds = float(self.get_parameter('command_retry_seconds').value)
        self.target_system = int(self.get_parameter('target_system').value)
        self.target_component = int(self.get_parameter('target_component').value)
        self.source_system = int(self.get_parameter('source_system').value)
        self.source_component = int(self.get_parameter('source_component').value)

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', px4_qos
        )
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, self.vehicle_status_topic, self._vehicle_status_cb, px4_qos
        )
        self.vehicle_local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            self.vehicle_local_position_topic,
            self._vehicle_local_position_cb,
            px4_qos,
        )
        self.vehicle_command_ack_sub = self.create_subscription(
            VehicleCommandAck,
            self.vehicle_command_ack_topic,
            self._vehicle_command_ack_cb,
            px4_qos,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        if self.ground_truth_tf_topic:
            self.create_subscription(
                TFMessage, self.ground_truth_tf_topic, self._ground_truth_tf_cb, 10
            )
        self.vehicle_status: Optional[VehicleStatus] = None
        self.vehicle_local_position: Optional[VehicleLocalPosition] = None
        self.setpoint_count = 0
        self.last_command_time = self.get_clock().now()
        self.takeoff_ready_since: Optional[Time] = None
        self.follow_unlocked = False

        period = 1.0 / max(self.rate_hz, 1.0)
        self.timer = self.create_timer(period, self._tick)

        self.get_logger().info(
            f'Following {self.ugv_frame} from {self.follow_distance:.1f} m behind '
            f'at {self.altitude:.1f} m altitude. PX4 local origin ENU='
            f'({self.uav_origin_x:.1f}, {self.uav_origin_y:.1f}, {self.uav_origin_z:.1f})'
        )

    def _vehicle_status_cb(self, msg):
        self.vehicle_status = msg

    def _vehicle_local_position_cb(self, msg):
        self.vehicle_local_position = msg

    def _vehicle_command_ack_cb(self, msg):
        interesting_commands = {
            getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176): 'set mode',
            getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400): 'arm/disarm',
        }
        if msg.command not in interesting_commands:
            return

        result_names = {
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_ACCEPTED', 0): 'accepted',
            getattr(
                VehicleCommandAck,
                'VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED',
                1,
            ): 'temporarily rejected',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_DENIED', 2): 'denied',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_UNSUPPORTED', 3): 'unsupported',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_FAILED', 4): 'failed',
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_IN_PROGRESS', 5): 'in progress',
        }
        result = result_names.get(msg.result, f'result {msg.result}')
        log = self.get_logger().info
        if msg.result not in (
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_ACCEPTED', 0),
            getattr(VehicleCommandAck, 'VEHICLE_CMD_RESULT_IN_PROGRESS', 5),
        ):
            log = self.get_logger().warn

        log(
            f'PX4 command ack: {interesting_commands[msg.command]} {result} '
            f'(result_param1={msg.result_param1}, result_param2={msg.result_param2})'
        )

    def _ground_truth_tf_cb(self, msg):
        for transform in msg.transforms:
            try:
                self.tf_buffer.set_transform(transform, self.get_name())
            except Exception as exc:
                self.get_logger().warn(
                    f'Could not add ground-truth transform to TF buffer: {exc}',
                    throttle_duration_sec=2.0,
                )

    def _timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def _is_offboard(self):
        if self.vehicle_status is None:
            return False
        offboard_state = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
        return self.vehicle_status.nav_state == offboard_state

    def _is_armed(self):
        if self.vehicle_status is None:
            return False
        armed_state = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)
        return self.vehicle_status.arming_state == armed_state

    def _local_position_ready(self):
        if self.vehicle_local_position is None:
            return False
        return (
            self.vehicle_local_position.xy_valid
            and self.vehicle_local_position.z_valid
            and math.isfinite(self.vehicle_local_position.x)
            and math.isfinite(self.vehicle_local_position.y)
            and math.isfinite(self.vehicle_local_position.z)
        )

    def _takeoff_hold_target(self):
        if self._local_position_ready():
            north = float(self.vehicle_local_position.x)
            east = float(self.vehicle_local_position.y)
            yaw_ned = float(self.vehicle_local_position.heading)
            if not math.isfinite(yaw_ned):
                yaw_ned = 0.0
        else:
            north = 0.0
            east = 0.0
            yaw_ned = 0.0

        return (north, east, -abs(self.altitude)), yaw_ned

    def _update_follow_unlock(self):
        if self.follow_unlocked:
            return

        if not (self._is_armed() and self._is_offboard() and self._local_position_ready()):
            self.takeoff_ready_since = None
            return

        altitude = -float(self.vehicle_local_position.z)
        if altitude < abs(self.altitude) * 0.8:
            self.takeoff_ready_since = None
            return

        now = self.get_clock().now()
        if self.takeoff_ready_since is None:
            self.takeoff_ready_since = now
            return

        elapsed = (now - self.takeoff_ready_since).nanoseconds * 1e-9
        if elapsed >= self.takeoff_hold_seconds:
            self.follow_unlocked = True
            self.get_logger().info('X500 takeoff hold complete; following UGV')

    def _latest_ugv_transform(self) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(
                self.world_frame, self.ugv_frame, Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'Waiting for TF {self.world_frame} <- {self.ugv_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None

    def _target_from_ugv(self, transform: TransformStamped):
        translation = transform.transform.translation
        yaw_enu = yaw_from_quaternion(transform.transform.rotation)

        forward_x = math.cos(yaw_enu)
        forward_y = math.sin(yaw_enu)
        left_x = -math.sin(yaw_enu)
        left_y = math.cos(yaw_enu)

        target_x = (
            translation.x
            - self.follow_distance * forward_x
            + self.lateral_offset * left_x
        )
        target_y = (
            translation.y
            - self.follow_distance * forward_y
            + self.lateral_offset * left_y
        )
        target_z = translation.z + self.altitude

        # Gazebo / ROS world is ENU. PX4 local position setpoints are NED.
        north = target_y - self.uav_origin_y
        east = target_x - self.uav_origin_x
        down = -(target_z - self.uav_origin_z)
        yaw_ned = normalize_angle(math.pi / 2.0 - yaw_enu)

        return (north, east, down), yaw_ned, (target_x, target_y, target_z)

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._timestamp_us()
        set_if_present(msg, 'position', True)
        set_if_present(msg, 'velocity', False)
        set_if_present(msg, 'acceleration', False)
        set_if_present(msg, 'attitude', False)
        set_if_present(msg, 'body_rate', False)
        set_if_present(msg, 'thrust_and_torque', False)
        set_if_present(msg, 'direct_actuator', False)
        self.offboard_control_mode_pub.publish(msg)

    def _publish_trajectory_setpoint(self, ned_position, yaw_ned):
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp_us()
        msg.position = [float(ned_position[0]), float(ned_position[1]), float(ned_position[2])]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        set_if_present(msg, 'jerk', [math.nan, math.nan, math.nan])
        msg.yaw = float(yaw_ned)
        msg.yawspeed = math.nan
        self.trajectory_setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._timestamp_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _engage_offboard(self):
        command = getattr(VehicleCommand, 'VEHICLE_CMD_DO_SET_MODE', 176)
        self._publish_vehicle_command(command, param1=1.0, param2=6.0)
        self.get_logger().info('Requested PX4 offboard mode')

    def _arm(self):
        command = getattr(VehicleCommand, 'VEHICLE_CMD_COMPONENT_ARM_DISARM', 400)
        self._publish_vehicle_command(command, param1=1.0)
        self.get_logger().info('Requested PX4 arm')

    def _maybe_send_mode_commands(self):
        if self.setpoint_count < self.startup_setpoints:
            self.setpoint_count += 1
            return

        if not self._local_position_ready():
            self.get_logger().warn(
                'Waiting for valid PX4 local position before arming/offboard',
                throttle_duration_sec=2.0,
            )
            return

        if self.vehicle_status is not None and self.vehicle_status.failsafe:
            self.get_logger().warn(
                'PX4 is in failsafe; not sending arm/offboard commands',
                throttle_duration_sec=2.0,
            )
            return

        elapsed = (self.get_clock().now() - self.last_command_time).nanoseconds * 1e-9
        if elapsed < self.command_retry_seconds:
            return

        if self.auto_offboard and not self._is_offboard():
            self._engage_offboard()
        if self.auto_arm and not self._is_armed():
            self._arm()

        self.last_command_time = self.get_clock().now()

    def _tick(self):
        target_enu = None
        if self.follow_unlocked:
            transform = self._latest_ugv_transform()
            if transform is not None:
                ned_position, yaw_ned, target_enu = self._target_from_ugv(transform)
            else:
                ned_position, yaw_ned = self._takeoff_hold_target()
        else:
            ned_position, yaw_ned = self._takeoff_hold_target()

        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint(ned_position, yaw_ned)
        self._maybe_send_mode_commands()
        self._update_follow_unlock()

        if target_enu is None:
            self.get_logger().info(
                f'X500 takeoff/hold NED=({ned_position[0]:.2f}, '
                f'{ned_position[1]:.2f}, {ned_position[2]:.2f}) '
                f'local_position_ready={self._local_position_ready()} '
                f'offboard={self._is_offboard()} armed={self._is_armed()}',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                f'X500 target ENU=({target_enu[0]:.2f}, {target_enu[1]:.2f}, {target_enu[2]:.2f}) '
                f'NED=({ned_position[0]:.2f}, {ned_position[1]:.2f}, {ned_position[2]:.2f}) '
                f'offboard={self._is_offboard()} armed={self._is_armed()}',
                throttle_duration_sec=1.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = X500FollowUGV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
