"""Repository-compatible cascaded controller and landing state machine."""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from .repo_landing_core import (
    CascadedPIAxis,
    RepoObservation,
    clamp,
    rotate_to_world,
)


class RepoLandingController(Node):
    """Track a moving platform and descend at a constant relative rate."""

    def __init__(self):
        super().__init__('repo_landing_controller')
        self.declare_parameter('observation_topic', '/repo_landing/observation')
        self.declare_parameter('cmd_topic', '/model/x500_flow_tag_0/cmd_vel')
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('automatic_landing', True)
        self.declare_parameter('tracking_settle_time', 5.0)
        self.declare_parameter('descent_speed', 0.10)
        self.declare_parameter('touchdown_height', 0.32)
        self.declare_parameter('touchdown_radius', 0.55)
        self.declare_parameter('low_altitude_gate', 0.85)
        self.declare_parameter('low_altitude_radius', 0.45)
        self.declare_parameter('initial_height', 4.0)
        self.declare_parameter('vertical_hold_kp', 0.8)
        self.declare_parameter('max_vertical_speed', 0.6)
        self.declare_parameter('position_kp', 3.0)
        self.declare_parameter('velocity_kp', 0.8)
        self.declare_parameter('velocity_ki', 0.15)
        self.declare_parameter('outer_velocity_limit', 3.39)
        self.declare_parameter('max_horizontal_speed', 2.0)
        self.declare_parameter('observation_timeout', 0.5)

        self.observation = None
        self.last_observation = None
        self.phase = 'WAITING_FOR_RELATIVE_STATE'
        self.first_ready_time = None
        self.landing_requested = bool(
            self.get_parameter('automatic_landing').value)
        self.previous_control_time = self.get_clock().now()

        axis_parameters = dict(
            position_kp=float(self.get_parameter('position_kp').value),
            velocity_kp=float(self.get_parameter('velocity_kp').value),
            velocity_ki=float(self.get_parameter('velocity_ki').value),
            velocity_limit=float(
                self.get_parameter('outer_velocity_limit').value),
            correction_limit=float(
                self.get_parameter('max_horizontal_speed').value),
        )
        self.x_controller = CascadedPIAxis(**axis_parameters)
        self.y_controller = CascadedPIAxis(**axis_parameters)

        self.command_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, '/repo_landing/status', 10)
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter('observation_topic').value),
            self._observation_callback,
            20,
        )
        self.create_service(
            Trigger, '/repo_landing/start_landing', self._start_landing)
        self.create_service(
            Trigger, '/repo_landing/stop', self._stop)
        frequency = max(1.0, float(self.get_parameter('control_hz').value))
        self.create_timer(1.0 / frequency, self._control)
        self.get_logger().info(
            'Repository-compatible cascaded PI landing controller started. '
            f'automatic_landing={self.landing_requested}')

    def _observation_callback(self, message: Float64MultiArray) -> None:
        try:
            self.observation = RepoObservation.from_array(message.data)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.last_observation = self.get_clock().now()

    def _start_landing(self, _request, response):
        self.landing_requested = True
        response.success = True
        response.message = 'repository landing sequence enabled'
        return response

    def _stop(self, _request, response):
        self.landing_requested = False
        self.phase = 'STOPPED'
        self.x_controller.reset()
        self.y_controller.reset()
        self.command_pub.publish(Twist())
        response.success = True
        response.message = 'repository landing controller stopped'
        return response

    def _fresh_observation(self, now) -> bool:
        return bool(
            self.observation is not None
            and self.last_observation is not None
            and (now - self.last_observation).nanoseconds * 1e-9
            <= float(self.get_parameter('observation_timeout').value)
        )

    def _publish_status(self, observation, command, position_setpoints) -> None:
        status = String()
        horizontal_error = None
        height = None
        if observation is not None:
            horizontal_error = math.hypot(
                observation.rel_p_x, observation.rel_p_y)
            height = max(0.0, -observation.rel_p_z)
        status.data = json.dumps({
            'mode': self.phase,
            'ready': observation is not None,
            'method': 'repository_cascaded_pi',
            'state_source': 'gazebo_ground_truth_vicon_equivalent',
            'landing_requested': self.landing_requested,
            'horizontal_error_m': horizontal_error,
            'height_above_platform_m': height,
            'relative_velocity_setpoint': position_setpoints,
            'command_enu_mps': [
                command.linear.x, command.linear.y, command.linear.z],
        })
        self.status_pub.publish(status)

    def _control(self) -> None:
        now = self.get_clock().now()
        dt = clamp(
            (now - self.previous_control_time).nanoseconds * 1e-9,
            0.001,
            0.1,
        )
        self.previous_control_time = now
        command = Twist()
        if not self._fresh_observation(now):
            self.phase = 'WAITING_FOR_RELATIVE_STATE'
            self.x_controller.reset()
            self.y_controller.reset()
            self.command_pub.publish(command)
            self._publish_status(None, command, [0.0, 0.0])
            return

        observation = self.observation
        if self.first_ready_time is None:
            self.first_ready_time = now
        correction_x, setpoint_x = self.x_controller.update(
            observation.rel_p_x, observation.rel_v_x, dt)
        correction_y, setpoint_y = self.y_controller.update(
            observation.rel_p_y, observation.rel_v_y, dt)
        correction_world_x, correction_world_y = rotate_to_world(
            correction_x, correction_y, observation.uav_yaw)
        command.linear.x = observation.platform_v_x + correction_world_x
        command.linear.y = observation.platform_v_y + correction_world_y
        maximum = float(self.get_parameter('max_horizontal_speed').value)
        horizontal_speed = math.hypot(command.linear.x, command.linear.y)
        if horizontal_speed > maximum:
            scale = maximum / horizontal_speed
            command.linear.x *= scale
            command.linear.y *= scale

        height = max(0.0, -observation.rel_p_z)
        horizontal_error = math.hypot(
            observation.rel_p_x, observation.rel_p_y)
        elapsed = (now - self.first_ready_time).nanoseconds * 1e-9
        settle_time = float(self.get_parameter('tracking_settle_time').value)
        if self.phase == 'STOPPED':
            command = Twist()
        elif height <= float(self.get_parameter('touchdown_height').value) \
                and horizontal_error <= float(
                    self.get_parameter('touchdown_radius').value):
            # The upstream README explicitly leaves motor deactivation to the
            # integrator.  For this velocity-controlled simulator, matching
            # platform velocity and commanding no descent is the safe analog.
            self.phase = 'TOUCHDOWN_HOLD'
            command.linear.x = observation.platform_v_x
            command.linear.y = observation.platform_v_y
            command.linear.z = 0.0
        elif height <= float(self.get_parameter('low_altitude_gate').value) \
                and horizontal_error > float(
                    self.get_parameter('low_altitude_radius').value):
            self.phase = 'LOW_ALTITUDE_REALIGN'
            command.linear.z = min(
                0.12,
                float(self.get_parameter('max_vertical_speed').value),
            )
        elif self.landing_requested and elapsed >= settle_time:
            self.phase = 'CONSTANT_RATE_DESCENT'
            command.linear.z = -abs(float(
                self.get_parameter('descent_speed').value))
        else:
            self.phase = 'MOVING_PLATFORM_TRACK'
            desired_height = float(
                self.get_parameter('initial_height').value)
            vertical = float(
                self.get_parameter('vertical_hold_kp').value) \
                * (desired_height - height)
            limit = float(self.get_parameter('max_vertical_speed').value)
            command.linear.z = clamp(vertical, -limit, limit)

        self.command_pub.publish(command)
        self._publish_status(
            observation, command, [setpoint_x, setpoint_y])


def main(args=None):
    rclpy.init(args=args)
    node = RepoLandingController()
    try:
        rclpy.spin(node)
    finally:
        node.command_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
