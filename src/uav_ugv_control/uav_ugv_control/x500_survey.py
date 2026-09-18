import math

import rclpy
from rclpy.executors import ExternalShutdownException

from uav_ugv_control.x500_follow_ugv import X500FollowUGV


class X500Survey(X500FollowUGV):
    """Fly a configurable lawnmower survey using PX4 offboard position control."""

    def __init__(self):
        super().__init__()
        self.timer.cancel()

        self.declare_parameter('survey_x_min', -30.0)
        self.declare_parameter('survey_x_max', 15.0)
        self.declare_parameter('survey_y_min', -30.0)
        self.declare_parameter('survey_y_max', 15.0)
        self.declare_parameter('lane_spacing', 5.0)
        self.declare_parameter('waypoint_tolerance', 1.0)
        self.declare_parameter('waypoints', '')

        self.survey_x_min = float(self.get_parameter('survey_x_min').value)
        self.survey_x_max = float(self.get_parameter('survey_x_max').value)
        self.survey_y_min = float(self.get_parameter('survey_y_min').value)
        self.survey_y_max = float(self.get_parameter('survey_y_max').value)
        self.lane_spacing = max(float(self.get_parameter('lane_spacing').value), 0.5)
        self.waypoint_tolerance = max(
            float(self.get_parameter('waypoint_tolerance').value), 0.2
        )

        explicit_waypoints = str(self.get_parameter('waypoints').value).strip()
        self.waypoints = (
            self._parse_waypoints(explicit_waypoints)
            if explicit_waypoints
            else self._build_lawnmower_path()
        )
        self.waypoint_index = 0
        self.survey_complete = False
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 1.0), self._tick)

        self.get_logger().info(
            f'Automatic orchard survey has {len(self.waypoints)} waypoints at '
            f'{self.altitude:.1f} m altitude: '
            + ' -> '.join(f'({x:.1f}, {y:.1f})' for x, y in self.waypoints)
        )

    def _parse_waypoints(self, waypoint_text):
        """Parse semicolon-separated world-frame x,y waypoint pairs."""
        waypoints = []
        try:
            for pair in waypoint_text.split(';'):
                x_text, y_text = pair.split(',')
                waypoints.append((float(x_text), float(y_text)))
        except ValueError as exc:
            raise ValueError(
                "waypoints must use the form 'x1,y1;x2,y2;...'"
            ) from exc
        if not waypoints:
            raise ValueError('at least one survey waypoint is required')
        return waypoints

    def _build_lawnmower_path(self):
        waypoints = []
        y = self.survey_y_min
        reverse = False
        while y <= self.survey_y_max + 1e-6:
            start_x = self.survey_x_max if reverse else self.survey_x_min
            end_x = self.survey_x_min if reverse else self.survey_x_max
            waypoints.extend([(start_x, y), (end_x, y)])
            reverse = not reverse
            y += self.lane_spacing
        return waypoints

    def _enu_to_ned(self, x, y, z):
        north = y - self.uav_origin_y
        east = x - self.uav_origin_x
        down = -(z - self.uav_origin_z)
        return north, east, down

    def _current_world_position(self):
        if not self._local_position_ready():
            return None
        return (
            self.uav_origin_x + float(self.vehicle_local_position.y),
            self.uav_origin_y + float(self.vehicle_local_position.x),
            self.uav_origin_z - float(self.vehicle_local_position.z),
        )

    def _survey_target(self):
        target_x, target_y = self.waypoints[self.waypoint_index]
        target_z = self.uav_origin_z + abs(self.altitude)
        current = self._current_world_position()

        if current is not None:
            distance = math.hypot(target_x - current[0], target_y - current[1])
            if distance <= self.waypoint_tolerance:
                if self.waypoint_index < len(self.waypoints) - 1:
                    self.waypoint_index += 1
                    target_x, target_y = self.waypoints[self.waypoint_index]
                    self.get_logger().info(
                        f'Advancing to survey waypoint '
                        f'{self.waypoint_index + 1}/{len(self.waypoints)}: '
                        f'world x={target_x:.1f}, y={target_y:.1f}'
                    )
                elif not self.survey_complete:
                    self.survey_complete = True
                    self.get_logger().info(
                        'Orchard survey complete; holding at the final waypoint'
                    )

        next_x, next_y = self.waypoints[self.waypoint_index]
        yaw_enu = math.atan2(next_y - target_y, next_x - target_x)
        if self.waypoint_index > 0:
            previous_x, previous_y = self.waypoints[self.waypoint_index - 1]
            yaw_enu = math.atan2(next_y - previous_y, next_x - previous_x)
        yaw_ned = math.atan2(
            math.sin(math.pi / 2.0 - yaw_enu),
            math.cos(math.pi / 2.0 - yaw_enu),
        )
        return self._enu_to_ned(target_x, target_y, target_z), yaw_ned

    def _maybe_send_mode_commands(self):
        """Keep requesting arm/offboard; PX4 remains the final safety authority."""
        if self.setpoint_count < self.startup_setpoints:
            self.setpoint_count += 1
            return
        if not self._local_position_ready():
            self.get_logger().warn(
                'Waiting for valid PX4 local position before automatic takeoff',
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
        if self.follow_unlocked:
            ned_position, yaw_ned = self._survey_target()
        else:
            ned_position, yaw_ned = self._takeoff_hold_target()

        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint(ned_position, yaw_ned)
        self._maybe_send_mode_commands()
        self._update_follow_unlock()

        phase = 'survey' if self.follow_unlocked else 'takeoff'
        self.get_logger().info(
            f'X500 {phase}: waypoint {self.waypoint_index + 1}/{len(self.waypoints)}, '
            f'offboard={self._is_offboard()} armed={self._is_armed()}',
            throttle_duration_sec=1.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = X500Survey()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
