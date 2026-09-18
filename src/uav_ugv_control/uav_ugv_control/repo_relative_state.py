"""Publish moving-platform state in the UAV stability frame.

Gazebo's exact model pose is the simulation replacement for the Vicon pose
used by the upstream project.  No GPS, marker detector, or world-frame
position is consumed by the controller after this adapter computes the
relative observation.
"""

from __future__ import annotations

import json
import math

import rclpy
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from tf2_msgs.msg import TFMessage

from .repo_landing_core import (
    FilteredDerivative,
    RepoObservation,
    rotate_to_stability,
)


def quaternion_yaw(quaternion) -> float:
    """Return yaw from a geometry quaternion."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y)
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z)
    return math.atan2(sin_yaw, cos_yaw)


class RepoRelativeState(Node):
    """Convert Gazebo UAV pose and Panther odometry to relative state."""

    def __init__(self):
        super().__init__('repo_relative_state')
        self.declare_parameter('uav_pose_topic', '/repo_landing/uav_pose')
        self.declare_parameter('platform_odom_topic', '/panther/native/odometry')
        self.declare_parameter('observation_topic', '/repo_landing/observation')
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('acceleration_cutoff_hz', 0.3)
        self.declare_parameter('data_timeout', 1.0)
        self.declare_parameter('platform_pose_offset_x', 0.0)
        self.declare_parameter('platform_pose_offset_y', 0.0)

        self.uav = None
        self.platform = None
        self.previous_uav = None
        self.uav_velocity = (0.0, 0.0, 0.0)
        self.last_uav_receive = None
        self.last_platform_receive = None
        cutoff = float(self.get_parameter('acceleration_cutoff_hz').value)
        self.accel_x = FilteredDerivative(cutoff)
        self.accel_y = FilteredDerivative(cutoff)
        self.accel_z = FilteredDerivative(cutoff)
        self.previous_publish_time = None

        self.observation_pub = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter('observation_topic').value),
            10,
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, '/repo_landing/relative_pose', 10)
        self.twist_pub = self.create_publisher(
            TwistStamped, '/repo_landing/relative_twist', 10)
        self.acceleration_pub = self.create_publisher(
            AccelStamped, '/repo_landing/relative_acceleration', 10)
        self.ready_pub = self.create_publisher(
            Bool, '/repo_landing/state_ready', 10)
        self.status_pub = self.create_publisher(
            String, '/repo_landing/relative_state_status', 10)

        self.create_subscription(
            TFMessage,
            str(self.get_parameter('uav_pose_topic').value),
            self._uav_callback,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('platform_odom_topic').value),
            self._platform_callback,
            20,
        )
        rate = max(1.0, float(self.get_parameter('publish_hz').value))
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Repository relative-state adapter waiting for Gazebo UAV pose '
            'and Panther odometry.')

    def _uav_callback(self, message: TFMessage) -> None:
        if not message.transforms:
            return
        transform = message.transforms[0]
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        translation = transform.transform.translation
        position = (
            float(translation.x),
            float(translation.y),
            float(translation.z),
        )
        if self.previous_uav is not None:
            previous_stamp, previous_position = self.previous_uav
            dt = stamp - previous_stamp
            if 1e-4 <= dt <= 0.5:
                raw = tuple(
                    (position[index] - previous_position[index]) / dt
                    for index in range(3))
                # Small smoothing prevents pose quantization from dominating
                # the derived relative acceleration.
                self.uav_velocity = tuple(
                    0.65 * self.uav_velocity[index] + 0.35 * raw[index]
                    for index in range(3))
        self.previous_uav = (stamp, position)
        self.uav = (
            position,
            quaternion_yaw(transform.transform.rotation),
            transform.transform.rotation,
        )
        self.last_uav_receive = now

    def _platform_callback(self, message: Odometry) -> None:
        now = self.get_clock().now()
        yaw = quaternion_yaw(message.pose.pose.orientation)
        body_v = message.twist.twist.linear
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        world_v_x = cosine * body_v.x - sine * body_v.y
        world_v_y = sine * body_v.x + cosine * body_v.y
        offset_x = float(
            self.get_parameter('platform_pose_offset_x').value)
        offset_y = float(
            self.get_parameter('platform_pose_offset_y').value)
        self.platform = (
            (
                float(message.pose.pose.position.x) + offset_x,
                float(message.pose.pose.position.y) + offset_y,
                float(message.pose.pose.position.z),
            ),
            yaw,
            (float(world_v_x), float(world_v_y), float(body_v.z)),
        )
        self.last_platform_receive = now

    def _fresh(self, now) -> bool:
        if self.last_uav_receive is None or self.last_platform_receive is None:
            return False
        timeout = float(self.get_parameter('data_timeout').value)
        return (
            (now - self.last_uav_receive).nanoseconds * 1e-9 <= timeout
            and (now - self.last_platform_receive).nanoseconds * 1e-9
            <= timeout
        )

    def _publish(self) -> None:
        now = self.get_clock().now()
        ready = self.uav is not None and self.platform is not None \
            and self._fresh(now)
        ready_message = Bool()
        ready_message.data = bool(ready)
        self.ready_pub.publish(ready_message)
        if not ready:
            status = String()
            status.data = json.dumps({
                'ready': False,
                'uav_pose_received': self.uav is not None,
                'platform_odometry_received': self.platform is not None,
            })
            self.status_pub.publish(status)
            return

        uav_position, uav_yaw, uav_quaternion = self.uav
        platform_position, platform_yaw, platform_velocity = self.platform
        relative_position_world = tuple(
            platform_position[index] - uav_position[index]
            for index in range(3))
        relative_velocity_world = tuple(
            platform_velocity[index] - self.uav_velocity[index]
            for index in range(3))
        rel_p_x, rel_p_y = rotate_to_stability(
            relative_position_world[0], relative_position_world[1], uav_yaw)
        rel_v_x, rel_v_y = rotate_to_stability(
            relative_velocity_world[0], relative_velocity_world[1], uav_yaw)
        rel_p_z = relative_position_world[2]
        rel_v_z = relative_velocity_world[2]

        stamp = now.nanoseconds * 1e-9
        if self.previous_publish_time is None:
            dt = 0.0
        else:
            dt = stamp - self.previous_publish_time
        self.previous_publish_time = stamp
        rel_a_x = self.accel_x.update(rel_v_x, dt)
        rel_a_y = self.accel_y.update(rel_v_y, dt)
        rel_a_z = self.accel_z.update(rel_v_z, dt)

        observation = RepoObservation(
            rel_p_x=rel_p_x,
            rel_p_y=rel_p_y,
            rel_p_z=rel_p_z,
            rel_v_x=rel_v_x,
            rel_v_y=rel_v_y,
            rel_v_z=rel_v_z,
            rel_a_x=rel_a_x,
            rel_a_y=rel_a_y,
            rel_a_z=rel_a_z,
            uav_x=uav_position[0],
            uav_y=uav_position[1],
            uav_z=uav_position[2],
            uav_yaw=uav_yaw,
            platform_x=platform_position[0],
            platform_y=platform_position[1],
            platform_z=platform_position[2],
            platform_v_x=platform_velocity[0],
            platform_v_y=platform_velocity[1],
        )
        array = Float64MultiArray()
        array.data = observation.to_array()
        self.observation_pub.publish(array)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'uav_stability_axes'
        pose.pose.position.x = rel_p_x
        pose.pose.position.y = rel_p_y
        pose.pose.position.z = rel_p_z
        relative_yaw = platform_yaw - uav_yaw
        pose.pose.orientation.z = math.sin(relative_yaw / 2.0)
        pose.pose.orientation.w = math.cos(relative_yaw / 2.0)
        self.pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x = rel_v_x
        twist.twist.linear.y = rel_v_y
        twist.twist.linear.z = rel_v_z
        self.twist_pub.publish(twist)

        acceleration = AccelStamped()
        acceleration.header = pose.header
        acceleration.accel.linear.x = rel_a_x
        acceleration.accel.linear.y = rel_a_y
        acceleration.accel.linear.z = rel_a_z
        self.acceleration_pub.publish(acceleration)

        status = String()
        status.data = json.dumps({
            'ready': True,
            'frame': 'uav_stability_axes',
            'source': 'gazebo_ground_truth_vicon_equivalent',
            'relative_position': [rel_p_x, rel_p_y, rel_p_z],
            'relative_velocity': [rel_v_x, rel_v_y, rel_v_z],
            'relative_acceleration': [rel_a_x, rel_a_y, rel_a_z],
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = RepoRelativeState()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
