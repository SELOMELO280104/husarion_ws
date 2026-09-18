"""Online Q-learning search node driven by the existing tracker status."""

from __future__ import annotations

import json
import time
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt8

from .rl_search_trainer import QSearchAgent
from .rl_search_environment import SearchState


class RLSearchTrainingNode(Node):
    def __init__(self):
        super().__init__('rl_search_training')
        self.declare_parameter('enabled', False)
        self.declare_parameter('episodes', 1000)
        self.declare_parameter('table_path', '/tmp/uav_search_q_table.json')
        self.agent = QSearchAgent(seed=7)
        self.enabled = bool(self.get_parameter('enabled').value)
        self.previous = None
        self.pixel_error = None
        self.last_action = 0
        self.episodes = 0
        self.successes = 0
        self.reward_sum = 0.0
        self.positive_reward = 0.0
        self.penalty_sum = 0.0
        self.step_count = 0
        self.was_visible = False
        self.centered_seconds = 0.0
        self.last_tick = time.monotonic()
        self.declare_parameter('center_tolerance_pixels', 20.0)
        self.declare_parameter('visible_reward', 0.1)
        self.declare_parameter('invisible_penalty', 0.2)
        self.declare_parameter('center_reward', 5.0)
        self.training_start = time.monotonic()
        self.episode_start = self.training_start
        self.declare_parameter('training_minutes', 5.0)
        self.declare_parameter('world_name', 'orchard')
        self.declare_parameter('resume', True)
        table_path = Path(str(self.get_parameter('table_path').value))
        if bool(self.get_parameter('resume').value) and table_path.exists():
            try:
                self.agent.load(table_path)
                self.get_logger().info(f'Resumed Q-table from {table_path}')
            except (OSError, ValueError, TypeError):
                self.get_logger().warning('Existing Q-table could not be loaded; starting fresh')
        self.dashboard_pub = self.create_publisher(String,
                                                    '/rl_search/training_status', 10)
        self.action_pub = self.create_publisher(UInt8,
                                                 '/rl_search_ros_adapter/action', 10)
        self.enable_pub = self.create_publisher(Bool,
                                                 '/rl_search_ros_adapter/enable', 10)
        self.create_subscription(String, '/uav/tag_follower/status', self.status, 10)
        self.create_timer(0.2, self.tick)

    def status(self, message):
        try:
            data = json.loads(message.data)
            self.previous = SearchState(
                float((data.get('marker_relative_xy_m') or [0, 0])[0]),
                float((data.get('marker_relative_xy_m') or [0, 0])[1]),
                float(data.get('depth_m') or 8.0),
                bool(data.get('tag_visible', False)),
                60.0)
            pixel = data.get('pixel_error')
            self.pixel_error = tuple(pixel) if pixel else None
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            self.previous = None

    def tick(self):
        self.enable_pub.publish(Bool(data=self.enabled))
        if not self.enabled or self.previous is None:
            return
        now = time.monotonic()
        if now - self.episode_start >= 60.0 * float(
                self.get_parameter('training_minutes').value):
            self.episodes += 1
            self.agent.end_episode()
            self._reset_world()
            self.episode_start = now
            self.was_visible = False
            self.latest_action = 0
            return
        state = self.previous
        action = self.agent.choose(state)
        now = time.monotonic()
        dt = min(1.0, max(0.0, now - self.last_tick))
        self.last_tick = now
        tolerance = float(self.get_parameter('center_tolerance_pixels').value)
        centered = (state.marker_visible and self.pixel_error is not None
                    and max(abs(float(self.pixel_error[0])),
                            abs(float(self.pixel_error[1]))) <= tolerance)
        if centered:
            self.centered_seconds += dt
            multiplier = max(1, int(self.centered_seconds // 5.0))
            reward = float(self.get_parameter('center_reward').value) * multiplier
        elif state.marker_visible:
            self.centered_seconds = 0.0
            reward = float(self.get_parameter('visible_reward').value)
        else:
            self.centered_seconds = 0.0
            reward = -float(self.get_parameter('invisible_penalty').value)
        self.reward_sum += reward
        if reward >= 0.0:
            self.positive_reward += reward
        else:
            self.penalty_sum += -reward
        self.step_count += 1
        self.agent.update(state, action, reward, state, state.marker_visible)
        self.action_pub.publish(UInt8(data=action))
        if state.marker_visible and not self.was_visible:
            self.episodes += 1
            self.successes += 1
            self.agent.end_episode()
            self.episode_start = time.monotonic()
        self.was_visible = state.marker_visible
        self.agent.save(str(self.get_parameter('table_path').value))
        now = time.monotonic()
        self.dashboard_pub.publish(String(data=json.dumps({
            'episodes': self.episodes,
            'success_rate': (self.successes / self.episodes
                             if self.episodes else 0.0),
            'average_reward_per_step': (self.reward_sum / self.step_count
                                        if self.step_count else 0.0),
            'cumulative_reward': self.positive_reward,
            'cumulative_penalty': self.penalty_sum,
            'net_reward': self.positive_reward - self.penalty_sum,
            'reward_rate_per_minute': self.positive_reward / max(
                1.0, (now - self.training_start) / 60.0),
            'penalty_rate_per_minute': self.penalty_sum / max(
                1.0, (now - self.training_start) / 60.0),
            'centered_seconds': self.centered_seconds,
            'last_reward': reward,
            'epsilon': self.agent.epsilon,
            'q_states': len(self.agent.q),
            'training_elapsed_seconds': now - self.training_start,
            'episode_elapsed_seconds': now - self.episode_start,
            'training_target_seconds': 60.0 * float(
                self.get_parameter('training_minutes').value),
            'table_path': str(self.get_parameter('table_path').value),
        })))

    def _reset_world(self):
        world = str(self.get_parameter('world_name').value)
        command = [
            'gz', 'service', '-s', f'/world/{world}/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '-p', 'reset: {all: true}',
        ]
        try:
            subprocess.run(command, check=False, timeout=3.0,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except (OSError, subprocess.TimeoutExpired):
            self.get_logger().warning('Gazebo reset command failed')


def main(args=None):
    rclpy.init(args=args)
    node = RLSearchTrainingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
