"""Online Double-Q training node for repository-compatible platform landing."""

from __future__ import annotations

from collections import deque
import json
import math
import os
from pathlib import Path
import random
import subprocess

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from .repo_landing_core import (
    DoubleQLearner,
    MultiResolutionDiscretizer,
    RepoObservation,
    apply_increment,
    clamp,
    rotate_to_world,
)


class RepoLandingTrainer(Node):
    """Train the repository's 1-D controller and compose it over x and y."""

    ACTION_MAX = math.radians(21.37723)
    ACTION_DELTA = math.radians(7.12574)
    CONTROL_HZ = 22.92

    def __init__(self):
        super().__init__('repo_landing_trainer')
        self.declare_parameter('observation_topic', '/repo_landing/observation')
        self.declare_parameter('cmd_topic', '/model/x500_flow_tag_0/cmd_vel')
        self.declare_parameter('model_name', 'x500_flow_tag_0')
        self.declare_parameter('policy_path', str(
            Path.home() / 'husarion_ws' / 'maps'
            / 'repo_landing_double_q.json'))
        self.declare_parameter('auto_resume', True)
        self.declare_parameter('max_episodes', 50000)
        self.declare_parameter('episode_seconds', 20.0)
        self.declare_parameter('initial_altitude', 4.0)
        self.declare_parameter('descent_speed', 0.10)
        self.declare_parameter('initial_sigma_x', 1.5)
        self.declare_parameter('tilt_to_velocity_gain', 4.0)
        self.declare_parameter('max_horizontal_speed', 2.0)
        self.declare_parameter('seed', -1)

        seed_value = int(self.get_parameter('seed').value)
        seed = None if seed_value < 0 else seed_value
        self.random = random.Random(seed)
        self.learner = DoubleQLearner(seed=seed)
        self.discretizer = MultiResolutionDiscretizer()
        self.policy_path = os.path.expanduser(str(
            self.get_parameter('policy_path').value))
        if bool(self.get_parameter('auto_resume').value) \
                and os.path.isfile(self.policy_path):
            self.learner.load(self.policy_path)
            self.discretizer.set_curriculum_step(
                self.learner.curriculum_step)
            self.get_logger().info(
                f'Resumed policy at episode {self.learner.episode}: '
                f'{self.policy_path}')

        self.command_pub = self.create_publisher(
            Twist, str(self.get_parameter('cmd_topic').value), 10)
        self.status_pub = self.create_publisher(
            String, '/repo_landing/training_status', 10)
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter('observation_topic').value),
            self._observation_callback,
            20,
        )

        self.observation = None
        self.last_observation_time = None
        self.phase = 'WAITING_FOR_RELATIVE_STATE'
        self.action_x = 0.0
        self.action_y = 0.0
        self.previous_state_x = None
        self.previous_state_y = None
        self.previous_action_x = None
        self.previous_action_y = None
        self.previous_shaping_x = None
        self.previous_shaping_y = None
        self.previous_step_x = 0
        self.previous_step_y = 0
        self.episode_start = None
        self.reset_until = None
        self.success_hold_start = None
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.success_window = deque(maxlen=100)
        self.last_loss = 0.0
        self.previous_timer = self.get_clock().now()
        self.create_timer(1.0 / self.CONTROL_HZ, self._step)
        self.get_logger().info(
            'Repository-compatible Double-Q trainer waiting for state. '
            'Training starts automatically; no button is required.')

    def _observation_callback(self, message: Float64MultiArray) -> None:
        try:
            self.observation = RepoObservation.from_array(message.data)
            self.last_observation_time = self.get_clock().now()
        except ValueError as error:
            self.get_logger().error(str(error))

    def _state(self, axis: str):
        if axis == 'x':
            values = (
                self.observation.rel_p_x,
                self.observation.rel_v_x,
                self.observation.rel_a_x,
                self.action_x,
            )
        else:
            # This sign inversion is the same symmetry operation used by the
            # upstream 2-D test code before applying the longitudinal policy.
            values = (
                -self.observation.rel_p_y,
                -self.observation.rel_v_y,
                -self.observation.rel_a_y,
                self.action_y,
            )
        return self.discretizer.state(
            *values, self.ACTION_MAX, self.ACTION_DELTA)

    @staticmethod
    def _normal(value: float, maximum: float) -> float:
        return clamp(value / maximum, -1.0, 1.0)

    def _shaping(self, axis: str, action: float) -> float:
        if axis == 'x':
            position = self.observation.rel_p_x
            velocity = self.observation.rel_v_x
        else:
            position = -self.observation.rel_p_y
            velocity = -self.observation.rel_v_y
        return (
            -100.0 * abs(self._normal(position, 4.5))
            - 10.0 * abs(self._normal(velocity, 3.39411))
            - 1.55 * abs(action / self.ACTION_MAX)
        )

    def _reward(self, axis: str, action: float, state,
                previous_shaping, previous_step: int) -> tuple[float, float]:
        shaping = self._shaping(axis, action)
        if previous_shaping is None:
            return 0.0, shaping
        velocity_limit = MultiResolutionDiscretizer.STEPS['rel_v'][state[0]]
        duration = -6.0 * (1.0 / self.CONTROL_HZ) * velocity_limit
        reward = clamp(shaping - previous_shaping, -5.0, 5.0) + duration
        if state[0] > previous_step:
            reward += 2.6
        elif state[0] < previous_step:
            reward -= 2.6
        return reward, shaping

    def _issue_command(self) -> None:
        command = Twist()
        gain = float(self.get_parameter('tilt_to_velocity_gain').value)
        correction_x = gain * self.action_x
        correction_y = -gain * self.action_y
        world_x, world_y = rotate_to_world(
            correction_x, correction_y, self.observation.uav_yaw)
        command.linear.x = self.observation.platform_v_x + world_x
        command.linear.y = self.observation.platform_v_y + world_y
        maximum = float(self.get_parameter('max_horizontal_speed').value)
        speed = math.hypot(command.linear.x, command.linear.y)
        if speed > maximum:
            scale = maximum / speed
            command.linear.x *= scale
            command.linear.y *= scale
        command.linear.z = -abs(float(
            self.get_parameter('descent_speed').value))
        self.command_pub.publish(command)

    def _teleport_uav(self) -> bool:
        sigma = float(self.get_parameter('initial_sigma_x').value)
        relative_x = clamp(self.random.gauss(0.0, sigma), -4.4, 4.4)
        x = self.observation.platform_x - relative_x
        y = self.observation.platform_y
        z = self.observation.platform_z + float(
            self.get_parameter('initial_altitude').value)
        executable = '/opt/ros/jazzy/lib/ros_gz_sim/set_entity_pose'
        command = [
            executable,
            '--name', str(self.get_parameter('model_name').value),
            '--type', '6',
            '--pos', str(x), str(y), str(z),
            '--euler', '0', '0', str(self.observation.uav_yaw),
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=4.0,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f'episode reset failed: {error}')
            return False
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            self.get_logger().error(
                f'episode reset returned {result.returncode}: {detail}')
            return False
        return True

    def _reset_episode(self, now) -> None:
        self.command_pub.publish(Twist())
        if not self._teleport_uav():
            self.phase = 'RESET_FAILED'
            return
        self.action_x = 0.0
        self.action_y = 0.0
        self.previous_state_x = None
        self.previous_state_y = None
        self.previous_action_x = None
        self.previous_action_y = None
        self.previous_shaping_x = None
        self.previous_shaping_y = None
        self.previous_step_x = 0
        self.previous_step_y = 0
        self.success_hold_start = None
        self.episode_reward = 0.0
        self.episode_steps = 0
        self.episode_start = None
        self.reset_until = now + Duration(seconds=0.5)
        self.phase = 'RESETTING_EPISODE'

    def _finish_episode(self, success: bool, reason: str, now) -> None:
        self.success_window.append(1 if success else 0)
        self.learner.end_episode()
        success_rate = sum(self.success_window) / len(self.success_window)
        if len(self.success_window) == 100 and success_rate >= 0.96 \
                and self.learner.curriculum_step < 4:
            self.learner.curriculum_step += 1
            self.discretizer.set_curriculum_step(
                self.learner.curriculum_step)
            self.success_window.clear()
            self.get_logger().info(
                'Advanced to curriculum step '
                f'{self.learner.curriculum_step}.')
        Path(self.policy_path).parent.mkdir(parents=True, exist_ok=True)
        self.learner.save(self.policy_path)
        self.get_logger().info(
            f'episode={self.learner.episode} reason={reason} '
            f'reward={self.episode_reward:.3f} '
            f'epsilon={self.learner.epsilon:.3f} '
            f'success_rate={success_rate:.1%}')
        self._reset_episode(now)

    def _publish_status(self, now, reason='') -> None:
        elapsed = 0.0
        if self.episode_start is not None:
            elapsed = max(
                0.0, (now - self.episode_start).nanoseconds * 1e-9)
        status = String()
        status.data = json.dumps({
            'mode': self.phase,
            'method': 'repository_multiresolution_double_q',
            'episode': self.learner.episode + 1,
            'episode_time_s': elapsed,
            'episode_limit_s': float(
                self.get_parameter('episode_seconds').value),
            'episode_reward': self.episode_reward,
            'epsilon': self.learner.epsilon,
            'curriculum_step': self.learner.curriculum_step,
            'learned_states': len(self.learner.q_a),
            'last_loss': self.last_loss,
            'success_rate_last_100': (
                sum(self.success_window) / len(self.success_window)
                if self.success_window else 0.0),
            'policy_path': self.policy_path,
            'detail': reason,
        })
        self.status_pub.publish(status)

    def _step(self) -> None:
        now = self.get_clock().now()
        if self.observation is None or self.last_observation_time is None \
                or (now - self.last_observation_time).nanoseconds * 1e-9 > 0.5:
            self.phase = 'WAITING_FOR_RELATIVE_STATE'
            self.command_pub.publish(Twist())
            self._publish_status(now)
            return
        if self.learner.episode >= int(
                self.get_parameter('max_episodes').value):
            self.phase = 'TRAINING_COMPLETE'
            self.command_pub.publish(Twist())
            self._publish_status(now)
            return
        if self.episode_start is None and self.reset_until is None:
            self._reset_episode(now)
            self._publish_status(now)
            return
        if self.reset_until is not None:
            if now < self.reset_until:
                self.command_pub.publish(Twist())
                self._publish_status(now)
                return
            self.reset_until = None
            self.episode_start = now
            self.phase = 'TRAINING'

        state_x = self._state('x')
        state_y = self._state('y')
        reward_x, shaping_x = self._reward(
            'x', self.action_x, state_x,
            self.previous_shaping_x, self.previous_step_x)
        reward_y, shaping_y = self._reward(
            'y', self.action_y, state_y,
            self.previous_shaping_y, self.previous_step_y)
        reward = (reward_x + reward_y) / 2.0
        height = max(0.0, -self.observation.rel_p_z)
        horizontal_failure = (
            abs(self.observation.rel_p_x) >= 4.5
            or abs(self.observation.rel_p_y) >= 4.5)
        timed_out = (
            (now - self.episode_start).nanoseconds * 1e-9
            >= float(self.get_parameter('episode_seconds').value))
        minimum_altitude = height <= 0.3

        centered = (
            state_x[0] == self.learner.curriculum_step
            and state_y[0] == self.learner.curriculum_step
            and state_x[1] == 1 and state_x[2] == 1
            and state_y[1] == 1 and state_y[2] == 1)
        if centered:
            if self.success_hold_start is None:
                self.success_hold_start = now
        else:
            self.success_hold_start = None
        success = bool(
            self.success_hold_start is not None
            and (now - self.success_hold_start).nanoseconds * 1e-9 >= 1.0)
        terminal = horizontal_failure or timed_out or minimum_altitude or success

        if self.previous_state_x is not None:
            self.last_loss = self.learner.update(
                self.previous_state_x,
                self.previous_action_x,
                reward_x,
                state_x,
                terminal,
            )
            self.last_loss = self.learner.update(
                self.previous_state_y,
                self.previous_action_y,
                reward_y,
                state_y,
                terminal,
            )
        self.episode_reward += reward
        self.episode_steps += 1
        if terminal:
            reason = (
                'success' if success else
                'horizontal_limit' if horizontal_failure else
                'minimum_altitude' if minimum_altitude else
                'time_limit')
            self._finish_episode(success, reason, now)
            self._publish_status(now, reason)
            return

        chosen_x = self.learner.choose(state_x, training=True)
        chosen_y = self.learner.choose(state_y, training=True)
        self.previous_state_x = state_x
        self.previous_state_y = state_y
        self.previous_action_x = chosen_x
        self.previous_action_y = chosen_y
        self.previous_shaping_x = shaping_x
        self.previous_shaping_y = shaping_y
        self.previous_step_x = state_x[0]
        self.previous_step_y = state_y[0]
        self.action_x = apply_increment(
            self.action_x, chosen_x, self.ACTION_DELTA, self.ACTION_MAX)
        self.action_y = apply_increment(
            self.action_y, chosen_y, self.ACTION_DELTA, self.ACTION_MAX)
        self._issue_command()
        self._publish_status(now)


def main(args=None):
    rclpy.init(args=args)
    node = RepoLandingTrainer()
    try:
        rclpy.spin(node)
    finally:
        node.command_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
