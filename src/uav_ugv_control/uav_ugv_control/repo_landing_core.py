"""Pure control and learning primitives for the repository-compatible port.

The upstream project represents motion in the UAV stability frame, trains a
one-dimensional tabular controller, and applies the same controller to the
longitudinal and lateral axes.  This module keeps those ideas independent of
ROS so that the math can be tested without running Gazebo.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import json
import math
import random


def clamp(value: float, lower: float, upper: float) -> float:
    """Clamp ``value`` to the closed interval ``[lower, upper]``."""
    return max(lower, min(upper, value))


def rotate_to_stability(x: float, y: float, yaw: float) -> tuple[float, float]:
    """Rotate an ENU vector into the yaw-only UAV stability frame."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x + sine * y, -sine * x + cosine * y


def rotate_to_world(x: float, y: float, yaw: float) -> tuple[float, float]:
    """Rotate a stability-frame vector into the ENU world frame."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * x - sine * y, sine * x + cosine * y


@dataclass
class RepoObservation:
    """Continuous state used by the repository-style controllers."""

    rel_p_x: float = 0.0
    rel_p_y: float = 0.0
    rel_p_z: float = 0.0
    rel_v_x: float = 0.0
    rel_v_y: float = 0.0
    rel_v_z: float = 0.0
    rel_a_x: float = 0.0
    rel_a_y: float = 0.0
    rel_a_z: float = 0.0
    uav_x: float = 0.0
    uav_y: float = 0.0
    uav_z: float = 0.0
    uav_yaw: float = 0.0
    platform_x: float = 0.0
    platform_y: float = 0.0
    platform_z: float = 0.0
    platform_v_x: float = 0.0
    platform_v_y: float = 0.0

    def to_array(self) -> list[float]:
        """Return the stable wire order used by the ROS adapter."""
        return [
            self.rel_p_x, self.rel_p_y, self.rel_p_z,
            self.rel_v_x, self.rel_v_y, self.rel_v_z,
            self.rel_a_x, self.rel_a_y, self.rel_a_z,
            self.uav_x, self.uav_y, self.uav_z, self.uav_yaw,
            self.platform_x, self.platform_y, self.platform_z,
            self.platform_v_x, self.platform_v_y,
        ]

    @classmethod
    def from_array(cls, values) -> 'RepoObservation':
        """Decode a ROS array and reject incomplete state messages."""
        values = list(values)
        if len(values) != 18:
            raise ValueError(
                f'repository observation needs 18 values, got {len(values)}')
        return cls(*[float(value) for value in values])


class FilteredDerivative:
    """First-order Butterworth derivative used by the upstream observer."""

    def __init__(self, cutoff_hz: float = 0.3):
        self.cutoff_hz = float(cutoff_hz)
        self.previous = None
        self.filtered = 0.0

    def reset(self) -> None:
        self.previous = None
        self.filtered = 0.0

    def update(self, value: float, dt: float) -> float:
        if self.previous is None or dt <= 0.0:
            self.previous = float(value)
            return self.filtered
        dt = clamp(float(dt), 1e-4, 0.5)
        raw = (float(value) - self.previous) / dt
        decay = math.exp(-2.0 * math.pi * self.cutoff_hz * dt)
        self.filtered = decay * self.filtered + (1.0 - decay) * raw
        self.previous = float(value)
        return self.filtered


class CascadedPIAxis:
    """Outer position P loop feeding an inner relative-velocity PI loop."""

    def __init__(
        self,
        position_kp: float = 3.0,
        velocity_kp: float = 0.8,
        velocity_ki: float = 0.15,
        velocity_limit: float = 3.39,
        correction_limit: float = 2.0,
        windup_limit: float = 5.0,
    ):
        self.position_kp = float(position_kp)
        self.velocity_kp = float(velocity_kp)
        self.velocity_ki = float(velocity_ki)
        self.velocity_limit = float(velocity_limit)
        self.correction_limit = float(correction_limit)
        self.windup_limit = float(windup_limit)
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(self, relative_position: float, relative_velocity: float,
               dt: float) -> tuple[float, float]:
        """Return UAV velocity correction and relative-velocity setpoint.

        Relative state is platform minus UAV.  The outer loop therefore asks
        for a negative relative velocity when the platform is ahead, while
        the direct Gazebo velocity interface needs the opposite sign.
        """
        relative_velocity_setpoint = clamp(
            -self.position_kp * relative_position,
            -self.velocity_limit,
            self.velocity_limit,
        )
        error = relative_velocity_setpoint - relative_velocity
        self.integral = clamp(
            self.integral + error * max(0.0, dt),
            -self.windup_limit,
            self.windup_limit,
        )
        correction = -(
            self.velocity_kp * error + self.velocity_ki * self.integral)
        return clamp(
            correction, -self.correction_limit, self.correction_limit), \
            relative_velocity_setpoint


class MultiResolutionDiscretizer:
    """Multi-resolution state discretization from the upstream project."""

    STATE_MAXIMUMS = {
        'rel_p': 4.5,
        'rel_v': 3.39411,
        'rel_a': 1.28,
    }
    STEPS = {
        'rel_p': [1.0, 0.64, 0.4096, 0.262144, 0.16777216],
        'rel_v': [1.0, 0.8, 0.64, 0.512, 0.4096],
        'rel_a': [1.0, 1.0, 1.0, 1.0, 1.0],
    }

    def __init__(self, curriculum_step: int = 0, intervals: int = 3,
                 beta: float = 1.0 / 3.0, sigma_a: float = 0.416):
        if intervals <= 0 or intervals % 2 == 0:
            raise ValueError('the repository discretization needs odd n_r')
        self.curriculum_step = int(clamp(curriculum_step, 0, 4))
        self.intervals = int(intervals)
        self.beta = float(beta)
        self.sigma_a = float(sigma_a)

    def set_curriculum_step(self, step: int) -> None:
        self.curriculum_step = int(clamp(step, 0, 4))

    def _normal(self, state: str, value: float) -> float:
        return clamp(value / self.STATE_MAXIMUMS[state], -1.0, 1.0)

    def latest_valid_step(self, values: dict[str, float]) -> int:
        latest = self.curriculum_step
        for state, value in values.items():
            normalized = abs(self._normal(state, value))
            valid = 0
            for step in range(self.curriculum_step + 1):
                if normalized <= self.STEPS[state][step] + 1e-12:
                    valid = step
            latest = min(latest, valid)
        return latest

    def _bin(self, state: str, value: float, step: int) -> int:
        normalized = self._normal(state, value)
        limit = self.STEPS[state][step]
        normalized = clamp(normalized, -limit, limit)
        if step < self.curriculum_step:
            ratio = self.STEPS[state][step + 1] / limit
        elif state == 'rel_a':
            ratio = self.beta * self.sigma_a
        else:
            ratio = self.beta
        goal = ratio * limit
        if -goal <= normalized <= goal:
            return self.intervals // 2
        if normalized < -goal:
            return 0
        return self.intervals - 1

    @staticmethod
    def action_bin(action_value: float, action_max: float,
                   action_delta: float) -> int:
        clipped = clamp(action_value, -action_max, action_max)
        return int(round((clipped + action_max) / action_delta))

    def state(self, relative_position: float, relative_velocity: float,
              relative_acceleration: float, action_value: float,
              action_max: float, action_delta: float) -> tuple[int, ...]:
        values = {
            'rel_p': relative_position,
            'rel_v': relative_velocity,
            'rel_a': relative_acceleration,
        }
        step = self.latest_valid_step(values)
        return (
            step,
            self._bin('rel_p', relative_position, step),
            self._bin('rel_v', relative_velocity, step),
            self._bin('rel_a', relative_acceleration, step),
            self.action_bin(action_value, action_max, action_delta),
        )


class DoubleQLearner:
    """Sparse Double-Q learner with resumable JSON persistence."""

    ACTIONS = ('increase', 'decrease', 'do_nothing')

    def __init__(self, gamma: float = 0.99, omega: float = 0.51,
                 alpha_min: float = 0.02949, seed: int | None = None):
        self.gamma = float(gamma)
        self.omega = float(omega)
        self.alpha_min = float(alpha_min)
        self.random = random.Random(seed)
        self.q_a = defaultdict(lambda: [0.0, 0.0, 0.0])
        self.q_b = defaultdict(lambda: [0.0, 0.0, 0.0])
        self.visits = defaultdict(lambda: [0, 0, 0])
        self.episode = 0
        self.epsilon = 1.0
        self.curriculum_step = 0

    @staticmethod
    def epsilon_for_episode(episode: int) -> float:
        if episode <= 800:
            return 1.0
        if episode >= 2000:
            return 0.01
        fraction = (episode - 800) / 1200.0
        return 1.0 + fraction * (0.01 - 1.0)

    @staticmethod
    def _best(values: list[float]) -> int:
        return max(range(len(values)), key=values.__getitem__)

    def choose(self, state: tuple[int, ...], training: bool = True) -> int:
        if training and self.random.random() < self.epsilon:
            return self.random.randrange(3)
        means = [
            (self.q_a[state][index] + self.q_b[state][index]) / 2.0
            for index in range(3)
        ]
        return self._best(means)

    def update(self, state: tuple[int, ...], action: int, reward: float,
               next_state: tuple[int, ...], terminal: bool) -> float:
        count = self.visits[state][action]
        alpha = max((1.0 / (count + 1)) ** self.omega, self.alpha_min)
        if self.random.randrange(2) == 0:
            selected = self._best(self.q_a[next_state])
            future = 0.0 if terminal else self.q_b[next_state][selected]
            delta = alpha * (
                reward + self.gamma * future - self.q_a[state][action])
            self.q_a[state][action] += delta
        else:
            selected = self._best(self.q_b[next_state])
            future = 0.0 if terminal else self.q_a[next_state][selected]
            delta = alpha * (
                reward + self.gamma * future - self.q_b[state][action])
            self.q_b[state][action] += delta
        self.visits[state][action] += 1
        return delta

    def end_episode(self) -> None:
        self.episode += 1
        self.epsilon = self.epsilon_for_episode(self.episode)

    def save(self, path: str) -> None:
        def encode(table):
            return {','.join(map(str, key)): list(value)
                    for key, value in table.items()}

        data = {
            'format': 'repo_compatible_double_q_v1',
            'episode': self.episode,
            'epsilon': self.epsilon,
            'curriculum_step': self.curriculum_step,
            'q_a': encode(self.q_a),
            'q_b': encode(self.q_b),
            'visits': encode(self.visits),
        }
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2, sort_keys=True)

    def load(self, path: str) -> None:
        with open(path, encoding='utf-8') as stream:
            data = json.load(stream)
        if data.get('format') != 'repo_compatible_double_q_v1':
            raise ValueError('unsupported repository policy format')

        def decode(source, integer=False):
            result = {}
            for key, values in source.items():
                parsed = tuple(int(item) for item in key.split(','))
                result[parsed] = [int(v) if integer else float(v)
                                  for v in values]
            return result

        self.q_a.update(decode(data.get('q_a', {})))
        self.q_b.update(decode(data.get('q_b', {})))
        self.visits.update(decode(data.get('visits', {}), integer=True))
        self.episode = int(data.get('episode', 0))
        self.epsilon = float(data.get(
            'epsilon', self.epsilon_for_episode(self.episode)))
        self.curriculum_step = int(data.get('curriculum_step', 0))


def apply_increment(value: float, action: int, delta: float,
                    maximum: float) -> float:
    """Apply the upstream increase/decrease/do-nothing action semantics."""
    if action == 0:
        value += delta
    elif action == 1:
        value -= delta
    elif action != 2:
        raise ValueError(f'unknown action {action}')
    return clamp(value, -maximum, maximum)
