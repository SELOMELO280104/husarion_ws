"""Safe landing-policy primitives adapted from rl_multi_rotor_landing.

The upstream project uses a discretised relative-state Q-learning policy and
cascaded PI control.  This module keeps that interface independent of ROS so
it can be tested and used by the existing ArUco tracker.  It deliberately
does not arm, land, or publish commands by itself.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class RelativeState:
    """Target position and velocity in the UAV camera/body frame (metres)."""

    x: float
    y: float
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0


class Discretizer:
    """Multi-resolution binning used by the upstream tabular agent."""

    def __init__(self, limits=None, bins=(5, 5, 5, 3, 3, 3)):
        self.limits = limits or ((-4.0, 4.0), (-4.0, 4.0), (0.2, 8.0),
                                 (-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0))
        self.bins = tuple(bins)

    def encode(self, state: RelativeState) -> tuple[int, ...]:
        values = (state.x, state.y, state.z, state.vx, state.vy, state.vz)
        encoded = []
        for value, (low, high), count in zip(values, self.limits, self.bins):
            value = min(max(value, low), high)
            index = int((value - low) / (high - low) * count)
            encoded.append(min(index, count - 1))
        return tuple(encoded)


class QLandingPolicy:
    """Inference-only Q policy; missing/untrained tables fail safe to hover."""

    ACTIONS = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (-1.0, 0.0, 0.0),
               (0.0, 1.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))

    def __init__(self, table_path=None, discretizer=None):
        self.discretizer = discretizer or Discretizer()
        self.table = {}
        if table_path:
            self.load(table_path)

    def load(self, table_path):
        data = json.loads(Path(table_path).read_text())
        self.table = {tuple(map(int, key.split(','))): list(values)
                      for key, values in data.items()}

    def action(self, state: RelativeState):
        values = self.table.get(self.discretizer.encode(state))
        if not values or not all(math.isfinite(float(v)) for v in values):
            return self.ACTIONS[0], False
        index = max(range(min(len(values), len(self.ACTIONS))),
                    key=lambda i: float(values[i]))
        return self.ACTIONS[index], True


def cascaded_velocity(state: RelativeState, kp=0.35, kd=0.10,
                      max_speed=0.8, landing_height=0.35):
    """Deterministic PI-style fallback, bounded and body-frame relative."""
    vx = kp * state.x + kd * state.vx
    vy = kp * state.y + kd * state.vy
    vz = 0.0 if state.z <= landing_height else -kp * (state.z - landing_height)
    scale = max(1.0, math.sqrt(vx * vx + vy * vy) / max_speed)
    return (vx / scale, vy / scale, max(-max_speed, min(max_speed, vz)))
