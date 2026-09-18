"""Small simulator-agnostic RL search environment for UAV-to-Panther search."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random

from .safe_spawn import SpawnPose, choose_safe_spawn


@dataclass
class SearchState:
    dx: float
    dy: float
    altitude: float
    marker_visible: bool
    time_remaining: float


class SearchEnvironment:
    """Gym-like episode logic; Gazebo adapters provide observations externally."""

    ACTIONS = ((0.0, 0.0, 0.0), (0.6, 0.0, 0.0), (-0.6, 0.0, 0.0),
               (0.0, 0.6, 0.0), (0.0, -0.6, 0.0), (0.0, 0.0, 0.4),
               (0.0, 0.0, -0.4))

    def __init__(self, bounds=(-20.0, 20.0, -20.0, 20.0), episode_seconds=60.0,
                 uav_radius=0.8, ugv_radius=1.0, seed=None):
        self.bounds = bounds
        self.episode_seconds = episode_seconds
        self.uav_radius = uav_radius
        self.ugv_radius = ugv_radius
        self.rng = random.Random(seed)
        self.elapsed = 0.0
        self.found = False

    def reset(self, uav_candidates, ugv_candidates, obstacles=()):
        ugv = choose_safe_spawn(ugv_candidates, obstacles, bounds=self.bounds)
        uav = choose_safe_spawn(
            uav_candidates, obstacles,
            other_poses=(ugv,), bounds=self.bounds, min_altitude=2.0)
        self.uav, self.ugv = uav, ugv
        self.elapsed = 0.0
        self.found = False
        return self.observe(False)

    def observe(self, marker_visible, marker_offset=None):
        dx, dy = marker_offset or (self.ugv.x - self.uav.x,
                                   self.ugv.y - self.uav.y)
        return SearchState(dx, dy, self.uav.z, bool(marker_visible),
                           max(0.0, self.episode_seconds - self.elapsed))

    def reward(self, state, action, dt=0.1):
        """Reward discovery, progress, and safe bounded search behavior."""
        self.elapsed += dt
        if state.marker_visible:
            self.found = True
            return 100.0
        distance = math.hypot(state.dx, state.dy)
        boundary_penalty = 20.0 if not self._inside_bounds() else 0.0
        altitude_penalty = 20.0 if state.altitude < 2.0 else 0.0
        return -0.05 - 0.01 * distance - boundary_penalty - altitude_penalty

    def _inside_bounds(self):
        xmin, xmax, ymin, ymax = self.bounds
        return xmin <= self.uav.x <= xmax and ymin <= self.uav.y <= ymax
