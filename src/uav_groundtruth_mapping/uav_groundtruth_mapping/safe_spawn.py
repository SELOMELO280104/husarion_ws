"""Collision-free spawn validation for simulation and RL episodes."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class SpawnPose:
    x: float
    y: float
    z: float
    radius: float


def _distance_xy(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def is_safe_spawn(pose: SpawnPose, obstacles=(), other_poses=(),
                  bounds=None, min_altitude=0.0) -> bool:
    """Return true only when a circular footprint clears all constraints.

    ``obstacles`` are ``(xmin, xmax, ymin, ymax)`` rectangles. ``bounds`` is
    an optional rectangle for the allowed training area.
    """
    if pose.z < min_altitude or not all(math.isfinite(v) for v in
                                       (pose.x, pose.y, pose.z)):
        return False
    if bounds is not None:
        xmin, xmax, ymin, ymax = bounds
        if not (xmin + pose.radius <= pose.x <= xmax - pose.radius and
                ymin + pose.radius <= pose.y <= ymax - pose.radius):
            return False
    for xmin, xmax, ymin, ymax in obstacles:
        dx = max(xmin - pose.x, 0.0, pose.x - xmax)
        dy = max(ymin - pose.y, 0.0, pose.y - ymax)
        if math.hypot(dx, dy) < pose.radius:
            return False
    return all(_distance_xy(pose, other) >= pose.radius + other.radius
               for other in other_poses)


def choose_safe_spawn(candidates, obstacles=(), other_poses=(), bounds=None,
                      min_altitude=0.0):
    """Return the first valid candidate, or raise ValueError."""
    for candidate in candidates:
        if is_safe_spawn(candidate, obstacles, other_poses, bounds,
                         min_altitude):
            return candidate
    raise ValueError('no collision-free spawn pose available')
