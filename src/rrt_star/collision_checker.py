"""Collision checker adapter wrapping MapHandler for RRT* algorithms.

Supports an optional safety_margin (obstacle expansion) per the BI-RRT* paper
(Fan et al., 2024, Stage 1: ExpansionDistance). When safety_margin > 0, any
point within that distance of an obstacle is treated as occupied, effectively
inflating all obstacles by the robot's half-width.
"""

import numpy as np
from typing import Tuple


class CollisionChecker:
    """Adapter that exposes MapHandler's collision checking for RRT* planners.

    Args:
        map_handler: MapHandler instance with is_line_collision, is_valid_world, etc.
        safety_margin: obstacle inflation radius in meters (default 0.0).
                       Set to robot_width / 2 for point-robot equivalence.
    """

    def __init__(self, map_handler, safety_margin: float = 0.0) -> None:
        self._map = map_handler
        self._margin = safety_margin

    def is_collision_free(self, x0: float, y0: float, x1: float, y1: float) -> bool:
        """Check if straight-line segment from (x0,y0) to (x1,y1) is collision free.

        When safety_margin > 0, samples points along the line and checks
        each has clearance >= safety_margin from nearest obstacle.
        """
        if self._margin <= 0:
            return not self._map.is_line_collision(x0, y0, x1, y1)

        # With safety margin: check clearance along the line
        d = np.hypot(x1 - x0, y1 - y0)
        if d < 1e-9:
            return self._is_clear(x0, y0)

        # Sample points along the line at resolution intervals
        steps = max(int(d / self._map.resolution), 2)
        for i in range(steps + 1):
            t = i / steps
            px = x0 + t * (x1 - x0)
            py = y0 + t * (y1 - y0)
            if not self._is_clear(px, py):
                return False
        return True

    def is_valid_point(self, x: float, y: float) -> bool:
        """Check if point is inside map bounds and has sufficient clearance."""
        if not self._map.is_valid_world(x, y):
            return False
        if self._map.is_obstacle_world(x, y):
            return False
        if self._margin > 0:
            return self._map.get_obstacle_distance_world(x, y) >= self._margin
        return True

    def _is_clear(self, x: float, y: float) -> bool:
        """Check single point has clearance >= safety_margin."""
        if not self._map.is_valid_world(x, y):
            return False
        if self._map.is_obstacle_world(x, y):
            return False
        if self._margin > 0:
            return self._map.get_obstacle_distance_world(x, y) >= self._margin
        return True

    @property
    def x_range(self) -> Tuple[float, float]:
        """Valid x-coordinate range (meters)."""
        return (0.0, self._map.width)

    @property
    def y_range(self) -> Tuple[float, float]:
        """Valid y-coordinate range (meters)."""
        return (0.0, self._map.height)

    @property
    def world_bounds(self) -> Tuple[float, float, float, float]:
        """(x_min, y_min, x_max, y_max) of the map."""
        return (0.0, 0.0, self._map.width, self._map.height)
