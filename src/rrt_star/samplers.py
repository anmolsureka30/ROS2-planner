"""Sampling strategies for RRT* algorithms."""

import numpy as np
from typing import Tuple, Optional, Dict


class UniformSampler:
    """Uniform random sampling over a rectangular domain.

    Args:
        x_range: (x_min, x_max) bounds
        y_range: (y_min, y_max) bounds
    """

    def __init__(self, x_range: Tuple[float, float],
                 y_range: Tuple[float, float]) -> None:
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range

    def sample(self) -> Tuple[float, float]:
        x = np.random.uniform(self.x_min, self.x_max)
        y = np.random.uniform(self.y_min, self.y_max)
        return (x, y)


class GoalBiasedSampler:
    """Samples uniformly with a probability of returning the goal point.

    Args:
        uniform_sampler: base uniform sampler
        goal_x: goal x-coordinate
        goal_y: goal y-coordinate
        goal_bias: probability [0, 1] of sampling the goal
    """

    def __init__(self, uniform_sampler: UniformSampler,
                 goal_x: float, goal_y: float,
                 goal_bias: float = 0.05) -> None:
        self._uniform = uniform_sampler
        self._goal_x = goal_x
        self._goal_y = goal_y
        self._goal_bias = goal_bias

    def sample(self) -> Tuple[float, float]:
        if np.random.random() < self._goal_bias:
            return (self._goal_x, self._goal_y)
        return self._uniform.sample()


class InformedSampler:
    """Ellipsoidal sampling for Informed RRT* and variants.

    Samples uniformly inside a prolate hyperspheroid (ellipse in 2D) defined
    by start/goal as foci and c_best as the major axis length.

    Uses direct sampling (unit disk -> scale -> rotate -> translate),
    NOT rejection sampling.

    Args:
        start_x, start_y: start point (focus 1)
        goal_x, goal_y: goal point (focus 2)
        x_range, y_range: map bounds for fallback uniform sampling
    """

    def __init__(self, start_x: float, start_y: float,
                 goal_x: float, goal_y: float,
                 x_range: Tuple[float, float],
                 y_range: Tuple[float, float]) -> None:
        self._start = np.array([start_x, start_y])
        self._goal = np.array([goal_x, goal_y])
        self._uniform = UniformSampler(x_range, y_range)

        # Precompute ellipse constants
        self.c_min = np.linalg.norm(self._goal - self._start)
        self._center = (self._start + self._goal) / 2.0

        # Rotation angle: direction from start to goal
        diff = self._goal - self._start
        self._theta = np.arctan2(diff[1], diff[0])
        self._cos_t = np.cos(self._theta)
        self._sin_t = np.sin(self._theta)

    def sample(self, c_best: float) -> Tuple[float, float]:
        """Sample a point from the informed ellipsoidal region.

        Args:
            c_best: current best path cost (defines ellipse size)

        Returns:
            (x, y) sampled point
        """
        if c_best >= 1e18 or c_best <= self.c_min:
            return self._uniform.sample()

        # Ellipse semi-axes
        a = c_best / 2.0  # semi-major
        b = np.sqrt(c_best * c_best - self.c_min * self.c_min) / 2.0  # semi-minor

        if b < 1e-9:
            b = 1e-9

        # Sample uniformly in unit disk (direct method)
        r = np.sqrt(np.random.random())
        alpha = np.random.uniform(0, 2 * np.pi)
        x_unit = r * np.cos(alpha)
        y_unit = r * np.sin(alpha)

        # Scale to ellipse
        x_e = a * x_unit
        y_e = b * y_unit

        # Rotate and translate
        x_final = self._cos_t * x_e - self._sin_t * y_e + self._center[0]
        y_final = self._sin_t * x_e + self._cos_t * y_e + self._center[1]

        return (x_final, y_final)

    def get_ellipse_params(self, c_best: float) -> Optional[Dict]:
        """Get ellipse parameters for visualization.

        Returns:
            Dict with center, angle_deg, semi_major, semi_minor, or None.
        """
        if c_best >= 1e18 or c_best <= self.c_min:
            return None

        a = c_best / 2.0
        b_sq = c_best * c_best - self.c_min * self.c_min
        b = np.sqrt(max(b_sq, 0.0)) / 2.0

        return {
            'center': (self._center[0], self._center[1]),
            'angle_deg': np.degrees(self._theta),
            'semi_major': a,
            'semi_minor': b,
        }
