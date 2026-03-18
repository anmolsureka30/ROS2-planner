"""Steering function for RRT* algorithms."""

import numpy as np
from .node import RRTNode


class StraightLineSteerer:
    """Steers from a node toward a target point by a fixed step size.

    Args:
        step_size: maximum distance to move per steer (meters)
    """

    def __init__(self, step_size: float) -> None:
        self.step_size = step_size

    def steer(self, from_node: RRTNode, to_x: float, to_y: float) -> RRTNode:
        """Create a new node by moving from from_node toward (to_x, to_y).

        If the target is closer than step_size, returns a node at the target.
        Does NOT set parent or cost — the Tree handles that.
        """
        dx = to_x - from_node.x
        dy = to_y - from_node.y
        d = np.hypot(dx, dy)

        if d < 1e-9:
            return RRTNode(to_x, to_y)

        if d <= self.step_size:
            return RRTNode(to_x, to_y)

        ratio = self.step_size / d
        nx = from_node.x + ratio * dx
        ny = from_node.y + ratio * dy
        return RRTNode(nx, ny)
