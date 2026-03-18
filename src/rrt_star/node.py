"""RRT Node for sampling-based planners."""

import numpy as np
from typing import Optional, Tuple, List


class RRTNode:
    """Node in an RRT tree.

    Attributes:
        x: x-coordinate (meters)
        y: y-coordinate (meters)
        parent: parent node in the tree
        cost: cumulative cost from root to this node
        children: child nodes (needed for cost propagation on rewire)
    """

    __slots__ = ('x', 'y', 'parent', 'cost', 'children')

    def __init__(self, x: float, y: float,
                 parent: Optional['RRTNode'] = None,
                 cost: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost
        self.children: List['RRTNode'] = []

    def distance_to(self, other: 'RRTNode') -> float:
        """Euclidean distance to another node."""
        return np.hypot(self.x - other.x, self.y - other.y)

    def distance_to_point(self, px: float, py: float) -> float:
        """Euclidean distance to a point."""
        return np.hypot(self.x - px, self.y - py)

    def as_tuple(self) -> Tuple[float, float]:
        return (self.x, self.y)

    def __repr__(self) -> str:
        return f"RRTNode(x={self.x:.2f}, y={self.y:.2f}, cost={self.cost:.2f})"
