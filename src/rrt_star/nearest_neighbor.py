"""KD-tree based nearest neighbor index for RRT* algorithms."""

import numpy as np
from scipy.spatial import KDTree
from typing import List, Optional
from .node import RRTNode


class NearestNeighborIndex:
    """Wraps scipy.spatial.KDTree for efficient nearest-neighbor queries.

    Automatically rebuilds the KD-tree after a configurable number of insertions
    to maintain query performance.

    Args:
        rebuild_threshold: rebuild KD-tree after this many insertions (default 1000)
    """

    def __init__(self, rebuild_threshold: int = 1000) -> None:
        self._nodes: List[RRTNode] = []
        self._points: List[List[float]] = []
        self._kdtree: Optional[KDTree] = None
        self._rebuild_threshold = rebuild_threshold
        self._insertions_since_rebuild = 0

    def add(self, node: RRTNode) -> None:
        """Add a node to the index."""
        self._nodes.append(node)
        self._points.append([node.x, node.y])
        self._insertions_since_rebuild += 1

        if self._insertions_since_rebuild >= self._rebuild_threshold:
            self.rebuild()

    def nearest(self, x: float, y: float) -> RRTNode:
        """Find the nearest node to point (x, y).

        Returns:
            The closest RRTNode.
        """
        self._ensure_built()
        _, idx = self._kdtree.query([x, y])
        return self._nodes[idx]

    def near(self, x: float, y: float, radius: float) -> List[RRTNode]:
        """Find all nodes within radius of point (x, y).

        Returns:
            List of RRTNodes within the given radius.
        """
        self._ensure_built()
        indices = self._kdtree.query_ball_point([x, y], radius)
        return [self._nodes[i] for i in indices]

    def rebuild(self) -> None:
        """Force rebuild the KD-tree from current nodes."""
        if len(self._points) > 0:
            self._kdtree = KDTree(np.array(self._points))
        self._insertions_since_rebuild = 0

    def _ensure_built(self) -> None:
        """Build KD-tree if not yet built or stale."""
        if self._kdtree is None or self._insertions_since_rebuild > 0:
            self.rebuild()

    @property
    def size(self) -> int:
        return len(self._nodes)
