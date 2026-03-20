"""KD-tree based nearest neighbor index for RRT* algorithms.

Uses a hybrid approach: a KD-tree for bulk indexed nodes, plus a small
buffer of recently-added nodes searched linearly. The KD-tree is rebuilt
only when the buffer reaches a threshold, avoiding the O(n log n) rebuild
cost on every query.
"""

import numpy as np
from scipy.spatial import KDTree
from typing import List, Optional
from .node import RRTNode


class NearestNeighborIndex:
    """Wraps scipy.spatial.KDTree for efficient nearest-neighbor queries.

    Args:
        rebuild_threshold: rebuild KD-tree after this many insertions (default 500)
    """

    def __init__(self, rebuild_threshold: int = 500) -> None:
        self._nodes: List[RRTNode] = []
        self._points: List[List[float]] = []
        self._kdtree: Optional[KDTree] = None
        self._rebuild_threshold = rebuild_threshold
        # Buffer: nodes added since last KD-tree rebuild
        self._buf_nodes: List[RRTNode] = []
        self._buf_points: List[List[float]] = []

    def add(self, node: RRTNode) -> None:
        """Add a node to the index."""
        self._nodes.append(node)
        self._points.append([node.x, node.y])
        self._buf_nodes.append(node)
        self._buf_points.append([node.x, node.y])

        if len(self._buf_nodes) >= self._rebuild_threshold:
            self.rebuild()

    def nearest(self, x: float, y: float) -> RRTNode:
        """Find the nearest node to point (x, y)."""
        best_node = None
        best_dist_sq = float('inf')

        # Query KD-tree (indexed nodes)
        if self._kdtree is not None:
            dist, idx = self._kdtree.query([x, y])
            best_dist_sq = dist * dist
            best_node = self._nodes[idx]

        # Linear scan of buffer (unindexed nodes)
        for i, pt in enumerate(self._buf_points):
            d_sq = (pt[0] - x) ** 2 + (pt[1] - y) ** 2
            if d_sq < best_dist_sq:
                best_dist_sq = d_sq
                best_node = self._buf_nodes[i]

        return best_node

    def near(self, x: float, y: float, radius: float) -> List[RRTNode]:
        """Find all nodes within radius of point (x, y)."""
        result = []

        # Query KD-tree (indexed nodes)
        if self._kdtree is not None:
            indices = self._kdtree.query_ball_point([x, y], radius)
            result = [self._nodes[i] for i in indices]

        # Linear scan of buffer (unindexed nodes)
        r_sq = radius * radius
        for i, pt in enumerate(self._buf_points):
            d_sq = (pt[0] - x) ** 2 + (pt[1] - y) ** 2
            if d_sq <= r_sq:
                result.append(self._buf_nodes[i])

        return result

    def rebuild(self) -> None:
        """Rebuild the KD-tree from all current nodes and clear the buffer."""
        if len(self._points) > 0:
            self._kdtree = KDTree(np.array(self._points))
        self._buf_nodes.clear()
        self._buf_points.clear()

    @property
    def size(self) -> int:
        return len(self._nodes)
