"""Tree data structure for RRT* algorithms."""

import numpy as np
from typing import List, Optional, Tuple
from .node import RRTNode


class Tree:
    """Tree structure maintaining vertices, edges, and parent/child relationships.

    Handles all pointer bookkeeping and cost propagation so planners
    never manipulate parent pointers directly.

    Args:
        root: root node of the tree
    """

    def __init__(self, root: RRTNode) -> None:
        root.cost = 0.0
        root.parent = None
        root.children = []
        self.root = root
        self.nodes: List[RRTNode] = [root]

    def add_node(self, node: RRTNode, parent: RRTNode) -> None:
        """Add a node to the tree with the given parent.

        Sets node.parent, updates node.cost, and maintains children list.
        """
        node.parent = parent
        node.cost = parent.cost + parent.distance_to(node)
        node.children = []
        parent.children.append(node)
        self.nodes.append(node)

    def rewire(self, node: RRTNode, new_parent: RRTNode, new_cost: float) -> None:
        """Change node's parent to new_parent and propagate cost updates.

        Args:
            node: node to rewire
            new_parent: new parent node
            new_cost: pre-computed cost through new_parent
        """
        # Remove from old parent's children
        old_parent = node.parent
        if old_parent is not None:
            try:
                old_parent.children.remove(node)
            except ValueError:
                pass

        # Set new parent
        node.parent = new_parent
        new_parent.children.append(node)
        node.cost = new_cost

        # Propagate cost update to all descendants (iterative BFS)
        self._propagate_cost(node)

    def _propagate_cost(self, node: RRTNode) -> None:
        """Iteratively update costs of all descendants after a rewire.

        Uses a stack (not recursion) to avoid stack overflow on deep trees.
        """
        stack = list(node.children)
        while stack:
            child = stack.pop()
            child.cost = child.parent.cost + child.parent.distance_to(child)
            stack.extend(child.children)

    def extract_path(self, goal_node: RRTNode) -> List[RRTNode]:
        """Trace path from goal_node back to root.

        Returns:
            List of nodes from root to goal_node (inclusive).
        """
        path = []
        current = goal_node
        while current is not None:
            path.append(current)
            current = current.parent
        path.reverse()
        return path

    def get_all_edges(self) -> List[Tuple[RRTNode, RRTNode]]:
        """Return all (parent, child) pairs for visualization."""
        edges = []
        for node in self.nodes:
            if node.parent is not None:
                edges.append((node.parent, node))
        return edges

    @property
    def size(self) -> int:
        return len(self.nodes)
