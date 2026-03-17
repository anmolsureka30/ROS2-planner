"""
Heuristic Functions for Hybrid A*
Supports Cost-Aware Obstacle Heuristic (Macenski et al., 2025)
"""

import numpy as np
import heapq
from typing import Dict, Tuple
from src.state import State
from src.map_handler import MapHandler
from src.hybrid_astar.reeds_shepp import ReedsSheppPath


class HeuristicCalculator:
    """
    Calculates heuristic estimates for Hybrid A*.

    Supports:
    - euclidean: Simple Euclidean distance
    - dubins: Non-holonomic distance (Reeds-Shepp, ignores obstacles)
    - 2d_astar: Cost-aware holonomic distance with obstacles
    - max: Maximum of dubins and 2d_astar (recommended)
    """

    def __init__(self, map_handler: MapHandler,
                 heuristic_type: str = "max",
                 turning_radius: float = 5.0,
                 cost_alpha: float = 1.0):
        self.map_handler = map_handler
        self.heuristic_type = heuristic_type
        self.turning_radius = turning_radius
        self.cost_alpha = cost_alpha

        self.rs_planner = ReedsSheppPath(turning_radius)

        # Cache for 2D heuristic distance maps
        self.holonomic_cache: Dict[Tuple[int, int], np.ndarray] = {}

        print(f"Heuristic Calculator: type={heuristic_type}, "
              f"turning_radius={turning_radius}m, cost_alpha={cost_alpha}")

    def compute(self, state: State, goal: State) -> float:
        """Compute heuristic value."""
        if self.heuristic_type == "euclidean":
            return self._euclidean_distance(state, goal)
        elif self.heuristic_type == "dubins":
            return self._reeds_shepp_distance(state, goal)
        elif self.heuristic_type == "2d_astar":
            return self._holonomic_with_obstacles(state, goal)
        elif self.heuristic_type == "max":
            h_rs = self._reeds_shepp_distance(state, goal)
            h_2d = self._holonomic_with_obstacles(state, goal)
            return max(h_rs, h_2d)
        else:
            return self._euclidean_distance(state, goal)

    def _euclidean_distance(self, state: State, goal: State) -> float:
        """Simple Euclidean distance (admissible)."""
        return np.hypot(state.x - goal.x, state.y - goal.y)

    def _reeds_shepp_distance(self, state: State, goal: State) -> float:
        """Reeds-Shepp distance (non-holonomic without obstacles, admissible)."""
        return self.rs_planner.get_distance(state, goal)

    def _holonomic_with_obstacles(self, state: State, goal: State) -> float:
        """
        Cost-Aware 2D Dijkstra distance considering obstacles and traversal costs.
        Admissible because it assumes holonomic motion (less constrained).
        Based on Smac Planner Cost-Aware Obstacle Heuristic.
        """
        start_px, start_py = self.map_handler.world_to_pixel(state.x, state.y)
        goal_px, goal_py = self.map_handler.world_to_pixel(goal.x, goal.y)

        cache_key = (goal_px, goal_py)
        if cache_key not in self.holonomic_cache:
            dist_map = self._compute_2d_astar_from_goal(goal_px, goal_py)
            self.holonomic_cache[cache_key] = dist_map

        dist_map = self.holonomic_cache[cache_key]

        if 0 <= start_py < dist_map.shape[0] and 0 <= start_px < dist_map.shape[1]:
            distance_pixels = dist_map[start_py, start_px]
        else:
            distance_pixels = float('inf')

        return distance_pixels * self.map_handler.resolution

    def _compute_2d_astar_from_goal(self, goal_px: int, goal_py: int) -> np.ndarray:
        """
        Compute Cost-Aware 2D Dijkstra distances from goal to all cells.
        Edge costs incorporate the cost field for obstacle proximity awareness.
        """
        height, width = self.map_handler.map_data.shape
        dist_map = np.full((height, width), float('inf'), dtype=np.float32)

        if not self.map_handler.is_valid_pixel(goal_px, goal_py):
            return dist_map
        if self.map_handler.is_obstacle_pixel(goal_px, goal_py):
            return dist_map

        pq = [(0.0, goal_px, goal_py)]
        dist_map[goal_py, goal_px] = 0.0

        # 8-connected neighbors with distance costs
        neighbors = [
            (1, 0, 1.0), (-1, 0, 1.0),
            (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414),
            (1, -1, 1.414), (-1, -1, 1.414)
        ]

        cost_field = self.map_handler.cost_field
        alpha = self.cost_alpha

        visited = set()

        while pq:
            dist, px, py = heapq.heappop(pq)

            if (px, py) in visited:
                continue
            visited.add((px, py))

            for dx, dy, edge_dist in neighbors:
                npx, npy = px + dx, py + dy

                if not self.map_handler.is_valid_pixel(npx, npy):
                    continue
                if self.map_handler.is_obstacle_pixel(npx, npy):
                    continue

                # Cost-Aware edge cost (Smac Planner Eq. 1)
                c_ij = cost_field[npy, npx]
                weighted_cost = edge_dist * (1.0 + alpha * c_ij)

                new_dist = dist + weighted_cost

                if new_dist < dist_map[npy, npx]:
                    dist_map[npy, npx] = new_dist
                    heapq.heappush(pq, (new_dist, npx, npy))

        return dist_map

    def precompute_holonomic_heuristic(self, goal: State):
        """Precompute holonomic heuristic for the given goal."""
        goal_px, goal_py = self.map_handler.world_to_pixel(goal.x, goal.y)
        dist_map = self._compute_2d_astar_from_goal(goal_px, goal_py)
        self.holonomic_cache[(goal_px, goal_py)] = dist_map
        print(f"  Precomputed cost-aware holonomic heuristic for "
              f"goal ({goal.x:.1f}, {goal.y:.1f})")
