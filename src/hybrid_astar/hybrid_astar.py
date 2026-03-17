"""
Hybrid A* Path Planner
Based on Dolgov et al. (2010): SE(2) search with Reeds-Shepp analytical expansion.
"""

import numpy as np
import heapq
import time
from typing import Optional, List, Tuple, Dict
from src.state import State, discretize_state
from src.map_handler import MapHandler
from src.hybrid_astar.motion_model import MotionModel, VehicleFootprint
from src.hybrid_astar.heuristic import HeuristicCalculator
from src.hybrid_astar.reeds_shepp import ReedsSheppPath


class HybridAStar:
    """
    Hybrid A* planner with Reeds-Shepp analytical expansion and dual heuristic.

    Algorithm:
        1. Expand states via bicycle kinematic motion primitives
        2. f(s) = g(s) + h(s) where h = max(RS_distance, 2D_Dijkstra)
        3. Try Reeds-Shepp shot to goal within shot_distance
        4. If RS path is collision-free → return immediately
    """

    def __init__(self,
                 map_handler: MapHandler,
                 motion_model: MotionModel,
                 vehicle_footprint: VehicleFootprint,
                 heuristic_calculator: HeuristicCalculator,
                 xy_resolution: float = 1.0,
                 theta_resolution: float = 5.0,
                 steering_penalty: float = 1.5,
                 reversing_penalty: float = 2.0,
                 steering_change_penalty: float = 1.5,
                 direction_switch_penalty: float = 10.0,
                 shot_distance: float = 15.0,
                 max_iterations: int = 50000):

        self.map_handler = map_handler
        self.motion_model = motion_model
        self.vehicle_footprint = vehicle_footprint
        self.heuristic_calculator = heuristic_calculator

        self.xy_resolution = xy_resolution
        self.theta_resolution = np.radians(theta_resolution)

        # Cost parameters (4 core penalties per Dolgov et al.)
        self.steering_penalty = steering_penalty
        self.reversing_penalty = reversing_penalty
        self.steering_change_penalty = steering_change_penalty
        self.direction_switch_penalty = direction_switch_penalty

        self.shot_distance = shot_distance
        self.max_iterations = max_iterations

        self.rs_planner = ReedsSheppPath(
            turning_radius=heuristic_calculator.turning_radius
        )

        self.collision_cache = {}

    def plan(self, start: State, goal: State) -> Optional[Tuple[List[State], Dict]]:
        """
        Plan a path from start to goal.

        Args:
            start: Start state (x, y, theta)
            goal: Goal state (x, y, theta)

        Returns:
            (path, info) tuple or None if no path found
        """
        start_time = time.time()
        nodes_expanded = 0
        nodes_visited = 0
        self.collision_cache.clear()

        # Validate start and goal
        if not self._is_valid_state(start):
            print("Start state is in collision!")
            return None
        if not self._is_valid_state(goal):
            print("Goal state is in collision!")
            return None

        # Precompute 2D heuristic if needed
        if self.heuristic_calculator.heuristic_type in ("2d_astar", "max"):
            print("Precomputing 2D heuristic...")
            self.heuristic_calculator.precompute_holonomic_heuristic(goal)

        # Initialize search
        start.g = 0.0
        start.h = self.heuristic_calculator.compute(start, goal)

        open_set = []
        counter = 0
        heapq.heappush(open_set, (start.f, counter, start))
        counter += 1

        closed_set: Dict[Tuple, State] = {}

        while open_set:
            _, _, current = heapq.heappop(open_set)
            nodes_expanded += 1

            dist_to_goal = current.distance_to(goal)

            # Progress reporting
            if nodes_expanded % 5000 == 0:
                print(f"  Nodes expanded: {nodes_expanded}, "
                      f"time: {time.time() - start_time:.2f}s")

            # === EXACT GOAL CHECK ===
            heading_diff = abs(self._angle_diff(current.theta, goal.theta))
            if dist_to_goal < 0.3 and heading_diff < np.radians(5):
                path = self._reconstruct_path(current)
                path.append(goal.copy())
                return self._build_result(path, start_time, nodes_expanded,
                                          nodes_visited, analytical=False)

            # === ANALYTICAL EXPANSION (Reeds-Shepp shot) ===
            if dist_to_goal < self.shot_distance:
                rs_result = self.rs_planner.plan(current, goal)
                if rs_result is not None:
                    rs_path, rs_length = rs_result
                    if self._is_path_collision_free(rs_path):
                        print(f"  Analytical shot! dist={dist_to_goal:.2f}m, "
                              f"RS_length={rs_length:.2f}m")
                        path = self._reconstruct_path(current) + rs_path[1:]
                        return self._build_result(path, start_time, nodes_expanded,
                                                  nodes_visited, analytical=True)

            # === EXPANSION ===
            current_idx = discretize_state(current, self.xy_resolution,
                                           self.theta_resolution)

            if current_idx in closed_set:
                if closed_set[current_idx].g <= current.g:
                    continue
            closed_set[current_idx] = current
            nodes_visited += 1

            # Generate and evaluate neighbors
            for neighbor in self._get_neighbors(current, goal):
                if not self._is_valid_state(neighbor):
                    continue

                neighbor_idx = discretize_state(neighbor, self.xy_resolution,
                                                self.theta_resolution)
                if neighbor_idx in closed_set:
                    if closed_set[neighbor_idx].g <= neighbor.g:
                        continue

                heapq.heappush(open_set, (neighbor.f, counter, neighbor))
                counter += 1

            # Iteration limit
            if nodes_expanded > self.max_iterations:
                print(f"  Max iterations ({self.max_iterations}) reached!")
                break

        print("  No path found.")
        return None

    def _get_neighbors(self, state: State, goal: State) -> List[State]:
        """Generate neighbor states using motion primitives."""
        neighbors = []

        for direction, steering in self.motion_model.get_motion_primitives():
            neighbor = self.motion_model.apply_motion(state, direction, steering)

            motion_cost = self.motion_model.compute_motion_cost(
                state, neighbor,
                self.steering_penalty,
                self.reversing_penalty,
                self.steering_change_penalty,
                self.direction_switch_penalty
            )

            neighbor.g = state.g + motion_cost
            neighbor.h = self.heuristic_calculator.compute(neighbor, goal)
            neighbor.parent = state
            neighbors.append(neighbor)

        return neighbors

    def _is_valid_state(self, state: State) -> bool:
        """Collision check with cache."""
        cache_key = (
            int(state.x / self.xy_resolution),
            int(state.y / self.xy_resolution),
            int(state.theta / self.theta_resolution)
        )

        if cache_key in self.collision_cache:
            return self.collision_cache[cache_key]

        if not self.map_handler.is_valid_world(state.x, state.y):
            self.collision_cache[cache_key] = False
            return False

        footprint = self.vehicle_footprint.get_footprint_fast(state)
        result = not self.map_handler.is_collision(footprint)

        self.collision_cache[cache_key] = result
        return result

    def _is_path_collision_free(self, path: List[State]) -> bool:
        """Check RS path for collisions at intervals <= vehicle_width/2."""
        if len(path) < 2:
            return False

        max_gap = self.vehicle_footprint.width / 2.0
        total_length = sum(
            path[i].distance_to(path[i + 1]) for i in range(len(path) - 1)
        )

        if total_length == 0:
            return self._is_valid_state(path[0])

        desired_checks = max(int(total_length / max_gap), 10)
        step = max(1, len(path) // desired_checks)

        for i in range(0, len(path), step):
            if not self._is_valid_state(path[i]):
                return False

        if not self._is_valid_state(path[-1]):
            return False

        return True

    def _reconstruct_path(self, state: State) -> List[State]:
        """Reconstruct path by following parent pointers."""
        path = []
        current = state
        while current is not None:
            path.append(current.copy())
            current = current.parent
        path.reverse()
        return path

    def _build_result(self, path: List[State], start_time: float,
                      nodes_expanded: int, nodes_visited: int,
                      analytical: bool) -> Tuple[List[State], Dict]:
        """Build result tuple with path and statistics."""
        search_time = time.time() - start_time
        path_length = sum(
            path[i].distance_to(path[i + 1]) for i in range(len(path) - 1)
        )

        info = {
            'success': True,
            'nodes_expanded': nodes_expanded,
            'nodes_visited': nodes_visited,
            'search_time': search_time,
            'path_length': path_length,
            'analytical': analytical,
        }

        print(f"\n  Success! length={path_length:.2f}m, "
              f"nodes={nodes_expanded}, time={search_time:.3f}s"
              f"{', analytical shot' if analytical else ''}")

        return path, info

    @staticmethod
    def _angle_diff(angle1: float, angle2: float) -> float:
        """Compute signed angle difference in [-pi, pi]."""
        diff = angle1 - angle2
        while diff > np.pi:
            diff -= 2 * np.pi
        while diff < -np.pi:
            diff += 2 * np.pi
        return diff
