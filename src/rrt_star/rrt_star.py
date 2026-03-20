"""
RRT* Algorithm Implementation
Based on: Karaman & Frazzoli, "Sampling-based algorithms for optimal motion
planning", IJRR 2011

Key correctness detail:
  After rewiring, nodes near the goal may have lower costs. We maintain
  X_soln (set of all goal-reaching nodes) and re-evaluate the best cost
  every iteration, not just when a new node enters the goal region.
"""

import time
import numpy as np
from typing import Optional, Tuple, List, Dict, Callable

from src.state import State
from .node import RRTNode
from .tree import Tree
from .collision_checker import CollisionChecker
from .steering import StraightLineSteerer
from .nearest_neighbor import NearestNeighborIndex
from .samplers import UniformSampler, GoalBiasedSampler


class RRTStarPlanner:
    """Base RRT* planner.

    Args:
        map_handler: MapHandler instance for collision checking
        config: full config dict (reads 'rrt_star' section)
    """

    def __init__(self, map_handler, config: Dict) -> None:
        cfg = config.get('rrt_star', config)
        self.max_iterations = cfg.get('max_iterations', 10000)
        self.step_size = cfg.get('step_size', 2.0)
        self.goal_radius = cfg.get('goal_radius', 2.0)
        self.goal_bias = cfg.get('goal_bias', 0.05)
        self.gamma = cfg.get('gamma', 20.0)
        self.dimension = cfg.get('dimension', 2)
        self.convergence_patience = cfg.get('convergence_patience', 2000)

        self.collision_checker = CollisionChecker(map_handler)
        self.steerer = StraightLineSteerer(self.step_size)
        self.tree: Optional[Tree] = None

    def plan(self, start: State, goal: State,
             on_iteration: Optional[Callable] = None
             ) -> Optional[Tuple[List[State], Dict]]:
        """Plan a path from start to goal using RRT*.

        Args:
            start: start state
            goal: goal state
            on_iteration: optional callback(iteration, tree, best_cost, new_edge)

        Returns:
            (path, info_dict) or None if no path found
        """
        t0 = time.time()

        root = RRTNode(start.x, start.y)
        self.tree = Tree(root)
        nn_index = NearestNeighborIndex()
        nn_index.add(root)

        uniform = UniformSampler(
            self.collision_checker.x_range,
            self.collision_checker.y_range)
        sampler = GoalBiasedSampler(uniform, goal.x, goal.y, self.goal_bias)

        # X_soln: set of all nodes that can reach the goal (Karaman §3.3)
        # After rewiring reduces a node's cost, we re-evaluate best path
        # through ALL goal-reaching nodes, not just the latest one.
        goal_nodes: List[RRTNode] = []
        best_cost = float('inf')
        best_goal_node: Optional[RRTNode] = None
        cost_history: List[float] = []
        nodes_expanded = 0
        iters_since_improvement = 0
        actual_iterations = 0

        for i in range(1, self.max_iterations + 1):
            actual_iterations = i
            new_edge = None

            sx, sy = sampler.sample()
            x_nearest = nn_index.nearest(sx, sy)
            x_new = self.steerer.steer(x_nearest, sx, sy)

            if (self.collision_checker.is_collision_free(
                    x_nearest.x, x_nearest.y, x_new.x, x_new.y) and
                    self.collision_checker.is_valid_point(x_new.x, x_new.y)):

                nodes_expanded += 1

                n = self.tree.size
                r_n = self._compute_rewire_radius(n)
                X_near = nn_index.near(x_new.x, x_new.y, r_n)

                # Choose best parent
                x_min = x_nearest
                c_min = x_nearest.cost + x_nearest.distance_to(x_new)
                for x_near in X_near:
                    c_new = x_near.cost + x_near.distance_to(x_new)
                    if c_new < c_min:
                        if self.collision_checker.is_collision_free(
                                x_near.x, x_near.y, x_new.x, x_new.y):
                            x_min = x_near
                            c_min = c_new

                self.tree.add_node(x_new, x_min)
                x_new.cost = c_min
                nn_index.add(x_new)
                new_edge = (x_min, x_new)

                # Rewire
                for x_near in X_near:
                    if x_near is x_min:
                        continue
                    potential_cost = x_new.cost + x_new.distance_to(x_near)
                    if potential_cost < x_near.cost:
                        if self.collision_checker.is_collision_free(
                                x_new.x, x_new.y, x_near.x, x_near.y):
                            self.tree.rewire(x_near, x_new, potential_cost)

                # Check if new node reaches goal → add to X_soln
                dist_to_goal = x_new.distance_to_point(goal.x, goal.y)
                if dist_to_goal <= self.goal_radius:
                    if self.collision_checker.is_collision_free(
                            x_new.x, x_new.y, goal.x, goal.y):
                        goal_nodes.append(x_new)

            # Re-evaluate best cost only when a node was added (optimization)
            if new_edge is not None and goal_nodes:
                prev_best = best_cost
                new_best = float('inf')
                new_best_node = None
                for gn in goal_nodes:
                    tc = gn.cost + gn.distance_to_point(goal.x, goal.y)
                    if tc < new_best:
                        new_best = tc
                        new_best_node = gn
                best_cost = new_best
                best_goal_node = new_best_node

                if best_cost < prev_best - 1e-6:
                    iters_since_improvement = 0
                else:
                    iters_since_improvement += 1
            elif best_cost < float('inf'):
                iters_since_improvement += 1

            cost_history.append(best_cost)

            if i % 2000 == 0:
                print(f"  RRT* iteration {i}/{self.max_iterations}, "
                      f"tree size: {self.tree.size}, "
                      f"best cost: {best_cost:.2f}")

            if on_iteration is not None:
                on_iteration(i, self.tree, best_cost, new_edge)

            # Early termination if converged
            if (self.convergence_patience > 0 and
                    iters_since_improvement >= self.convergence_patience and
                    best_cost < float('inf')):
                print(f"  RRT* converged at iteration {i} "
                      f"(no improvement for {self.convergence_patience} iters)")
                break

        search_time = time.time() - t0

        if best_goal_node is None:
            print("  No path found.")
            return None

        node_path = self.tree.extract_path(best_goal_node)
        node_path.append(RRTNode(goal.x, goal.y))

        path = self._nodes_to_states(node_path)
        path_length = sum(
            np.hypot(path[j+1].x - path[j].x, path[j+1].y - path[j].y)
            for j in range(len(path) - 1))

        info = {
            'success': True,
            'search_time': search_time,
            'path_length': path_length,
            'nodes_expanded': nodes_expanded,
            'iterations': actual_iterations,
            'cost_history': cost_history,
            'tree_size': self.tree.size,
            'algorithm': 'rrt_star',
        }

        print(f"\n{'='*40}")
        print(f"  RRT* Result")
        print(f"  {'='*36}")
        print(f"  Success:        True")
        print(f"  Tree Size:      {self.tree.size}")
        print(f"  Search Time:    {search_time:.3f} s")
        print(f"  Path Length:    {path_length:.2f} m")
        print(f"{'='*40}\n")

        return path, info

    def _compute_rewire_radius(self, n: int) -> float:
        """r_n = gamma * (log(n) / n) ^ (1/d), capped at step_size * 3."""
        if n <= 1:
            return self.step_size * 3
        r = self.gamma * (np.log(n) / n) ** (1.0 / self.dimension)
        return min(r, self.step_size * 3)

    def _nodes_to_states(self, node_path: List[RRTNode]) -> List[State]:
        """Convert RRTNode path to List[State] with theta from tangent direction."""
        states = []
        for i, node in enumerate(node_path):
            if i < len(node_path) - 1:
                dx = node_path[i+1].x - node.x
                dy = node_path[i+1].y - node.y
                theta = np.arctan2(dy, dx)
            else:
                theta = states[-1].theta if states else 0.0
            states.append(State(node.x, node.y, theta, g=node.cost))
        return states
