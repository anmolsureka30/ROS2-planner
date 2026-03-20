"""
Informed RRT* Algorithm Implementation
Based on: Gammell, Srinivasa & Barfoot, "Informed RRT*: Optimal sampling-based
path planning focused via direct sampling of an admissible ellipsoidal
heuristic", IROS 2014

Key difference from base RRT*:
  - Before first solution: GoalBiasedSampler (same as RRT*)
  - After first solution: InformedSampler (ellipsoidal, focused on improving region)
  - Ellipse shrinks as better solutions are found
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
from .samplers import UniformSampler, GoalBiasedSampler, InformedSampler


class InformedRRTStarPlanner:
    """Informed RRT* planner with ellipsoidal sampling.

    Args:
        map_handler: MapHandler instance
        config: full config dict (reads 'informed_rrt_star' section)
    """

    def __init__(self, map_handler, config: Dict) -> None:
        cfg = config.get('informed_rrt_star', config)
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
        """Plan using Informed RRT*.

        Args:
            start: start state
            goal: goal state
            on_iteration: optional callback(iteration, tree, best_cost, new_edge)

        Returns:
            (path, info_dict) or None
        """
        t0 = time.time()

        # Initialize tree
        root = RRTNode(start.x, start.y)
        self.tree = Tree(root)
        nn_index = NearestNeighborIndex()
        nn_index.add(root)

        # Samplers
        uniform = UniformSampler(
            self.collision_checker.x_range,
            self.collision_checker.y_range)
        goal_biased = GoalBiasedSampler(uniform, goal.x, goal.y, self.goal_bias)
        informed = InformedSampler(
            start.x, start.y, goal.x, goal.y,
            self.collision_checker.x_range,
            self.collision_checker.y_range)

        goal_nodes: List[RRTNode] = []
        best_goal_node: Optional[RRTNode] = None
        best_cost = float('inf')
        cost_history: List[float] = []
        nodes_expanded = 0
        iters_since_improvement = 0
        actual_iterations = 0

        for i in range(1, self.max_iterations + 1):
            actual_iterations = i
            new_edge = None

            if best_cost < float('inf'):
                sx, sy = informed.sample(best_cost)
            else:
                sx, sy = goal_biased.sample()

            x_nearest = nn_index.nearest(sx, sy)
            x_new = self.steerer.steer(x_nearest, sx, sy)

            if (self.collision_checker.is_collision_free(
                    x_nearest.x, x_nearest.y, x_new.x, x_new.y) and
                    self.collision_checker.is_valid_point(x_new.x, x_new.y)):

                nodes_expanded += 1

                n = self.tree.size
                r_n = self._compute_rewire_radius(n)
                X_near = nn_index.near(x_new.x, x_new.y, r_n)

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

                for x_near in X_near:
                    if x_near is x_min:
                        continue
                    potential_cost = x_new.cost + x_new.distance_to(x_near)
                    if potential_cost < x_near.cost:
                        if self.collision_checker.is_collision_free(
                                x_new.x, x_new.y, x_near.x, x_near.y):
                            self.tree.rewire(x_near, x_new, potential_cost)

                dist_to_goal = x_new.distance_to_point(goal.x, goal.y)
                if dist_to_goal <= self.goal_radius:
                    if self.collision_checker.is_collision_free(
                            x_new.x, x_new.y, goal.x, goal.y):
                        goal_nodes.append(x_new)

            # Re-evaluate best only when a node was added
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
                print(f"  Informed RRT* iteration {i}/{self.max_iterations}, "
                      f"tree: {self.tree.size}, c_best: {best_cost:.2f}")

            if on_iteration is not None:
                on_iteration(i, self.tree, best_cost, new_edge)

            # Early termination if converged
            if (self.convergence_patience > 0 and
                    iters_since_improvement >= self.convergence_patience and
                    best_cost < float('inf')):
                print(f"  Informed RRT* converged at iteration {i} "
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

        ellipse_params = informed.get_ellipse_params(best_cost)

        info = {
            'success': True,
            'search_time': search_time,
            'path_length': path_length,
            'nodes_expanded': nodes_expanded,
            'iterations': actual_iterations,
            'cost_history': cost_history,
            'tree_size': self.tree.size,
            'algorithm': 'informed_rrt_star',
            'ellipse_params': ellipse_params,
        }

        print(f"\n{'='*40}")
        print(f"  Informed RRT* Result")
        print(f"  {'='*36}")
        print(f"  Success:        True")
        print(f"  Tree Size:      {self.tree.size}")
        print(f"  Search Time:    {search_time:.3f} s")
        print(f"  Path Length:    {path_length:.2f} m")
        print(f"{'='*40}\n")

        return path, info

    def _compute_rewire_radius(self, n: int) -> float:
        if n <= 1:
            return self.step_size * 3
        r = self.gamma * (np.log(n) / n) ** (1.0 / self.dimension)
        return min(r, self.step_size * 3)

    def _nodes_to_states(self, node_path: List[RRTNode]) -> List[State]:
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
