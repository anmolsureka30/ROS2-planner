"""
BI-RRT* Algorithm Implementation
Based on: Fan et al., "BI-RRT*: An improved path planning algorithm for
secure and trustworthy mobile robots systems", Heliyon 10 (2024) e26403

Three-phase pipeline:
  Phase 1: Bidirectional RRT-Connect for fast initial solution
  Phase 2: Path pruning (greedy farthest-visible waypoint skipping)
  Phase 3: Informed RRT* optimization with pruned cost as initial c_best
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
from .samplers import UniformSampler, InformedSampler


class BIRRTStarPlanner:
    """BI-RRT* planner with bidirectional search + pruning + informed optimization.

    Args:
        map_handler: MapHandler instance
        config: full config dict (reads 'bi_rrt_star' section)
    """

    def __init__(self, map_handler, config: Dict) -> None:
        cfg = config.get('bi_rrt_star', config)
        self.max_bidir_iterations = cfg.get('max_bidir_iterations', 5000)
        self.max_opt_iterations = cfg.get('max_opt_iterations', 5000)
        self.step_size = cfg.get('step_size', 2.0)
        self.goal_radius = cfg.get('goal_radius', 2.0)
        self.gamma = cfg.get('gamma', 20.0)
        self.dimension = cfg.get('dimension', 2)

        # Stage 1 (Fan et al., 2024): Obstacle expansion by robot half-width.
        # safety_margin can be set explicitly in config, or defaults to
        # vehicle.width / 2. Set to 0 for maps with thin border obstacles.
        if 'safety_margin' in cfg:
            self.safety_margin = cfg['safety_margin']
        else:
            vehicle_width = config.get('vehicle', {}).get('width', 0.0)
            self.safety_margin = vehicle_width / 2.0
        self.convergence_patience = cfg.get('convergence_patience', 2000)

        self.collision_checker = CollisionChecker(map_handler, self.safety_margin)
        self.steerer = StraightLineSteerer(self.step_size)
        self.tree: Optional[Tree] = None

    def plan(self, start: State, goal: State,
             on_iteration: Optional[Callable] = None
             ) -> Optional[Tuple[List[State], Dict]]:
        """Plan using BI-RRT*.

        Args:
            start: start state
            goal: goal state
            on_iteration: optional callback(iteration, tree, best_cost, new_edge)

        Returns:
            (path, info_dict) or None
        """
        t0 = time.time()

        # Phase 1: Bidirectional search
        print("  BI-RRT* Phase 1: Bidirectional search...")
        initial_path = self._bidirectional_search(start, goal)
        if initial_path is None:
            print("  FAILED: No initial path found.")
            return None

        init_cost = self._path_length(initial_path)
        print(f"    Initial solution cost: {init_cost:.2f}")

        # Phase 2: Prune
        print("  BI-RRT* Phase 2: Path pruning...")
        pruned_path = self._prune_path(initial_path)
        pruned_cost = self._path_length(pruned_path)
        print(f"    Pruned cost: {pruned_cost:.2f} "
              f"({len(initial_path)} -> {len(pruned_path)} waypoints)")

        # Phase 3: Informed optimization
        print("  BI-RRT* Phase 3: Informed optimization...")
        opt_path, opt_cost, cost_history = self._informed_optimize(
            start, goal, pruned_cost, on_iteration)

        search_time = time.time() - t0

        # Use best result
        if opt_path is not None and opt_cost < pruned_cost:
            final_nodes = opt_path
        else:
            final_nodes = pruned_path

        path = self._nodes_to_states(final_nodes)
        path_length = sum(
            np.hypot(path[j+1].x - path[j].x, path[j+1].y - path[j].y)
            for j in range(len(path) - 1))

        info = {
            'success': True,
            'search_time': search_time,
            'path_length': path_length,
            'nodes_expanded': self.tree.size if self.tree else 0,
            'iterations': self.max_opt_iterations,
            'cost_history': cost_history,
            'tree_size': self.tree.size if self.tree else 0,
            'algorithm': 'bi_rrt_star',
        }

        print(f"\n{'='*40}")
        print(f"  BI-RRT* Result")
        print(f"  {'='*36}")
        print(f"  Success:        True")
        print(f"  Tree Size:      {self.tree.size if self.tree else 0}")
        print(f"  Search Time:    {search_time:.3f} s")
        print(f"  Path Length:    {path_length:.2f} m")
        print(f"{'='*40}\n")

        return path, info

    # ── Phase 1: Bidirectional RRT-Connect ──

    def _bidirectional_search(self, start: State,
                               goal: State) -> Optional[List[RRTNode]]:
        """Find initial path using bidirectional RRT-Connect.

        Two trees grow toward each other. On each iteration:
          1. Sample random point (with 20% bias toward other tree)
          2. Extend tree_a toward sample
          3. Greedily connect tree_b toward new node
          4. Swap trees for balance
        """
        root_a = RRTNode(start.x, start.y)
        root_b = RRTNode(goal.x, goal.y)
        tree_a = [root_a]
        tree_b = [root_b]

        nn_a = NearestNeighborIndex()
        nn_a.add(root_a)
        nn_b = NearestNeighborIndex()
        nn_b.add(root_b)

        uniform = UniformSampler(
            self.collision_checker.x_range,
            self.collision_checker.y_range)

        goal_bias = 0.20  # 20% as recommended in paper

        for k in range(self.max_bidir_iterations):
            # Sample with bias toward other tree (Algorithm 2, step 2a)
            if np.random.random() < goal_bias:
                target = tree_b[np.random.randint(len(tree_b))]
                sx, sy = target.x, target.y
            else:
                sx, sy = uniform.sample()

            # Extend tree_a toward sample (Algorithm 2, step 2b)
            x_near = nn_a.nearest(sx, sy)
            x_new = self.steerer.steer(x_near, sx, sy)

            # Edge collision check enforces safety margin via CollisionChecker.
            # No separate is_valid_point — edge check covers the endpoint.
            if self.collision_checker.is_collision_free(
                    x_near.x, x_near.y, x_new.x, x_new.y):

                x_new.parent = x_near
                x_new.cost = x_near.cost + x_near.distance_to(x_new)
                tree_a.append(x_new)
                nn_a.add(x_new)

                # Connect tree_b toward x_new (Algorithm 2, step 2c)
                x_connect = nn_b.nearest(x_new.x, x_new.y)
                while True:
                    x_step = self.steerer.steer(x_connect, x_new.x, x_new.y)
                    if not self.collision_checker.is_collision_free(
                            x_connect.x, x_connect.y, x_step.x, x_step.y):
                        break

                    x_step.parent = x_connect
                    x_step.cost = x_connect.cost + x_connect.distance_to(x_step)
                    tree_b.append(x_step)
                    nn_b.add(x_step)

                    if x_step.distance_to(x_new) < self.step_size * 0.5:
                        # Trees connected! Build path.
                        path_a = self._trace_to_root(x_new)
                        path_b = self._trace_to_root(x_step)
                        path_b.reverse()
                        return path_a + path_b

                    x_connect = x_step

            # Swap trees for balance (Algorithm 2, step 2d)
            if len(tree_a) > len(tree_b):
                tree_a, tree_b = tree_b, tree_a
                nn_a, nn_b = nn_b, nn_a

        return None

    def _trace_to_root(self, node: RRTNode) -> List[RRTNode]:
        """Walk parent pointers from node to root, return root-to-node order."""
        path = []
        current = node
        while current is not None:
            path.append(current)
            current = current.parent
        path.reverse()
        return path

    # ── Phase 2: Path Pruning ──

    def _prune_path(self, path: List[RRTNode]) -> List[RRTNode]:
        """Greedy waypoint skipping: from each anchor, jump to farthest visible point."""
        if len(path) <= 2:
            return path

        pruned = [path[0]]
        left = 0

        while left < len(path) - 1:
            right = left + 1
            while right + 1 < len(path):
                if self.collision_checker.is_collision_free(
                        path[left].x, path[left].y,
                        path[right + 1].x, path[right + 1].y):
                    right += 1
                else:
                    break
            pruned.append(path[right])
            left = right

        return pruned

    # ── Phase 3: Informed RRT* Optimization ──

    def _informed_optimize(self, start: State, goal: State,
                            c_best: float,
                            on_iteration: Optional[Callable] = None
                            ) -> Tuple[Optional[List[RRTNode]], float, List[float]]:
        """Run Informed RRT* with c_best from pruning as initial ellipse size."""
        root = RRTNode(start.x, start.y)
        self.tree = Tree(root)
        nn_index = NearestNeighborIndex()
        nn_index.add(root)

        informed = InformedSampler(
            start.x, start.y, goal.x, goal.y,
            self.collision_checker.x_range,
            self.collision_checker.y_range)

        goal_nodes: List[RRTNode] = []
        best_goal_node: Optional[RRTNode] = None
        cost_history: List[float] = []
        iters_since_improvement = 0

        for i in range(1, self.max_opt_iterations + 1):
            new_edge = None
            sx, sy = informed.sample(c_best)

            x_nearest = nn_index.nearest(sx, sy)
            x_new = self.steerer.steer(x_nearest, sx, sy)

            if (self.collision_checker.is_collision_free(
                    x_nearest.x, x_nearest.y, x_new.x, x_new.y) and
                    self.collision_checker.is_valid_point(x_new.x, x_new.y)):

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
                prev_best = c_best
                new_best = float('inf')
                new_best_node = None
                for gn in goal_nodes:
                    tc = gn.cost + gn.distance_to_point(goal.x, goal.y)
                    if tc < new_best:
                        new_best = tc
                        new_best_node = gn
                if new_best < c_best:
                    c_best = new_best
                    best_goal_node = new_best_node

                if c_best < prev_best - 1e-6:
                    iters_since_improvement = 0
                else:
                    iters_since_improvement += 1
            else:
                iters_since_improvement += 1

            cost_history.append(c_best)

            if i % 2000 == 0:
                print(f"    Opt iteration {i}/{self.max_opt_iterations}, "
                      f"tree: {self.tree.size}, c_best: {c_best:.2f}")

            if on_iteration is not None:
                on_iteration(i, self.tree, c_best, new_edge)

            # Early termination if converged
            if (self.convergence_patience > 0 and
                    iters_since_improvement >= self.convergence_patience):
                print(f"    Opt converged at iteration {i} "
                      f"(no improvement for {self.convergence_patience} iters)")
                break

        if best_goal_node is not None:
            path = self.tree.extract_path(best_goal_node)
            path.append(RRTNode(goal.x, goal.y))
            return path, c_best, cost_history

        return None, c_best, cost_history

    # ── Helpers ──

    def _compute_rewire_radius(self, n: int) -> float:
        if n <= 1:
            return self.step_size * 3
        r = self.gamma * (np.log(n) / n) ** (1.0 / self.dimension)
        return min(r, self.step_size * 3)

    def _path_length(self, path: List[RRTNode]) -> float:
        total = 0.0
        for i in range(len(path) - 1):
            total += path[i].distance_to(path[i+1])
        return total

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
