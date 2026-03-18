#!/usr/bin/env python3
"""
RRT* Family — Progressive Tree Growth Visualization

Records rich per-iteration data (new edges, rewired edges, current best path,
sample points) then replays frame-by-frame showing exactly how the algorithm
explores, connects, rewires, and optimises.

What you see:
  - Tree grows progressively outward from the start node
  - Each new node: sample point (faint dot) + edge to chosen parent
  - Rewiring: old edge removed (disappears), new edge added (yellow flash)
  - Path appears ONLY when goal is first reached, then updates as it improves
  - Informed/BI-RRT*: ellipse appears after first solution, shrinks with cost
  - BI-RRT*: shows Phase 1 (bidirectional), Phase 2 (pruned path), Phase 3 (optimization)
  - Cost convergence plot updates alongside

Usage:
    python scripts/rrt_simulation.py --algorithm rrt_star
    python scripts/rrt_simulation.py --algorithm informed_rrt_star
    python scripts/rrt_simulation.py --algorithm bi_rrt_star
    python scripts/rrt_simulation.py --compare
    python scripts/rrt_simulation.py --speed 10    # slower (10 edges/frame)
"""

import argparse
import os
import sys
import numpy as np
import yaml
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Ellipse, Circle
from matplotlib.animation import FuncAnimation

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.map_handler import MapHandler
from src.state import State
from src.rrt_star.node import RRTNode
from src.rrt_star.tree import Tree
from src.rrt_star.collision_checker import CollisionChecker
from src.rrt_star.steering import StraightLineSteerer
from src.rrt_star.nearest_neighbor import NearestNeighborIndex
from src.rrt_star.samplers import UniformSampler, GoalBiasedSampler, InformedSampler


# ══════════════════════════════════════════════════════════════
# Recording: run each algorithm step-by-step, capture everything
# ══════════════════════════════════════════════════════════════

def record_rrt_star(map_handler, config, start, goal):
    """Run RRT* and record every iteration's visual state."""
    cfg = config.get('rrt_star', {})
    max_iter = cfg.get('max_iterations', 10000)
    step_size = cfg.get('step_size', 2.0)
    goal_radius = cfg.get('goal_radius', 2.0)
    goal_bias = cfg.get('goal_bias', 0.05)
    gamma = cfg.get('gamma', 20.0)
    dim = cfg.get('dimension', 2)

    cc = CollisionChecker(map_handler)
    steerer = StraightLineSteerer(step_size)

    root = RRTNode(start.x, start.y)
    tree = Tree(root)
    nn = NearestNeighborIndex()
    nn.add(root)

    uniform = UniformSampler(cc.x_range, cc.y_range)
    sampler = GoalBiasedSampler(uniform, goal.x, goal.y, goal_bias)

    goal_nodes = []  # X_soln: all nodes that reached goal
    best_goal_node = None
    best_cost = float('inf')
    prev_cost = float('inf')
    frames = []
    improvement_count = 0

    def get_path():
        if best_goal_node is None:
            return None
        path = tree.extract_path(best_goal_node)
        return [(n.x, n.y) for n in path] + [(goal.x, goal.y)]

    for i in range(1, max_iter + 1):
        sx, sy = sampler.sample()
        x_nearest = nn.nearest(sx, sy)
        x_new = steerer.steer(x_nearest, sx, sy)

        if not (cc.is_collision_free(x_nearest.x, x_nearest.y, x_new.x, x_new.y)
                and cc.is_valid_point(x_new.x, x_new.y)):
            continue

        n = tree.size
        if n <= 1:
            r_n = step_size * 3
        else:
            r_n = min(gamma * (np.log(n) / n) ** (1.0 / dim), step_size * 3)

        X_near = nn.near(x_new.x, x_new.y, r_n)

        x_min = x_nearest
        c_min = x_nearest.cost + x_nearest.distance_to(x_new)
        for xn in X_near:
            c = xn.cost + xn.distance_to(x_new)
            if c < c_min and cc.is_collision_free(xn.x, xn.y, x_new.x, x_new.y):
                x_min = xn
                c_min = c

        tree.add_node(x_new, x_min)
        x_new.cost = c_min
        nn.add(x_new)
        new_edge = [(x_min.x, x_min.y), (x_new.x, x_new.y)]

        rewired = []
        for xn in X_near:
            if xn is x_min:
                continue
            pot = x_new.cost + x_new.distance_to(xn)
            if pot < xn.cost and cc.is_collision_free(x_new.x, x_new.y, xn.x, xn.y):
                old_parent = xn.parent
                rewired.append({
                    'old_edge': [(old_parent.x, old_parent.y), (xn.x, xn.y)],
                    'new_edge': [(x_new.x, x_new.y), (xn.x, xn.y)],
                })
                tree.rewire(xn, x_new, pot)

        # Add to X_soln if reaches goal
        d = x_new.distance_to_point(goal.x, goal.y)
        if d <= goal_radius:
            if cc.is_collision_free(x_new.x, x_new.y, goal.x, goal.y):
                goal_nodes.append(x_new)

        # Re-evaluate best through ALL goal nodes (costs change via rewiring)
        path_snapshot = None
        if goal_nodes:
            new_best = float('inf')
            new_best_node = None
            for gn in goal_nodes:
                tc = gn.cost + gn.distance_to_point(goal.x, goal.y)
                if tc < new_best:
                    new_best = tc
                    new_best_node = gn
            best_cost = new_best
            best_goal_node = new_best_node

            if best_cost < prev_cost - 0.01:
                improvement_count += 1
                path_snapshot = get_path()
                prev_cost = best_cost

        phase = 'Exploring' if best_goal_node is None else 'Optimizing'
        frames.append({
            'iteration': i,
            'sample': (sx, sy),
            'nearest': (x_nearest.x, x_nearest.y),
            'new_edge': new_edge,
            'rewired': rewired,
            'best_cost': best_cost,
            'tree_size': tree.size,
            'path': path_snapshot,
            'improvement': improvement_count if path_snapshot else 0,
            'phase': f'RRT*: {phase}',
        })

        if i % 2000 == 0:
            print(f"  RRT* iter {i}/{max_iter}, nodes={tree.size}, "
                  f"cost={best_cost:.2f}, improvements={improvement_count}")

    frames.append({
        'iteration': max_iter,
        'new_edge': None, 'sample': None, 'nearest': None, 'rewired': [],
        'best_cost': best_cost, 'tree_size': tree.size,
        'path': get_path(), 'improvement': improvement_count,
        'phase': f'RRT*: Complete ({improvement_count} improvements)',
    })

    print(f"  RRT* done: {tree.size} nodes, cost={best_cost:.2f}, "
          f"improvements={improvement_count}")
    return frames


def record_informed_rrt_star(map_handler, config, start, goal):
    """Run Informed RRT* and record every iteration."""
    cfg = config.get('informed_rrt_star', {})
    max_iter = cfg.get('max_iterations', 10000)
    step_size = cfg.get('step_size', 2.0)
    goal_radius = cfg.get('goal_radius', 2.0)
    goal_bias = cfg.get('goal_bias', 0.05)
    gamma = cfg.get('gamma', 20.0)
    dim = cfg.get('dimension', 2)

    cc = CollisionChecker(map_handler)
    steerer = StraightLineSteerer(step_size)

    root = RRTNode(start.x, start.y)
    tree = Tree(root)
    nn = NearestNeighborIndex()
    nn.add(root)

    uniform = UniformSampler(cc.x_range, cc.y_range)
    goal_biased = GoalBiasedSampler(uniform, goal.x, goal.y, goal_bias)
    informed = InformedSampler(start.x, start.y, goal.x, goal.y,
                                cc.x_range, cc.y_range)

    goal_nodes = []
    best_goal_node = None
    best_cost = float('inf')
    prev_cost = float('inf')
    frames = []
    improvement_count = 0

    def get_path():
        if best_goal_node is None:
            return None
        path = tree.extract_path(best_goal_node)
        return [(n.x, n.y) for n in path] + [(goal.x, goal.y)]

    for i in range(1, max_iter + 1):
        if best_cost < float('inf'):
            sx, sy = informed.sample(best_cost)
        else:
            sx, sy = goal_biased.sample()

        x_nearest = nn.nearest(sx, sy)
        x_new = steerer.steer(x_nearest, sx, sy)

        if not (cc.is_collision_free(x_nearest.x, x_nearest.y, x_new.x, x_new.y)
                and cc.is_valid_point(x_new.x, x_new.y)):
            continue

        n = tree.size
        r_n = min(gamma * (np.log(max(n, 2)) / max(n, 2)) ** (1.0/dim), step_size*3)
        X_near = nn.near(x_new.x, x_new.y, r_n)

        x_min = x_nearest
        c_min = x_nearest.cost + x_nearest.distance_to(x_new)
        for xn in X_near:
            c = xn.cost + xn.distance_to(x_new)
            if c < c_min and cc.is_collision_free(xn.x, xn.y, x_new.x, x_new.y):
                x_min = xn
                c_min = c

        tree.add_node(x_new, x_min)
        x_new.cost = c_min
        nn.add(x_new)
        new_edge = [(x_min.x, x_min.y), (x_new.x, x_new.y)]

        rewired = []
        for xn in X_near:
            if xn is x_min:
                continue
            pot = x_new.cost + x_new.distance_to(xn)
            if pot < xn.cost and cc.is_collision_free(x_new.x, x_new.y, xn.x, xn.y):
                old_p = xn.parent
                rewired.append({
                    'old_edge': [(old_p.x, old_p.y), (xn.x, xn.y)],
                    'new_edge': [(x_new.x, x_new.y), (xn.x, xn.y)],
                })
                tree.rewire(xn, x_new, pot)

        d = x_new.distance_to_point(goal.x, goal.y)
        if d <= goal_radius:
            if cc.is_collision_free(x_new.x, x_new.y, goal.x, goal.y):
                goal_nodes.append(x_new)

        path_snapshot = None
        if goal_nodes:
            new_best = float('inf')
            new_best_node = None
            for gn in goal_nodes:
                tc = gn.cost + gn.distance_to_point(goal.x, goal.y)
                if tc < new_best:
                    new_best = tc
                    new_best_node = gn
            best_cost = new_best
            best_goal_node = new_best_node

            if best_cost < prev_cost - 0.01:
                improvement_count += 1
                path_snapshot = get_path()
                prev_cost = best_cost

        ellipse = informed.get_ellipse_params(best_cost)
        phase = 'Exploring' if best_goal_node is None else 'Optimizing (ellipse)'

        frames.append({
            'iteration': i,
            'sample': (sx, sy),
            'nearest': (x_nearest.x, x_nearest.y),
            'new_edge': new_edge,
            'rewired': rewired,
            'best_cost': best_cost,
            'tree_size': tree.size,
            'path': path_snapshot,
            'ellipse': ellipse,
            'improvement': improvement_count if path_snapshot else 0,
            'phase': f'Informed RRT*: {phase}',
        })

        if i % 2000 == 0:
            print(f"  Informed RRT* iter {i}/{max_iter}, nodes={tree.size}, "
                  f"cost={best_cost:.2f}, improvements={improvement_count}")

    frames.append({
        'iteration': max_iter,
        'new_edge': None, 'sample': None, 'nearest': None, 'rewired': [],
        'best_cost': best_cost, 'tree_size': tree.size,
        'path': get_path(), 'ellipse': informed.get_ellipse_params(best_cost),
        'improvement': improvement_count,
        'phase': f'Informed RRT*: Complete ({improvement_count} improvements)',
    })

    print(f"  Informed RRT* done: {tree.size} nodes, cost={best_cost:.2f}, "
          f"improvements={improvement_count}")
    return frames


def record_bi_rrt_star(map_handler, config, start, goal):
    """Run BI-RRT* and record all 3 phases with rich per-iteration data."""
    cfg = config.get('bi_rrt_star', {})
    max_bidir = cfg.get('max_bidir_iterations', 5000)
    max_opt = cfg.get('max_opt_iterations', 5000)
    step_size = cfg.get('step_size', 2.0)
    goal_radius = cfg.get('goal_radius', 2.0)
    gamma = cfg.get('gamma', 20.0)
    dim = cfg.get('dimension', 2)

    cc = CollisionChecker(map_handler)
    steerer = StraightLineSteerer(step_size)
    uniform = UniformSampler(cc.x_range, cc.y_range)

    frames = []

    # ── Phase 1: Bidirectional RRT-Connect ──
    print("  BI-RRT* Phase 1: Bidirectional search...")
    root_a = RRTNode(start.x, start.y)
    root_b = RRTNode(goal.x, goal.y)
    tree_a, tree_b = [root_a], [root_b]
    nn_a, nn_b = NearestNeighborIndex(), NearestNeighborIndex()
    nn_a.add(root_a)
    nn_b.add(root_b)

    initial_path = None

    for k in range(max_bidir):
        if np.random.random() < 0.20:
            tgt = tree_b[np.random.randint(len(tree_b))]
            sx, sy = tgt.x, tgt.y
        else:
            sx, sy = uniform.sample()

        x_near = nn_a.nearest(sx, sy)
        x_new = steerer.steer(x_near, sx, sy)

        if not cc.is_collision_free(x_near.x, x_near.y, x_new.x, x_new.y):
            tree_a, tree_b = tree_b, tree_a
            nn_a, nn_b = nn_b, nn_a
            continue

        x_new.parent = x_near
        x_new.cost = x_near.cost + x_near.distance_to(x_new)
        tree_a.append(x_new)
        nn_a.add(x_new)

        # Record this expansion
        frames.append({
            'iteration': k,
            'new_edge': [(x_near.x, x_near.y), (x_new.x, x_new.y)],
            'sample': (sx, sy),
            'nearest': (x_near.x, x_near.y),
            'rewired': [],
            'best_cost': float('inf'),
            'tree_size': len(tree_a) + len(tree_b),
            'path': None,
            'phase': 'Phase 1: Bidirectional',
        })

        # Connect tree_b toward x_new
        x_connect = nn_b.nearest(x_new.x, x_new.y)
        while True:
            x_step = steerer.steer(x_connect, x_new.x, x_new.y)
            if not cc.is_collision_free(x_connect.x, x_connect.y, x_step.x, x_step.y):
                break
            x_step.parent = x_connect
            x_step.cost = x_connect.cost + x_connect.distance_to(x_step)
            tree_b.append(x_step)
            nn_b.add(x_step)

            frames.append({
                'iteration': k,
                'new_edge': [(x_connect.x, x_connect.y), (x_step.x, x_step.y)],
                'sample': None, 'nearest': None, 'rewired': [],
                'best_cost': float('inf'),
                'tree_size': len(tree_a) + len(tree_b),
                'path': None,
                'phase': 'Phase 1: Connecting',
            })

            if x_step.distance_to(x_new) < step_size * 0.5:
                # Connected!
                def trace(node):
                    p = []
                    while node:
                        p.append(node)
                        node = node.parent
                    p.reverse()
                    return p
                path_a = trace(x_new)
                path_b = trace(x_step)
                path_b.reverse()
                initial_path = path_a + path_b
                break
            x_connect = x_step

        if initial_path is not None:
            break

        if len(tree_a) > len(tree_b):
            tree_a, tree_b = tree_b, tree_a
            nn_a, nn_b = nn_b, nn_a

    if initial_path is None:
        print("  Phase 1 FAILED")
        return frames

    init_cost = sum(initial_path[i].distance_to(initial_path[i+1])
                     for i in range(len(initial_path)-1))
    init_path_xy = [(n.x, n.y) for n in initial_path]
    print(f"    Initial cost: {init_cost:.2f}")

    # Mark the initial path frame
    frames.append({
        'iteration': len(frames),
        'new_edge': None, 'sample': None, 'nearest': None, 'rewired': [],
        'best_cost': init_cost,
        'tree_size': len(tree_a) + len(tree_b),
        'path': init_path_xy,
        'phase': 'Phase 1: Path Found!',
    })

    # ── Phase 2: Pruning ──
    print("  BI-RRT* Phase 2: Pruning...")
    pruned = [initial_path[0]]
    left = 0
    while left < len(initial_path) - 1:
        right = left + 1
        while right + 1 < len(initial_path):
            if cc.is_collision_free(initial_path[left].x, initial_path[left].y,
                                     initial_path[right+1].x, initial_path[right+1].y):
                right += 1
            else:
                break
        pruned.append(initial_path[right])
        left = right

    pruned_cost = sum(pruned[i].distance_to(pruned[i+1]) for i in range(len(pruned)-1))
    pruned_path_xy = [(n.x, n.y) for n in pruned]
    print(f"    Pruned cost: {pruned_cost:.2f} ({len(initial_path)}->{len(pruned)} pts)")

    frames.append({
        'iteration': len(frames),
        'new_edge': None, 'sample': None, 'nearest': None, 'rewired': [],
        'best_cost': pruned_cost,
        'tree_size': len(tree_a) + len(tree_b),
        'path': pruned_path_xy,
        'phase': 'Phase 2: Pruned Path',
    })

    # ── Phase 3: Informed optimization ──
    print("  BI-RRT* Phase 3: Informed optimization...")
    opt_root = RRTNode(start.x, start.y)
    opt_tree = Tree(opt_root)
    opt_nn = NearestNeighborIndex()
    opt_nn.add(opt_root)
    informed = InformedSampler(start.x, start.y, goal.x, goal.y,
                                cc.x_range, cc.y_range)

    best_goal_node = None
    c_best = pruned_cost

    opt_goal_nodes = []
    opt_improvement_count = 0
    opt_prev_cost = c_best

    def get_opt_path():
        if best_goal_node is None:
            return None
        p = opt_tree.extract_path(best_goal_node)
        return [(n.x, n.y) for n in p] + [(goal.x, goal.y)]

    for i in range(1, max_opt + 1):
        sx, sy = informed.sample(c_best)
        x_nearest = opt_nn.nearest(sx, sy)
        x_new = steerer.steer(x_nearest, sx, sy)

        if not (cc.is_collision_free(x_nearest.x, x_nearest.y, x_new.x, x_new.y)
                and cc.is_valid_point(x_new.x, x_new.y)):
            continue

        n = opt_tree.size
        r_n = min(gamma * (np.log(max(n,2))/max(n,2))**(1.0/dim), step_size*3)
        X_near = opt_nn.near(x_new.x, x_new.y, r_n)

        x_min = x_nearest
        c_min = x_nearest.cost + x_nearest.distance_to(x_new)
        for xn in X_near:
            c = xn.cost + xn.distance_to(x_new)
            if c < c_min and cc.is_collision_free(xn.x, xn.y, x_new.x, x_new.y):
                x_min = xn
                c_min = c

        opt_tree.add_node(x_new, x_min)
        x_new.cost = c_min
        opt_nn.add(x_new)

        rewired = []
        for xn in X_near:
            if xn is x_min:
                continue
            pot = x_new.cost + x_new.distance_to(xn)
            if pot < xn.cost and cc.is_collision_free(x_new.x, x_new.y, xn.x, xn.y):
                old_p = xn.parent
                rewired.append({
                    'old_edge': [(old_p.x, old_p.y), (xn.x, xn.y)],
                    'new_edge': [(x_new.x, x_new.y), (xn.x, xn.y)],
                })
                opt_tree.rewire(xn, x_new, pot)

        d = x_new.distance_to_point(goal.x, goal.y)
        if d <= goal_radius:
            if cc.is_collision_free(x_new.x, x_new.y, goal.x, goal.y):
                opt_goal_nodes.append(x_new)

        path_snapshot = None
        if opt_goal_nodes:
            new_best = float('inf')
            new_best_node = None
            for gn in opt_goal_nodes:
                tc = gn.cost + gn.distance_to_point(goal.x, goal.y)
                if tc < new_best:
                    new_best = tc
                    new_best_node = gn
            if new_best < c_best:
                c_best = new_best
                best_goal_node = new_best_node
            if c_best < opt_prev_cost - 0.01:
                opt_improvement_count += 1
                path_snapshot = get_opt_path()
                opt_prev_cost = c_best

        ellipse = informed.get_ellipse_params(c_best)

        frames.append({
            'iteration': i,
            'new_edge': [(x_min.x, x_min.y), (x_new.x, x_new.y)],
            'sample': (sx, sy),
            'nearest': (x_nearest.x, x_nearest.y),
            'rewired': rewired,
            'best_cost': c_best,
            'tree_size': opt_tree.size,
            'path': path_snapshot,
            'ellipse': ellipse,
            'improvement': opt_improvement_count if path_snapshot else 0,
            'phase': 'Phase 3: Optimizing',
        })

        if i % 2000 == 0:
            print(f"    Opt iter {i}/{max_opt}, nodes={opt_tree.size}, "
                  f"cost={c_best:.2f}, improvements={opt_improvement_count}")

    # Final frame
    final_path = get_opt_path() or pruned_path_xy
    frames.append({
        'iteration': max_opt,
        'new_edge': None, 'sample': None, 'nearest': None, 'rewired': [],
        'best_cost': c_best, 'tree_size': opt_tree.size,
        'path': final_path, 'ellipse': informed.get_ellipse_params(c_best),
        'improvement': opt_improvement_count,
        'phase': f'BI-RRT*: Complete ({opt_improvement_count} Phase 3 improvements)',
    })

    print(f"  BI-RRT* done: cost={c_best:.2f}, improvements={opt_improvement_count}")
    return frames


RECORDERS = {
    'rrt_star': ('RRT*', record_rrt_star),
    'informed_rrt_star': ('Informed RRT*', record_informed_rrt_star),
    'bi_rrt_star': ('BI-RRT*', record_bi_rrt_star),
}


# ══════════════════════════════════════════════════════════════
# Animation: progressive replay of recorded frames
# ══════════════════════════════════════════════════════════════

class Simulation:
    """Progressive frame-by-frame replay of an RRT* algorithm."""

    def __init__(self, alg_label, frames, map_handler, start, goal,
                 edges_per_frame=20):
        self.alg_label = alg_label
        self.frames = frames
        self.start = start
        self.goal = goal
        self.epf = edges_per_frame
        self.cursor = 0

        # Accumulated state
        self.all_edges = []          # all tree edges currently alive
        self.edge_set = set()        # for fast removal on rewire
        self.improvement_count = 0   # how many times path improved
        self.all_node_xs = []        # all node positions for scatter
        self.all_node_ys = []
        self.cost_iters = []
        self.cost_vals = []
        self.current_path = None
        self.prev_cost = float('inf')

        # ── Figure ──
        self.fig = plt.figure(figsize=(16, 9))
        gs = self.fig.add_gridspec(2, 2, width_ratios=[2.2, 1],
                                    height_ratios=[3, 1], hspace=0.35, wspace=0.25)
        self.ax_map = self.fig.add_subplot(gs[:, 0])
        self.ax_cost = self.fig.add_subplot(gs[0, 1])
        self.ax_info = self.fig.add_subplot(gs[1, 1])
        self.ax_info.axis('off')

        # Map
        map_handler.plot(self.ax_map)
        Circle_goal = Circle((goal.x, goal.y), 2.0, fill=False,
                              edgecolor='red', linewidth=1, linestyle=':', alpha=0.4)
        self.ax_map.add_patch(Circle_goal)

        self.ax_map.plot(start.x, start.y, 'o', color='limegreen', markersize=14,
                          zorder=20, markeredgecolor='black', markeredgewidth=1.5)
        self.ax_map.plot(goal.x, goal.y, '^', color='red', markersize=14,
                          zorder=20, markeredgecolor='black', markeredgewidth=1.5)
        aw = dict(head_width=1.2, zorder=21, length_includes_head=True)
        self.ax_map.arrow(start.x, start.y, 4*np.cos(start.theta), 4*np.sin(start.theta),
                           color='limegreen', **aw)
        self.ax_map.arrow(goal.x, goal.y, 4*np.cos(goal.theta), 4*np.sin(goal.theta),
                           color='red', **aw)

        # Artists
        self.tree_lc = LineCollection([], colors='lightsteelblue',
                                       linewidths=0.4, alpha=0.6, zorder=2)
        self.ax_map.add_collection(self.tree_lc)

        self.new_lc = LineCollection([], colors='dodgerblue',
                                      linewidths=1.5, alpha=0.9, zorder=4)
        self.ax_map.add_collection(self.new_lc)

        self.rewire_lc = LineCollection([], colors='gold',
                                         linewidths=1.8, alpha=0.9, zorder=5)
        self.ax_map.add_collection(self.rewire_lc)

        # Node scatter: all accumulated nodes (faint small circles)
        self.node_scatter = self.ax_map.scatter(
            [], [], s=6, c='steelblue', alpha=0.3, zorder=3, linewidths=0)

        # New nodes this frame (bright, slightly larger)
        self.new_node_scatter = self.ax_map.scatter(
            [], [], s=30, c='dodgerblue', alpha=0.8, zorder=6,
            edgecolors='navy', linewidths=0.5)

        # Sample points (tiny red x — shows where random sample was drawn)
        self.sample_scatter = self.ax_map.scatter(
            [], [], s=15, c='red', alpha=0.4, zorder=6, marker='x', linewidths=0.7)

        self.path_line, = self.ax_map.plot([], [], '-', color='orangered',
                                            linewidth=3.5, zorder=15)
        self.ellipse_artist = None

        # Cost
        self.cost_line, = self.ax_cost.plot([], [], 'b-', linewidth=1.5)
        self.ax_cost.set_xlabel('Iteration')
        self.ax_cost.set_ylabel('Best Cost (m)')
        self.ax_cost.set_title('Cost Convergence', fontsize=11)
        self.ax_cost.grid(True, alpha=0.3)

        # Info
        self.info_text = self.ax_info.text(
            0.05, 0.95, '', transform=self.ax_info.transAxes,
            verticalalignment='top', fontsize=11, family='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))

        self.ax_map.set_title(f'{alg_label} — Ready', fontsize=14, fontweight='bold')

    def _update(self, frame_num):
        if self.cursor >= len(self.frames):
            return []

        end = min(self.cursor + self.epf, len(self.frames))
        new_segs = []
        rewire_segs = []
        new_node_xs, new_node_ys = [], []
        sample_xs, sample_ys = [], []
        last_f = self.frames[self.cursor]

        for idx in range(self.cursor, end):
            f = self.frames[idx]
            last_f = f

            # Add new edge and track the new node position
            if f.get('new_edge') is not None:
                seg = tuple(map(tuple, f['new_edge']))
                self.all_edges.append(list(f['new_edge']))
                self.edge_set.add(seg)
                new_segs.append(list(f['new_edge']))

                # New node is the endpoint of the edge
                nx, ny = f['new_edge'][1]
                self.all_node_xs.append(nx)
                self.all_node_ys.append(ny)
                new_node_xs.append(nx)
                new_node_ys.append(ny)

            # Sample point (where the random sample was drawn)
            if f.get('sample') is not None:
                sample_xs.append(f['sample'][0])
                sample_ys.append(f['sample'][1])

            # Handle rewiring: remove old edges, add new ones
            for rw in f.get('rewired', []):
                old_seg = tuple(map(tuple, rw['old_edge']))
                if old_seg in self.edge_set:
                    self.edge_set.discard(old_seg)
                    old_list = list(rw['old_edge'])
                    if old_list in self.all_edges:
                        self.all_edges.remove(old_list)

                new_seg = tuple(map(tuple, rw['new_edge']))
                self.all_edges.append(list(rw['new_edge']))
                self.edge_set.add(new_seg)
                rewire_segs.append(list(rw['new_edge']))

            # Track cost
            cost = f['best_cost']
            if cost < 1e18:
                self.cost_iters.append(f['iteration'])
                self.cost_vals.append(cost)

            # Update path ONLY when cost actually changes (new better path found)
            if f.get('path') is not None and cost < self.prev_cost:
                self.current_path = f['path']
                self.prev_cost = cost
                self.improvement_count = f.get('improvement', self.improvement_count)

        self.cursor = end

        # ── Update artists ──
        self.tree_lc.set_segments(self.all_edges)
        self.new_lc.set_segments(new_segs)
        self.rewire_lc.set_segments(rewire_segs)

        # All accumulated nodes (faint)
        self.node_scatter.set_offsets(
            np.column_stack([self.all_node_xs, self.all_node_ys])
            if self.all_node_xs else np.empty((0, 2)))

        # New nodes this frame (bright, temporary highlight)
        self.new_node_scatter.set_offsets(
            np.column_stack([new_node_xs, new_node_ys])
            if new_node_xs else np.empty((0, 2)))

        # Sample points this frame (tiny red x, temporary)
        self.sample_scatter.set_offsets(
            np.column_stack([sample_xs, sample_ys])
            if sample_xs else np.empty((0, 2)))

        # Path
        if self.current_path is not None:
            xs, ys = zip(*self.current_path)
            self.path_line.set_data(xs, ys)

        # Cost convergence
        if self.cost_iters:
            self.cost_line.set_data(self.cost_iters, self.cost_vals)
            self.ax_cost.relim()
            self.ax_cost.autoscale_view()

        # Ellipse
        if self.ellipse_artist is not None:
            self.ellipse_artist.remove()
            self.ellipse_artist = None
        ep = last_f.get('ellipse')
        if ep is not None:
            self.ellipse_artist = Ellipse(
                xy=ep['center'],
                width=2 * ep['semi_major'], height=2 * ep['semi_minor'],
                angle=ep['angle_deg'],
                fill=True, facecolor='palegreen', edgecolor='green',
                linewidth=1.5, linestyle='--', alpha=0.12, zorder=1)
            self.ax_map.add_patch(self.ellipse_artist)

        # Title
        phase = last_f.get('phase', self.alg_label)
        cost_str = f'{last_f["best_cost"]:.1f}m' if last_f['best_cost'] < 1e18 else 'searching'
        self.ax_map.set_title(
            f'{phase}  |  nodes {last_f["tree_size"]}  |  cost: {cost_str}',
            fontsize=13, fontweight='bold')

        # Info
        progress = self.cursor / len(self.frames) * 100
        lines = [
            f'Algorithm:    {self.alg_label}',
            f'Phase:        {phase}',
            f'Iteration:    {last_f["iteration"]}',
            f'Tree nodes:   {last_f["tree_size"]}',
            f'Best cost:    {cost_str}',
            f'Improvements: {self.improvement_count}',
            f'Progress:     {progress:.0f}%',
        ]
        if self.improvement_count > 0 and self.current_path is not None:
            lines.append(f'Path updated  #{self.improvement_count}')
        self.info_text.set_text('\n'.join(lines))

        return [self.tree_lc, self.new_lc, self.rewire_lc,
                self.node_scatter, self.new_node_scatter, self.sample_scatter,
                self.path_line, self.cost_line, self.info_text]

    def run(self):
        n = (len(self.frames) // self.epf) + 30
        self.anim = FuncAnimation(self.fig, self._update, frames=n,
                                   interval=33, blit=False, repeat=False)
        plt.show()


# ══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='RRT* Family — Progressive Tree Growth Visualization')
    parser.add_argument('--algorithm', type=str, default='rrt_star',
                        choices=list(RECORDERS.keys()))
    parser.add_argument('--compare', action='store_true')
    parser.add_argument('--map', type=str, default=None)
    parser.add_argument('--speed', type=int, default=20,
                        help='Edges per animation frame (default 20)')
    args = parser.parse_args()

    config_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'planner_config.yaml')
    with open(config_path) as f:
        config = yaml.safe_load(f)
    if args.map:
        config['map']['path'] = args.map

    mh = MapHandler(config['map']['path'], config['map']['resolution'],
                     config['map']['obstacle_threshold'])
    start = State(10, 10, 0)
    goal = State(80, 80, np.pi / 2)

    print(f"Map: {config['map']['path']}")
    print(f"Start: ({start.x},{start.y}), Goal: ({goal.x},{goal.y})")
    print(f"Speed: {args.speed} edges/frame\n")

    if args.compare:
        fig, axes = plt.subplots(1, 3, figsize=(22, 8))
        all_frames = {}
        for key, (label, recorder) in RECORDERS.items():
            print(f"--- {label} ---")
            all_frames[key] = (label, recorder(mh, config, start, goal))

        # For compare mode, use separate Simulation per panel
        # but share figure — simplified: run sequentially
        print("\nLaunching animations...")
        for idx, (key, (label, frames)) in enumerate(all_frames.items()):
            print(f"  {label}: {len(frames)} frames recorded")

        # Run first algorithm as demonstration
        first_key = args.algorithm
        label, frames = all_frames.get(first_key, list(all_frames.values())[0])
        sim = Simulation(label, frames, mh, start, goal, args.speed)
        sim.run()
    else:
        label, recorder = RECORDERS[args.algorithm]
        print(f"Recording {label}...")
        frames = recorder(mh, config, start, goal)
        print(f"\n{len(frames)} frames recorded. Launching animation...\n")
        sim = Simulation(label, frames, mh, start, goal, args.speed)
        sim.run()


if __name__ == '__main__':
    main()
