"""Visualization utilities for RRT* family algorithms."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Ellipse
from typing import Optional, List, Dict, Tuple

from src.state import State


class RRTVisualizer:
    """Visualizer for RRT* tree, path, ellipse, and comparison plots.

    Args:
        map_handler: MapHandler instance for background plotting
    """

    def __init__(self, map_handler) -> None:
        self._map = map_handler

    def plot_results(self, start: State, goal: State,
                     path: List[State], info: Dict,
                     tree=None,
                     ellipse_params: Optional[Dict] = None,
                     save_path: Optional[str] = None) -> None:
        """Plot a single algorithm result with tree, path, and stats.

        Args:
            start: start state
            goal: goal state
            path: planned path as List[State]
            info: info dict from planner
            tree: Tree object (optional, for drawing edges)
            ellipse_params: dict with center, angle_deg, semi_major, semi_minor
            save_path: file path to save figure (optional)
        """
        fig, ax = plt.subplots(figsize=(10, 10))
        self._map.plot(ax)

        # Draw tree edges
        if tree is not None:
            edges = tree.get_all_edges()
            if edges:
                segments = [[(p.x, p.y), (c.x, c.y)] for p, c in edges]
                lc = LineCollection(segments, colors='lightblue',
                                    linewidths=0.3, alpha=0.4)
                ax.add_collection(lc)

        # Draw ellipse
        if ellipse_params is not None:
            ellipse = Ellipse(
                xy=ellipse_params['center'],
                width=2 * ellipse_params['semi_major'],
                height=2 * ellipse_params['semi_minor'],
                angle=ellipse_params['angle_deg'],
                fill=False, edgecolor='green', linewidth=1.5,
                linestyle='--', alpha=0.7)
            ax.add_patch(ellipse)

        # Draw path
        if path:
            xs = [s.x for s in path]
            ys = [s.y for s in path]
            ax.plot(xs, ys, 'b-', linewidth=2.5, label='Path', zorder=5)

        # Start and goal markers
        ax.plot(start.x, start.y, 'gs', markersize=12, zorder=10, label='Start')
        ax.plot(goal.x, goal.y, 'r^', markersize=12, zorder=10, label='Goal')

        # Draw heading arrows
        arrow_len = 3.0
        for s, color in [(start, 'green'), (goal, 'red')]:
            ax.arrow(s.x, s.y,
                     arrow_len * np.cos(s.theta),
                     arrow_len * np.sin(s.theta),
                     head_width=1.0, color=color, zorder=11)

        # Stats text
        alg_name = info.get('algorithm', 'RRT*')
        stats = (f"{alg_name}\n"
                 f"Time: {info.get('search_time', 0):.3f}s\n"
                 f"Length: {info.get('path_length', 0):.1f}m\n"
                 f"Nodes: {info.get('tree_size', 0)}")
        ax.text(0.02, 0.98, stats, transform=ax.transAxes,
                verticalalignment='top', fontsize=10,
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        ax.set_title(f"{alg_name} Result", fontsize=14)
        ax.legend(loc='lower right')
        plt.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  Comparison saved to: {save_path}")

    def plot_comparison(self, results: Dict[str, Tuple[List[State], Dict]],
                        start: State, goal: State,
                        save_path: Optional[str] = None) -> None:
        """Plot side-by-side comparison of multiple algorithms.

        Args:
            results: dict mapping algorithm name -> (path, info)
            start: start state
            goal: goal state
            save_path: file path to save figure
        """
        n = len(results)
        cols = min(n, 2)
        rows = (n + cols - 1) // cols
        fig, axes = plt.subplots(rows, cols, figsize=(8 * cols, 8 * rows))
        if n == 1:
            axes = [axes]
        else:
            axes = axes.flatten()

        for idx, (name, (path, info)) in enumerate(results.items()):
            ax = axes[idx]
            self._map.plot(ax)

            if path:
                xs = [s.x for s in path]
                ys = [s.y for s in path]
                ax.plot(xs, ys, 'b-', linewidth=2, zorder=5)

            ax.plot(start.x, start.y, 'gs', markersize=10, zorder=10)
            ax.plot(goal.x, goal.y, 'r^', markersize=10, zorder=10)

            success = info.get('success', False)
            t = info.get('search_time', 0)
            length = info.get('path_length', 0)
            tree_sz = info.get('tree_size', 0)
            ax.set_title(f"{name}\n{'OK' if success else 'FAIL'} | "
                         f"{t:.2f}s | {length:.1f}m | {tree_sz} nodes",
                         fontsize=11)

        # Hide unused axes
        for idx in range(n, len(axes)):
            axes[idx].set_visible(False)

        plt.tight_layout()
        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  Comparison saved to: {save_path}")

    def plot_cost_convergence(self, ax, cost_history: List[float],
                              label: str = '', color: str = 'blue') -> None:
        """Plot cost convergence on an existing axes.

        Args:
            ax: matplotlib Axes
            cost_history: list of best costs per iteration
            label: legend label
            color: line color
        """
        # Filter out inf values for plotting
        finite = [(i, c) for i, c in enumerate(cost_history)
                  if c < 1e18]
        if not finite:
            return
        xs, ys = zip(*finite)
        ax.plot(xs, ys, color=color, linewidth=1.5, label=label)
        ax.set_xlabel('Iteration')
        ax.set_ylabel('Best Cost (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
