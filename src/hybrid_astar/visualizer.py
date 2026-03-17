"""
Visualization for Hybrid A* planner results.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from typing import List, Dict, Optional
from src.state import State
from src.map_handler import MapHandler
from src.hybrid_astar.motion_model import VehicleFootprint


class Visualizer:
    """Visualization for path planning results."""

    def __init__(self, map_handler: MapHandler,
                 vehicle_footprint: VehicleFootprint):
        self.map_handler = map_handler
        self.vehicle_footprint = vehicle_footprint

    def plot_results(self, start: State, goal: State,
                     path: Optional[List[State]] = None,
                     info: Optional[Dict] = None,
                     save_path: str = None,
                     show_vehicle: bool = True,
                     vehicle_interval: int = 10):
        """
        Plot complete planning result.

        Args:
            start: Start state
            goal: Goal state
            path: Planned path (None if failed)
            info: Planning statistics
            save_path: Path to save figure
            show_vehicle: Show vehicle footprints along path
            vehicle_interval: Show vehicle every N states
        """
        fig, ax = plt.subplots(figsize=(14, 12))

        self.map_handler.plot(ax)

        # Start and goal
        self._plot_state(ax, start, color='green', label='Start',
                         marker='o', size=200)
        self._plot_state(ax, goal, color='red', label='Goal',
                         marker='*', size=300)

        # Path
        if path is not None and len(path) > 0:
            x_coords = [s.x for s in path]
            y_coords = [s.y for s in path]
            ax.plot(x_coords, y_coords, 'b-', linewidth=2,
                    label='Path', alpha=0.7)

            if show_vehicle:
                for i in range(0, len(path), vehicle_interval):
                    self._plot_vehicle(ax, path[i], alpha=0.3,
                                       edgecolor='blue', facecolor='cyan')
                self._plot_vehicle(ax, start, alpha=0.5,
                                   edgecolor='green', facecolor='lightgreen')
                self._plot_vehicle(ax, goal, alpha=0.5,
                                   edgecolor='red', facecolor='lightcoral')

        # Statistics
        if info is not None:
            stats_text = self._format_statistics(info)
            ax.text(0.02, 0.98, stats_text,
                    transform=ax.transAxes,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    fontsize=10, family='monospace')

        ax.legend(loc='upper right', fontsize=12)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"Figure saved to: {save_path}")

        return fig, ax

    def _plot_state(self, ax, state: State, color: str, label: str,
                    marker: str = 'o', size: int = 100):
        """Plot state as point with heading arrow."""
        ax.scatter(state.x, state.y, c=color, s=size,
                   marker=marker, label=label, zorder=5,
                   edgecolors='black', linewidths=1.5)

        arrow_length = 2.0
        dx = arrow_length * np.cos(state.theta)
        dy = arrow_length * np.sin(state.theta)
        ax.arrow(state.x, state.y, dx, dy,
                 head_width=0.5, head_length=0.5,
                 fc=color, ec='black', linewidth=1.5,
                 zorder=5, alpha=0.7)

    def _plot_vehicle(self, ax, state: State, alpha: float = 0.3,
                      edgecolor: str = 'blue', facecolor: str = 'cyan'):
        """Plot vehicle footprint rectangle at given state."""
        vertices = self.vehicle_footprint.get_vertices_world(state)
        polygon = Polygon(vertices, closed=True,
                          edgecolor=edgecolor,
                          facecolor=facecolor,
                          alpha=alpha, linewidth=1.5)
        ax.add_patch(polygon)

    @staticmethod
    def _format_statistics(info: Dict) -> str:
        """Format statistics for display."""
        lines = [
            "Planning Statistics",
            "=" * 30,
            f"Success: {'Yes' if info.get('success') else 'No'}",
        ]

        if info.get('success'):
            lines.extend([
                f"Path Length: {info.get('path_length', 0):.2f} m",
                f"Nodes Expanded: {info.get('nodes_expanded', 0)}",
                f"Nodes Visited: {info.get('nodes_visited', 0)}",
                f"Search Time: {info.get('search_time', 0):.3f} s",
            ])
            if info.get('analytical'):
                lines.append("Analytical Shot: Yes")

        return '\n'.join(lines)
