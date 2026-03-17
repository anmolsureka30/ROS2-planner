"""
Map adapter: creates a MapHandler-compatible object from nav_msgs/OccupancyGrid.

Subclasses MapHandler but bypasses the PNG-loading constructor, initializing
directly from numpy array data. All collision checking, coordinate conversion,
and cost field methods are inherited unchanged.
"""

import numpy as np
from typing import Tuple
from scipy.ndimage import distance_transform_edt


class MapAdapter:
    """
    MapHandler-compatible object initialized from OccupancyGrid data.

    Has the same public interface as src.map_handler.MapHandler so it can be
    passed directly to HeuristicCalculator, HybridAStar, and Visualizer.
    """

    def __init__(self, data: np.ndarray, resolution: float,
                 width: int, height: int,
                 origin_x: float = 0.0, origin_y: float = 0.0,
                 obstacle_threshold: int = 50):
        """
        Args:
            data: Flat int8 array from OccupancyGrid.data (length = width * height).
                  Values: -1 = unknown, 0 = free, 100 = occupied.
            resolution: Meters per cell.
            width: Grid width in cells.
            height: Grid height in cells.
            origin_x: X coordinate of cell (0,0) in world frame.
            origin_y: Y coordinate of cell (0,0) in world frame.
            obstacle_threshold: OccupancyGrid values >= this are obstacles.
        """
        self.resolution = resolution
        self.obstacle_threshold = obstacle_threshold
        self.pixel_width = width
        self.pixel_height = height
        self._origin_x = origin_x
        self._origin_y = origin_y

        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
        # MapHandler convention: 0=free, 1=obstacle
        grid = np.array(data, dtype=np.int8).reshape((height, width))
        # Treat unknown (-1) as obstacle for safety
        self.map_data = np.where(
            (grid >= obstacle_threshold) | (grid < 0), 1, 0
        ).astype(np.uint8)

        # OccupancyGrid uses bottom-left origin — same as MapHandler after flipud.
        # No flip needed here.

        self.width = width * resolution
        self.height = height * resolution

        # Compute cost field (same logic as MapHandler._compute_cost_field)
        self.cost_field = None
        self.distance_field = None
        self._compute_cost_field()

    def _compute_cost_field(self):
        """Compute distance-based traversal cost field."""
        free_space = (1 - self.map_data).astype(np.float64)
        self.distance_field = distance_transform_edt(free_space)
        self.max_obstacle_distance = self.distance_field.max()

        if self.max_obstacle_distance > 0:
            self.cost_field = 1.0 - np.clip(
                self.distance_field / self.max_obstacle_distance, 0.0, 1.0
            )
        else:
            self.cost_field = np.zeros_like(self.map_data, dtype=np.float64)

    # --- Coordinate conversion (with origin offset) ---

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        px = int(round((x - self._origin_x) / self.resolution))
        py = int(round((y - self._origin_y) / self.resolution))
        return px, py

    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        x = px * self.resolution + self._origin_x
        y = py * self.resolution + self._origin_y
        return x, y

    def is_valid_pixel(self, px: int, py: int) -> bool:
        return 0 <= px < self.pixel_width and 0 <= py < self.pixel_height

    def is_valid_world(self, x: float, y: float) -> bool:
        return (self._origin_x <= x < self._origin_x + self.width and
                self._origin_y <= y < self._origin_y + self.height)

    # --- Obstacle checking ---

    def is_obstacle_pixel(self, px: int, py: int) -> bool:
        if not self.is_valid_pixel(px, py):
            return True
        return self.map_data[py, px] == 1

    def is_obstacle_world(self, x: float, y: float) -> bool:
        px, py = self.world_to_pixel(x, y)
        return self.is_obstacle_pixel(px, py)

    def is_collision(self, points: np.ndarray) -> bool:
        """Check if any point in Nx2 array collides with obstacle."""
        for point in points:
            if self.is_obstacle_world(point[0], point[1]):
                return True
        return False

    def is_line_collision(self, x0: float, y0: float,
                          x1: float, y1: float) -> bool:
        """Check line segment for collision using Bresenham's algorithm."""
        px0, py0 = self.world_to_pixel(x0, y0)
        px1, py1 = self.world_to_pixel(x1, y1)
        for px, py in self._bresenham(px0, py0, px1, py1):
            if self.is_obstacle_pixel(px, py):
                return True
        return False

    # --- Cost queries ---

    def get_traversal_cost(self, x: float, y: float, alpha: float = 1.0) -> float:
        px, py = self.world_to_pixel(x, y)
        if not self.is_valid_pixel(px, py):
            return float('inf')
        c_ij = self.cost_field[py, px]
        return 1.0 + alpha * c_ij

    def get_obstacle_distance_world(self, x: float, y: float) -> float:
        px, py = self.world_to_pixel(x, y)
        if not self.is_valid_pixel(px, py):
            return 0.0
        return self.distance_field[py, px] * self.resolution

    # --- Visualization support ---

    def plot(self, ax=None, show_grid=False):
        """Plot the occupancy grid."""
        import matplotlib.pyplot as plt
        if ax is None:
            _, ax = plt.subplots(figsize=(10, 10))

        extent = [self._origin_x, self._origin_x + self.width,
                  self._origin_y, self._origin_y + self.height]
        ax.imshow(self.map_data, cmap='gray_r', origin='lower', extent=extent)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Occupancy Grid Map')
        if show_grid:
            ax.grid(True, alpha=0.3)
        return ax

    # --- Internal helpers ---

    @staticmethod
    def _bresenham(x0: int, y0: int, x1: int, y1: int):
        """Bresenham's line algorithm yielding (x, y) pixel coordinates."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            yield (x0, y0)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
