"""
Map Handler for loading and processing obstacle maps
Supports cost field generation for Cost-Aware planning (Macenski et al., 2025)
"""

import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
from typing import Tuple
from scipy.ndimage import distance_transform_edt


class MapHandler:
    """
    Handles map loading, coordinate conversion, and obstacle checking
    
    Attributes:
        map_data (np.ndarray): Binary occupancy grid (1 = obstacle, 0 = free)
        resolution (float): meters per pixel
        width (float): map width in meters
        height (float): map height in meters
        pixel_width (int): map width in pixels
        pixel_height (int): map height in pixels
    """
    
    def __init__(self, map_path: str, resolution: float = 0.1, 
                 obstacle_threshold: int = 128):
        """
        Initialize map handler
        
        Args:
            map_path: Path to map image (PNG)
            resolution: meters per pixel
            obstacle_threshold: Pixel values below this are obstacles
        """
        self.resolution = resolution
        self.obstacle_threshold = obstacle_threshold
        
        # Load map image
        img = Image.open(map_path).convert('L')  # Convert to grayscale
        self.pixel_width = img.width
        self.pixel_height = img.height
        
        # Convert to numpy array and create binary occupancy grid
        img_array = np.array(img)
        
        # Invert: black (0) = obstacle (1), white (255) = free (0)
        self.map_data = (img_array < obstacle_threshold).astype(np.uint8)
        
        # Flip vertically to match mathematical coordinate system
        self.map_data = np.flipud(self.map_data)
        
        # Calculate map dimensions in meters
        self.width = self.pixel_width * resolution
        self.height = self.pixel_height * resolution
        
        # Cost field: distance-based traversal cost for cost-aware planning
        self.cost_field = None
        self.distance_field = None
        self._compute_cost_field()

        print(f"Map loaded: {self.pixel_width}x{self.pixel_height} pixels")
        print(f"Map size: {self.width:.1f}m x {self.height:.1f}m")
        print(f"Resolution: {resolution} m/pixel")
        print(f"Cost field computed: max_distance={self.max_obstacle_distance:.1f} pixels")
        
    def _compute_cost_field(self):
        """
        Compute cost field using distance transform (Smac Planner, Macenski et al.).
        Cells near obstacles have higher traversal cost.
        cost_field values in [0, 1]: 0 = far from obstacles, 1 = at obstacle.
        """
        # Distance from each free cell to nearest obstacle (in pixels)
        free_space = (1 - self.map_data).astype(np.float64)
        self.distance_field = distance_transform_edt(free_space)

        self.max_obstacle_distance = self.distance_field.max()

        if self.max_obstacle_distance > 0:
            # Normalized cost: 1 near obstacles, 0 far away
            self.cost_field = 1.0 - np.clip(
                self.distance_field / self.max_obstacle_distance, 0.0, 1.0
            )
        else:
            self.cost_field = np.zeros_like(self.map_data, dtype=np.float64)

    def get_traversal_cost(self, x: float, y: float, alpha: float = 1.0) -> float:
        """
        Get cost-aware traversal multiplier at world coordinates.
        Based on Smac Planner Eq. 1: C = d * (1 + alpha * c_ij / c_max)

        Args:
            x, y: World coordinates
            alpha: Cost penalty weight (0 = ignore costs, higher = more avoidance)

        Returns:
            Traversal cost multiplier >= 1.0
        """
        px, py = self.world_to_pixel(x, y)
        if not self.is_valid_pixel(px, py):
            return float('inf')

        c_ij = self.cost_field[py, px]
        return 1.0 + alpha * c_ij

    def get_obstacle_distance_world(self, x: float, y: float) -> float:
        """Get distance to nearest obstacle in meters."""
        px, py = self.world_to_pixel(x, y)
        if not self.is_valid_pixel(px, py):
            return 0.0
        return self.distance_field[py, px] * self.resolution

    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to pixel coordinates
        
        Args:
            x, y: World coordinates in meters
            
        Returns:
            (px, py): Pixel coordinates
        """
        px = int(round(x / self.resolution))
        py = int(round(y / self.resolution))
        return px, py
    
    def pixel_to_world(self, px: int, py: int) -> Tuple[float, float]:
        """
        Convert pixel coordinates to world coordinates
        
        Args:
            px, py: Pixel coordinates
            
        Returns:
            (x, y): World coordinates in meters
        """
        x = px * self.resolution
        y = py * self.resolution
        return x, y
    
    def is_valid_pixel(self, px: int, py: int) -> bool:
        """Check if pixel coordinates are within map bounds"""
        return 0 <= px < self.pixel_width and 0 <= py < self.pixel_height
    
    def is_valid_world(self, x: float, y: float) -> bool:
        """Check if world coordinates are within map bounds"""
        return 0 <= x < self.width and 0 <= y < self.height
    
    def is_obstacle_pixel(self, px: int, py: int) -> bool:
        """
        Check if pixel contains obstacle
        
        Args:
            px, py: Pixel coordinates
            
        Returns:
            True if obstacle, False if free
        """
        if not self.is_valid_pixel(px, py):
            return True  # Out of bounds = obstacle
        return self.map_data[py, px] == 1
    
    def is_obstacle_world(self, x: float, y: float) -> bool:
        """
        Check if world coordinate contains obstacle
        
        Args:
            x, y: World coordinates in meters
            
        Returns:
            True if obstacle, False if free
        """
        px, py = self.world_to_pixel(x, y)
        return self.is_obstacle_pixel(px, py)
    
    def is_collision(self, points: np.ndarray) -> bool:
        """
        Check if any point in array collides with obstacle
        
        Args:
            points: Nx2 array of (x, y) world coordinates
            
        Returns:
            True if collision detected
        """
        for point in points:
            if self.is_obstacle_world(point[0], point[1]):
                return True
        return False
    
    def bresenham_line(self, x0: int, y0: int, x1: int, y1: int):
        """
        Bresenham's line algorithm for pixel coordinates
        Yields all pixels on the line from (x0, y0) to (x1, y1)
        """
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
    
    def is_line_collision(self, x0: float, y0: float, 
                         x1: float, y1: float) -> bool:
        """
        Check if line segment collides with obstacle
        
        Args:
            x0, y0: Start point in world coordinates
            x1, y1: End point in world coordinates
            
        Returns:
            True if collision detected
        """
        px0, py0 = self.world_to_pixel(x0, y0)
        px1, py1 = self.world_to_pixel(x1, y1)
        
        for px, py in self.bresenham_line(px0, py0, px1, py1):
            if self.is_obstacle_pixel(px, py):
                return True
        return False
    
    def plot(self, ax=None, show_grid=False):
        """
        Plot the map
        
        Args:
            ax: Matplotlib axis (creates new if None)
            show_grid: Whether to show grid lines
        """
        if ax is None:
            fig, ax = plt.subplots(figsize=(10, 10))
        
        # Display map with correct orientation
        ax.imshow(self.map_data, cmap='gray_r', origin='lower',
                 extent=[0, self.width, 0, self.height])
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Occupancy Grid Map', fontsize=14)
        
        if show_grid:
            ax.grid(True, alpha=0.3)
        
        return ax