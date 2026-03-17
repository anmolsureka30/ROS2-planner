"""
State Node Class for Hybrid A* Algorithm
Represents a state in SE(2): (x, y, theta)
"""

import numpy as np
from typing import Optional


class State:
    """
    Represents a state in the search space
    
    Attributes:
        x (float): x-coordinate in continuous space (meters)
        y (float): y-coordinate in continuous space (meters)
        theta (float): heading angle in radians
        g (float): cost-so-far from start
        h (float): heuristic cost-to-goal
        parent (State): parent state in the path
        direction (int): motion direction (1: forward, -1: reverse)
        steering (float): steering angle used to reach this state
    """
    
    def __init__(self, x: float, y: float, theta: float, 
                 g: float = 0.0, h: float = 0.0,
                 parent: Optional['State'] = None,
                 direction: int = 1, steering: float = 0.0):
        self.x = x
        self.y = y
        self.theta = self._normalize_angle(theta)
        self.g = g
        self.h = h
        self.parent = parent
        self.direction = direction  # 1 for forward, -1 for reverse
        self.steering = steering  # steering angle in radians
        
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        return np.arctan2(np.sin(angle), np.cos(angle))
    
    @property
    def f(self) -> float:
        """Total cost f = g + h"""
        return self.g + self.h
    
    def __lt__(self, other: 'State') -> bool:
        """Comparison for priority queue (min-heap)"""
        return self.f < other.f
    
    def __eq__(self, other: 'State') -> bool:
        """Check if two states are equal (same position and heading)"""
        return (np.isclose(self.x, other.x, atol=0.01) and 
                np.isclose(self.y, other.y, atol=0.01) and
                np.isclose(self.theta, other.theta, atol=0.1))
    
    def __hash__(self) -> int:
        """Hash for using State in sets/dicts"""
        return hash((round(self.x, 2), round(self.y, 2), round(self.theta, 2)))
    
    def __repr__(self) -> str:
        return f"State(x={self.x:.2f}, y={self.y:.2f}, theta={np.degrees(self.theta):.1f}°, g={self.g:.2f}, h={self.h:.2f})"
    
    def distance_to(self, other: 'State') -> float:
        """Euclidean distance to another state"""
        return np.hypot(self.x - other.x, self.y - other.y)
    
    def copy(self) -> 'State':
        """Create a copy of this state"""
        return State(self.x, self.y, self.theta, self.g, self.h, 
                    self.parent, self.direction, self.steering)


def discretize_state(state: State, xy_resolution: float, 
                     theta_resolution: float) -> tuple:
    """
    Discretize continuous state to grid indices
    
    Args:
        state: State to discretize
        xy_resolution: Grid resolution for x, y (meters)
        theta_resolution: Angular resolution (radians)
    
    Returns:
        tuple: (grid_x, grid_y, theta_bin)
    """
    grid_x = int(round(state.x / xy_resolution))
    grid_y = int(round(state.y / xy_resolution))
    
    # Normalize theta to [0, 2*pi) for binning
    theta_normalized = state.theta % (2 * np.pi)
    theta_bin = int(round(theta_normalized / theta_resolution))
    
    # Handle wrap-around for theta
    num_bins = int(round(2 * np.pi / theta_resolution))
    theta_bin = theta_bin % num_bins
    
    return (grid_x, grid_y, theta_bin)




