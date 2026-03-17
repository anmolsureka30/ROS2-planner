"""
Vehicle Motion Model - Bicycle Kinematics
Based on Dolgov et al. (2010) and Meng et al. (2023)
"""

import numpy as np
from typing import List, Tuple
from src.state import State


class MotionModel:
    """
    Bicycle kinematic model for non-holonomic vehicle motion.
    Uses exact arc geometry for motion primitive generation.
    """

    def __init__(self, wheel_base: float = 2.5,
                 max_steering_angle: float = 40.0,
                 step_size: float = 0.5,
                 num_steering_angles: int = 5,
                 allow_reverse: bool = True):

        self.wheel_base = wheel_base
        self.max_steering_angle = np.radians(max_steering_angle)
        self.step_size = step_size
        self.allow_reverse = allow_reverse

        # Generate steering angles
        if num_steering_angles == 1:
            self.steering_angles = [0.0]
        else:
            self.steering_angles = np.linspace(
                -self.max_steering_angle,
                self.max_steering_angle,
                num_steering_angles
            ).tolist()

        self.directions = [1]
        if allow_reverse:
            self.directions.append(-1)

        self.min_turning_radius = wheel_base / np.tan(self.max_steering_angle)

        print(f"Motion Model: wheel_base={wheel_base}m, step={step_size}m, "
              f"steering_angles={num_steering_angles}, min_radius={self.min_turning_radius:.2f}m")

    def get_motion_primitives(self) -> List[Tuple[int, float]]:
        """Get all (direction, steering_angle) primitives."""
        primitives = []
        for direction in self.directions:
            for steering in self.steering_angles:
                primitives.append((direction, steering))
        return primitives

    def apply_motion(self, state: State, direction: int,
                     steering_angle: float) -> State:
        """
        Apply a single motion primitive using exact arc geometry.

        Args:
            state: Current state
            direction: 1 (forward) or -1 (reverse)
            steering_angle: Steering angle in radians

        Returns:
            New state after applying motion
        """
        d = self.step_size * direction

        if abs(steering_angle) < 1e-6:
            # Straight motion
            x_new = state.x + d * np.cos(state.theta)
            y_new = state.y + d * np.sin(state.theta)
            theta_new = state.theta
        else:
            # Arc motion using center-of-rotation
            R = self.wheel_base / np.tan(steering_angle)
            cx = state.x - R * np.sin(state.theta)
            cy = state.y + R * np.cos(state.theta)
            d_theta = d / R
            theta_new = state.theta + d_theta
            x_new = cx + R * np.sin(theta_new)
            y_new = cy - R * np.cos(theta_new)

        return State(x_new, y_new, theta_new,
                     direction=direction, steering=steering_angle)

    def compute_motion_cost(self, state: State, next_state: State,
                            steering_penalty: float = 1.5,
                            reversing_penalty: float = 2.0,
                            steering_change_penalty: float = 1.5,
                            direction_switch_penalty: float = 10.0) -> float:
        """
        Compute motion cost with 4 core penalties per Dolgov et al.

        Cost = step_size
             * (1 + steering_penalty * |δ|/δ_max)
             * (reversing_penalty if reverse)
             * (1 + steering_change_penalty * |Δδ|/δ_max)
             + direction_switch_penalty if gear change
        """
        cost = self.step_size

        # Proportional steering penalty: sharper turns cost more
        steer_ratio = abs(next_state.steering) / self.max_steering_angle
        cost *= (1.0 + steering_penalty * steer_ratio)

        # Reverse driving penalty
        if next_state.direction < 0:
            cost *= reversing_penalty

        # Steering angle change cost (proportional to change magnitude)
        if state.parent is not None:
            steer_diff = abs(next_state.steering - state.steering) / self.max_steering_angle
            cost *= (1.0 + steering_change_penalty * steer_diff)

        # Direction switch penalty (forward <-> reverse gear change)
        if state.direction != next_state.direction:
            cost += direction_switch_penalty

        return cost


class VehicleFootprint:
    """
    Vehicle footprint for collision checking.
    Uses sparse critical points for fast collision checks.
    """

    def __init__(self, length: float = 4.5, width: float = 2.0,
                 rear_axle_to_back: float = 1.0):

        self.length = length
        self.width = width
        self.rear_axle_to_back = rear_axle_to_back

        # 4 corners of the bounding box
        self.vertices = np.array([
            [-rear_axle_to_back, -width / 2],
            [-rear_axle_to_back, width / 2],
            [length - rear_axle_to_back, width / 2],
            [length - rear_axle_to_back, -width / 2]
        ])

        self.collision_points = self._generate_collision_points()

        print(f"Vehicle: {length}m x {width}m, collision points: "
              f"{len(self.collision_points)}")

    def _generate_collision_points(self) -> np.ndarray:
        """Generate critical points for collision checking (~16-20 points)."""
        points = []

        # 4 corners
        points.extend(self.vertices.tolist())

        # 4 midpoints of edges
        for i in range(4):
            v1 = self.vertices[i]
            v2 = self.vertices[(i + 1) % 4]
            midpoint = (v1 + v2) / 2
            points.append(midpoint.tolist())

        # Front and rear center
        front_x = self.length - self.rear_axle_to_back
        rear_x = -self.rear_axle_to_back
        points.append([front_x, 0])
        points.append([rear_x, 0])

        # Quarter-width points along length
        for x_frac in [0.25, 0.5, 0.75]:
            x_pos = rear_x + x_frac * self.length
            points.append([x_pos, -self.width / 4])
            points.append([x_pos, self.width / 4])

        return np.array(points)

    def get_footprint_fast(self, state: State) -> np.ndarray:
        """Transform collision points to world coordinates."""
        cos_theta = np.cos(state.theta)
        sin_theta = np.sin(state.theta)
        rotation = np.array([[cos_theta, -sin_theta],
                             [sin_theta, cos_theta]])

        points_world = (rotation @ self.collision_points.T).T
        points_world[:, 0] += state.x
        points_world[:, 1] += state.y

        return points_world

    def get_vertices_world(self, state: State) -> np.ndarray:
        """Get 4 corners in world coordinates for visualization."""
        cos_theta = np.cos(state.theta)
        sin_theta = np.sin(state.theta)
        rotation = np.array([[cos_theta, -sin_theta],
                             [sin_theta, cos_theta]])

        vertices_world = (rotation @ self.vertices.T).T
        vertices_world[:, 0] += state.x
        vertices_world[:, 1] += state.y

        return vertices_world
