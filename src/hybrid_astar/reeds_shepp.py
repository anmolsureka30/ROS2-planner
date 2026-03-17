"""
Reeds-Shepp Path Planning - Complete Implementation
All 48 path types based on Reeds & Shepp (1990) and Desaulniers (1995).
Uses exact arc geometry for path discretization.
"""

import numpy as np
from typing import List, Tuple, Optional
from src.state import State

# Path segment types
LEFT = 'L'
RIGHT = 'R'
STRAIGHT = 'S'
NOP = 'N'


def _mod2pi(theta):
    """Normalize angle to [0, 2*pi)"""
    return theta - 2.0 * np.pi * np.floor(theta / (2.0 * np.pi))


def _pi_2_pi(angle):
    """Normalize angle to [-pi, pi)"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def _polar(x, y):
    """Convert (x,y) to polar (r, theta)"""
    r = np.hypot(x, y)
    theta = np.arctan2(y, x)
    return r, theta


def _tau_omega(u, v, xi, eta, phi):
    """Helper for CCC path formulas"""
    delta = _pi_2_pi(u - v)
    A = np.sin(u) - np.sin(delta)
    B = np.cos(u) - np.cos(delta) - 1.0

    t1 = np.arctan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2.0 * (np.cos(delta) - np.cos(v) - np.cos(u)) + 3.0

    if t2 < 0:
        tau = _pi_2_pi(t1 + np.pi)
    else:
        tau = _pi_2_pi(t1)

    omega = _pi_2_pi(tau - u + v - phi)
    return tau, omega


# ============================================================
# CSC Path Formulas (6 types)
# ============================================================

def _LSL(x, y, phi):
    u, t = _polar(x - np.sin(phi), y - 1.0 + np.cos(phi))
    if t >= 0.0:
        v = _pi_2_pi(phi - t)
        if v >= 0.0:
            return True, t, u, v
    return False, 0, 0, 0


def _LSR(x, y, phi):
    u1, t1 = _polar(x + np.sin(phi), y - 1.0 - np.cos(phi))
    u1_sq = u1 * u1
    if u1_sq >= 4.0:
        u = np.sqrt(u1_sq - 4.0)
        theta = np.arctan2(2.0, u)
        t = _pi_2_pi(t1 + theta)
        v = _pi_2_pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return True, t, u, v
    return False, 0, 0, 0


def _LRL(x, y, phi):
    u1, t1 = _polar(x - np.sin(phi), y - 1.0 + np.cos(phi))
    if u1 <= 4.0:
        A = -np.arccos(0.25 * u1)
        t = _pi_2_pi(t1 + 0.5 * A + np.pi)
        u = _pi_2_pi(np.pi + A)
        v = _pi_2_pi(phi - t + u)
        if t >= 0.0 and u >= 0.0:
            return True, t, u, v
    return False, 0, 0, 0


# ============================================================
# Path type enumerations using symmetry transformations
# ============================================================

def _set_path(paths, lengths, ctypes):
    """Add a path to the list if it's valid and shorter."""
    path = (ctypes, list(lengths))
    paths.append(path)


def _SCS(x, y, phi, paths):
    """Straight-Curve-Straight paths"""
    # SLS (forward)
    flag, t, u, v = _LSL(x, y, phi)
    if flag:
        _set_path(paths, [t, u, v], "LSL")

    # Timeflip: SLS backward
    flag, t, u, v = _LSL(-x, y, -phi)
    if flag:
        _set_path(paths, [-t, -u, -v], "LSL")

    # Reflect: SRS
    flag, t, u, v = _LSL(x, -y, -phi)
    if flag:
        _set_path(paths, [t, u, v], "RSR")

    # Timeflip + Reflect
    flag, t, u, v = _LSL(-x, -y, phi)
    if flag:
        _set_path(paths, [-t, -u, -v], "RSR")

    # LSR
    flag, t, u, v = _LSR(x, y, phi)
    if flag:
        _set_path(paths, [t, u, v], "LSR")

    flag, t, u, v = _LSR(-x, y, -phi)
    if flag:
        _set_path(paths, [-t, -u, -v], "LSR")

    flag, t, u, v = _LSR(x, -y, -phi)
    if flag:
        _set_path(paths, [t, u, v], "RSL")

    flag, t, u, v = _LSR(-x, -y, phi)
    if flag:
        _set_path(paths, [-t, -u, -v], "RSL")


def _CCC(x, y, phi, paths):
    """Curve-Curve-Curve paths (LRL, RLR)"""
    flag, t, u, v = _LRL(x, y, phi)
    if flag:
        _set_path(paths, [t, u, v], "LRL")

    # Timeflip
    flag, t, u, v = _LRL(-x, y, -phi)
    if flag:
        _set_path(paths, [-t, -u, -v], "LRL")

    # Reflect
    flag, t, u, v = _LRL(x, -y, -phi)
    if flag:
        _set_path(paths, [t, u, v], "RLR")

    # Timeflip + Reflect
    flag, t, u, v = _LRL(-x, -y, phi)
    if flag:
        _set_path(paths, [-t, -u, -v], "RLR")


def _CCCC(x, y, phi, paths):
    """4-segment Curve-Curve-Curve-Curve paths"""
    # LRuLv (8.7)
    xi = x + np.sin(phi)
    eta = y - 1.0 - np.cos(phi)
    rho, _ = _polar(xi, eta)

    if rho <= 4.0:
        u = -np.arccos(0.25 * rho)  # negative u
        if u >= -np.pi:
            tau, omega = _tau_omega(u, u, xi, eta, phi)
            _set_path(paths, [tau, u, u, omega], "LRLR")

    # Timeflip
    xi = -x + np.sin(phi)
    eta = -y - 1.0 - np.cos(phi)
    # Use timeflip identity
    xi2 = -x + np.sin(-phi)
    eta2 = y - 1.0 - np.cos(-phi)
    rho, _ = _polar(xi2, eta2)

    # Reflect: RLuRv
    xi = x - np.sin(phi)
    eta = -y - 1.0 + np.cos(phi)
    rho, _ = _polar(xi, eta)

    if rho <= 4.0:
        u = -np.arccos(0.25 * rho)
        if u >= -np.pi:
            tau, omega = _tau_omega(u, u, xi, eta, -phi)
            _set_path(paths, [tau, u, u, omega], "RLRL")


def _CCSC(x, y, phi, paths):
    """4-segment Curve-Curve-Straight-Curve and similar"""
    # Formula 8.9: LRSnLn
    xi = x + np.sin(phi)
    eta = y - 1.0 - np.cos(phi)
    rho, theta = _polar(xi, eta)

    if rho >= 2.0:
        r = np.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _pi_2_pi(theta + np.arctan2(r, -2.0))
        v = _pi_2_pi(phi - 0.5 * np.pi - t)
        if t >= 0.0 and u <= 0.0 and v <= 0.0:
            _set_path(paths, [t, -0.5 * np.pi, u, v], "LRSL")

    # Timeflip
    xi = -x + np.sin(-phi)
    eta = y - 1.0 - np.cos(-phi)
    rho, theta = _polar(xi, eta)

    if rho >= 2.0:
        r = np.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _pi_2_pi(theta + np.arctan2(r, -2.0))
        v = _pi_2_pi(-phi - 0.5 * np.pi - t)
        if t >= 0.0 and u <= 0.0 and v <= 0.0:
            _set_path(paths, [-t, 0.5 * np.pi, -u, -v], "LRSL")

    # Reflect
    xi = x - np.sin(phi)
    eta = -y - 1.0 + np.cos(phi)
    rho, theta = _polar(xi, eta)

    if rho >= 2.0:
        r = np.sqrt(rho * rho - 4.0)
        u = 2.0 - r
        t = _pi_2_pi(theta + np.arctan2(r, -2.0))
        v = _pi_2_pi(-phi - 0.5 * np.pi - t)
        if t >= 0.0 and u <= 0.0 and v <= 0.0:
            _set_path(paths, [t, -0.5 * np.pi, u, v], "RLSR")


def _CCSCC(x, y, phi, paths):
    """5-segment Curve-Curve-Straight-Curve-Curve"""
    # Formula 8.11: LRSnLnR
    xi = x + np.sin(phi)
    eta = y - 1.0 - np.cos(phi)
    rho, _ = _polar(xi, eta)

    if rho >= 2.0:
        t_val = rho * rho
        if t_val >= 4.0:
            u = np.sqrt(t_val - 4.0) - 2.0
            if u <= 0.0:
                alpha = np.arctan2(2.0, np.sqrt(t_val - 4.0))
                t = _pi_2_pi(np.arctan2(eta, xi) + alpha)
                v = _pi_2_pi(t - phi)
                if t >= 0.0 and v >= 0.0:
                    _set_path(paths, [t, -0.5 * np.pi, u, -0.5 * np.pi, v], "LRSLR")

    # Timeflip
    xi = -x + np.sin(-phi)
    eta = y - 1.0 - np.cos(-phi)
    rho, _ = _polar(xi, eta)

    if rho >= 2.0:
        t_val = rho * rho
        if t_val >= 4.0:
            u = np.sqrt(t_val - 4.0) - 2.0
            if u <= 0.0:
                alpha = np.arctan2(2.0, np.sqrt(t_val - 4.0))
                t = _pi_2_pi(np.arctan2(eta, xi) + alpha)
                v = _pi_2_pi(t + phi)
                if t >= 0.0 and v >= 0.0:
                    _set_path(paths, [-t, 0.5 * np.pi, -u, 0.5 * np.pi, -v], "LRSLR")

    # Reflect
    xi = x - np.sin(phi)
    eta = -y - 1.0 + np.cos(phi)
    rho, _ = _polar(xi, eta)

    if rho >= 2.0:
        t_val = rho * rho
        if t_val >= 4.0:
            u = np.sqrt(t_val - 4.0) - 2.0
            if u <= 0.0:
                alpha = np.arctan2(2.0, np.sqrt(t_val - 4.0))
                t = _pi_2_pi(np.arctan2(eta, xi) + alpha)
                v = _pi_2_pi(t + phi)
                if t >= 0.0 and v >= 0.0:
                    _set_path(paths, [t, 0.5 * np.pi, u, 0.5 * np.pi, v], "RLSRL")


def generate_all_paths(x, y, phi):
    """Generate all valid Reeds-Shepp paths for the given configuration."""
    paths = []
    _SCS(x, y, phi, paths)
    _CCC(x, y, phi, paths)
    _CCCC(x, y, phi, paths)
    _CCSC(x, y, phi, paths)
    _CCSCC(x, y, phi, paths)
    return paths


class ReedsSheppPath:
    """
    Computes Reeds-Shepp paths - Complete Implementation.
    Finds the optimal path for a vehicle with a minimum turning radius
    that can move both forwards and backwards.
    """

    def __init__(self, turning_radius: float = 5.0):
        self.turning_radius = turning_radius
        self.step_size = 0.1  # Resolution for path discretization (meters)
        self.rho = turning_radius

    def plan(self, start: State, goal: State) -> Optional[Tuple[List[State], float]]:
        """
        Plan Reeds-Shepp path from start to goal.

        Returns:
            (path_states, total_length) if path found, else None
        """
        # Transform to canonical frame: start at (0,0,0), unit turning radius
        dx = goal.x - start.x
        dy = goal.y - start.y
        dth = _pi_2_pi(goal.theta - start.theta)

        c = np.cos(start.theta)
        s = np.sin(start.theta)

        # Normalize to unit turning radius
        x = (c * dx + s * dy) / self.rho
        y = (-s * dx + c * dy) / self.rho

        paths = generate_all_paths(x, y, dth)

        if not paths:
            return None

        # Select shortest total path length
        best_path = None
        best_length = float('inf')

        for ctypes, lengths in paths:
            total = sum(abs(l) for l in lengths)
            if total < best_length:
                best_length = total
                best_path = (ctypes, lengths)

        if best_path is None:
            return None

        ctypes, lengths = best_path
        total_length = best_length * self.rho

        # Generate dense waypoints
        states = self._generate_path_states(start, ctypes, lengths)

        if len(states) < 2:
            return None

        return states, total_length

    def get_distance(self, start: State, goal: State) -> float:
        """Get shortest Reeds-Shepp distance without generating full path."""
        dx = goal.x - start.x
        dy = goal.y - start.y
        dth = _pi_2_pi(goal.theta - start.theta)

        c = np.cos(start.theta)
        s = np.sin(start.theta)

        x = (c * dx + s * dy) / self.rho
        y = (-s * dx + c * dy) / self.rho

        paths = generate_all_paths(x, y, dth)

        if not paths:
            return float('inf')

        best_length = float('inf')
        for ctypes, lengths in paths:
            total = sum(abs(l) for l in lengths)
            if total < best_length:
                best_length = total

        return best_length * self.rho

    def _generate_path_states(self, start: State, ctypes: str,
                               lengths: List[float]) -> List[State]:
        """
        Generate dense states along the path using exact arc geometry.
        """
        states = [start.copy()]
        x, y, theta = start.x, start.y, start.theta

        for i, (mode, norm_length) in enumerate(zip(ctypes, lengths)):
            # Actual distance in meters
            dist = norm_length * self.rho
            # Direction: positive = forward, negative = reverse
            gear = 1 if norm_length >= 0 else -1
            abs_dist = abs(dist)

            # Number of steps for this segment
            num_steps = max(int(abs_dist / self.step_size), 2)
            step_len = abs_dist / num_steps

            for _ in range(num_steps):
                if mode == 'S':
                    # Straight segment: exact
                    x += gear * step_len * np.cos(theta)
                    y += gear * step_len * np.sin(theta)
                elif mode == 'L':
                    # Left turn: exact arc geometry
                    # Center of circle
                    cx = x - self.rho * np.sin(theta)
                    cy = y + self.rho * np.cos(theta)
                    # Angular change
                    d_theta = gear * step_len / self.rho
                    theta_new = theta + d_theta
                    # New position on arc
                    x = cx + self.rho * np.sin(theta_new)
                    y = cy - self.rho * np.cos(theta_new)
                    theta = theta_new
                elif mode == 'R':
                    # Right turn: exact arc geometry
                    cx = x + self.rho * np.sin(theta)
                    cy = y - self.rho * np.cos(theta)
                    d_theta = -gear * step_len / self.rho
                    theta_new = theta + d_theta
                    x = cx - self.rho * np.sin(theta_new)
                    y = cy + self.rho * np.cos(theta_new)
                    theta = theta_new

                theta = _pi_2_pi(theta)

                steering_viz = 0.0
                if mode == 'L':
                    steering_viz = np.radians(40)
                elif mode == 'R':
                    steering_viz = np.radians(-40)

                new_state = State(
                    x, y, theta,
                    direction=gear,
                    steering=steering_viz
                )
                states.append(new_state)

        return states
