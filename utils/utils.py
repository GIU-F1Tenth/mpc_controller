"""
Utility functions for MPC Controller.

This module provides utility functions for trajectory generation,
coordinate transformations, and other helper functions.
"""

import numpy as np
from typing import Tuple, Optional
import logging

logger = logging.getLogger(__name__)


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi] range.

    Args:
        angle: Angle in radians

    Returns:
        Normalized angle in [-pi, pi]
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def compute_distance(point1: np.ndarray, point2: np.ndarray) -> float:
    """
    Compute Euclidean distance between two points.

    Args:
        point1: First point [x, y]
        point2: Second point [x, y]

    Returns:
        Euclidean distance
    """
    return np.linalg.norm(point1 - point2)


def generate_circular_trajectory(
    center: Tuple[float, float] = (0.0, 0.0),
    radius: float = 2.0,
    velocity: float = 2.0,
    num_points: int = 100
) -> np.ndarray:
    """
    Generate a circular reference trajectory.

    Args:
        center: Circle center (x, y)
        radius: Circle radius
        velocity: Constant velocity along trajectory
        num_points: Number of trajectory points

    Returns:
        Trajectory array (4 x num_points) [x, y, v, theta]
    """
    angles = np.linspace(0, 2 * np.pi, num_points)

    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    v = np.full(num_points, velocity)
    theta = angles + np.pi / 2  # Tangent to circle

    return np.vstack([x, y, v, theta])


def generate_straight_trajectory(
    start: Tuple[float, float],
    end: Tuple[float, float],
    velocity: float = 2.0,
    num_points: int = 50
) -> np.ndarray:
    """
    Generate a straight line reference trajectory.

    Args:
        start: Start point (x, y)
        end: End point (x, y)
        velocity: Constant velocity along trajectory
        num_points: Number of trajectory points

    Returns:
        Trajectory array (4 x num_points) [x, y, v, theta]
    """
    x = np.linspace(start[0], end[0], num_points)
    y = np.linspace(start[1], end[1], num_points)
    v = np.full(num_points, velocity)

    # Calculate heading angle
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    theta = np.full(num_points, np.arctan2(dy, dx))

    return np.vstack([x, y, v, theta])


def generate_figure_eight_trajectory(
    center: Tuple[float, float] = (0.0, 0.0),
    radius: float = 2.0,
    velocity: float = 2.0,
    num_points: int = 200
) -> np.ndarray:
    """
    Generate a figure-eight reference trajectory.

    Args:
        center: Trajectory center (x, y)
        radius: Trajectory scale factor
        velocity: Constant velocity along trajectory
        num_points: Number of trajectory points

    Returns:
        Trajectory array (4 x num_points) [x, y, v, theta]
    """
    t = np.linspace(0, 4 * np.pi, num_points)

    # Parametric figure-eight equations
    x = center[0] + radius * np.sin(t)
    y = center[1] + radius * np.sin(t) * np.cos(t)
    v = np.full(num_points, velocity)

    # Calculate heading from derivatives
    dx_dt = radius * np.cos(t)
    dy_dt = radius * (np.cos(t)**2 - np.sin(t)**2)
    theta = np.arctan2(dy_dt, dx_dt)

    return np.vstack([x, y, v, theta])


def interpolate_trajectory(
    trajectory: np.ndarray,
    target_length: int
) -> np.ndarray:
    """
    Interpolate trajectory to desired length.

    Args:
        trajectory: Input trajectory (4 x N)
        target_length: Desired number of points

    Returns:
        Interpolated trajectory (4 x target_length)
    """
    if trajectory.shape[0] != 4:
        raise ValueError("Trajectory must have 4 rows [x, y, v, theta]")

    current_length = trajectory.shape[1]

    if current_length == target_length:
        return trajectory

    # Create interpolation indices
    old_indices = np.linspace(0, current_length - 1, current_length)
    new_indices = np.linspace(0, current_length - 1, target_length)

    # Interpolate each state component
    interpolated = np.zeros((4, target_length))
    for i in range(4):
        if i == 3:  # Special handling for angles
            # Unwrap angles for interpolation
            unwrapped = np.unwrap(trajectory[i, :])
            interpolated[i, :] = np.interp(new_indices, old_indices, unwrapped)
            # Normalize back to [-pi, pi]
            interpolated[i, :] = normalize_angle(interpolated[i, :])
        else:
            interpolated[i, :] = np.interp(
                new_indices, old_indices, trajectory[i, :])

    return interpolated


def trajectory_curvature(trajectory: np.ndarray) -> np.ndarray:
    """
    Compute trajectory curvature.

    Args:
        trajectory: Trajectory array (4 x N) [x, y, v, theta]

    Returns:
        Curvature array (N-1,)
    """
    x, y = trajectory[0, :], trajectory[1, :]

    # First derivatives
    dx = np.diff(x)
    dy = np.diff(y)

    # Second derivatives
    ddx = np.diff(dx)
    ddy = np.diff(dy)

    # Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
    numerator = np.abs(dx[:-1] * ddy - dy[:-1] * ddx)
    denominator = (dx[:-1]**2 + dy[:-1]**2)**(3/2)

    # Avoid division by zero
    denominator = np.maximum(denominator, 1e-8)

    return numerator / denominator


def smooth_trajectory(
    trajectory: np.ndarray,
    window_size: int = 5
) -> np.ndarray:
    """
    Smooth trajectory using moving average filter.

    Args:
        trajectory: Input trajectory (4 x N)
        window_size: Moving average window size

    Returns:
        Smoothed trajectory (4 x N)
    """
    if window_size < 1 or window_size > trajectory.shape[1]:
        raise ValueError("Invalid window size")

    smoothed = np.copy(trajectory)

    for i in range(4):
        if i == 3:  # Special handling for angles
            # Convert to complex representation for smoothing
            complex_angles = np.exp(1j * trajectory[i, :])
            # Apply moving average
            kernel = np.ones(window_size) / window_size
            smoothed_complex = np.convolve(complex_angles, kernel, mode='same')
            # Convert back to angles
            smoothed[i, :] = np.angle(smoothed_complex)
        else:
            # Regular moving average for position and velocity
            kernel = np.ones(window_size) / window_size
            smoothed[i, :] = np.convolve(trajectory[i, :], kernel, mode='same')

    return smoothed


def trajectory_statistics(trajectory: np.ndarray) -> dict:
    """
    Compute trajectory statistics.

    Args:
        trajectory: Trajectory array (4 x N) [x, y, v, theta]

    Returns:
        Dictionary of trajectory statistics
    """
    x, y, v, theta = trajectory

    # Compute path length
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    path_length = np.sum(distances)

    # Compute curvature statistics
    curvature = trajectory_curvature(trajectory)

    # Compute velocity statistics
    velocity_stats = {
        'mean': np.mean(v),
        'std': np.std(v),
        'min': np.min(v),
        'max': np.max(v)
    }

    # Compute position statistics
    position_stats = {
        'x_range': [np.min(x), np.max(x)],
        'y_range': [np.min(y), np.max(y)],
        'centroid': [np.mean(x), np.mean(y)]
    }

    return {
        'path_length': path_length,
        'num_points': len(x),
        'velocity': velocity_stats,
        'position': position_stats,
        'curvature': {
            'mean': np.mean(curvature),
            'max': np.max(curvature),
            'std': np.std(curvature)
        }
    }
