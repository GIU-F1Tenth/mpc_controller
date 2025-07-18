"""
Configuration file for MPC Controller.

This module contains configuration parameters and utility functions
for the Model Predictive Controller.

Author: F1Tenth Team
License: MIT
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass
class VehicleParameters:
    """Vehicle-specific parameters."""
    wheelbase: float = 0.33           # Vehicle wheelbase (m)
    max_velocity: float = 5.0         # Maximum velocity (m/s)
    max_acceleration: float = 2.0     # Maximum acceleration (m/s²)
    min_acceleration: float = -2.0    # Maximum deceleration (m/s²)
    max_steering_angle: float = 0.4   # Maximum steering angle (rad)


@dataclass
class MPCParameters:
    """MPC-specific parameters."""
    prediction_horizon: int = 10      # Prediction horizon steps
    time_horizon: float = 1.0         # Time horizon (s)
    state_weights: np.ndarray = None  # State cost weights [x, y, v, theta]
    control_weights: np.ndarray = None  # Control cost weights [a, delta]

    def __post_init__(self):
        """Initialize default weight matrices if not provided."""
        if self.state_weights is None:
            self.state_weights = np.diag([10.0, 10.0, 1.0, 1.0])

        if self.control_weights is None:
            self.control_weights = np.diag([1.0, 1.0])


@dataclass
class ControllerConfig:
    """Complete controller configuration."""
    vehicle: VehicleParameters = None
    mpc: MPCParameters = None
    control_frequency: float = 10.0   # Control loop frequency (Hz)

    def __post_init__(self):
        """Initialize default configurations if not provided."""
        if self.vehicle is None:
            self.vehicle = VehicleParameters()

        if self.mpc is None:
            self.mpc = MPCParameters()


def load_config_from_file(filepath: str) -> ControllerConfig:
    """
    Load configuration from YAML file.

    Args:
        filepath: Path to configuration file

    Returns:
        Controller configuration

    Note:
        This is a placeholder for future YAML configuration support.
        For now, returns default configuration.
    """
    # TODO: Implement YAML loading when needed
    return ControllerConfig()


def validate_config(config: ControllerConfig) -> bool:
    """
    Validate controller configuration.

    Args:
        config: Controller configuration to validate

    Returns:
        True if configuration is valid

    Raises:
        ValueError: If configuration is invalid
    """
    # Validate vehicle parameters
    if config.vehicle.wheelbase <= 0:
        raise ValueError("Vehicle wheelbase must be positive")

    if config.vehicle.max_velocity <= 0:
        raise ValueError("Maximum velocity must be positive")

    if config.vehicle.max_acceleration <= 0:
        raise ValueError("Maximum acceleration must be positive")

    if config.vehicle.min_acceleration >= 0:
        raise ValueError("Minimum acceleration must be negative")

    if config.vehicle.max_steering_angle <= 0:
        raise ValueError("Maximum steering angle must be positive")

    # Validate MPC parameters
    if config.mpc.prediction_horizon <= 0:
        raise ValueError("Prediction horizon must be positive")

    if config.mpc.time_horizon <= 0:
        raise ValueError("Time horizon must be positive")

    if config.mpc.state_weights.shape != (4, 4):
        raise ValueError("State weights must be 4x4 matrix")

    if config.mpc.control_weights.shape != (2, 2):
        raise ValueError("Control weights must be 2x2 matrix")

    # Validate control frequency
    if config.control_frequency <= 0:
        raise ValueError("Control frequency must be positive")

    return True


# Default configuration instance
DEFAULT_CONFIG = ControllerConfig()
