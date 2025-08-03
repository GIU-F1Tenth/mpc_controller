"""
F1TENTH MPC Controller Package

This package provides Model Predictive Control (MPC) implementations for F1TENTH
autonomous racing platforms, including both traditional optimized MPC and
adaptive MPC with real-time parameter tuning.

Main modules:
    - optimized_mpc_controller: Traditional MPC with fixed parameters
    - adaptive_mpc_controller: Adaptive MPC with real-time parameter tuning
    - mpc_node: ROS2 node for traditional MPC
    - adaptive_mpc_node: ROS2 node for adaptive MPC
    - kinematic_bicycle_model: Vehicle dynamics models
    - dynamic_bicycle_model: Advanced vehicle dynamics

Author: Mohammed Azab <mohammed@azab.io>
License: MIT
"""

from .optimized_mpc_controller import OptimizedMPCController
from .adaptive_mpc_controller import AdaptiveMPCController, AdaptiveParameters
from .kinematic_bicycle_model import MPCType, KinematicBicycleModel

__version__ = "3.0.0"
__author__ = "Mohammed Azab"
__email__ = "mohammed@azab.io"

__all__ = [
    'OptimizedMPCController',
    'AdaptiveMPCController', 
    'AdaptiveParameters',
    'MPCType',
    'KinematicBicycleModel'
]