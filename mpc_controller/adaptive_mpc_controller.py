"""
F1TENTH Adaptive Model Predictive Controller

This module provides an intelligent, self-adapting Model Predictive Controller (MPC) for
F1TENTH autonomous racing that dynamically adjusts parameters based on real-time conditions.

Key Adaptive Features:
    - Dynamic horizon adjustment based on speed and curvature
    - Real-time cost weight adaptation based on tracking error
    - Obstacle-aware parameter tuning using LiDAR data
    - Intelligent timestep adjustment for performance optimization
    - Safe parameter bounds with constraint validation

Adaptation Metrics:
    - Path tracking error (lateral, heading, velocity)
    - Obstacle proximity from LiDAR
    - Track curvature analysis
    - Vehicle speed and dynamics
    - Control effort and stability

Main Classes:
    AdaptiveMPCController: Core adaptive MPC with parameter adjustment logic
    AdaptationEngine: Manages parameter adaptation algorithms
    SafetyManager: Ensures parameter changes maintain safety constraints
    MetricsCollector: Collects and processes real-time performance metrics

Author: Mohammed Azab <mohammed@azab.io>
License: MIT
Version: 3.0.0
"""

import numpy as np
import time
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum
import math

try:
    from .optimized_mpc_controller import OptimizedMPCController
    from .kinematic_bicycle_model import MPCType
except ImportError:
    from optimized_mpc_controller import OptimizedMPCController
    from kinematic_bicycle_model import MPCType


class AdaptationMode(Enum):
    """Adaptation modes for different racing scenarios"""
    CONSERVATIVE = "conservative"
    BALANCED = "balanced"
    AGGRESSIVE = "aggressive"
    EMERGENCY = "emergency"


@dataclass
class TrackingMetrics:
    """Container for tracking performance metrics"""
    lateral_error: float = 0.0
    heading_error: float = 0.0
    velocity_error: float = 0.0
    cross_track_error: float = 0.0
    timestamp: float = 0.0


@dataclass
class EnvironmentMetrics:
    """Container for environment-based metrics"""
    min_obstacle_distance: float = float('inf')
    track_curvature: float = 0.0
    average_speed: float = 0.0
    road_width: float = 1.0
    confidence_level: float = 1.0
    timestamp: float = 0.0


@dataclass
class AdaptiveParameters:
    """Container for adaptive parameter bounds and current values"""
    horizon_N: int = 25
    horizon_T: float = 0.3
    
    # Parameter bounds
    horizon_N_range: Tuple[int, int] = (5, 50)
    horizon_T_range: Tuple[float, float] = (0.1, 1.0)
    
    # Cost weights
    position_weight: float = 5.0
    heading_weight: float = 2.0
    velocity_weight: float = 1.0
    steering_weight: float = 0.7
    acceleration_weight: float = 0.5
    
    # Weight adaptation ranges
    position_weight_range: Tuple[float, float] = (1.0, 20.0)
    heading_weight_range: Tuple[float, float] = (0.5, 10.0)
    velocity_weight_range: Tuple[float, float] = (0.1, 5.0)
    steering_weight_range: Tuple[float, float] = (0.1, 2.0)
    acceleration_weight_range: Tuple[float, float] = (0.1, 2.0)


class AdaptationEngine:
    """
    Engine for computing optimal parameter adaptations based on real-time metrics
    """
    
    def __init__(self, adaptation_rate: float = 0.1, smoothing_factor: float = 0.8):
        self.adaptation_rate = adaptation_rate
        self.smoothing_factor = smoothing_factor
        self.adaptation_history = []
        self.last_adaptation_time = 0.0
        
        # Tracking error thresholds
        self.low_error_threshold = 0.05
        self.high_error_threshold = 0.2
        
        # Speed-based adaptation parameters
        self.low_speed_threshold = 1.0
        self.high_speed_threshold = 5.0
        
        # Curvature-based adaptation
        self.low_curvature_threshold = 0.1
        self.high_curvature_threshold = 0.5

    def compute_horizon_adaptation(self, 
                                 speed: float, 
                                 curvature: float,
                                 tracking_error: float,
                                 current_N: int,
                                 current_T: float,
                                 bounds: AdaptiveParameters) -> Tuple[int, float]:
        """
        Compute adaptive horizon based on speed, curvature, and tracking performance
        """
        # Speed-based adaptation
        if speed < self.low_speed_threshold:
            # Low speed: shorter horizon for responsiveness
            target_N = max(bounds.horizon_N_range[0], int(current_N * 0.7))
            target_T = min(bounds.horizon_T_range[1], current_T * 1.2)
        elif speed > self.high_speed_threshold:
            # High speed: longer horizon for stability
            target_N = min(bounds.horizon_N_range[1], int(current_N * 1.3))
            target_T = max(bounds.horizon_T_range[0], current_T * 0.8)
        else:
            target_N = current_N
            target_T = current_T
        
        # Curvature-based adaptation
        if curvature > self.high_curvature_threshold:
            # High curvature: shorter horizon for agility
            target_N = max(bounds.horizon_N_range[0], int(target_N * 0.8))
            target_T = max(bounds.horizon_T_range[0], target_T * 0.9)
        elif curvature < self.low_curvature_threshold:
            # Low curvature: longer horizon for efficiency
            target_N = min(bounds.horizon_N_range[1], int(target_N * 1.1))
            target_T = min(bounds.horizon_T_range[1], target_T * 1.1)
        
        # Error-based fine-tuning
        if tracking_error > self.high_error_threshold:
            # High error: more aggressive adaptation
            target_N = max(bounds.horizon_N_range[0], int(target_N * 0.9))
        
        # Apply smoothing
        adapted_N = int(current_N + self.adaptation_rate * (target_N - current_N))
        adapted_T = current_T + self.adaptation_rate * (target_T - current_T)
        
        return (
            np.clip(adapted_N, bounds.horizon_N_range[0], bounds.horizon_N_range[1]),
            np.clip(adapted_T, bounds.horizon_T_range[0], bounds.horizon_T_range[1])
        )

    def compute_weight_adaptation(self,
                                tracking_metrics: TrackingMetrics,
                                environment_metrics: EnvironmentMetrics,
                                current_weights: Dict[str, float],
                                bounds: AdaptiveParameters) -> Dict[str, float]:
        """
        Compute adaptive cost weights based on tracking performance and environment
        """
        adapted_weights = current_weights.copy()
        
        # Lateral error adaptation
        if tracking_metrics.lateral_error > self.high_error_threshold:
            # Increase position weight for better tracking
            target_pos_weight = min(bounds.position_weight_range[1], 
                                  current_weights['position_weight'] * 1.5)
            adapted_weights['position_weight'] = current_weights['position_weight'] + \
                self.adaptation_rate * (target_pos_weight - current_weights['position_weight'])
        
        # Heading error adaptation
        if tracking_metrics.heading_error > self.high_error_threshold:
            # Increase heading weight for better orientation tracking
            target_head_weight = min(bounds.heading_weight_range[1],
                                   current_weights['heading_weight'] * 1.3)
            adapted_weights['heading_weight'] = current_weights['heading_weight'] + \
                self.adaptation_rate * (target_head_weight - current_weights['heading_weight'])
        
        # Velocity error adaptation
        if tracking_metrics.velocity_error > self.high_error_threshold:
            # Increase velocity weight for better speed tracking
            target_vel_weight = min(bounds.velocity_weight_range[1],
                                  current_weights['velocity_weight'] * 1.2)
            adapted_weights['velocity_weight'] = current_weights['velocity_weight'] + \
                self.adaptation_rate * (target_vel_weight - current_weights['velocity_weight'])
        
        # Obstacle proximity adaptation
        if environment_metrics.min_obstacle_distance < 2.0:
            # Close obstacles: increase steering responsiveness, reduce aggressiveness
            target_steer_weight = max(bounds.steering_weight_range[0],
                                    current_weights['steering_weight'] * 0.7)
            adapted_weights['steering_weight'] = current_weights['steering_weight'] + \
                self.adaptation_rate * (target_steer_weight - current_weights['steering_weight'])
        
        # High curvature adaptation
        if environment_metrics.track_curvature > self.high_curvature_threshold:
            # Tight corners: prioritize position tracking over speed
            target_pos_weight = min(bounds.position_weight_range[1],
                                  current_weights['position_weight'] * 1.2)
            target_vel_weight = max(bounds.velocity_weight_range[0],
                                  current_weights['velocity_weight'] * 0.8)
            
            adapted_weights['position_weight'] = current_weights['position_weight'] + \
                self.adaptation_rate * (target_pos_weight - current_weights['position_weight'])
            adapted_weights['velocity_weight'] = current_weights['velocity_weight'] + \
                self.adaptation_rate * (target_vel_weight - current_weights['velocity_weight'])
        
        # Apply bounds
        for weight_name in adapted_weights:
            if weight_name.endswith('_weight'):
                param_name = weight_name.replace('_weight', '_weight_range')
                if hasattr(bounds, param_name):
                    weight_range = getattr(bounds, param_name)
                    adapted_weights[weight_name] = np.clip(adapted_weights[weight_name],
                                                         weight_range[0], weight_range[1])
        
        return adapted_weights


class SafetyManager:
    """
    Manages safety constraints and validates parameter adaptations
    """
    
    def __init__(self, emergency_distance_threshold: float = 0.5):
        self.emergency_distance_threshold = emergency_distance_threshold
        self.safety_violations = 0
        self.last_emergency_time = 0.0
        
    def validate_parameters(self, 
                          adapted_params: AdaptiveParameters,
                          current_metrics: EnvironmentMetrics) -> Tuple[bool, AdaptiveParameters]:
        """
        Validate that adapted parameters maintain safety constraints
        """
        validated_params = adapted_params
        is_safe = True
        
        # Emergency mode for close obstacles
        if current_metrics.min_obstacle_distance < self.emergency_distance_threshold:
            validated_params.horizon_N = max(5, validated_params.horizon_N // 2)
            validated_params.horizon_T = max(0.1, validated_params.horizon_T * 0.5)
            validated_params.position_weight = min(20.0, validated_params.position_weight * 2.0)
            is_safe = False
            self.safety_violations += 1
            self.last_emergency_time = time.time()
        
        # Ensure minimum performance requirements
        if validated_params.horizon_N < 5:
            validated_params.horizon_N = 5
            
        if validated_params.horizon_T < 0.1:
            validated_params.horizon_T = 0.1
            
        return is_safe, validated_params


class AdaptiveMPCController:
    """
    Adaptive Model Predictive Controller with real-time parameter optimization
    """
    
    def __init__(self, 
                 initial_params: Dict[str, Any],
                 adaptation_rate: float = 0.1,
                 enable_adaptation: bool = True):
        """
        Initialize adaptive MPC controller
        
        Args:
            initial_params: Initial MPC parameters
            adaptation_rate: Rate of parameter adaptation (0.0 to 1.0)
            enable_adaptation: Whether to enable adaptive behavior
        """
        self.enable_adaptation = enable_adaptation
        self.adaptation_rate = adaptation_rate
        
        # Initialize base MPC controller
        self.mpc_controller = OptimizedMPCController(**initial_params)
        
        # Initialize adaptive components
        self.adaptation_engine = AdaptationEngine(adaptation_rate)
        self.safety_manager = SafetyManager()
        
        # Initialize adaptive parameters with bounds
        self.adaptive_params = AdaptiveParameters()
        self._update_adaptive_params_from_initial(initial_params)
        
        # Metrics storage
        self.tracking_metrics = TrackingMetrics()
        self.environment_metrics = EnvironmentMetrics()
        
        # Performance monitoring
        self.adaptation_history = []
        self.performance_stats = {
            'adaptations_count': 0,
            'safety_violations': 0,
            'avg_tracking_error': 0.0,
            'adaptation_frequency': 0.0
        }
        
        # Timing
        self.last_adaptation_time = 0.0
        self.adaptation_interval = 0.1  # Adapt every 100ms

    def _update_adaptive_params_from_initial(self, initial_params: Dict[str, Any]):
        """Update adaptive parameters from initial configuration"""
        if 'horizon_N' in initial_params:
            self.adaptive_params.horizon_N = initial_params['horizon_N']
        if 'horizon_T' in initial_params:
            self.adaptive_params.horizon_T = initial_params['horizon_T']
        
        # Update weight parameters
        cost_weights = initial_params.get('cost_function_weights', {})
        if 'position_weight' in cost_weights:
            self.adaptive_params.position_weight = cost_weights['position_weight']
        if 'heading_weight' in cost_weights:
            self.adaptive_params.heading_weight = cost_weights['heading_weight']
        if 'velocity_weight' in cost_weights:
            self.adaptive_params.velocity_weight = cost_weights['velocity_weight']
        if 'steering_weight' in cost_weights:
            self.adaptive_params.steering_weight = cost_weights['steering_weight']
        if 'acceleration_weight' in cost_weights:
            self.adaptive_params.acceleration_weight = cost_weights['acceleration_weight']

    def update_tracking_metrics(self, 
                              current_state: Dict[str, float],
                              reference_trajectory: np.ndarray):
        """
        Update tracking performance metrics
        
        Args:
            current_state: Current vehicle state
            reference_trajectory: Reference trajectory to track
        """
        if len(reference_trajectory) == 0:
            return
            
        # Get closest reference point
        ref_point = reference_trajectory[0]
        
        # Calculate tracking errors
        self.tracking_metrics.lateral_error = abs(
            np.sqrt((current_state['x'] - ref_point[0])**2 + 
                   (current_state['y'] - ref_point[1])**2)
        )
        
        self.tracking_metrics.heading_error = abs(
            current_state.get('theta', 0.0) - ref_point[3] if len(ref_point) > 3 else 0.0
        )
        
        self.tracking_metrics.velocity_error = abs(
            current_state.get('v', 0.0) - ref_point[2] if len(ref_point) > 2 else 0.0
        )
        
        self.tracking_metrics.timestamp = time.time()

    def update_environment_metrics(self, 
                                 lidar_data: Optional[List[float]] = None,
                                 track_info: Optional[Dict[str, float]] = None):
        """
        Update environment-based metrics
        
        Args:
            lidar_data: LiDAR range measurements
            track_info: Track information (curvature, width, etc.)
        """
        # Process LiDAR data
        if lidar_data:
            valid_ranges = [r for r in lidar_data if not np.isinf(r) and not np.isnan(r) and r > 0.0]
            if valid_ranges:
                self.environment_metrics.min_obstacle_distance = min(valid_ranges)
            else:
                self.environment_metrics.min_obstacle_distance = float('inf')
        
        # Process track information
        if track_info:
            self.environment_metrics.track_curvature = track_info.get('curvature', 0.0)
            self.environment_metrics.road_width = track_info.get('width', 1.0)
            self.environment_metrics.average_speed = track_info.get('speed', 0.0)
        
        self.environment_metrics.timestamp = time.time()

    def adapt_parameters(self) -> bool:
        """
        Perform parameter adaptation based on current metrics
        
        Returns:
            bool: True if parameters were adapted
        """
        if not self.enable_adaptation:
            return False
            
        current_time = time.time()
        if current_time - self.last_adaptation_time < self.adaptation_interval:
            return False
        
        # Compute tracking error magnitude
        tracking_error = np.sqrt(
            self.tracking_metrics.lateral_error**2 + 
            self.tracking_metrics.heading_error**2 + 
            self.tracking_metrics.velocity_error**2
        )
        
        # Adapt horizon parameters
        new_N, new_T = self.adaptation_engine.compute_horizon_adaptation(
            speed=self.environment_metrics.average_speed,
            curvature=self.environment_metrics.track_curvature,
            tracking_error=tracking_error,
            current_N=self.adaptive_params.horizon_N,
            current_T=self.adaptive_params.horizon_T,
            bounds=self.adaptive_params
        )
        
        # Adapt cost weights
        current_weights = {
            'position_weight': self.adaptive_params.position_weight,
            'heading_weight': self.adaptive_params.heading_weight,
            'velocity_weight': self.adaptive_params.velocity_weight,
            'steering_weight': self.adaptive_params.steering_weight,
            'acceleration_weight': self.adaptive_params.acceleration_weight
        }
        
        adapted_weights = self.adaptation_engine.compute_weight_adaptation(
            tracking_metrics=self.tracking_metrics,
            environment_metrics=self.environment_metrics,
            current_weights=current_weights,
            bounds=self.adaptive_params
        )
        
        # Update adaptive parameters
        params_changed = False
        if new_N != self.adaptive_params.horizon_N or abs(new_T - self.adaptive_params.horizon_T) > 1e-6:
            self.adaptive_params.horizon_N = new_N
            self.adaptive_params.horizon_T = new_T
            params_changed = True
        
        for weight_name, new_value in adapted_weights.items():
            current_value = getattr(self.adaptive_params, weight_name)
            if abs(new_value - current_value) > 1e-6:
                setattr(self.adaptive_params, weight_name, new_value)
                params_changed = True
        
        # Validate safety
        is_safe, validated_params = self.safety_manager.validate_parameters(
            self.adaptive_params, self.environment_metrics
        )
        
        if not is_safe:
            self.adaptive_params = validated_params
            self.performance_stats['safety_violations'] += 1
        
        # Update MPC controller parameters if changed
        if params_changed:
            self._update_mpc_parameters()
            self.performance_stats['adaptations_count'] += 1
            self.last_adaptation_time = current_time
            
            # Store adaptation history
            self.adaptation_history.append({
                'timestamp': current_time,
                'horizon_N': self.adaptive_params.horizon_N,
                'horizon_T': self.adaptive_params.horizon_T,
                'weights': adapted_weights.copy(),
                'tracking_error': tracking_error,
                'is_safe': is_safe
            })
        
        return params_changed

    def _update_mpc_parameters(self):
        """Update the underlying MPC controller with new parameters"""
        try:
            # Update horizon parameters
            self.mpc_controller.N = self.adaptive_params.horizon_N
            self.mpc_controller.T = self.adaptive_params.horizon_T
            
            # Update cost weights
            if hasattr(self.mpc_controller, 'cost_function_weights'):
                self.mpc_controller.cost_function_weights.update({
                    'position_weight': self.adaptive_params.position_weight,
                    'heading_weight': self.adaptive_params.heading_weight,
                    'velocity_weight': self.adaptive_params.velocity_weight,
                    'steering_weight': self.adaptive_params.steering_weight,
                    'acceleration_weight': self.adaptive_params.acceleration_weight
                })
            
            # Force MPC controller to rebuild optimization problem
            if hasattr(self.mpc_controller, '_setup_optimization_problem'):
                self.mpc_controller._setup_optimization_problem()
                
        except Exception as e:
            # Log error but don't crash
            pass

    def solve_mpc(self, 
                  current_state: Dict[str, float], 
                  reference_trajectory: np.ndarray,
                  lidar_data: Optional[List[float]] = None,
                  track_info: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
        """
        Solve MPC optimization with adaptive parameter adjustment
        
        Args:
            current_state: Current vehicle state
            reference_trajectory: Reference trajectory to follow
            lidar_data: Optional LiDAR data for obstacle awareness
            track_info: Optional track information
            
        Returns:
            Dict containing control commands and adaptation info
        """
        # Update metrics
        self.update_tracking_metrics(current_state, reference_trajectory)
        self.update_environment_metrics(lidar_data, track_info)
        
        # Perform adaptation
        adapted = self.adapt_parameters()
        
        # Solve MPC
        result = self.mpc_controller.solve_mpc(current_state, reference_trajectory)
        
        # Add adaptation information to result
        result['adapted'] = adapted
        result['current_horizon_N'] = self.adaptive_params.horizon_N
        result['current_horizon_T'] = self.adaptive_params.horizon_T
        result['tracking_error'] = np.sqrt(
            self.tracking_metrics.lateral_error**2 + 
            self.tracking_metrics.heading_error**2 + 
            self.tracking_metrics.velocity_error**2
        )
        result['min_obstacle_distance'] = self.environment_metrics.min_obstacle_distance
        
        return result

    def get_adaptation_status(self) -> Dict[str, Any]:
        """Get current adaptation status and statistics"""
        return {
            'adaptive_params': {
                'horizon_N': self.adaptive_params.horizon_N,
                'horizon_T': self.adaptive_params.horizon_T,
                'position_weight': self.adaptive_params.position_weight,
                'heading_weight': self.adaptive_params.heading_weight,
                'velocity_weight': self.adaptive_params.velocity_weight,
                'steering_weight': self.adaptive_params.steering_weight,
                'acceleration_weight': self.adaptive_params.acceleration_weight
            },
            'metrics': {
                'lateral_error': self.tracking_metrics.lateral_error,
                'heading_error': self.tracking_metrics.heading_error,
                'velocity_error': self.tracking_metrics.velocity_error,
                'min_obstacle_distance': self.environment_metrics.min_obstacle_distance,
                'track_curvature': self.environment_metrics.track_curvature
            },
            'performance': self.performance_stats.copy(),
            'enable_adaptation': self.enable_adaptation
        }

    def set_adaptation_enabled(self, enabled: bool):
        """Enable or disable adaptive behavior"""
        self.enable_adaptation = enabled

    def reset_adaptation_history(self):
        """Reset adaptation history and statistics"""
        self.adaptation_history.clear()
        self.performance_stats = {
            'adaptations_count': 0,
            'safety_violations': 0,
            'avg_tracking_error': 0.0,
            'adaptation_frequency': 0.0
        }
