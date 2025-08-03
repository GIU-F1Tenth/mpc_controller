#!/usr/bin/env python3

"""
Test script for Adaptive MPC Controller

This script tests the basic functionality of the adaptive MPC controller
without requiring ROS2 to be running.

Author: Mohammed Azab <mohammed@azab.io>
"""

import sys
import os
import numpy as np

# Add the package to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_adaptive_mpc_import():
    """Test that adaptive MPC can be imported"""
    try:
        from mpc_controller.adaptive_mpc_controller import (
            AdaptiveMPCController, 
            AdaptiveParameters,
            TrackingMetrics,
            EnvironmentMetrics,
            AdaptationEngine,
            SafetyManager
        )
        print("✅ Successfully imported adaptive MPC classes")
        return True
    except ImportError as e:
        print(f"❌ Failed to import adaptive MPC: {e}")
        return False

def test_adaptive_mpc_initialization():
    """Test adaptive MPC initialization"""
    try:
        from mpc_controller.adaptive_mpc_controller import AdaptiveMPCController
        from mpc_controller.kinematic_bicycle_model import MPCType
        
        # Test parameters
        initial_params = {
            'N': 10,
            'T': 0.5,
            'wheelbase': 0.33,
            'max_speed': 5.0,
            'max_steering_angle': 0.5,
            'max_acceleration': 3.0,
            'max_deceleration': 3.0,
            'min_speed': 0.1,
            'mpc_type': MPCType.KINEMATIC,
            'solver_type': 'ipopt',
            'cost_function_weights': {
                'position_weight': 5.0,
                'heading_weight': 2.0,
                'velocity_weight': 1.0,
                'steering_weight': 0.7,
                'acceleration_weight': 0.5
            }
        }
        
        # Initialize adaptive MPC
        adaptive_mpc = AdaptiveMPCController(
            initial_params=initial_params,
            adaptation_rate=0.1,
            enable_adaptation=True
        )
        
        print("✅ Successfully initialized adaptive MPC controller")
        print(f"   - Horizon N: {adaptive_mpc.adaptive_params.horizon_N}")
        print(f"   - Horizon T: {adaptive_mpc.adaptive_params.horizon_T}")
        print(f"   - Adaptation enabled: {adaptive_mpc.enable_adaptation}")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to initialize adaptive MPC: {e}")
        return False

def test_metrics_update():
    """Test metrics update functionality"""
    try:
        from mpc_controller.adaptive_mpc_controller import AdaptiveMPCController
        from mpc_controller.kinematic_bicycle_model import MPCType
        
        # Initialize controller
        initial_params = {
            'N': 10, 'T': 0.5, 'wheelbase': 0.33, 'max_speed': 5.0,
            'max_steering_angle': 0.5, 'mpc_type': MPCType.KINEMATIC,
            'cost_function_weights': {
                'position_weight': 5.0, 'heading_weight': 2.0, 'velocity_weight': 1.0,
                'steering_weight': 0.7, 'acceleration_weight': 0.5
            }
        }
        
        adaptive_mpc = AdaptiveMPCController(initial_params=initial_params)
        
        # Test tracking metrics update
        current_state = {'x': 0.0, 'y': 0.0, 'v': 2.0, 'theta': 0.0}
        reference_trajectory = np.array([[1.0, 0.0, 2.0, 0.0]])
        
        adaptive_mpc.update_tracking_metrics(current_state, reference_trajectory)
        
        print("✅ Successfully updated tracking metrics")
        print(f"   - Lateral error: {adaptive_mpc.tracking_metrics.lateral_error:.3f}")
        print(f"   - Heading error: {adaptive_mpc.tracking_metrics.heading_error:.3f}")
        
        # Test environment metrics update
        lidar_data = [1.0, 1.5, 2.0, 1.8, 1.2]  # Simulated LiDAR
        track_info = {'curvature': 0.1, 'width': 2.0, 'speed': 2.0}
        
        adaptive_mpc.update_environment_metrics(lidar_data, track_info)
        
        print("✅ Successfully updated environment metrics")
        print(f"   - Min obstacle distance: {adaptive_mpc.environment_metrics.min_obstacle_distance:.3f}")
        print(f"   - Track curvature: {adaptive_mpc.environment_metrics.track_curvature:.3f}")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to update metrics: {e}")
        return False

def test_adaptation_engine():
    """Test adaptation engine functionality"""
    try:
        from mpc_controller.adaptive_mpc_controller import (
            AdaptationEngine, 
            AdaptiveParameters,
            TrackingMetrics,
            EnvironmentMetrics
        )
        
        # Initialize adaptation engine
        engine = AdaptationEngine(adaptation_rate=0.1)
        bounds = AdaptiveParameters()
        
        # Test horizon adaptation
        new_N, new_T = engine.compute_horizon_adaptation(
            speed=3.0,
            curvature=0.2,
            tracking_error=0.1,
            current_N=25,
            current_T=0.3,
            bounds=bounds
        )
        
        print("✅ Successfully computed horizon adaptation")
        print(f"   - New N: {new_N}")
        print(f"   - New T: {new_T:.3f}")
        
        # Test weight adaptation
        tracking_metrics = TrackingMetrics(lateral_error=0.1, heading_error=0.05, velocity_error=0.2)
        environment_metrics = EnvironmentMetrics(min_obstacle_distance=2.0, track_curvature=0.1)
        
        current_weights = {
            'position_weight': 5.0,
            'heading_weight': 2.0,
            'velocity_weight': 1.0,
            'steering_weight': 0.7,
            'acceleration_weight': 0.5
        }
        
        adapted_weights = engine.compute_weight_adaptation(
            tracking_metrics=tracking_metrics,
            environment_metrics=environment_metrics,
            current_weights=current_weights,
            bounds=bounds
        )
        
        print("✅ Successfully computed weight adaptation")
        for key, value in adapted_weights.items():
            print(f"   - {key}: {value:.3f}")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to test adaptation engine: {e}")
        return False

def main():
    """Run all tests"""
    print("🧪 Testing Adaptive MPC Controller")
    print("=" * 50)
    
    tests = [
        test_adaptive_mpc_import,
        test_adaptive_mpc_initialization,
        test_metrics_update,
        test_adaptation_engine
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        print(f"\n🔍 Running {test.__name__}...")
        if test():
            passed += 1
        print("-" * 40)
    
    print(f"\n📊 Results: {passed}/{total} tests passed")
    
    if passed == total:
        print("🎉 All tests passed! Adaptive MPC is ready.")
        return True
    else:
        print("❌ Some tests failed. Please check the implementation.")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
