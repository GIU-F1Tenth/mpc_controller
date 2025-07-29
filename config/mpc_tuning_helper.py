#!/usr/bin/env python3
"""
MPC Tuning Helper Script
Usage: python3 mpc_tuning_helper.py
"""

import yaml
import os

def create_tuning_variant(base_config, variant_name, modifications):
    """Create a tuning variant config file"""
    config = base_config.copy()
    
    # Apply modifications
    for key_path, value in modifications.items():
        keys = key_path.split('.')
        current = config['optimized_mpc_controller']['ros__parameters']
        
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        
        current[keys[-1]] = value
    
    # Save variant
    filename = f"params_tuning_{variant_name}.yaml"
    filepath = os.path.join(os.path.dirname(__file__), filename)
    
    with open(filepath, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    print(f"Created {filename}")
    return filepath

def main():
    # Base configuration
    base_config = {
        'optimized_mpc_controller': {
            'ros__parameters': {
                'horizon_N': 10,
                'horizon_T': 0.6,
                'max_steering_angle': 0.5,
                'max_acceleration': 5.0,
                'max_deceleration': 3.0,
                'max_speed': 8.0,
                'cost_function_weights': {
                    'position_weight': 5.0,
                    'heading_weight': 2.0,
                    'velocity_weight': 1.0,
                    'steering_weight': 1.0,
                    'acceleration_weight': 0.5,
                    'jerk_weight': 0.1
                },
                'control_hz': 15.0,
                'mpc_type': 'kinematic',
                'solver_type': 'ipopt'
            }
        }
    }
    
    # Create tuning variants
    variants = {
        'aggressive': {
            'cost_function_weights.position_weight': 10.0,
            'cost_function_weights.velocity_weight': 3.0,
            'cost_function_weights.steering_weight': 0.5,
            'max_acceleration': 8.0,
            'control_hz': 25.0
        },
        'smooth': {
            'cost_function_weights.steering_weight': 3.0,
            'cost_function_weights.acceleration_weight': 2.0,
            'cost_function_weights.jerk_weight': 1.0,
            'max_acceleration': 3.0,
            'control_hz': 10.0
        },
        'fast_solver': {
            'horizon_N': 8,
            'solver_type': 'sqpmethod',
            'control_hz': 30.0
        },
        'high_speed': {
            'horizon_N': 15,
            'horizon_T': 1.0,
            'max_speed': 12.0,
            'mpc_type': 'dynamic'
        }
    }
    
    for variant_name, modifications in variants.items():
        create_tuning_variant(base_config, variant_name, modifications)
    
    print("\nTuning variants created!")
    print("\nTo test a variant:")
    print("ros2 run mpc_controller mpc_node --ros-args --params-file config/params_tuning_<variant>.yaml")
    
    print("\nMonitor performance with:")
    print("ros2 topic echo /mpc/solve_time")
    print("ros2 topic echo /mpc/diagnostics")

if __name__ == "__main__":
    main()
