#!/usr/bin/env python3

"""
Test script to verify MPC node functionality and configuration usage
"""

import yaml
import numpy as np
from mpc_controller.optimized_mpc_controller import OptimizedMPCController
from mpc_controller.dynamic_bicycle_model import DynamicBicycleModel
from mpc_controller.kinematic_bicycle_model import KinematicBicycleModel, MPCType
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'mpc_controller'))


def test_configuration_loading():
    """Test if configuration files are properly structured"""
    print("🔧 Testing Configuration Loading...")

    config_files = [
        'config/params.yaml',
        'config/params_aggressive.yaml',
        'config/params_conservative.yaml',
        'config/params_precision.yaml'
    ]

    for config_file in config_files:
        if os.path.exists(config_file):
            try:
                with open(config_file, 'r') as f:
                    config = yaml.safe_load(f)

                if 'optimized_mpc_controller' in config and 'ros__parameters' in config['optimized_mpc_controller']:
                    params = config['optimized_mpc_controller']['ros__parameters']

                    # Check required parameters
                    required_params = [
                        'wheelbase', 'horizon_N', 'horizon_T', 'max_steering_angle',
                        'max_acceleration', 'max_speed', 'mpc_type', 'solver_type'
                    ]

                    missing_params = [p for p in required_params if p not in params]
                    if missing_params:
                        print(f"❌ {config_file}: Missing parameters: {missing_params}")
                    else:
                        print(f"✅ {config_file}: All required parameters present")

                        # Validate parameter values
                        if params['mpc_type'] not in ['kinematic', 'dynamic']:
                            print(f"⚠️  {config_file}: Invalid mpc_type: {params['mpc_type']}")
                        if params['solver_type'] not in ['ipopt', 'sqpmethod']:
                            print(f"⚠️  {config_file}: Invalid solver_type: {params['solver_type']}")

                else:
                    print(f"❌ {config_file}: Invalid structure - missing 'optimized_mpc_controller/ros__parameters'")

            except Exception as e:
                print(f"❌ {config_file}: Error loading - {e}")
        else:
            print(f"❌ {config_file}: File not found")

    print()


def test_kinematic_model():
    """Test kinematic bicycle model"""
    print("🚗 Testing Kinematic Bicycle Model...")

    try:
        model = KinematicBicycleModel(wheelbase=0.33, dt=0.05)
        dynamics = model.get_dynamics()

        # Test with sample inputs
        x, y, v, theta = 0.0, 0.0, 1.0, 0.0
        a, delta = 0.5, 0.1

        result = dynamics(x, y, v, theta, a, delta)
        print(f"✅ Kinematic model test passed - output shape: {result.shape}")

    except Exception as e:
        print(f"❌ Kinematic model test failed: {e}")

    print()


def test_dynamic_model():
    """Test dynamic bicycle model"""
    print("🏎️ Testing Dynamic Bicycle Model...")

    try:
        model = DynamicBicycleModel(wheelbase=0.33, dt=0.05)
        dynamics = model.get_dynamics()

        # Test with sample inputs
        x, y, v, theta, beta, r = 0.0, 0.0, 1.0, 0.0, 0.0, 0.0
        a, delta = 0.5, 0.1

        result = dynamics(x, y, v, theta, beta, r, a, delta)
        print(f"✅ Dynamic model test passed - output shape: {result.shape}")

    except Exception as e:
        print(f"❌ Dynamic model test failed: {e}")

    print()


def test_mpc_controller_initialization():
    """Test MPC controller initialization with different configurations"""
    print("🎯 Testing MPC Controller Initialization...")

    # Test kinematic configuration
    try:
        kinematic_controller = OptimizedMPCController(
            N=10,
            T=1.0,
            wheelbase=0.33,
            mpc_type=MPCType.KINEMATIC,
            solver_type='ipopt',
            enable_logging=False,
            max_steering_angle=0.5,
            max_acceleration=1.0,
            max_deceleration=1.0,
            min_speed=0.1,
            max_speed=2.0,
            enable_cost_function_weights=True,
            cost_function_weights={
                'steering_weight': 0.1,
                'acceleration_weight': 0.1,
                'jerk_weight': 0.1,
                'heading_weight': 0.1,
                'position_weight': 0.1,
                'velocity_weight': 0.1
            },
            enable_hard_constraints=True,
            hard_constraints={
                'max_steering_angle': 0.5,
                'max_acceleration': 1.0,
                'max_deceleration': 1.0
            },
            enable_obstacle_avoidance=False,
            obstacle_avoidance_weight=0.5,
            enable_speed_control=True,
            speed_control_weight=0.3,
            enable_trajectory_tracking=True,
            trajectory_tracking_weight=0.2,
            enable_safety_checks=True,
            safety_check_distance=0.5,
            lookahead_distance=0.7
        )
        print("✅ Kinematic MPC controller initialized successfully")

    except Exception as e:
        print(f"❌ Kinematic MPC controller initialization failed: {e}")

    # Test dynamic configuration
    try:
        dynamic_controller = OptimizedMPCController(
            N=10,
            T=1.0,
            wheelbase=0.33,
            mpc_type=MPCType.DYNAMIC,
            solver_type='ipopt',
            enable_logging=False,
            max_steering_angle=0.5,
            max_acceleration=1.0,
            max_deceleration=1.0,
            min_speed=0.1,
            max_speed=2.0,
            enable_cost_function_weights=True,
            cost_function_weights={
                'steering_weight': 0.1,
                'acceleration_weight': 0.1,
                'jerk_weight': 0.1,
                'heading_weight': 0.1,
                'position_weight': 0.1,
                'velocity_weight': 0.1
            },
            enable_hard_constraints=True,
            hard_constraints={
                'max_steering_angle': 0.5,
                'max_acceleration': 1.0,
                'max_deceleration': 1.0
            },
            enable_obstacle_avoidance=False,
            obstacle_avoidance_weight=0.5,
            enable_speed_control=True,
            speed_control_weight=0.3,
            enable_trajectory_tracking=True,
            trajectory_tracking_weight=0.2,
            enable_safety_checks=True,
            safety_check_distance=0.5,
            lookahead_distance=0.7
        )
        print("✅ Dynamic MPC controller initialized successfully")

    except Exception as e:
        print(f"❌ Dynamic MPC controller initialization failed: {e}")

    print()


def test_mpc_solve():
    """Test MPC solve functionality"""
    print("🧮 Testing MPC Solve Functionality...")

    try:
        controller = OptimizedMPCController(
            N=5,  # Smaller horizon for testing
            T=0.5,
            wheelbase=0.33,
            mpc_type=MPCType.KINEMATIC,
            solver_type='ipopt',
            enable_logging=False,
            max_steering_angle=0.5,
            max_acceleration=1.0,
            max_deceleration=1.0,
            min_speed=0.1,
            max_speed=2.0,
            enable_cost_function_weights=True
        )

        # Create test current state
        current_state = {
            'x': 0.0,
            'y': 0.0,
            'v': 1.0,
            'theta': 0.0
        }

        # Create test reference trajectory
        reference_trajectory = np.array([
            [0.0, 0.0, 1.0, 0.0],  # x, y, v, theta
            [0.5, 0.0, 1.0, 0.0],
            [1.0, 0.0, 1.0, 0.0],
            [1.5, 0.0, 1.0, 0.0],
            [2.0, 0.0, 1.0, 0.0],
            [2.5, 0.0, 1.0, 0.0]
        ])

        # Set initial guess for optimization variables
        try:
            # Initialize control inputs to reasonable values
            U_init = np.zeros((2, controller.N))
            controller.opti.set_initial(controller.U, U_init)

            # Initialize states with simple trajectory following
            X_init = np.zeros((controller.n_states, controller.N + 1))
            for i in range(controller.N + 1):
                if i < len(reference_trajectory):
                    X_init[:controller.n_states, i] = reference_trajectory[i, :controller.n_states]
                else:
                    X_init[:controller.n_states, i] = reference_trajectory[-1, :controller.n_states]
            controller.opti.set_initial(controller.X, X_init)
        except BaseException:
            pass  # Initial guess setup failed, continue anyway

        # Solve MPC
        result = controller.solve_mpc(current_state, reference_trajectory)

        if result['success']:
            print(f"✅ MPC solve successful:")
            print(f"   - Solve time: {result['solve_time']:.4f}s")
            print(f"   - Acceleration: {result['acceleration']:.3f}")
            print(f"   - Steering: {result['steering']:.3f}")
        else:
            print(f"❌ MPC solve failed: {result['error']}")

    except Exception as e:
        print(f"❌ MPC solve test failed: {e}")

    print()


def test_parameter_usage():
    """Test if all files properly use configuration parameters"""
    print("📋 Testing Parameter Usage Across Files...")

    files_to_check = [
        'mpc_controller/mpc_node.py',
        'mpc_controller/optimized_mpc_controller.py',
        'mpc_controller/kinematic_bicycle_model.py',
        'mpc_controller/dynamic_bicycle_model.py'
    ]

    parameters_from_yaml = [
        'wheelbase', 'horizon_N', 'horizon_T', 'max_steering_angle',
        'max_acceleration', 'max_deceleration', 'min_speed', 'max_speed',
        'enable_cost_function_weights', 'cost_function_weights',
        'enable_hard_constraints', 'hard_constraints',
        'enable_obstacle_avoidance', 'obstacle_avoidance_weight',
        'enable_speed_control', 'speed_control_weight',
        'enable_trajectory_tracking', 'trajectory_tracking_weight',
        'enable_safety_checks', 'safety_check_distance',
        'mpc_type', 'solver_type', 'control_hz',
        'safety_timeout', 'emergency_brake_threshold'
    ]

    for file_path in files_to_check:
        if os.path.exists(file_path):
            try:
                with open(file_path, 'r') as f:
                    content = f.read()

                used_params = []
                for param in parameters_from_yaml:
                    if param in content:
                        used_params.append(param)

                print(f"📄 {file_path}:")
                print(f"   - Uses {len(used_params)}/{len(parameters_from_yaml)} parameters")

                if len(used_params) < 5:  # Flag files that use very few parameters
                    print(f"   ⚠️  May not be fully utilizing configuration")

            except Exception as e:
                print(f"❌ Error checking {file_path}: {e}")
        else:
            print(f"❌ {file_path}: File not found")

    print()


def main():
    """Run all tests"""
    print("🏁 F1TENTH MPC Node Functionality Test")
    print("=" * 50)

    test_configuration_loading()
    test_kinematic_model()
    test_dynamic_model()
    test_mpc_controller_initialization()
    test_mpc_solve()
    test_parameter_usage()

    print("=" * 50)
    print("🏁 Test Complete!")
    print("\n📋 Expected Output When Running:")
    print("1. Configuration files should load without errors")
    print("2. Both kinematic and dynamic models should initialize")
    print("3. MPC controller should solve optimization problems")
    print("4. All parameter files should be properly utilized")
    print("\n🚀 To run the actual node:")
    print("ros2 launch mpc_controller mpc_controller.launch.py")
    print("ros2 launch mpc_controller mpc_controller.launch.py config:=params_aggressive")


if __name__ == '__main__':
    main()
