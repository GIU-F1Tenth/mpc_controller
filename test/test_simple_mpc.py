#!/usr/bin/env python3

"""
Simple MPC test to verify basic functionality
"""

import numpy as np
import casadi as ca
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'mpc_controller'))


def test_simple_kinematic_model():
    """Test simple kinematic model directly"""
    print("🧪 Testing Simple Kinematic Model...")

    # State variables: [x, y, v, theta]
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    v = ca.SX.sym('v')
    theta = ca.SX.sym('theta')

    # Control inputs: [acceleration, steering_angle]
    a = ca.SX.sym('a')
    delta = ca.SX.sym('delta')

    # Simple kinematic bicycle model
    L = 0.33  # wheelbase
    dt = 0.1  # time step

    # State derivatives
    xdot = v * ca.cos(theta)
    ydot = v * ca.sin(theta)
    vdot = a
    thetadot = (v / L) * ca.tan(delta)

    # Euler integration
    state_next = ca.vertcat(
        x + dt * xdot,
        y + dt * ydot,
        v + dt * vdot,
        theta + dt * thetadot
    )

    # Create function
    dynamics = ca.Function('dynamics',
                           [x, y, v, theta, a, delta],
                           [state_next])

    # Test the function
    try:
        result = dynamics(0, 0, 1, 0, 0.5, 0.1)
        print(f"✅ Simple kinematic model works: {result}")
        return True
    except Exception as e:
        print(f"❌ Simple kinematic model failed: {e}")
        return False


def test_simple_mpc():
    """Test simple MPC setup"""
    print("🎯 Testing Simple MPC Setup...")

    try:
        # MPC parameters
        N = 5  # horizon length

        # Set up optimization
        opti = ca.Opti()

        # Decision variables
        U = opti.variable(2, N)      # [acceleration, steering]
        X = opti.variable(4, N + 1)  # [x, y, v, theta]

        # Parameters
        X_ref = opti.parameter(4, N + 1)
        X0 = opti.parameter(4, 1)

        # Simple cost function
        cost = 0
        Q = np.diag([10, 10, 1, 5])  # state weights
        R = np.diag([1, 1])          # control weights

        for i in range(N):
            state_error = X[:, i] - X_ref[:, i]
            control_input = U[:, i]

            cost += ca.mtimes([state_error.T, Q, state_error])
            cost += ca.mtimes([control_input.T, R, control_input])

        # Terminal cost
        terminal_error = X[:, -1] - X_ref[:, -1]
        cost += ca.mtimes([terminal_error.T, Q * 2, terminal_error])

        opti.minimize(cost)

        # Initial condition
        opti.subject_to(X[:, 0] == X0)

        # Dynamics - simple version
        L = 0.33
        dt = 0.1

        for i in range(N):
            x_i = X[0, i]
            y_i = X[1, i]
            v_i = X[2, i]
            theta_i = X[3, i]
            a_i = U[0, i]
            delta_i = U[1, i]

            # Simple dynamics
            x_next = x_i + dt * v_i * ca.cos(theta_i)
            y_next = y_i + dt * v_i * ca.sin(theta_i)
            v_next = v_i + dt * a_i
            theta_next = theta_i + dt * (v_i / L) * ca.tan(delta_i)

            opti.subject_to(X[0, i + 1] == x_next)
            opti.subject_to(X[1, i + 1] == y_next)
            opti.subject_to(X[2, i + 1] == v_next)
            opti.subject_to(X[3, i + 1] == theta_next)

        # Constraints
        opti.subject_to(opti.bounded(-1.0, U[0, :], 1.0))  # acceleration
        opti.subject_to(opti.bounded(-0.5, U[1, :], 0.5))  # steering
        opti.subject_to(opti.bounded(0.1, X[2, :], 2.0))   # velocity

        # Solver
        solver_opts = {
            'ipopt.print_level': 0,
            'ipopt.max_iter': 100,
            'print_time': False
        }
        opti.solver('ipopt', solver_opts)

        # Test solve
        current_state = np.array([[0], [0], [1], [0]])
        reference = np.array([
            [0, 0.5, 1.0, 1.5, 2.0, 2.5],  # x
            [0, 0, 0, 0, 0, 0],             # y
            [1, 1, 1, 1, 1, 1],             # v
            [0, 0, 0, 0, 0, 0]              # theta
        ])

        opti.set_value(X0, current_state)
        opti.set_value(X_ref, reference)

        # Initial guess
        opti.set_initial(U, np.zeros((2, N)))
        opti.set_initial(X, reference)

        sol = opti.solve()

        print("✅ Simple MPC solve successful!")
        print(f"   - First control: a={sol.value(U[0, 0]):.3f}, δ={sol.value(U[1, 0]):.3f}")
        return True

    except Exception as e:
        print(f"❌ Simple MPC failed: {e}")
        return False


def main():
    print("🧪 Simple MPC Tests")
    print("=" * 30)

    test1 = test_simple_kinematic_model()
    test2 = test_simple_mpc()

    print("=" * 30)
    if test1 and test2:
        print("✅ All simple tests passed!")
        print("💡 The basic MPC setup is working correctly.")
        print("   The issue might be in the complex model implementation.")
    else:
        print("❌ Some tests failed")
        print("🔧 Check CasADi installation and basic setup")


if __name__ == '__main__':
    main()
