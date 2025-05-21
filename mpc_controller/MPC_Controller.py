import casadi as ca
import numpy as np
import pandas as pd

class MPC_Controller:
    def __init__(self, N=10, T=1.0, L=0.33, Q=None, R=None):
        self.N = N       # Prediction horizon
        self.T = T       # Total time horizon
        self.dt = T / N  # Time step
        self.L = L       # Wheelbase

        # Cost function weights (can be customized)
        self.Q = Q if Q is not None else np.diag([10, 10, 1, 1])
        self.R = R if R is not None else np.diag([1, 1])

        self._setup_optimizer()  # Create optimization problem

    def _setup_optimizer(self):
        # Define state and control variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        v = ca.SX.sym('v')
        theta = ca.SX.sym('theta')
        a = ca.SX.sym('a')   
        delta = ca.SX.sym('delta')

        # System dynamics (Bicycle model)
        xdot = v * ca.cos(theta)
        ydot = v * ca.sin(theta)
        vdot = a
        thetadot = v * ca.tan(delta) / self.L

        # System dynamics function
        self.f = ca.Function('f', [x, y, v, theta, a, delta], [xdot, ydot, vdot, thetadot])

        # Optimization problem setup
        self.opti = ca.Opti()
        self.U = self.opti.variable(2, self.N)         # [acceleration, steering]
        self.X = self.opti.variable(4, self.N + 1)      # [x, y, v, theta]
        self.X_ref = self.opti.parameter(4, self.N + 1) # Reference trajectory
        self.X0 = self.opti.parameter(4)                # Initial state

        # Initial state constraint
        self.opti.subject_to(self.X[:, 0] == self.X0)

        # Cost function
        cost = 0
        for i in range(self.N):
            state_error = self.X[:, i] - self.X_ref[:, i]
            control_effort = self.U[:, i]
            cost += ca.mtimes([state_error.T, self.Q, state_error]) + ca.mtimes([control_effort.T, self.R, control_effort])

        # Optional terminal cost
        terminal_error = self.X[:, self.N] - self.X_ref[:, self.N]
        cost += ca.mtimes([terminal_error.T, self.Q * 10, terminal_error])  # Heavier terminal weight

        self.opti.minimize(cost)

        # Dynamics constraints
        g = []
        for i in range(self.N):
            x_next = self.X[:, i] + self.dt * ca.vertcat(*self.f(
                self.X[0, i], self.X[1, i], self.X[2, i], self.X[3, i], self.U[0, i], self.U[1, i]
            ))
            g.append(self.X[:, i + 1] - x_next)
        self.opti.subject_to(ca.vertcat(*g) == 0)

        # Control bounds
        self.opti.subject_to(self.opti.bounded(-2.0, self.U[0, :], 2.0))       # Acceleration
        self.opti.subject_to(self.opti.bounded(-0.4, self.U[1, :], 0.4))       # Steering angle

        # Solver setup
        self.opti.solver('ipopt')

    def solve_mpc(self, x, y, v, theta, X_ref):
        try:
            # Set initial and reference state
            self.opti.set_value(self.X0, [x, y, v, theta])
            self.opti.set_value(self.X_ref, X_ref)

            # Warm start
            self.opti.set_initial(self.U, np.zeros((2, self.N)))
            self.opti.set_initial(self.X, np.tile(np.array([[x], [y], [v], [theta]]), (1, self.N + 1)))

            # Solve the optimization problem
            sol = self.opti.solve()
            optimal_accel = sol.value(self.U[0, 0])
            optimal_steer = sol.value(self.U[1, 0])
            predicted_states = sol.value(self.X)

            return optimal_accel, optimal_steer, predicted_states

        except RuntimeError as e:
            print("[MPC] Optimization failed:", e)
            return 0.0, 0.0, None
