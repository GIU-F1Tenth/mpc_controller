import casadi as ca
import numpy as np
import pandas as pd

class MPC_Controller:
    def __init__(self, N=10, T=1.0, L=0.33):
        self.N = N       # Prediction horizon
        self.T = T       # Total time horizon
        self.dt = T / N  # Time step
        self.L = L       # Wheelbase

        self._setup_optimizer()     # Create optimization problem

    def _setup_optimizer(self):

        # State variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        v = ca.SX.sym('v')
        theta = ca.SX.sym('theta')

        # Control variables
        a = ca.SX.sym('a')   
        delta = ca.SX.sym('delta')

        # System dynamics (Bicycle model)
        xdot = v * ca.cos(theta)
        ydot = v * ca.sin(theta)
        vdot = a
        thetadot = v * ca.tan(delta) / self.L

        # Define system function
        self.f = ca.Function('f', [x, y, v, theta, a, delta], [xdot, ydot, vdot, thetadot])

        # Optimization variables
        self.opti = ca.Opti()
        self.U = self.opti.variable(2, self.N)  # [Acceleration, Steering]
        self.X = self.opti.variable(4, self.N + 1)  # [x, y, v, theta]
        

        # Cost function weights
        self.Q = np.diag([10, 10, 1, 1])  # Penalize position & velocity error
        self.R = np.diag([1, 1])  # Penalize control input

        # Reference trajectory
        self.X_ref = self.opti.parameter(4, self.N + 1)

        # The Cost function
        cost = 0
        for i in range(self.N):
            state_error = self.X[:, i] - self.X_ref[:, i]
            control_effort = self.U[:, i]
            cost += ca.mtimes([state_error.T, self.Q, state_error]) + ca.mtimes([control_effort.T, self.R, control_effort])
        
        self.opti.minimize(cost)

        # Constraints
        g = []
        for i in range(self.N):
            x_next = self.X[:, i] + self.dt * ca.vertcat(*self.f(self.X[0, i], self.X[1, i], self.X[2, i], self.X[3, i], self.U[0, i], self.U[1, i]))
            g.append(self.X[:, i+1] - x_next)

        self.opti.subject_to(ca.vertcat(*g) == 0)

        # Control limits
        self.opti.subject_to(self.opti.bounded(-2, self.U[0, :], 2))             # Acceleration limits
        self.opti.subject_to(self.opti.bounded(-0.4, self.U[1, :], 0.4))         # Steering limits

        # Solver
        self.opti.solver('ipopt')

    def solve_mpc(self, x, y, v, theta, X_ref):

        try:
            self.opti.set_value(self.X_ref, X_ref)
            sol = self.opti.solve()
            optimal_accel = sol.value(self.U[0, 0])
            optimal_steer = sol.value(self.U[1, 0])
            return optimal_accel, optimal_steer
        except RuntimeError:
            return 0.0, 0.0  


# Main function to test
#if __name__ == "__main__":
 #   print("Initializing MPC Controller...")
  #  mpc = MPC_Controller()
