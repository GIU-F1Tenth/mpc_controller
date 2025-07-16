import casadi as ca
import numpy as np

class MPC_Controller:
    def __init__(self, N=10, T=1.0, L=0.33):
        """
        Model Predictive Controller using bicycle model.

        Parameters:
            N (int): Prediction horizon
            T (float): Time horizon in seconds
            L (float): Wheelbase of the vehicle
        """
        self.N = N       # Prediction steps
        self.T = T       # Total time horizon
        self.dt = T / N  # Time step
        self.L = L       # Wheelbase

        self._setup_optimizer()

    def _setup_optimizer(self):
        # State and control variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        v = ca.SX.sym('v')
        theta = ca.SX.sym('theta')

        a = ca.SX.sym('a')       # Acceleration
        delta = ca.SX.sym('delta')  # Steering angle

        # System dynamics using bicycle model
        xdot = v * ca.cos(theta)
        ydot = v * ca.sin(theta)
        vdot = a
        thetadot = v * ca.tan(delta) / self.L

        # State transition function
        self.f = ca.Function('f', [x, y, v, theta, a, delta], [xdot, ydot, vdot, thetadot])

        # Optimization problem
        self.opti = ca.Opti()

        self.U = self.opti.variable(2, self.N)         # Control inputs: [acceleration, steering]
        self.X = self.opti.variable(4, self.N + 1)     # States: [x, y, v, theta]
        self.X_ref = self.opti.parameter(4, self.N + 1)  # Reference trajectory

        # Cost weights
        self.Q = np.diag([10, 10, 1, 1])  # State error weights
        self.R = np.diag([1, 1])          # Control input weights

        # Cost function
        cost = 0
        for i in range(self.N):
            state_error = self.X[:, i] - self.X_ref[:, i]
            control_effort = self.U[:, i]
            cost += ca.mtimes([state_error.T, self.Q, state_error]) + \
                    ca.mtimes([control_effort.T, self.R, control_effort])
        self.opti.minimize(cost)

        # Dynamics constraints
        g = []
        for i in range(self.N):
            x_next = self.X[:, i] + self.dt * ca.vertcat(*self.f(
                self.X[0, i], self.X[1, i], self.X[2, i], self.X[3, i],
                self.U[0, i], self.U[1, i]
            ))
            g.append(self.X[:, i + 1] - x_next)
        self.opti.subject_to(ca.vertcat(*g) == 0)

        # Control input limits
        self.opti.subject_to(self.opti.bounded(-2.0, self.U[0, :], 2.0))   # Acceleration
        self.opti.subject_to(self.opti.bounded(-0.4, self.U[1, :], 0.4))  # Steering

        # Solver setup
        self.opti.solver('ipopt')

    def solve_mpc(self, x, y, v, theta, X_ref):
        """
        Solve the MPC optimization problem.

        Parameters:
            x, y, v, theta (float): Current state
            X_ref (np.ndarray): Reference trajectory of shape (4, N+1)

        Returns:
            Tuple (acceleration, steering)
        """
        try:
            self.opti.set_value(self.X_ref, X_ref)

            # Set initial state constraint
            self.opti.set_initial(self.X[:, 0], [x, y, v, theta])

            sol = self.opti.solve()

            optimal_accel = sol.value(self.U[0, 0])
            optimal_steer = sol.value(self.U[1, 0])
            return optimal_accel, optimal_steer
        except RuntimeError:
            # If solver fails, return zero controls
            return 0.0, 0.0
