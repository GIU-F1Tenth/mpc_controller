"""
Model Predictive Controller for F1Tenth autonomous vehicle.

This module implements a Model Predictive Controller using the kinematic bicycle model
for trajectory tracking of an F1Tenth vehicle.
"""

import logging
from typing import Tuple, Optional
import casadi as ca
import numpy as np

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MPCController:
    """
    Model Predictive Controller for F1Tenth vehicle using kinematic bicycle model.

    This controller optimizes acceleration and steering commands to follow a reference
    trajectory while respecting vehicle dynamics and constraints.

    Attributes:
        N (int): Prediction horizon
        T (float): Total time horizon
        dt (float): Time step
        L (float): Vehicle wheelbase
        Q (np.ndarray): State weight matrix
        R (np.ndarray): Control weight matrix
    """

    def __init__(self, N: int = 10, T: float = 1.0, L: float = 0.33):
        """
        Initialize the MPC controller.

        Args:
            N: Prediction horizon (number of steps)
            T: Total time horizon (seconds)
            L: Vehicle wheelbase (meters)
        """
        self.N = N
        self.T = T
        self.dt = T / N
        self.L = L

        # Cost function weights
        # State weights [x, y, v, theta]
        self.Q = np.diag([4.0, 4.0, 1.0, 2.0])
        self.R = np.diag([1.0, 1.0])              # Control weights [a, delta]

        # Control limits
        self.max_acceleration = 2.0    # m/s²
        self.min_acceleration = -2.0   # m/s²
        self.max_steering = 0.4        # rad
        self.min_steering = -0.4       # rad

        self._setup_optimizer()
        logger.info(f"MPC Controller initialized with N={N}, T={T}, L={L}")

    def _setup_optimizer(self) -> None:
        """Set up the CasADi optimization problem."""

    def _setup_optimizer(self) -> None:
        """Set up the CasADi optimization problem."""
        # State variables [x, y, v, theta]
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        v = ca.SX.sym('v')
        theta = ca.SX.sym('theta')

        # Control variables [acceleration, steering_angle]
        a = ca.SX.sym('a')
        delta = ca.SX.sym('delta')

        # Kinematic bicycle model dynamics
        x_dot = v * ca.cos(theta)
        y_dot = v * ca.sin(theta)
        v_dot = a
        theta_dot = v * ca.tan(delta) / self.L

        # System dynamics function
        self.f = ca.Function(
            'f',
            [x, y, v, theta, a, delta],
            [x_dot, y_dot, v_dot, theta_dot],
            ['x', 'y', 'v', 'theta', 'a', 'delta'],
            ['x_dot', 'y_dot', 'v_dot', 'theta_dot']
        )

        # Initialize optimization problem
        self.opti = ca.Opti()

        # Decision variables
        self.U = self.opti.variable(2, self.N)      # Control inputs [a, delta]
        self.X = self.opti.variable(4, self.N + 1)  # States [x, y, v, theta]

        # Parameters
        self.X_ref = self.opti.parameter(4, self.N + 1)  # Reference trajectory
        self.x0 = self.opti.parameter(4, 1)              # Initial state

        # Objective function
        self._setup_cost_function()

        # Constraints
        self._setup_constraints()

        # Solver configuration
        opts = {
            'ipopt.print_level': 0,
            'ipopt.max_iter': 100,
            'print_time': 0,
            'verbose': False
        }
        self.opti.solver('ipopt', opts)

    def _setup_cost_function(self) -> None:
        """Set up the MPC cost function."""
        cost = 0

        # Stage costs
        for k in range(self.N):
            # State tracking error
            state_error = self.X[:, k] - self.X_ref[:, k]
            cost += ca.mtimes([state_error.T, self.Q, state_error])

            # Control effort
            control_effort = self.U[:, k]
            cost += ca.mtimes([control_effort.T, self.R, control_effort])

        # Terminal cost
        terminal_error = self.X[:, self.N] - self.X_ref[:, self.N]
        cost += ca.mtimes([terminal_error.T, self.Q, terminal_error])

        self.opti.minimize(cost)

    def _setup_constraints(self) -> None:
        """Set up optimization constraints."""
        # Initial condition constraint
        self.opti.subject_to(self.X[:, 0] == self.x0)

        # System dynamics constraints
        for k in range(self.N):
            x_next = self.X[:, k] + self.dt * ca.vertcat(
                *self.f(
                    self.X[0, k], self.X[1, k], self.X[2, k], self.X[3, k],
                    self.U[0, k], self.U[1, k]
                )
            )
            self.opti.subject_to(self.X[:, k + 1] == x_next)

        # Control constraints
        self.opti.subject_to(
            self.opti.bounded(self.min_acceleration,
                              self.U[0, :], self.max_acceleration)
        )
        self.opti.subject_to(
            self.opti.bounded(self.min_steering,
                              self.U[1, :], self.max_steering)
        )

        # State constraints (optional - can be added based on requirements)
        # Velocity constraints
        self.opti.subject_to(self.X[2, :] >= 0)  # Non-negative velocity

    def solve_mpc(
        self,
        x: float,
        y: float,
        v: float,
        theta: float,
        X_ref: np.ndarray
    ) -> Tuple[float, float]:
        """
        Solve the MPC optimization problem.

        Args:
            x: Current x position
            y: Current y position  
            v: Current velocity
            theta: Current heading angle
            X_ref: Reference trajectory (4 x N+1 array)

        Returns:
            Tuple of (optimal_acceleration, optimal_steering_angle)
        """
        try:
            # Set initial state
            current_state = np.array([x, y, v, theta]).reshape(-1, 1)
            self.opti.set_value(self.x0, current_state)

            # Set reference trajectory
            if X_ref.shape != (4, self.N + 1):
                raise ValueError(
                    f"X_ref must have shape (4, {self.N + 1}), got {X_ref.shape}")

            self.opti.set_value(self.X_ref, X_ref)

            # Solve optimization problem
            sol = self.opti.solve()

            # Extract optimal control inputs
            optimal_acceleration = float(sol.value(self.U[0, 0]))
            optimal_steering = float(sol.value(self.U[1, 0]))

            logger.debug(
                f"MPC solved: a={optimal_acceleration:.3f}, δ={optimal_steering:.3f}")

            return optimal_acceleration, optimal_steering

        except RuntimeError as e:
            logger.warning(f"MPC solver failed: {e}. Using fallback control.")
            return 0.0, 0.0
        except Exception as e:
            logger.error(f"Unexpected error in MPC solve: {e}")
            return 0.0, 0.0

    def update_weights(self, Q: Optional[np.ndarray] = None, R: Optional[np.ndarray] = None) -> None:
        """
        Update the cost function weights.

        Args:
            Q: New state weight matrix (4x4)
            R: New control weight matrix (2x2)
        """
        if Q is not None:
            if Q.shape != (4, 4):
                raise ValueError("Q must be a 4x4 matrix")
            self.Q = Q

        if R is not None:
            if R.shape != (2, 2):
                raise ValueError("R must be a 2x2 matrix")
            self.R = R

        if Q is not None or R is not None:
            logger.info("MPC weights updated. Reinitializing optimizer...")
            self._setup_optimizer()

    def get_prediction(self, sol) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract the full predicted trajectory from the solution.

        Args:
            sol: CasADi solution object

        Returns:
            Tuple of (predicted_states, predicted_controls)
        """
        predicted_states = sol.value(self.X)
        predicted_controls = sol.value(self.U)
        return predicted_states, predicted_controls


# Legacy class name for backward compatibility
MPC_Controller = MPCController
