import casadi as ca
import numpy as np
from enum import Enum


class MPCType(Enum):
    KINEMATIC = "kinematic"
    DYNAMIC = "dynamic"


class VehicleModel:
    """Base vehicle dynamics model"""

    def __init__(self, wheelbase=0.33, dt=0.05):
        self.L = wheelbase
        self.dt = dt

    def get_dynamics(self):
        """Override in subclasses"""
        raise NotImplementedError


class KinematicBicycleModel(VehicleModel):
    """Kinematic bicycle model for high-speed racing"""

    def __init__(self, wheelbase=0.33, dt=0.05):
        super().__init__(wheelbase, dt)

    def get_dynamics(self):
        """Get kinematic bicycle model dynamics"""
        # State variables: [x, y, v, theta]
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        v = ca.SX.sym('v')
        theta = ca.SX.sym('theta')

        # Control inputs: [acceleration, steering_angle]
        a = ca.SX.sym('a')
        delta = ca.SX.sym('delta')

        # Kinematic bicycle model with slip angle
        beta = ca.atan(0.5 * ca.tan(delta))  # Slip angle at center of mass

        # State derivatives
        xdot = v * ca.cos(theta + beta)
        ydot = v * ca.sin(theta + beta)
        vdot = a
        thetadot = v * ca.sin(beta) / (self.L / 2)

        # Discrete-time model using RK4 integration
        k1 = ca.vertcat(xdot, ydot, vdot, thetadot)
        k2 = ca.vertcat(
            (v + 0.5 * self.dt * a) * ca.cos(theta + 0.5 * self.dt * thetadot + beta),
            (v + 0.5 * self.dt * a) * ca.sin(theta + 0.5 * self.dt * thetadot + beta),
            a,
            (v + 0.5 * self.dt * a) * ca.sin(beta) / (self.L / 2)
        )
        k3 = k2  # Simplified for bicycle model
        k4 = ca.vertcat(
            (v + self.dt * a) * ca.cos(theta + self.dt * thetadot + beta),
            (v + self.dt * a) * ca.sin(theta + self.dt * thetadot + beta),
            a,
            (v + self.dt * a) * ca.sin(beta) / (self.L / 2)
        )

        state_next = ca.vertcat(x, y, v, theta) + self.dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        return ca.Function('dynamics',
                           [x, y, v, theta, a, delta],
                           [state_next],
                           ['state', 'control'],
                           ['state_next'])


class KinematicCostFunction:
    """Racing-optimized cost function for Kinematic model"""

    def __init__(self,
                 enable_cost_function_weights=True,
                 cost_function_weights=None,
                 enable_speed_control=True,
                 speed_control_weight=0.3,
                 enable_trajectory_tracking=True,
                 trajectory_tracking_weight=0.2,
                 obstacle_avoidance_weight=0.5):

        self.enable_cost_function_weights = enable_cost_function_weights
        self.enable_speed_control = enable_speed_control
        self.speed_control_weight = speed_control_weight
        self.enable_trajectory_tracking = enable_trajectory_tracking
        self.trajectory_tracking_weight = trajectory_tracking_weight
        self.obstacle_avoidance_weight = obstacle_avoidance_weight

        # Default weights for Kinematic model
        default_weights = {
            'position_weight': 10.0,
            'heading_weight': 5.0,
            'velocity_weight': 1.0,
            'steering_weight': 1.0,
            'acceleration_weight': 0.5,
            'jerk_weight': 0.1
        }

        # Use provided weights or defaults
        if cost_function_weights and enable_cost_function_weights:
            self.weights = {**default_weights, **cost_function_weights}
        else:
            self.weights = default_weights

        # Set up cost matrices for Kinematic model
        self.Q = np.diag([
            self.weights['position_weight'],      # x
            self.weights['position_weight'],      # y
            self.weights['velocity_weight'],      # v
            self.weights['heading_weight']        # theta
        ])
        self.R = np.diag([
            self.weights['acceleration_weight'],  # acceleration
            self.weights['steering_weight']       # steering
        ])

        # Terminal cost with higher weights
        self.Q_terminal = self.Q * 2.0

    def compute_stage_cost(self, state_error, control_input, prev_control=None):
        """Compute stage cost with all enabled features"""
        # Base tracking cost
        cost = ca.mtimes([state_error.T, self.Q, state_error]) + \
            ca.mtimes([control_input.T, self.R, control_input])

        # Add jerk penalty if previous control is available
        if prev_control is not None and self.weights['jerk_weight'] > 0:
            control_diff = control_input - prev_control
            jerk_weight_matrix = np.diag([
                self.weights['jerk_weight'],  # acceleration jerk
                self.weights['jerk_weight']   # steering jerk
            ])
            cost += ca.mtimes([control_diff.T, jerk_weight_matrix, control_diff])

        return cost

    def compute_terminal_cost(self, terminal_error):
        """Compute terminal cost with higher weight"""
        return ca.mtimes([terminal_error.T, self.Q_terminal, terminal_error])

    def compute_speed_control_cost(self, current_velocity, reference_velocity):
        """Compute speed control cost"""
        if not self.enable_speed_control:
            return 0.0

        speed_error = current_velocity - reference_velocity
        return self.speed_control_weight * speed_error**2

    def compute_trajectory_tracking_cost(self, position_error):
        """Compute trajectory tracking cost"""
        if not self.enable_trajectory_tracking:
            return 0.0

        return self.trajectory_tracking_weight * ca.sumsqr(position_error)


class KinematicConstraintsManager:
    """Manage vehicle and racing constraints for Kinematic model"""

    def __init__(self,
                 max_steering_angle=0.5,
                 max_acceleration=1.0,
                 max_deceleration=1.0,
                 min_speed=0.1,
                 max_speed=2.0,
                 enable_hard_constraints=True,
                 hard_constraints=None,
                 enable_safety_checks=True,
                 safety_check_distance=0.5):

        self.enable_hard_constraints = enable_hard_constraints
        self.enable_safety_checks = enable_safety_checks
        self.safety_check_distance = safety_check_distance

        # Vehicle constraints from parameters
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.max_acceleration = max_acceleration
        self.max_deceleration = -abs(max_deceleration)  # Ensure negative
        self.max_steering_angle = max_steering_angle
        self.max_steering_rate = 3.0  # Default value

        # Override with hard constraints if provided
        if hard_constraints and enable_hard_constraints:
            self.max_steering_angle = hard_constraints.get('max_steering_angle', self.max_steering_angle)
            self.max_acceleration = hard_constraints.get('max_acceleration', self.max_acceleration)
            self.max_deceleration = -abs(hard_constraints.get('max_deceleration', abs(self.max_deceleration)))

    def apply_constraints(self, opti, U, X, N):
        """Apply all constraints to the optimization problem for Kinematic model"""

        # Control input constraints
        opti.subject_to(opti.bounded(self.max_deceleration, U[0, :], self.max_acceleration))
        opti.subject_to(opti.bounded(-self.max_steering_angle, U[1, :], self.max_steering_angle))

        # Speed constraints (X[2] is velocity for kinematic model)
        opti.subject_to(opti.bounded(self.min_speed, X[2, :], self.max_speed))

        # Steering rate constraints (if hard constraints enabled)
        if self.enable_hard_constraints:
            for i in range(N - 1):
                steering_rate = (U[1, i + 1] - U[1, i]) / 0.05  # Assuming dt = 0.05
                opti.subject_to(opti.bounded(-self.max_steering_rate, steering_rate, self.max_steering_rate))

        # Safety constraints (simplified implementation)
        if self.enable_safety_checks:
            # Add minimum distance between consecutive points
            for i in range(N):
                # Ensure minimum forward progress
                if i > 0:
                    position_diff = ca.sqrt((X[0, i] - X[0, i - 1])**2 + (X[1, i] - X[1, i - 1])**2)
                    opti.subject_to(position_diff >= 0.01)  # Minimum movement
