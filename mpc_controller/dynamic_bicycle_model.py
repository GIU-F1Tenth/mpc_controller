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


class DynamicBicycleModel(VehicleModel):
    """Dynamic bicycle model with tire dynamics for precise racing"""

    def __init__(self, wheelbase=0.33, dt=0.05, mass=3.17, Iz=0.04712, Cf=4.718, Cr=5.4562):
        super().__init__(wheelbase, dt)
        self.m = mass
        self.Iz = Iz
        self.Cf = Cf  # Front cornering stiffness
        self.Cr = Cr  # Rear cornering stiffness
        self.lf = self.L * 0.5  # Distance to front axle
        self.lr = self.L * 0.5  # Distance to rear axle

    def get_dynamics(self):
        """Get dynamic bicycle model dynamics with tire forces"""
        # State variables: [x, y, v, theta, beta, r]
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        v = ca.SX.sym('v')
        theta = ca.SX.sym('theta')
        beta = ca.SX.sym('beta')    # Sideslip angle
        r = ca.SX.sym('r')          # Yaw rate

        # Control inputs: [acceleration, steering_angle]
        a = ca.SX.sym('a')
        delta = ca.SX.sym('delta')

        # Tire slip angles with safeguard against division by zero
        v_safe = ca.fmax(v, 0.1)  # Ensure minimum velocity
        alpha_f = delta - ca.atan((v_safe * ca.sin(beta) + self.lf * r) / (v_safe * ca.cos(beta) + 1e-6))
        alpha_r = -ca.atan((v_safe * ca.sin(beta) - self.lr * r) / (v_safe * ca.cos(beta) + 1e-6))

        # Tire forces
        Fyf = self.Cf * alpha_f
        Fyr = self.Cr * alpha_r

        # State derivatives
        xdot = v_safe * ca.cos(theta + beta)
        ydot = v_safe * ca.sin(theta + beta)
        vdot = a
        thetadot = r
        betadot = (Fyf * ca.cos(delta) + Fyr) / (self.m * v_safe) - r
        rdot = (self.lf * Fyf * ca.cos(delta) - self.lr * Fyr) / self.Iz

        state_next = ca.vertcat(x, y, v, theta, beta, r) + self.dt * ca.vertcat(
            xdot, ydot, vdot, thetadot, betadot, rdot
        )

        return ca.Function('dynamics',
                           [x, y, v, theta, beta, r, a, delta],
                           [state_next])


class DynamicCostFunction:
    """Racing-optimized cost function for Dynamic model"""

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

        # Default weights for Dynamic model
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

        # Set up cost matrices for Dynamic model (6 states)
        self.Q = np.diag([
            self.weights['position_weight'],      # x
            self.weights['position_weight'],      # y
            self.weights['velocity_weight'],      # v
            self.weights['heading_weight'],       # theta
            self.weights['velocity_weight'] * 0.5,  # beta
            self.weights['heading_weight'] * 0.3    # r
        ])
        self.R = np.diag([
            self.weights['acceleration_weight'] * 0.7,  # More aggressive
            self.weights['steering_weight'] * 0.8
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


class DynamicConstraintsManager:
    """Manage vehicle and racing constraints for Dynamic model"""

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

        # Dynamic model specific constraints
        self.max_slip_angle = 0.2
        self.max_yaw_rate = 2.0

    def apply_constraints(self, opti, U, X, N):
        """Apply all constraints to the optimization problem for Dynamic model"""

        # Control input constraints
        opti.subject_to(opti.bounded(self.max_deceleration, U[0, :], self.max_acceleration))
        opti.subject_to(opti.bounded(-self.max_steering_angle, U[1, :], self.max_steering_angle))

        # Speed constraints (X[2] is velocity for dynamic model)
        opti.subject_to(opti.bounded(self.min_speed, X[2, :], self.max_speed))

        # Steering rate constraints (if hard constraints enabled)
        if self.enable_hard_constraints:
            for i in range(N - 1):
                steering_rate = (U[1, i + 1] - U[1, i]) / 0.05  # Assuming dt = 0.05
                opti.subject_to(opti.bounded(-self.max_steering_rate, steering_rate, self.max_steering_rate))

        # Dynamic model-specific constraints
        opti.subject_to(opti.bounded(-self.max_slip_angle, X[4, :], self.max_slip_angle))  # Beta
        opti.subject_to(opti.bounded(-self.max_yaw_rate, X[5, :], self.max_yaw_rate))      # Yaw rate

        # Safety constraints (simplified implementation)
        if self.enable_safety_checks:
            # Add minimum distance between consecutive points
            for i in range(N):
                # Ensure minimum forward progress
                if i > 0:
                    position_diff = ca.sqrt((X[0, i] - X[0, i - 1])**2 + (X[1, i] - X[1, i - 1])**2)
                    opti.subject_to(position_diff >= 0.01)  # Minimum movement
