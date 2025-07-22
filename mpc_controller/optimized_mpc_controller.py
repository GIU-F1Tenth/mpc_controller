import casadi as ca
import numpy as np
from enum import Enum
import time


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

        # Tire slip angles
        alpha_f = delta - ca.atan((v * ca.sin(beta) + self.lf * r) / (v * ca.cos(beta)))
        alpha_r = -ca.atan((v * ca.sin(beta) - self.lr * r) / (v * ca.cos(beta)))

        # Tire forces
        Fyf = self.Cf * alpha_f
        Fyr = self.Cr * alpha_r

        # State derivatives
        xdot = v * ca.cos(theta + beta)
        ydot = v * ca.sin(theta + beta)
        vdot = a
        thetadot = r
        betadot = (Fyf * ca.cos(delta) + Fyr) / (self.m * v) - r
        rdot = (self.lf * Fyf * ca.cos(delta) - self.lr * Fyr) / self.Iz

        state_next = ca.vertcat(x, y, v, theta, beta, r) + self.dt * ca.vertcat(
            xdot, ydot, vdot, thetadot, betadot, rdot
        )

        return ca.Function('dynamics',
                           [x, y, v, theta, beta, r, a, delta],
                           [state_next],
                           ['state', 'control'],
                           ['state_next'])


class CostFunction:
    """Racing-optimized cost function using all parameters"""

    def __init__(self, mpc_type=MPCType.KINEMATIC,
                 enable_cost_function_weights=True,
                 cost_function_weights=None,
                 enable_speed_control=True,
                 speed_control_weight=0.3,
                 enable_trajectory_tracking=True,
                 trajectory_tracking_weight=0.2,
                 obstacle_avoidance_weight=0.5):

        self.mpc_type = mpc_type
        self.enable_cost_function_weights = enable_cost_function_weights
        self.enable_speed_control = enable_speed_control
        self.speed_control_weight = speed_control_weight
        self.enable_trajectory_tracking = enable_trajectory_tracking
        self.trajectory_tracking_weight = trajectory_tracking_weight
        self.obstacle_avoidance_weight = obstacle_avoidance_weight

        # Default weights
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

        # Set up cost matrices based on model type
        if mpc_type == MPCType.KINEMATIC:
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
        else:  # DYNAMIC
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


class ConstraintsManager:
    """Manage vehicle and racing constraints using all parameters"""

    def __init__(self, mpc_type=MPCType.KINEMATIC,
                 max_steering_angle=0.5,
                 max_acceleration=1.0,
                 max_deceleration=1.0,
                 min_speed=0.1,
                 max_speed=2.0,
                 enable_hard_constraints=True,
                 hard_constraints=None,
                 enable_safety_checks=True,
                 safety_check_distance=0.5):

        self.mpc_type = mpc_type
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

        # Model-specific constraints
        if mpc_type == MPCType.DYNAMIC:
            self.max_slip_angle = 0.2
            self.max_yaw_rate = 2.0

    def apply_constraints(self, opti, U, X, N):
        """Apply all constraints to the optimization problem"""

        # Control input constraints
        opti.subject_to(opti.bounded(self.max_deceleration, U[0, :], self.max_acceleration))
        opti.subject_to(opti.bounded(-self.max_steering_angle, U[1, :], self.max_steering_angle))

        # Speed constraints
        opti.subject_to(opti.bounded(self.min_speed, X[2, :], self.max_speed))

        # Steering rate constraints (if hard constraints enabled)
        if self.enable_hard_constraints:
            for i in range(N - 1):
                steering_rate = (U[1, i + 1] - U[1, i]) / 0.05  # Assuming dt = 0.05
                opti.subject_to(opti.bounded(-self.max_steering_rate, steering_rate, self.max_steering_rate))

        # Model-specific constraints
        if self.mpc_type == MPCType.DYNAMIC:
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


class SolverConfiguration:
    """Solver configuration for optimal performance"""

    @staticmethod
    def get_solver_options(solver_type='ipopt'):
        """Get optimized solver options for racing MPC"""

        if solver_type == 'ipopt':
            return {
                'ipopt.print_level': 0,
                'ipopt.max_iter': 150,
                'ipopt.tol': 1e-4,
                'ipopt.acceptable_tol': 1e-3,
                'ipopt.acceptable_iter': 10,
                'ipopt.warm_start_init_point': 'yes',
                'ipopt.warm_start_bound_push': 1e-6,
                'ipopt.warm_start_mult_bound_push': 1e-6,
                'print_time': False
            }
        elif solver_type == 'sqpmethod':
            return {
                'qpsol': 'qrqp',
                'print_header': False,
                'print_iteration': False,
                'max_iter': 100
            }
        else:
            return {}


class OptimizedMPCController:
    """Optimized Model Predictive Controller for F1TENTH Racing using all parameters"""

    def __init__(self,
                 N=10,
                 T=1.0,
                 wheelbase=0.33,
                 mpc_type=MPCType.KINEMATIC,
                 solver_type='ipopt',
                 enable_logging=True,
                 # All parameters from params.yaml
                 max_steering_angle=0.5,
                 max_acceleration=1.0,
                 max_deceleration=1.0,
                 min_speed=0.1,
                 max_speed=2.0,
                 enable_cost_function_weights=True,
                 cost_function_weights=None,
                 enable_hard_constraints=True,
                 hard_constraints=None,
                 enable_obstacle_avoidance=False,
                 obstacle_avoidance_weight=0.5,
                 enable_speed_control=True,
                 speed_control_weight=0.3,
                 enable_trajectory_tracking=True,
                 trajectory_tracking_weight=0.2,
                 enable_safety_checks=True,
                 safety_check_distance=0.5,
                 lookahead_distance=0.7):
        """
        Initialize Optimized MPC Controller with all parameters

        Parameters from params.yaml are directly used here
        """

        self.N = N
        self.T = T
        self.dt = T / N
        self.wheelbase = wheelbase
        self.mpc_type = mpc_type
        self.solver_type = solver_type
        self.enable_logging = enable_logging
        self.lookahead_distance = lookahead_distance

        # Store all parameters
        self.max_steering_angle = max_steering_angle
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.enable_obstacle_avoidance = enable_obstacle_avoidance
        self.enable_safety_checks = enable_safety_checks

        # Performance tracking
        self.solve_times = []
        self.solve_success_rate = []
        self.iteration_count = 0

        # Warm start storage
        self.previous_solution_U = None
        self.previous_solution_X = None
        self.previous_control = None

        # Initialize cost function with all parameters
        self.cost_function = CostFunction(
            mpc_type=mpc_type,
            enable_cost_function_weights=enable_cost_function_weights,
            cost_function_weights=cost_function_weights,
            enable_speed_control=enable_speed_control,
            speed_control_weight=speed_control_weight,
            enable_trajectory_tracking=enable_trajectory_tracking,
            trajectory_tracking_weight=trajectory_tracking_weight,
            obstacle_avoidance_weight=obstacle_avoidance_weight
        )

        # Initialize constraints manager with all parameters
        self.constraints_manager = ConstraintsManager(
            mpc_type=mpc_type,
            max_steering_angle=max_steering_angle,
            max_acceleration=max_acceleration,
            max_deceleration=max_deceleration,
            min_speed=min_speed,
            max_speed=max_speed,
            enable_hard_constraints=enable_hard_constraints,
            hard_constraints=hard_constraints,
            enable_safety_checks=enable_safety_checks,
            safety_check_distance=safety_check_distance
        )

        self._setup_mpc_problem()

        if self.enable_logging:
            print(f"✅ Optimized MPC Controller initialized:")
            print(f"   - Type: {mpc_type.value}")
            print(f"   - Horizon: {N} steps, {T}s total")
            print(f"   - Solver: {solver_type}")
            print(f"   - Speed limits: {min_speed}-{max_speed} m/s")
            print(f"   - Max steering: {max_steering_angle:.2f} rad")
            print(f"   - Safety checks: {enable_safety_checks}")
            print(f"   - Obstacle avoidance: {enable_obstacle_avoidance}")

    def _setup_mpc_problem(self):
        """Setup the complete MPC optimization problem"""

        # Initialize vehicle model
        if self.mpc_type == MPCType.KINEMATIC:
            self.vehicle_model = KinematicBicycleModel(self.wheelbase, self.dt)
            self.n_states = 4
        else:  # DYNAMIC
            self.vehicle_model = DynamicBicycleModel(self.wheelbase, self.dt)
            self.n_states = 6

        self.dynamics = self.vehicle_model.get_dynamics()

        # Setup optimization problem
        self._setup_optimization_problem()

    def _setup_optimization_problem(self):
        """Setup CasADi optimization problem"""

        self.opti = ca.Opti()

        # Decision variables
        self.U = self.opti.variable(2, self.N)                    # Control inputs
        self.X = self.opti.variable(self.n_states, self.N + 1)    # States

        # Parameters
        self.X_ref = self.opti.parameter(self.n_states, self.N + 1)  # Reference trajectory
        self.X0 = self.opti.parameter(self.n_states, 1)              # Initial state

        # Objective function
        cost = 0

        # Stage costs with jerk penalty
        for i in range(self.N):
            state_error = self.X[:, i] - self.X_ref[:, i]
            control_input = self.U[:, i]

            # Get previous control for jerk penalty
            prev_control = self.U[:, i - 1] if i > 0 else None

            stage_cost = self.cost_function.compute_stage_cost(
                state_error, control_input, prev_control
            )
            cost += stage_cost

            # Add speed control cost if enabled
            if self.cost_function.enable_speed_control:
                speed_cost = self.cost_function.compute_speed_control_cost(
                    self.X[2, i], self.X_ref[2, i]
                )
                cost += speed_cost

            # Add trajectory tracking cost if enabled
            if self.cost_function.enable_trajectory_tracking:
                position_error = self.X[0:2, i] - self.X_ref[0:2, i]
                tracking_cost = self.cost_function.compute_trajectory_tracking_cost(
                    position_error
                )
                cost += tracking_cost

        # Terminal cost
        terminal_error = self.X[:, -1] - self.X_ref[:, -1]
        cost += self.cost_function.compute_terminal_cost(terminal_error)

        self.opti.minimize(cost)

        # Initial condition constraint
        self.opti.subject_to(self.X[:, 0] == self.X0)

        # Dynamics constraints
        for i in range(self.N):
            if self.mpc_type == MPCType.KINEMATIC:
                x_next = self.dynamics(self.X[:, i], self.U[:, i])
            else:  # DYNAMIC
                x_next = self.dynamics(self.X[:, i], self.U[:, i])
            self.opti.subject_to(self.X[:, i + 1] == x_next)

        # Apply constraints
        self.constraints_manager.apply_constraints(self.opti, self.U, self.X, self.N)

        # Solver configuration
        solver_options = SolverConfiguration.get_solver_options(self.solver_type)
        self.opti.solver(self.solver_type, solver_options)

    def solve_mpc(self, current_state, reference_trajectory):
        """
        Solve MPC optimization problem with all parameter features

        Parameters:
            current_state: dict with keys based on model type
            reference_trajectory: numpy array of reference states

        Returns:
            dict with solution and metadata
        """

        start_time = time.time()
        self.iteration_count += 1

        try:
            # Prepare state vector based on model type
            if self.mpc_type == MPCType.KINEMATIC:
                state_vector = np.array([
                    current_state['x'],
                    current_state['y'],
                    current_state['v'],
                    current_state['theta']
                ])
            else:  # DYNAMIC
                state_vector = np.array([
                    current_state['x'],
                    current_state['y'],
                    current_state['v'],
                    current_state['theta'],
                    current_state.get('beta', 0.0),
                    current_state.get('r', 0.0)
                ])

            # Set parameters
            self.opti.set_value(self.X0, state_vector.reshape(-1, 1))
            self.opti.set_value(self.X_ref, reference_trajectory.T)

            # Warm start if available
            if self.previous_solution_U is not None:
                # Shift previous solution and add last control input
                warm_start_U = np.roll(self.previous_solution_U, -1, axis=1)
                warm_start_U[:, -1] = warm_start_U[:, -2]  # Repeat last control
                self.opti.set_initial(self.U, warm_start_U)

            if self.previous_solution_X is not None:
                warm_start_X = np.roll(self.previous_solution_X, -1, axis=1)
                warm_start_X[:, -1] = warm_start_X[:, -2]  # Repeat last state
                self.opti.set_initial(self.X, warm_start_X)

            # Solve optimization
            sol = self.opti.solve()

            # Extract solution
            optimal_U = sol.value(self.U)
            optimal_X = sol.value(self.X)

            # Store for warm start and jerk calculation
            self.previous_solution_U = optimal_U
            self.previous_solution_X = optimal_X
            self.previous_control = optimal_U[:, 0]

            # Calculate solve time
            solve_time = time.time() - start_time

            # Track performance
            self.solve_times.append(solve_time)
            self.solve_success_rate.append(1.0)

            result = {
                'acceleration': float(optimal_U[0, 0]),
                'steering': float(optimal_U[1, 0]),
                'success': True,
                'solve_time': solve_time,
                'predicted_states': optimal_X,
                'control_sequence': optimal_U
            }

            if self.enable_logging and self.iteration_count % 50 == 0:
                print(f"MPC solve #{self.iteration_count}: {solve_time:.4f}s, "
                      f"a={result['acceleration']:.3f}, δ={result['steering']:.3f}")

            return result

        except Exception as e:
            solve_time = time.time() - start_time

            # Track failed solve
            self.solve_times.append(solve_time)
            self.solve_success_rate.append(0.0)

            if self.enable_logging:
                print(f"❌ MPC solve failed: {e}")

            return {
                'acceleration': 0.0,
                'steering': 0.0,
                'success': False,
                'solve_time': solve_time,
                'error': str(e)
            }

    def get_performance_stats(self):
        """Get comprehensive performance statistics"""
        if not self.solve_times:
            return {}

        recent_times = self.solve_times[-100:] if len(self.solve_times) > 100 else self.solve_times
        recent_success = self.solve_success_rate[-100:] if len(
            self.solve_success_rate) > 100 else self.solve_success_rate

        return {
            'avg_solve_time': np.mean(recent_times),
            'max_solve_time': np.max(recent_times),
            'min_solve_time': np.min(recent_times),
            'std_solve_time': np.std(recent_times),
            'success_rate': np.mean(recent_success),
            'total_iterations': self.iteration_count,
            'real_time_factor': self.dt / np.mean(recent_times) if recent_times else 0
        }

    def reset_performance_tracking(self):
        """Reset performance tracking metrics"""
        self.solve_times.clear()
        self.solve_success_rate.clear()
        self.iteration_count = 0
        self.previous_solution_U = None
        self.previous_solution_X = None
        self.previous_control = None

    def update_parameters(self, **kwargs):
        """Update MPC parameters dynamically"""
        if 'max_speed' in kwargs:
            self.constraints_manager.max_speed = kwargs['max_speed']
            self.max_speed = kwargs['max_speed']
        if 'max_steering_angle' in kwargs:
            self.constraints_manager.max_steering_angle = kwargs['max_steering_angle']
            self.max_steering_angle = kwargs['max_steering_angle']
        if 'enable_safety_checks' in kwargs:
            self.constraints_manager.enable_safety_checks = kwargs['enable_safety_checks']
            self.enable_safety_checks = kwargs['enable_safety_checks']
        # Add more parameter updates as needed
