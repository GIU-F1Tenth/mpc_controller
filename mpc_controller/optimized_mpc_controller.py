import casadi as ca
import numpy as np
import time

# Import separated model classes
try:
    # Try relative import first (for package use)
    from .kinematic_bicycle_model import (
        MPCType, VehicleModel, KinematicBicycleModel,
        KinematicCostFunction, KinematicConstraintsManager
    )
    from .dynamic_bicycle_model import (
        DynamicBicycleModel, DynamicCostFunction, DynamicConstraintsManager
    )
except ImportError:
    # Fall back to absolute import (for standalone use)
    from kinematic_bicycle_model import (
        MPCType, VehicleModel, KinematicBicycleModel,
        KinematicCostFunction, KinematicConstraintsManager
    )
    from dynamic_bicycle_model import (
        DynamicBicycleModel, DynamicCostFunction, DynamicConstraintsManager
    )


class SolverConfiguration:
    """Solver configuration for optimal performance"""

    @staticmethod
    def get_solver_options(solver_type='ipopt'):
        """Get optimized solver options for racing MPC with maximum numerical stability"""

        if solver_type == 'ipopt':
            # Minimalist, validated IPOPT configuration
            return {
                'ipopt.print_level': 0,
                'ipopt.max_iter': 100,
                'ipopt.tol': 1e-3,
                'ipopt.acceptable_tol': 1e-2,
                'ipopt.acceptable_iter': 5,
                'ipopt.linear_solver': 'mumps',  # Most reliable solver
                'ipopt.mu_init': 1e-1,
                'ipopt.nlp_scaling_method': 'gradient-based',
                'ipopt.bound_push': 1e-2,
                'ipopt.bound_frac': 1e-2,
                'print_time': False,
                'verbose': False
            }
        elif solver_type == 'sqpmethod':
            return {
                'qpsol': 'qrqp',
                'print_header': False,
                'print_iteration': False,
                'max_iter': 20,          # Very conservative
                'tol_pr': 1e-2,          # Relaxed primal feasibility
                'tol_du': 1e-2,          # Relaxed dual feasibility
                'regularize': True,       # Enable regularization
                'reg_threshold': 1e-6,    # More aggressive regularization
                'beta': 0.8,             # Line search parameter
                'merit_memory': 4        # Merit function memory
            }
        else:
            return {}


class OptimizedMPCController:
    """
    Optimized Model Predictive Controller for F1TENTH Racing using all parameters
    
    This controller computes optimal steering angles and acceleration commands
    based on position and velocity references from the trajectory publisher.
    The trajectory publisher only provides (x, y, v, theta) references - 
    steering angles are computed by this MPC controller, not pre-calculated.
    """

    def __init__(self,
                 N=10,
                 T=1.0,
                 wheelbase=0.33,
                 mpc_type=MPCType.KINEMATIC,
                 solver_type='ipopt',
                 enable_logging=True,
                 logger=None,  # ROS2 logger for proper logging
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
        self.logger = logger  # ROS2 logger for proper logging
        self.lookahead_distance = lookahead_distance

        # Store all parameters
        self.max_steering_angle = max_steering_angle
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.enable_obstacle_avoidance = enable_obstacle_avoidance
        self.enable_safety_checks = enable_safety_checks

        # Performance tracking and failure handling
        self.solve_times = []
        self.solve_success_rate = []
        self.iteration_count = 0
        self.consecutive_failures = 0  # Track consecutive failures

        # Warm start storage
        self.previous_solution_U = None
        self.previous_solution_X = None
        self.previous_control = None

        # Initialize cost function with all parameters based on model type
        if mpc_type == MPCType.KINEMATIC:
            self.cost_function = KinematicCostFunction(
                enable_cost_function_weights=enable_cost_function_weights,
                cost_function_weights=cost_function_weights,
                enable_speed_control=enable_speed_control,
                speed_control_weight=speed_control_weight,
                enable_trajectory_tracking=enable_trajectory_tracking,
                trajectory_tracking_weight=trajectory_tracking_weight,
                obstacle_avoidance_weight=obstacle_avoidance_weight
            )
        else:  # DYNAMIC
            self.cost_function = DynamicCostFunction(
                enable_cost_function_weights=enable_cost_function_weights,
                cost_function_weights=cost_function_weights,
                enable_speed_control=enable_speed_control,
                speed_control_weight=speed_control_weight,
                enable_trajectory_tracking=enable_trajectory_tracking,
                trajectory_tracking_weight=trajectory_tracking_weight,
                obstacle_avoidance_weight=obstacle_avoidance_weight
            )

        # Initialize constraints manager with all parameters based on model type
        if mpc_type == MPCType.KINEMATIC:
            self.constraints_manager = KinematicConstraintsManager(
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
        else:  # DYNAMIC
            self.constraints_manager = DynamicConstraintsManager(
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

        # Initialize logging with proper output
        if self.enable_logging:
            self._log_initialization(mpc_type, N, T, solver_type, min_speed, max_speed, 
                                   max_steering_angle, enable_safety_checks, enable_obstacle_avoidance)

    def _log(self, level, message):
        """Helper method for consistent logging"""
        if self.logger:
            if level == 'info':
                self.logger.info(message)
            elif level == 'warn':
                self.logger.warn(message)
            elif level == 'error':
                self.logger.error(message)
            elif level == 'debug':
                self.logger.debug(message)
        elif self.enable_logging:
            print(f"[{level.upper()}] {message}")

    def _log_initialization(self, mpc_type, N, T, solver_type, min_speed, max_speed, 
                          max_steering_angle, enable_safety_checks, enable_obstacle_avoidance):
        """Log initialization information"""
        self._log('info', f"[MPC] ✅ Optimized MPC Controller initialized:")
        self._log('info', f"[MPC]    - Type: {mpc_type.value}")
        self._log('info', f"[MPC]    - Horizon: {N} steps, {T}s total")
        self._log('info', f"[MPC]    - Solver: {solver_type}")
        self._log('info', f"[MPC]    - Speed limits: {min_speed}-{max_speed} m/s")
        self._log('info', f"[MPC]    - Max steering: {max_steering_angle:.2f} rad")
        self._log('info', f"[MPC]    - Safety checks: {enable_safety_checks}")
        self._log('info', f"[MPC]    - Obstacle avoidance: {enable_obstacle_avoidance}")

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
                x_next = self.dynamics(self.X[0, i], self.X[1, i], self.X[2, i], self.X[3, i],
                                       self.U[0, i], self.U[1, i])
            else:  # DYNAMIC
                x_next = self.dynamics(self.X[0, i], self.X[1, i], self.X[2, i], self.X[3, i],
                                       self.X[4, i], self.X[5, i], self.U[0, i], self.U[1, i])
            self.opti.subject_to(self.X[:, i + 1] == x_next)

        # Apply constraints
        self.constraints_manager.apply_constraints(self.opti, self.U, self.X, self.N)

        # Solver configuration
        solver_options = SolverConfiguration.get_solver_options(self.solver_type)
        self.opti.solver(self.solver_type, solver_options)

    def solve_mpc(self, current_state, reference_trajectory):
        """
        Solve MPC optimization problem with robust numerical validation

        Parameters:
            current_state: dict with keys based on model type
            reference_trajectory: numpy array of reference states

        Returns:
            dict with solution and metadata
        """

        start_time = time.time()
        self.iteration_count += 1

        try:
            # Validate inputs first
            if not self._validate_inputs(current_state, reference_trajectory):
                return {
                    'acceleration': 0.0,
                    'steering': 0.0,
                    'success': False,
                    'solve_time': time.time() - start_time,
                    'error': 'Input validation failed'
                }

            # Prepare state vector based on model type
            if self.mpc_type == MPCType.KINEMATIC:
                state_vector = np.array([
                    float(current_state['x']),
                    float(current_state['y']),
                    float(max(0.01, current_state['v'])),  # Ensure minimum positive velocity
                    float(current_state['theta'])
                ])
            else:  # DYNAMIC
                state_vector = np.array([
                    float(current_state['x']),
                    float(current_state['y']),
                    float(max(0.01, current_state['v'])),  # Ensure minimum positive velocity
                    float(current_state['theta']),
                    float(current_state.get('beta', 0.0)),
                    float(current_state.get('r', 0.0))
                ])

            # Validate state vector
            if not np.all(np.isfinite(state_vector)):
                return {
                    'acceleration': 0.0,
                    'steering': 0.0,
                    'success': False,
                    'solve_time': time.time() - start_time,
                    'error': 'State vector contains NaN/Inf values'
                }

            # Set parameters
            self.opti.set_value(self.X0, state_vector.reshape(-1, 1))
            self.opti.set_value(self.X_ref, reference_trajectory.T)

            # Warm start if available and not too many recent failures
            if (self.previous_solution_U is not None and
                self.previous_solution_X is not None and
                    self.consecutive_failures < 3):  # Reset warm start after 3 consecutive failures
                # Shift previous solution and add last control input
                warm_start_U = np.roll(self.previous_solution_U, -1, axis=1)
                warm_start_U[:, -1] = warm_start_U[:, -2]  # Repeat last control
                self.opti.set_initial(self.U, warm_start_U)

                warm_start_X = np.roll(self.previous_solution_X, -1, axis=1)
                warm_start_X[:, -1] = warm_start_X[:, -2]  # Repeat last state
                self.opti.set_initial(self.X, warm_start_X)
            elif self.consecutive_failures >= 3:
                # Reset warm start after consecutive failures
                if self.enable_logging:
                    self._log('warn', f"[MPC] ⚠️ Resetting warm start after {self.consecutive_failures} consecutive failures")
                self.previous_solution_U = None
                self.previous_solution_X = None

            # Solve optimization with debug capability
            try:
                sol = self.opti.solve()
            except Exception as solve_error:
                # Try to get debug information if available
                if self.enable_logging:
                    self._log('error', f"[MPC] 🐛 Solver failed, attempting debug...")
                    try:
                        # Get debug values to understand what went wrong
                        debug_U = self.opti.debug.value(self.U)
                        debug_X = self.opti.debug.value(self.X)
                        debug_X0 = self.opti.debug.value(self.X0)
                        debug_X_ref = self.opti.debug.value(self.X_ref)

                        print(f"  Debug U shape: {debug_U.shape if debug_U is not None else 'None'}")
                        print(f"  Debug X shape: {debug_X.shape if debug_X is not None else 'None'}")
                        print(f"  Debug X0 finite: {np.all(np.isfinite(debug_X0)) if debug_X0 is not None else 'None'}")
                        print(
                            f"  Debug X_ref finite: {np.all(np.isfinite(debug_X_ref)) if debug_X_ref is not None else 'None'}")

                        # Check for specific problematic values
                        if debug_X_ref is not None:
                            print(f"  X_ref range: X=[{np.min(debug_X_ref[0,:]):.3f}, {np.max(debug_X_ref[0,:]):.3f}], "
                                  f"Y=[{np.min(debug_X_ref[1,:]):.3f}, {np.max(debug_X_ref[1,:]):.3f}], "
                                  f"V=[{np.min(debug_X_ref[2,:]):.3f}, {np.max(debug_X_ref[2,:]):.3f}]")
                    except BaseException:
                        print(f"  Could not extract debug information")

                raise solve_error

            # Extract solution
            optimal_U = sol.value(self.U)
            optimal_X = sol.value(self.X)

            # Validate solution
            if not (np.all(np.isfinite(optimal_U)) and np.all(np.isfinite(optimal_X))):
                return {
                    'acceleration': 0.0,
                    'steering': 0.0,
                    'success': False,
                    'solve_time': time.time() - start_time,
                    'error': 'Solution contains NaN/Inf values'
                }

            # Store for warm start and jerk calculation
            self.previous_solution_U = optimal_U
            self.previous_solution_X = optimal_X
            self.previous_control = optimal_U[:, 0]

            # Calculate solve time
            solve_time = time.time() - start_time

            # Track performance
            self.solve_times.append(solve_time)
            self.solve_success_rate.append(1.0)
            self.consecutive_failures = 0  # Reset consecutive failure counter

            # Apply safety limits to optimal control outputs using configured parameters
            # These are the MPC-computed steering angle and acceleration commands
            acceleration = float(np.clip(optimal_U[0, 0], 
                                       -self.max_deceleration, 
                                       self.max_acceleration))
            steering = float(np.clip(optimal_U[1, 0], 
                                   -self.max_steering_angle, 
                                   self.max_steering_angle))

            result = {
                'acceleration': acceleration,
                'steering': steering,
                'success': True,
                'solve_time': solve_time,
                'predicted_states': optimal_X,
                'control_sequence': optimal_U
            }

            if self.enable_logging and self.iteration_count % 50 == 0:
                self._log('info', f"[MPC] MPC solve #{self.iteration_count}: {solve_time:.4f}s, "
                         f"a={result['acceleration']:.3f}, δ={result['steering']:.3f}")

            return result

        except Exception as e:
            solve_time = time.time() - start_time

            # Track failed solve
            self.solve_times.append(solve_time)
            self.solve_success_rate.append(0.0)
            self.consecutive_failures += 1

            if self.enable_logging:
                self._log('error', f"[MPC] ❌ MPC solve failed (consecutive: {self.consecutive_failures}): {e}")

            return {
                'acceleration': 0.0,
                'steering': 0.0,
                'success': False,
                'solve_time': solve_time,
                'error': str(e)
            }

    def _validate_inputs(self, current_state, reference_trajectory):
        """Validate inputs for numerical stability"""
        try:
            # Validate current state
            required_keys = ['x', 'y', 'v', 'theta']
            if self.mpc_type == MPCType.DYNAMIC:
                required_keys.extend(['beta', 'r'])

            for key in required_keys[:4]:  # Always check basic kinematic states
                if key not in current_state:
                    return False
                if not np.isfinite(current_state[key]):
                    return False

            # Check reasonable ranges
            if abs(current_state['x']) > 1000 or abs(current_state['y']) > 1000:
                return False
            if current_state['v'] < -10 or current_state['v'] > 50:
                return False
            if abs(current_state['theta']) > 4 * np.pi:  # Allow some wrapping
                return False

            # Validate reference trajectory
            if reference_trajectory is None or reference_trajectory.size == 0:
                return False
            if not np.all(np.isfinite(reference_trajectory)):
                return False

            # Check trajectory dimensions
            expected_states = 4 if self.mpc_type == MPCType.KINEMATIC else 6
            if reference_trajectory.shape[1] != expected_states:
                return False
            if reference_trajectory.shape[0] != self.N + 1:
                return False
            
            # Check for problematic velocity values that cause infeasibility
            velocities = reference_trajectory[:, 2]  # v is always the 3rd column
            if np.any(velocities < 0.01):  # Too low velocities cause infeasibility
                return False
            if np.any(velocities > 50.0):  # Unreasonably high velocities
                return False

            return True
        except BaseException:
            return False

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
        """Reset performance tracking metrics and clear warm start"""
        self.solve_times.clear()
        self.solve_success_rate.clear()
        self.iteration_count = 0
        self.consecutive_failures = 0
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
