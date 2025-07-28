#!/usr/bin/env python3

"""
F1TENTH Optimized MPC Controller Node with All Parameters

This ROS2 node implements an advanced Model Predictive Controller for trajectory tracking
in F1TENTH autonomous racing cars using all parameters from params.yaml.

Author: Mohammed Azab <mohammed@azab.io>
License: MIT
Version: 2.0.0
"""
import rclpy
import numpy as np
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from giu_f1t_interfaces.msg import VehicleStateArray
from tf_transformations import euler_from_quaternion

try:
    from .optimized_mpc_controller import OptimizedMPCController
    from .kinematic_bicycle_model import MPCType
except ImportError:
    from optimized_mpc_controller import OptimizedMPCController
    from kinematic_bicycle_model import MPCType
# Import configuration defaults
import sys, os
try:
    # Try multiple paths to find config
    possible_config_paths = [
        os.path.join(os.path.dirname(__file__), '..', 'config'),  # Source tree
        '/home/mohammedazab/ws/src/race_stack/config',  # Absolute path
        os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config')  # Alternative relative
    ]
    
    config_imported = False
    for config_path in possible_config_paths:
        if os.path.exists(config_path) and config_path not in sys.path:
            sys.path.insert(0, config_path)
            try:
                import config as default_config
                print(f"Using config.py defaults from {config_path}")
                config_imported = True
                break
            except ImportError:
                continue
    
    if not config_imported:
        raise ImportError("Config module not found in any expected location")
        
except ImportError as e:
    # Fallback if config.py is not available
    print(f"Config file not found ({e}), using hardcoded defaults")
    class default_config:
        enable_trajectory_generation = True
        optimal_trajectory_path = "/home/mohammedazab/ws/src/race_stack/mpc_controller/trajectory/optimal_trajectory.csv"
        reference_trajectory_path = "/home/mohammedazab/ws/src/race_stack/mpc_controller/trajectory/ref_trajectory.csv"
        horizon_N = 10
        wheelbase = 0.33
        max_steering_angle = 0.5
        min_speed = 0.1


class MPCNode(Node):
    def __init__(self):
        super().__init__('optimized_mpc_node')

        # Check if YAML config override is enabled
        self.yaml_config_enabled = getattr(default_config, 'yaml_config_enabled', True)
        
        self._declare_all_parameters()
        self._load_all_parameters()
        self._initialize_optimized_mpc()
        self._initialize_state()
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()

        self.get_logger().info("MPC Node has been started 🏎️ ")

    def _declare_all_parameters(self):
        """Declare all ROS2 parameters using config.py defaults"""

        # Check if YAML config should override defaults
        yaml_enabled = getattr(default_config, 'yaml_config_enabled', True)
        
        if yaml_enabled:
            self.get_logger().info("🔧 Using config.py defaults with YAML override enabled")
        else:
            self.get_logger().info("🔧 Using config.py defaults with YAML override disabled")

        # Trajectory settings
        self.declare_parameter('enable_trajectory_generation', 
                             getattr(default_config, 'enable_trajectory_generation', True))
        self.declare_parameter('optimal_trajectory_path', 
                             getattr(default_config, 'optimal_trajectory_path', ''))
        self.declare_parameter('reference_trajectory_path', 
                             getattr(default_config, 'reference_trajectory_path', ''))

        # Vehicle parameters
        self.declare_parameter('wheelbase', 
                             getattr(default_config, 'wheelbase', 0.33))

        # MPC Horizon
        self.declare_parameter('horizon_N', 
                             getattr(default_config, 'horizon_N', 8))
        self.declare_parameter('horizon_T', 
                             getattr(default_config, 'horizon_T', 0.8))
        self.declare_parameter('lookahead_distance', 
                             getattr(default_config, 'lookahead_distance', 0.5))

        # Vehicle limits
        self.declare_parameter('max_steering_angle', 
                             getattr(default_config, 'max_steering_angle', 0.9))
        self.declare_parameter('max_acceleration', 
                             getattr(default_config, 'max_acceleration', 2.8))
        self.declare_parameter('max_deceleration', 
                             getattr(default_config, 'max_deceleration', 2.8))
        self.declare_parameter('min_speed', 
                             getattr(default_config, 'min_speed', 0.1))
        self.declare_parameter('max_speed', 
                             getattr(default_config, 'max_speed', 8.0))

        # Cost function weights
        self.declare_parameter('enable_cost_function_weights', 
                             getattr(default_config, 'enable_cost_function_weights', True))
        self.declare_parameter('cost_function_weights.steering_weight', 
                             getattr(default_config, 'steering_weight', 1.0))
        self.declare_parameter('cost_function_weights.acceleration_weight', 
                             getattr(default_config, 'acceleration_weight', 0.5))
        self.declare_parameter('cost_function_weights.jerk_weight', 
                             getattr(default_config, 'jerk_weight', 0.1))
        self.declare_parameter('cost_function_weights.heading_weight', 
                             getattr(default_config, 'heading_weight', 2.0))
        self.declare_parameter('cost_function_weights.position_weight', 
                             getattr(default_config, 'position_weight', 5.0))
        self.declare_parameter('cost_function_weights.velocity_weight', 
                             getattr(default_config, 'velocity_weight', 1.0))

        # Hard constraints
        self.declare_parameter('enable_hard_constraints', 
                             getattr(default_config, 'enable_hard_constraints', False))
        self.declare_parameter('hard_constraints.max_steering_angle', 
                             getattr(default_config, 'hard_max_steering_angle', 0.4))
        self.declare_parameter('hard_constraints.max_acceleration', 
                             getattr(default_config, 'hard_max_acceleration', 0.8))
        self.declare_parameter('hard_constraints.max_deceleration', 
                             getattr(default_config, 'hard_max_deceleration', 0.8))

        # Obstacle avoidance
        self.declare_parameter('enable_obstacle_avoidance', 
                             getattr(default_config, 'enable_obstacle_avoidance', False))
        self.declare_parameter('obstacle_avoidance_weight', 
                             getattr(default_config, 'obstacle_avoidance_weight', 0.5))

        # Speed control
        self.declare_parameter('enable_speed_control', 
                             getattr(default_config, 'enable_speed_control', True))
        self.declare_parameter('speed_control_weight', 
                             getattr(default_config, 'speed_control_weight', 0.2))

        # Trajectory tracking
        self.declare_parameter('enable_trajectory_tracking', 
                             getattr(default_config, 'enable_trajectory_tracking', True))
        self.declare_parameter('trajectory_tracking_weight', 
                             getattr(default_config, 'trajectory_tracking_weight', 0.1))

        # Safety checks
        self.declare_parameter('enable_safety_checks', 
                             getattr(default_config, 'enable_safety_checks', True))
        self.declare_parameter('safety_check_distance', 
                             getattr(default_config, 'safety_check_distance', 0.5))

        # Logging
        self.declare_parameter('enable_logging', 
                             getattr(default_config, 'enable_logging', True))

        # Additional MPC parameters
        self.declare_parameter('mpc_type', 
                             getattr(default_config, 'mpc_type', 'kinematic'))
        self.declare_parameter('solver_type', 
                             getattr(default_config, 'solver_type', 'ipopt'))
        self.declare_parameter('control_hz', 
                             getattr(default_config, 'control_hz', 20.0))

        # Safety Parameters
        self.declare_parameter('safety_timeout', 
                             getattr(default_config, 'safety_timeout', 1.0))
        self.declare_parameter('emergency_brake_threshold', 
                             getattr(default_config, 'emergency_brake_threshold', 2.0))

        # Topics
        self.declare_parameter('odom_topic', 
                             getattr(default_config, 'odom_topic', 'car_state/odom'))
        self.declare_parameter('reference_topic', 
                             getattr(default_config, 'reference_topic', '/mpc/reference_trajectory'))
        self.declare_parameter('status_topic', 
                             getattr(default_config, 'status_topic', '/mpc/path_ready'))
        self.declare_parameter('control_topic', 
                             getattr(default_config, 'control_topic', '/drive'))
        self.declare_parameter('pose_estimate_topic', '/initialpose')

        # QoS
        self.declare_parameter('qos_depth', 
                             getattr(default_config, 'qos_depth', 10))
        
        # Debug and Logging settings
        self.declare_parameter('debug_logging_enabled', 
                             getattr(default_config, 'debug_logging_enabled', False))
        self.declare_parameter('performance_logging_enabled', 
                             getattr(default_config, 'performance_logging_enabled', True))
        self.declare_parameter('state_logging_enabled', 
                             getattr(default_config, 'state_logging_enabled', False))
        self.declare_parameter('control_logging_enabled', 
                             getattr(default_config, 'control_logging_enabled', True))
        self.declare_parameter('trajectory_logging_enabled', 
                             getattr(default_config, 'trajectory_logging_enabled', False))
        self.declare_parameter('solver_logging_enabled', 
                             getattr(default_config, 'solver_logging_enabled', False))
        self.declare_parameter('log_frequency_divider', 
                             getattr(default_config, 'log_frequency_divider', 10))

    def _load_all_parameters(self):
        """Load all parameters from ROS2 parameter server"""

        # Trajectory settings
        self.enable_trajectory_generation = self.get_parameter('enable_trajectory_generation').value
        self.optimal_trajectory_path = self.get_parameter('optimal_trajectory_path').value
        self.reference_trajectory_path = self.get_parameter('reference_trajectory_path').value

        # Vehicle parameters
        self.wheelbase = self.get_parameter('wheelbase').value

        # MPC Horizon
        self.horizon_N = self.get_parameter('horizon_N').value
        self.horizon_T = self.get_parameter('horizon_T').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        # Vehicle limits
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_deceleration = self.get_parameter('max_deceleration').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value

        # Cost function weights
        self.enable_cost_function_weights = self.get_parameter('enable_cost_function_weights').value
        self.cost_function_weights = {
            'steering_weight': self.get_parameter('cost_function_weights.steering_weight').value,
            'acceleration_weight': self.get_parameter('cost_function_weights.acceleration_weight').value,
            'jerk_weight': self.get_parameter('cost_function_weights.jerk_weight').value,
            'heading_weight': self.get_parameter('cost_function_weights.heading_weight').value,
            'position_weight': self.get_parameter('cost_function_weights.position_weight').value,
            'velocity_weight': self.get_parameter('cost_function_weights.velocity_weight').value,
        }

        # Hard constraints
        self.enable_hard_constraints = self.get_parameter('enable_hard_constraints').value
        self.hard_constraints = {
            'max_steering_angle': self.get_parameter('hard_constraints.max_steering_angle').value,
            'max_acceleration': self.get_parameter('hard_constraints.max_acceleration').value,
            'max_deceleration': self.get_parameter('hard_constraints.max_deceleration').value,
        }

        # Obstacle avoidance
        self.enable_obstacle_avoidance = self.get_parameter('enable_obstacle_avoidance').value
        self.obstacle_avoidance_weight = self.get_parameter('obstacle_avoidance_weight').value

        # Speed control
        self.enable_speed_control = self.get_parameter('enable_speed_control').value
        self.speed_control_weight = self.get_parameter('speed_control_weight').value

        # Trajectory tracking
        self.enable_trajectory_tracking = self.get_parameter('enable_trajectory_tracking').value
        self.trajectory_tracking_weight = self.get_parameter('trajectory_tracking_weight').value

        # Safety checks
        self.enable_safety_checks = self.get_parameter('enable_safety_checks').value
        self.safety_check_distance = self.get_parameter('safety_check_distance').value

        # Logging
        self.enable_logging = self.get_parameter('enable_logging').value
        
        # Debug and Advanced Logging settings
        self.debug_logging_enabled = self.get_parameter('debug_logging_enabled').value
        self.performance_logging_enabled = self.get_parameter('performance_logging_enabled').value
        self.state_logging_enabled = self.get_parameter('state_logging_enabled').value
        self.control_logging_enabled = self.get_parameter('control_logging_enabled').value
        self.trajectory_logging_enabled = self.get_parameter('trajectory_logging_enabled').value
        self.solver_logging_enabled = self.get_parameter('solver_logging_enabled').value
        self.log_frequency_divider = self.get_parameter('log_frequency_divider').value

        # Optimized MPC Parameters
        mpc_type_str = self.get_parameter('mpc_type').value
        self.mpc_type = MPCType.KINEMATIC if mpc_type_str == 'kinematic' else MPCType.DYNAMIC
        self.solver_type = self.get_parameter('solver_type').value
        self.control_hz = self.get_parameter('control_hz').value

        # Safety Parameters
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.emergency_brake_threshold = self.get_parameter('emergency_brake_threshold').value

        # Topics
        self.odom_topic = self.get_parameter('odom_topic').value
        self.reference_topic = self.get_parameter('reference_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.pose_estimate_topic = self.get_parameter('pose_estimate_topic').value

        self.qos_depth = self.get_parameter('qos_depth').value

    def _initialize_optimized_mpc(self):
        """Initialize the optimized MPC controller with all parameters"""

        try:
            self.mpc_controller = OptimizedMPCController(
                N=self.horizon_N,
                T=self.horizon_T,
                wheelbase=self.wheelbase,
                mpc_type=self.mpc_type,
                solver_type=self.solver_type,
                enable_logging=self.enable_logging,
                # All parameters from params.yaml
                max_steering_angle=self.max_steering_angle,
                max_acceleration=self.max_acceleration,
                max_deceleration=self.max_deceleration,
                min_speed=self.min_speed,
                max_speed=self.max_speed,
                enable_cost_function_weights=self.enable_cost_function_weights,
                cost_function_weights=self.cost_function_weights,
                enable_hard_constraints=self.enable_hard_constraints,
                hard_constraints=self.hard_constraints,
                enable_obstacle_avoidance=self.enable_obstacle_avoidance,
                obstacle_avoidance_weight=self.obstacle_avoidance_weight,
                enable_speed_control=self.enable_speed_control,
                speed_control_weight=self.speed_control_weight,
                enable_trajectory_tracking=self.enable_trajectory_tracking,
                trajectory_tracking_weight=self.trajectory_tracking_weight,
                enable_safety_checks=self.enable_safety_checks,
                safety_check_distance=self.safety_check_distance,
                lookahead_distance=self.lookahead_distance,
                logger=self.get_logger()  # Pass ROS2 logger to controller
            )

            self.get_logger().info("✅ Optimized MPC Controller initialized successfully")
            self.get_logger().info(f"   - Type: {self.mpc_type.value}")
            self.get_logger().info(f"   - Horizon: N={self.horizon_N}, T={self.horizon_T}s")
            self.get_logger().info(f"   - Solver: {self.solver_type}")
            self.get_logger().info(f"   - Control Hz: {self.control_hz}")

        except Exception as e:
            self.get_logger().error(f"❌ Failed to initialize MPC controller: {e}")
            raise e

    def _initialize_state(self):
        """Initialize node state variables with safe defaults"""

        # Vehicle state
        self.current_pose = None
        self.current_velocity = 0.0
        self.current_yaw = 0.0  # Initialize to 0 instead of None
        self.current_steering_angle = 0.0

        # RViz 2D Pose Estimate for debugging/validation
        self.rviz_pose_estimate = None
        self.last_pose_estimate_time = None

        # Additional states for dynamic model
        self.current_beta = 0.0  # Sideslip angle
        self.current_yaw_rate = 0.0

        # Reference trajectory
        self.reference_trajectory = []
        self.path_ready = False

        # Safety and performance tracking
        self.last_trajectory_time = None
        self.last_odom_time = None
        self.control_active = False
        self.emergency_stop = False
        self.emergency_stop_time = None  # Track when emergency stop was activated
        self.emergency_recovery_timeout = 2.0  # Allow recovery after 2 seconds

        # Performance metrics
        self.control_loop_times = []

        # Add state validation flags and failure tracking
        self.state_initialized = False
        self.first_odom_received = False
        self.consecutive_mpc_failures = 0
        self.last_successful_solve_time = None
        
        # Debug logging state tracking
        self.log_counter = 0
        self.total_mpc_solves = 0
        self.successful_mpc_solves = 0
        self.failed_mpc_solves = 0
        self.total_solve_time = 0.0
        self.max_solve_time = 0.0
        self.min_solve_time = float('inf')
        
        # State logging buffers
        self.recent_states = []
        self.recent_controls = []
        self.recent_solve_times = []
        
        # Trajectory tracking metrics
        self.trajectory_updates_received = 0
        self.valid_trajectory_updates = 0
        self.current_trajectory_length = 0
        
        # Performance alerts
        self.slow_solve_threshold = 0.05  # 50ms
        self.slow_solve_count = 0

    def _setup_subscriptions(self):
        """Setup ROS2 subscriptions"""

        # Odometry subscription
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            self.qos_depth
        )

        # Reference trajectory subscription
        self.create_subscription(
            VehicleStateArray,
            self.reference_topic,
            self._reference_callback,
            self.qos_depth
        )

        # Path ready status subscription
        self.create_subscription(
            Bool,
            self.status_topic,
            self._status_callback,
            self.qos_depth
        )

        # RViz 2D Pose Estimate subscription for debugging/validation
        self.create_subscription(
            PoseStamped,
            self.pose_estimate_topic,
            self._pose_estimate_callback,
            self.qos_depth
        )

    def _setup_publishers(self):
        """Setup ROS2 publishers"""

        # Control command publisher
        self.control_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.control_topic,
            self.qos_depth
        )

        # Performance metrics publishers
        self.solve_time_publisher = self.create_publisher(
            Float32,
            '/mpc/solve_time',
            self.qos_depth
        )

        # Diagnostics publisher
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            '/mpc/diagnostics',
            self.qos_depth
        )

    def _setup_timers(self):
        """Setup ROS2 timers"""

        # Main control loop
        self.create_timer(1.0 / self.control_hz, self._control_loop)

        # Diagnostics timer (lower frequency)
        self.create_timer(1.0, self._publish_diagnostics)

    def _odom_callback(self, msg: Odometry):
        """Process odometry data with validation"""

        try:
            # Validate odometry message
            if not self._validate_odometry_msg(msg):
                if self.enable_logging and not self.first_odom_received:
                    #self.get_logger().warn("Received invalid odometry message, skipping update")
                    pass
                return

            self.current_pose = msg.pose.pose

            # Extract velocity with validation
            linear_vel = msg.twist.twist.linear
            velocity_magnitude = np.sqrt(linear_vel.x**2 + linear_vel.y**2)

            # Validate and clamp velocity
            if np.isfinite(velocity_magnitude) and velocity_magnitude < 100:  # Reasonable upper bound
                self.current_velocity = velocity_magnitude
            else:
                if self.enable_logging:
                    self.get_logger().warn(f"Invalid velocity detected: {velocity_magnitude}, keeping previous value")

            # Extract yaw angle with validation
            orientation = msg.pose.pose.orientation
            try:
                _, _, yaw = euler_from_quaternion([
                    orientation.x, orientation.y, orientation.z, orientation.w
                ])
                if np.isfinite(yaw):
                    self.current_yaw = yaw
                else:
                    if self.enable_logging:
                        self.get_logger().warn("Invalid yaw angle detected, keeping previous value")
            except BaseException:
                if self.enable_logging:
                    self.get_logger().warn("Failed to extract yaw angle from quaternion")

            # Extract additional states for dynamic model
            if self.mpc_type == MPCType.DYNAMIC:
                yaw_rate = msg.twist.twist.angular.z
                if np.isfinite(yaw_rate) and abs(yaw_rate) < 50:  # Reasonable bound
                    self.current_yaw_rate = yaw_rate

                # Estimate sideslip angle (simplified)
                if self.current_velocity > 0.1:
                    lateral_vel = linear_vel.y  # Assuming body frame
                    if np.isfinite(lateral_vel):
                        beta = np.arctan(lateral_vel / self.current_velocity)
                        if abs(beta) < np.pi / 2:  # Reasonable bound
                            self.current_beta = beta

            self.last_odom_time = self.get_clock().now()

            # State logging for debugging
            if self.state_logging_enabled and self.log_counter % (self.log_frequency_divider * 2) == 0:
                self.get_logger().info(
                    f"[STATE] Vehicle: X={self.current_pose.position.x:.3f}, Y={self.current_pose.position.y:.3f}, "
                    f"V={self.current_velocity:.3f}, Yaw={np.degrees(self.current_yaw):.1f}°")
                if self.mpc_type == MPCType.DYNAMIC:
                    self.get_logger().info(
                        f"[STATE] Dynamic: Beta={np.degrees(self.current_beta):.1f}°, "
                        f"YawRate={np.degrees(self.current_yaw_rate):.1f}°/s")
            
            # Track recent states for debugging
            current_state_data = {
                'time': time.time(),
                'x': self.current_pose.position.x,
                'y': self.current_pose.position.y,
                'v': self.current_velocity,
                'yaw': self.current_yaw
            }
            if self.mpc_type == MPCType.DYNAMIC:
                current_state_data.update({
                    'beta': self.current_beta,
                    'yaw_rate': self.current_yaw_rate
                })
            
            self.recent_states.append(current_state_data)
            if len(self.recent_states) > 50:  # Keep last 50 states
                self.recent_states.pop(0)

            if not self.first_odom_received:
                self.first_odom_received = True
                self.state_initialized = True
                if self.enable_logging:
                    self.get_logger().info("[STATE] ✅ First valid odometry received, MPC ready for operation")
                    self.get_logger().info(f"[STATE] Initial state: X={self.current_pose.position.x:.3f}, "
                                         f"Y={self.current_pose.position.y:.3f}, V={self.current_velocity:.3f}")

        except Exception as e:
            if self.enable_logging:
                self.get_logger().error(f"[ERROR] Error processing odometry: {e}")
                if self.debug_logging_enabled:
                    import traceback
                    self.get_logger().error(f"[DEBUG] Odometry error traceback: {traceback.format_exc()}")

    def _validate_odometry_msg(self, msg):
        """Validate odometry message for numerical stability"""
        try:
            # Check position
            pos = msg.pose.pose.position
            if not (np.isfinite(pos.x) and np.isfinite(pos.y) and np.isfinite(pos.z)):
                return False
            if abs(pos.x) > 10000 or abs(pos.y) > 10000:  # Reasonable position bounds
                return False

            # Check orientation
            ori = msg.pose.pose.orientation
            if not (np.isfinite(ori.x) and np.isfinite(ori.y) and np.isfinite(ori.z) and np.isfinite(ori.w)):
                return False

            # Check if quaternion is normalized (approximately)
            norm = np.sqrt(ori.x**2 + ori.y**2 + ori.z**2 + ori.w**2)
            if abs(norm - 1.0) > 0.1:  # Allow some tolerance
                return False

            # Check velocities
            lin_vel = msg.twist.twist.linear
            ang_vel = msg.twist.twist.angular
            velocities = [lin_vel.x, lin_vel.y, lin_vel.z, ang_vel.x, ang_vel.y, ang_vel.z]
            if not all(np.isfinite(v) for v in velocities):
                return False

            return True
        except BaseException:
            return False

    def _reference_callback(self, msg: VehicleStateArray):
        """Process reference trajectory with robust numerical validation"""
        
        # Update trajectory tracking metrics
        self.trajectory_updates_received += 1

        if len(msg.states) < 2:  # Need at least 2 points to calculate heading
            self.get_logger().warn(f"[TRAJ] Reference trajectory too short: {len(msg.states)} < 2 (minimum)")
            return

        # Log trajectory reception
        if self.trajectory_logging_enabled and self.debug_logging_enabled:
            self.get_logger().info(f"[TRAJ] Received trajectory with {len(msg.states)} states")

        # Convert to numpy array for MPC with validation
        self.reference_trajectory = []
        invalid_states = 0
        
        for i, state in enumerate(msg.states):
            # Validate state values for numerical stability
            if not self._validate_state_values(state):
                invalid_states += 1
                if self.debug_logging_enabled:
                    self.get_logger().warn(f"[DEBUG] Invalid state values at index {i}: x={state.x}, y={state.y}, v={state.v}")
                if invalid_states > 3:  # Reject trajectory if too many invalid states
                    self.get_logger().error(f"[TRAJ] Too many invalid states ({invalid_states}), rejecting trajectory update")
                    if self.debug_logging_enabled:
                        # Log some sample invalid states for debugging
                        self.get_logger().error(f"[DEBUG] First invalid state example: x={msg.states[0].x}, y={msg.states[0].y}, v={msg.states[0].v}")
                        if len(msg.states) > 1:
                            self.get_logger().error(f"[DEBUG] Last invalid state example: x={msg.states[-1].x}, y={msg.states[-1].y}, v={msg.states[-1].v}")
                    return

            if self.mpc_type == MPCType.KINEMATIC:
                # Use theta from VehicleState if available and valid, otherwise calculate from trajectory points
                if hasattr(state, 'theta') and abs(state.theta) > 1e-6:  # More robust check
                    theta = state.theta
                else:
                    # Calculate heading from trajectory points
                    if i < len(msg.states) - 1:
                        next_state = msg.states[i + 1]
                        dx = next_state.x - state.x
                        dy = next_state.y - state.y
                        # Validate heading calculation inputs
                        distance = np.sqrt(dx*dx + dy*dy)
                        if distance < 1e-6:  # Points are too close, use previous theta or current yaw
                            if i > 0 and len(self.reference_trajectory) > 0:
                                # Use previous trajectory point's theta
                                theta = self.reference_trajectory[-1][3]
                            else:
                                # Use current yaw as fallback
                                theta = self.current_yaw if self.current_yaw is not None and np.isfinite(self.current_yaw) else 0.0
                        else:
                            theta = np.arctan2(dy, dx)
                    else:
                        # For the last point, use previous trajectory point's theta or current yaw
                        if i > 0 and len(self.reference_trajectory) > 0:
                            theta = self.reference_trajectory[-1][3]
                        else:
                            theta = self.current_yaw if self.current_yaw is not None and np.isfinite(self.current_yaw) else 0.0

                # Ensure theta is normalized to [-pi, pi] and finite
                if not np.isfinite(theta):
                    theta = 0.0
                    if self.debug_logging_enabled:
                        self.get_logger().warn(f"[DEBUG] Non-finite theta at index {i}, using 0.0")
                else:
                    theta = np.arctan2(np.sin(theta), np.cos(theta))

                # Build reference trajectory state vector (x, y, v, theta only)
                # NOTE: Steering angles are computed by MPC, not from trajectory publisher
                self.reference_trajectory.append([
                    float(state.x), float(state.y), float(state.v if state.v > 0.01 else 0.01), float(theta)
                ])
            else:  # DYNAMIC
                # Use theta from VehicleState if available and valid, otherwise calculate from trajectory points
                if hasattr(state, 'theta') and abs(state.theta) > 1e-6:
                    theta = state.theta
                else:
                    # Calculate heading from trajectory points
                    if i < len(msg.states) - 1:
                        next_state = msg.states[i + 1]
                        dx = next_state.x - state.x
                        dy = next_state.y - state.y
                        # Validate heading calculation inputs
                        distance = np.sqrt(dx*dx + dy*dy)
                        if distance < 1e-6:  # Points are too close
                            if i > 0 and len(self.reference_trajectory) > 0:
                                # Use previous trajectory point's theta
                                theta = self.reference_trajectory[-1][3]
                            else:
                                # Use current yaw as fallback
                                theta = self.current_yaw if self.current_yaw is not None and np.isfinite(self.current_yaw) else 0.0
                        else:
                            theta = np.arctan2(dy, dx)
                    else:
                        # For the last point, use previous trajectory point's theta or current yaw
                        if i > 0 and len(self.reference_trajectory) > 0:
                            theta = self.reference_trajectory[-1][3]
                        else:
                            theta = self.current_yaw if self.current_yaw is not None and np.isfinite(self.current_yaw) else 0.0

                # Ensure theta is normalized to [-pi, pi] and finite
                if not np.isfinite(theta):
                    theta = 0.0
                    if self.debug_logging_enabled:
                        self.get_logger().warn(f"[DEBUG] Non-finite theta at index {i}, using 0.0")
                else:
                    theta = np.arctan2(np.sin(theta), np.cos(theta))

                # Build reference trajectory state vector (x, y, v, theta, beta=0, r=0)
                # NOTE: Steering angles are computed by MPC, not from trajectory publisher
                self.reference_trajectory.append([
                    float(state.x), float(state.y), float(state.v if state.v > 0.01 else 0.01), float(theta), 0.0, 0.0  # beta=0, r=0 for reference
                ])

        self.reference_trajectory = np.array(self.reference_trajectory)

        # Final validation of the complete trajectory
        if not self._validate_trajectory(self.reference_trajectory):
            self.get_logger().error("[TRAJ] Reference trajectory contains invalid values, rejecting update")
            if self.debug_logging_enabled:
                # Log detailed trajectory information for debugging
                traj_array = np.array(self.reference_trajectory)
                self.get_logger().error(f"[DEBUG] Failed trajectory shape: {traj_array.shape}")
                self.get_logger().error(f"[DEBUG] Failed trajectory sample (first 3 points):")
                for i in range(min(3, len(traj_array))):
                    self.get_logger().error(f"[DEBUG] Point {i}: {traj_array[i]}")
                if len(traj_array) > 3:
                    self.get_logger().error(f"[DEBUG] Failed trajectory sample (last 3 points):")
                    for i in range(max(0, len(traj_array)-3), len(traj_array)):
                        self.get_logger().error(f"[DEBUG] Point {i}: {traj_array[i]}")
                # Log statistics
                if len(traj_array) > 0:
                    self.get_logger().error(f"[DEBUG] Trajectory stats:")
                    self.get_logger().error(f"[DEBUG] X: min={np.min(traj_array[:, 0]):.3f}, max={np.max(traj_array[:, 0]):.3f}")
                    self.get_logger().error(f"[DEBUG] Y: min={np.min(traj_array[:, 1]):.3f}, max={np.max(traj_array[:, 1]):.3f}")
                    self.get_logger().error(f"[DEBUG] V: min={np.min(traj_array[:, 2]):.3f}, max={np.max(traj_array[:, 2]):.3f}")
                    if traj_array.shape[1] > 3:
                        self.get_logger().error(f"[DEBUG] Theta: min={np.min(traj_array[:, 3]):.3f}, max={np.max(traj_array[:, 3]):.3f}")
            return

        # Update tracking metrics
        self.valid_trajectory_updates += 1
        self.current_trajectory_length = len(self.reference_trajectory)
        self.last_trajectory_time = self.get_clock().now()

        # Enhanced trajectory logging
        if self.trajectory_logging_enabled:
            traj_array = np.array(self.reference_trajectory)
            self.get_logger().info(
                f"[TRAJ] ✅ Accepted trajectory: {len(self.reference_trajectory)} points, "
                f"X: [{np.min(traj_array[:, 0]):.2f}, {np.max(traj_array[:, 0]):.2f}], "
                f"Y: [{np.min(traj_array[:, 1]):.2f}, {np.max(traj_array[:, 1]):.2f}], "
                f"V: [{np.min(traj_array[:, 2]):.2f}, {np.max(traj_array[:, 2]):.2f}]")
        elif self.enable_logging:
            pass
            #self.get_logger().info(
            #    f"[TRAJ] Received reference trajectory with {len(self.reference_trajectory)} points (horizon_N={self.horizon_N})")
        
        # Log trajectory update statistics periodically
        if self.performance_logging_enabled and self.trajectory_updates_received % 50 == 0:
            success_rate = (self.valid_trajectory_updates / self.trajectory_updates_received) * 100
            self.get_logger().info(
                f"[TRAJ] Trajectory stats: {self.trajectory_updates_received} received, "
                f"{self.valid_trajectory_updates} valid ({success_rate:.1f}%), "
                f"Current length: {self.current_trajectory_length}, Invalid states: {invalid_states}")

    def _validate_state_values(self, state):
        """Validate individual state values for numerical stability"""
        try:
            # Check for NaN or infinite values
            values = [state.x, state.y, state.v]
            if hasattr(state, 'theta'):
                values.append(state.theta)

            for i, val in enumerate(values):
                if not np.isfinite(val):
                    if self.debug_logging_enabled:
                        field_names = ['x', 'y', 'v', 'theta']
                        self.get_logger().error(f"[DEBUG] Invalid state value: {field_names[i] if i < len(field_names) else f'field_{i}'} = {val}")
                    return False

            # Check for reasonable ranges
            if abs(state.x) > 1000 or abs(state.y) > 1000:  # Position bounds
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[DEBUG] State position out of bounds: x={state.x}, y={state.y}")
                return False
            if state.v < -2 or state.v > 25:  # F1TENTH appropriate velocity bounds
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[DEBUG] State velocity out of bounds: v={state.v}")
                return False
            if hasattr(state, 'theta') and abs(state.theta) > 10:  # Angle bounds
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[DEBUG] State theta out of bounds: theta={state.theta}")
                return False

            return True
        except Exception as e:
            if self.debug_logging_enabled:
                self.get_logger().error(f"[DEBUG] State validation exception: {e}")
            return False

    def _validate_trajectory(self, trajectory):
        """Validate complete trajectory for numerical stability"""
        try:
            if trajectory is None or len(trajectory) == 0:
                if self.debug_logging_enabled:
                    self.get_logger().error("[DEBUG] Trajectory validation failed: None or empty trajectory")
                return False

            # Check for NaN or infinite values
            if not np.all(np.isfinite(trajectory)):
                if self.debug_logging_enabled:
                    nan_mask = ~np.isfinite(trajectory)
                    nan_indices = np.where(nan_mask)
                    self.get_logger().error(f"[DEBUG] Trajectory validation failed: NaN/Inf values at indices {list(zip(nan_indices[0], nan_indices[1]))}")
                    # Log first few invalid values for debugging
                    for i in range(min(5, len(nan_indices[0]))):
                        row, col = nan_indices[0][i], nan_indices[1][i]
                        self.get_logger().error(f"[DEBUG] Invalid value at [{row},{col}]: {trajectory[row, col]}")
                return False

            # Check trajectory shape
            expected_cols = 4 if self.mpc_type == MPCType.KINEMATIC else 6
            if trajectory.shape[1] != expected_cols:
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[DEBUG] Trajectory validation failed: Wrong shape {trajectory.shape}, expected columns: {expected_cols}")
                return False

            # Check trajectory smoothness (no sudden jumps)
            if len(trajectory) > 1:
                diffs = np.diff(trajectory, axis=0)
                position_diffs = np.linalg.norm(diffs[:, :2], axis=1)  # x,y differences

                # Check for unreasonably large position jumps (>5m between points)
                large_jumps = position_diffs > 5.0
                if np.any(large_jumps):
                    if self.debug_logging_enabled:
                        jump_indices = np.where(large_jumps)[0]
                        self.get_logger().error(f"[DEBUG] Trajectory validation failed: Large position jumps detected")
                        for idx in jump_indices[:3]:  # Log first 3 large jumps
                            self.get_logger().error(f"[DEBUG] Large jump at index {idx}: {position_diffs[idx]:.3f}m between points {idx} and {idx+1}")
                            self.get_logger().error(f"[DEBUG] Point {idx}: [{trajectory[idx, 0]:.3f}, {trajectory[idx, 1]:.3f}]")
                            self.get_logger().error(f"[DEBUG] Point {idx+1}: [{trajectory[idx+1, 0]:.3f}, {trajectory[idx+1, 1]:.3f}]")
                    return False

                # Check for unreasonably large velocity jumps (>20 m/s between points for F1TENTH)
                velocity_diffs = np.abs(diffs[:, 2])
                large_vel_jumps = velocity_diffs > 20.0
                if np.any(large_vel_jumps):
                    if self.debug_logging_enabled:
                        vel_jump_indices = np.where(large_vel_jumps)[0]
                        self.get_logger().error(f"[DEBUG] Trajectory validation failed: Large velocity jumps detected")
                        for idx in vel_jump_indices[:3]:  # Log first 3 large velocity jumps
                            self.get_logger().error(f"[DEBUG] Large velocity jump at index {idx}: {velocity_diffs[idx]:.3f}m/s between points {idx} and {idx+1}")
                            self.get_logger().error(f"[DEBUG] Velocity {idx}: {trajectory[idx, 2]:.3f}m/s")
                            self.get_logger().error(f"[DEBUG] Velocity {idx+1}: {trajectory[idx+1, 2]:.3f}m/s")
                    return False

            # Additional range checks
            x_vals = trajectory[:, 0]
            y_vals = trajectory[:, 1]
            v_vals = trajectory[:, 2]

            # Check position bounds
            if np.any(np.abs(x_vals) > 1000) or np.any(np.abs(y_vals) > 1000):
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[DEBUG] Trajectory validation failed: Position out of bounds")
                    self.get_logger().error(f"[DEBUG] X range: [{np.min(x_vals):.3f}, {np.max(x_vals):.3f}]")
                    self.get_logger().error(f"[DEBUG] Y range: [{np.min(y_vals):.3f}, {np.max(y_vals):.3f}]")
                return False

            # Check velocity bounds (more appropriate for F1TENTH racing)
            if np.any(v_vals < -2) or np.any(v_vals > 25):
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[DEBUG] Trajectory validation failed: Velocity out of bounds")
                    self.get_logger().error(f"[DEBUG] Velocity range: [{np.min(v_vals):.3f}, {np.max(v_vals):.3f}]")
                return False

            return True
        except Exception as e:
            if self.debug_logging_enabled:
                self.get_logger().error(f"[DEBUG] Trajectory validation failed with exception: {e}")
                import traceback
                self.get_logger().error(f"[DEBUG] Validation exception traceback: {traceback.format_exc()}")
            return False

    def _status_callback(self, msg: Bool):
        """Process path ready status"""
        self.path_ready = msg.data

    def _pose_estimate_callback(self, msg: PoseStamped):
        """Process RViz 2D Pose Estimate for debugging/validation"""

        self.rviz_pose_estimate = msg.pose
        self.last_pose_estimate_time = self.get_clock().now()

        # Log pose difference for debugging if logging is enabled
        if self.enable_logging and self.current_pose is not None:
            # Calculate position difference
            pos_diff_x = msg.pose.position.x - self.current_pose.position.x
            pos_diff_y = msg.pose.position.y - self.current_pose.position.y
            pos_diff_magnitude = np.sqrt(pos_diff_x**2 + pos_diff_y**2)

            # Calculate yaw difference
            rviz_orientation = msg.pose.orientation
            _, _, rviz_yaw = euler_from_quaternion([
                rviz_orientation.x, rviz_orientation.y, rviz_orientation.z, rviz_orientation.w
            ])
            yaw_diff = np.abs(rviz_yaw - self.current_yaw)
            if yaw_diff > np.pi:
                yaw_diff = 2 * np.pi - yaw_diff

            # Log significant differences (> 10cm position or > 5 degrees heading)
            if pos_diff_magnitude > 0.1 or yaw_diff > np.radians(5):
                self.get_logger().info(
                    f"🎯 Pose difference - Position: {pos_diff_magnitude:.3f}m, "
                    f"Heading: {np.degrees(yaw_diff):.1f}°"
                )

    def _control_loop(self):
        """Main MPC control loop"""

        loop_start_time = time.time()

        # Check for emergency stop recovery
        if self.emergency_stop and self.emergency_stop_time is not None:
            time_since_emergency = time.time() - self.emergency_stop_time
            if time_since_emergency > self.emergency_recovery_timeout:
                if self.debug_logging_enabled:
                    self.get_logger().info(f"[RECOVERY] Attempting emergency stop recovery after {time_since_emergency:.1f}s")
                self.emergency_stop = False
                self.emergency_stop_time = None

        # Safety checks
        if not self._safety_checks():
            self._publish_emergency_stop()
            return

        # Check if we have all required data
        if not self._data_ready():
            self._publish_zero_control()
            return

        # Solve MPC optimization
        try:
            result = self._solve_mpc_optimization()

            if result['success']:
                self._publish_control_command(
                    result['acceleration'],
                    result['steering']
                )

                # Publish solve time
                solve_time_msg = Float32()
                solve_time_msg.data = result['solve_time']
                self.solve_time_publisher.publish(solve_time_msg)

                self.control_active = True
                self.consecutive_mpc_failures = 0  # Reset failure counter
                self.last_successful_solve_time = self.get_clock().now()

            else:
                self.consecutive_mpc_failures += 1

                if self.enable_logging:
                    self.get_logger().warn(
                        f"MPC optimization failed (consecutive: {self.consecutive_mpc_failures}), publishing zero control. Result: {result}")

                # Try reset after many consecutive failures
                if self.consecutive_mpc_failures >= 5:
                    if self.enable_logging:
                        self.get_logger().warn("Resetting MPC controller due to persistent failures")
                    self.mpc_controller.reset_performance_tracking()
                    self.consecutive_mpc_failures = 0

                self._publish_zero_control()
                self.control_active = False

        except Exception as e:
            self.consecutive_mpc_failures += 1
            self.get_logger().error(f"Control loop error (consecutive: {self.consecutive_mpc_failures}): {e}")

            if self.enable_logging:
                # Additional debug info
                self.get_logger().error(f"Current state available: {self.current_pose is not None}")
                self.get_logger().error(f"Path ready: {self.path_ready}")
                self.get_logger().error(
                    f"Reference trajectory length: {len(self.reference_trajectory) if hasattr(self, 'reference_trajectory') else 'None'}")
                self.get_logger().error(f"Horizon N: {self.horizon_N}")

            # Emergency reset after critical failures
            if self.consecutive_mpc_failures >= 10:
                if self.enable_logging:
                    self.get_logger().error("Critical MPC failures detected, performing emergency reset")
                self.mpc_controller.reset_performance_tracking()
                self.consecutive_mpc_failures = 0

            self._publish_emergency_stop()
            self.control_active = False

        # Track performance
        loop_time = time.time() - loop_start_time
        self.control_loop_times.append(loop_time)

        if len(self.control_loop_times) > 100:
            self.control_loop_times.pop(0)

    def _safety_checks(self):
        """Perform safety checks using parameters"""

        if not self.enable_safety_checks:
            return True

        current_time = self.get_clock().now()

        # Check for data timeouts
        if self.last_odom_time:
            odom_age = (current_time - self.last_odom_time).nanoseconds / 1e9
            if odom_age > self.safety_timeout:
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[SAFETY] Odometry timeout: {odom_age:.3f}s > {self.safety_timeout:.3f}s")
                return False

        if self.last_trajectory_time:
            traj_age = (current_time - self.last_trajectory_time).nanoseconds / 1e9
            if traj_age > self.safety_timeout:
                if self.debug_logging_enabled:
                    self.get_logger().error(f"[SAFETY] Trajectory timeout: {traj_age:.3f}s > {self.safety_timeout:.3f}s")
                return False

        # Check for excessive velocity
        velocity_threshold = self.emergency_brake_threshold * self.max_speed
        if self.current_velocity > velocity_threshold:
            if self.debug_logging_enabled:
                self.get_logger().error(f"[SAFETY] Excessive velocity: {self.current_velocity:.3f}m/s > {velocity_threshold:.3f}m/s")
            return False

        return True

    def _data_ready(self):
        """Check if all required data is available with enhanced validation"""

        checks = [
            self.current_pose is not None,
            self.state_initialized,
            self.first_odom_received,
            self.path_ready,
            len(self.reference_trajectory) >= 2,  # Need at least 2 points, will extend if needed
            self._validate_current_state()  # Additional numerical validation
        ]

        # Debug logging for data readiness
        if self.enable_logging or self.debug_logging_enabled:
            ready = all(checks)
            if not ready:
                failed_checks = []
                if not checks[0]:
                    failed_checks.append("no_pose")
                if not checks[1]:
                    failed_checks.append("not_initialized")
                if not checks[2]:
                    failed_checks.append("no_odom")
                if not checks[3]:
                    failed_checks.append("path_not_ready")
                if not checks[4]:
                    failed_checks.append(f"traj_too_short({len(self.reference_trajectory)})")
                if not checks[5]:
                    failed_checks.append("invalid_state")

                # Log more frequently when debug is enabled
                if self.debug_logging_enabled or self.log_counter % (self.log_frequency_divider * 5) == 0:
                    self.get_logger().warn(f"[DATA] Data not ready - Failed checks: {', '.join(failed_checks)}")
                    # Additional context
                    if not checks[3]:  # path_not_ready
                        self.get_logger().warn(f"[DATA] Path ready status: {self.path_ready}")
                    if not checks[4]:  # trajectory too short
                        self.get_logger().warn(f"[DATA] Trajectory length: {len(self.reference_trajectory)}, required: >=2")
                else:
                    self.get_logger().debug(f"Data not ready - Failed checks: {', '.join(failed_checks)}")

        return all(checks)

    def _solve_mpc_optimization(self):
        """Solve MPC optimization problem with robust numerical validation"""

        # Validate current state before solving
        if not self._validate_current_state():
            if self.enable_logging:
                self.get_logger().warn("Current state contains invalid values, cannot solve MPC")
            return {
                'success': False,
                'acceleration': 0.0,
                'steering': 0.0,
                'solve_time': 0.0,
                'error': 'Invalid current state'}

        # Prepare current state
        if self.mpc_type == MPCType.KINEMATIC:
            current_state = {
                'x': float(self.current_pose.position.x),
                'y': float(self.current_pose.position.y),
                'v': float(max(0.01, self.current_velocity)),  # Ensure minimum positive velocity
                'theta': float(self.current_yaw)
            }
        else:  # DYNAMIC
            current_state = {
                'x': float(self.current_pose.position.x),
                'y': float(self.current_pose.position.y),
                'v': float(max(0.01, self.current_velocity)),  # Ensure minimum positive velocity
                'theta': float(self.current_yaw),
                'beta': float(self.current_beta),
                'r': float(self.current_yaw_rate)
            }

        # Ensure reference trajectory has exactly the right size (N+1 points)
        if len(self.reference_trajectory) >= self.horizon_N + 1:
            reference_traj = self.reference_trajectory[:self.horizon_N + 1]
        else:
            # If we don't have enough points, extend the trajectory by repeating the last point
            reference_traj = list(self.reference_trajectory)  # Convert to list for easier manipulation

            if len(reference_traj) > 0:
                last_point = reference_traj[-1].copy()

                # Extend with the last point to reach N+1 total points
                while len(reference_traj) < self.horizon_N + 1:
                    reference_traj.append(last_point.copy())

                if self.enable_logging:
                    pass
                    #self.get_logger().info(
                       # f"Extended reference trajectory from {len(self.reference_trajectory)} to {len(reference_traj)} points")
            else:   
                # Fallback: create a trajectory with current state
                if self.mpc_type == MPCType.KINEMATIC:
                    fallback_point = [
                        current_state['x'],
                        current_state['y'],
                        max(0.1, current_state['v']),  # Ensure positive velocity
                        current_state['theta']
                    ]
                else:
                    fallback_point = [
                        current_state['x'],
                        current_state['y'],
                        max(0.1, current_state['v']),  # Ensure positive velocity
                        current_state['theta'],
                        0.0, 0.0
                    ]

                reference_traj = [fallback_point] * (self.horizon_N + 1)

                if self.enable_logging:
                    self.get_logger().warn(
                        f"Created fallback reference trajectory with {len(reference_traj)} points using current state")

        # Convert to numpy array if it isn't already
        reference_traj = np.array(reference_traj)

        # Final validation of reference trajectory
        if not np.all(np.isfinite(reference_traj)):
            if self.enable_logging:
                self.get_logger().error("Reference trajectory contains NaN or infinite values")
            return {'success': False, 'acceleration': 0.0, 'steering': 0.0,
                    'solve_time': 0.0, 'error': 'Invalid reference trajectory'}

        # Debug logging
        if self.enable_logging:
            self.get_logger().debug(
                f"Reference trajectory shape: {reference_traj.shape}, Expected: ({self.horizon_N + 1}, {4 if self.mpc_type == MPCType.KINEMATIC else 6})")
            self.get_logger().debug(f"Current state: {current_state}")
            # Log reference trajectory range for debugging
            if self.debug_logging_enabled and self.log_counter % self.log_frequency_divider == 0:
                self.get_logger().debug(
                    f"[DEBUG] Ref traj ranges - X: [{np.min(reference_traj[:, 0]):.2f}, {np.max(reference_traj[:, 0]):.2f}], "
                    f"Y: [{np.min(reference_traj[:, 1]):.2f}, {np.max(reference_traj[:, 1]):.2f}], "
                    f"V: [{np.min(reference_traj[:, 2]):.2f}, {np.max(reference_traj[:, 2]):.2f}]")
                self.get_logger().debug(f"[DEBUG] Current state: {current_state}")

        # Log solve attempt
        self.total_mpc_solves += 1
        self.log_counter += 1
        solve_start_time = time.time()

        # Solve MPC
        result = self.mpc_controller.solve_mpc(current_state, reference_traj)
        
        # Calculate solve time
        solve_time = time.time() - solve_start_time
        result['solve_time'] = solve_time
        
        # Update performance metrics
        self.total_solve_time += solve_time
        self.max_solve_time = max(self.max_solve_time, solve_time)
        self.min_solve_time = min(self.min_solve_time, solve_time)
        
        # Track recent solve times
        self.recent_solve_times.append(solve_time)
        if len(self.recent_solve_times) > 100:  # Keep last 100 solve times
            self.recent_solve_times.pop(0)
        
        # Log results based on success/failure
        if result.get('success', False):
            self.successful_mpc_solves += 1
            self.consecutive_mpc_failures = 0
            self.last_successful_solve_time = time.time()
            
            # Performance logging
            if self.performance_logging_enabled:
                if solve_time > self.slow_solve_threshold:
                    self.slow_solve_count += 1
                    self.get_logger().warn(f"[PERF] Slow MPC solve: {solve_time:.4f}s (threshold: {self.slow_solve_threshold:.3f}s)")
                
                # Log performance summary periodically
                if self.log_counter % (self.log_frequency_divider * 10) == 0:
                    avg_solve_time = self.total_solve_time / self.total_mpc_solves
                    success_rate = (self.successful_mpc_solves / self.total_mpc_solves) * 100
                    self.get_logger().info(
                        f"[PERF] MPC Performance Summary - Solves: {self.total_mpc_solves}, "
                        f"Success: {success_rate:.1f}%, Avg: {avg_solve_time:.4f}s, "
                        f"Min/Max: {self.min_solve_time:.4f}s/{self.max_solve_time:.4f}s, "
                        f"Slow solves: {self.slow_solve_count}")
            
            # Control logging
            if self.control_logging_enabled and self.log_counter % self.log_frequency_divider == 0:
                self.get_logger().info(
                    f"[CTRL] MPC #{self.total_mpc_solves}: ✅ Success in {solve_time:.4f}s, "
                    f"Accel: {result.get('acceleration', 0.0):.3f}, "
                    f"Steering: {result.get('steering', 0.0):.3f} rad ({np.degrees(result.get('steering', 0.0)):.1f}°)")
            
            # Track recent successful controls
            self.recent_controls.append({
                'time': time.time(),
                'acceleration': result.get('acceleration', 0.0),
                'steering': result.get('steering', 0.0),
                'solve_time': solve_time
            })
            if len(self.recent_controls) > 50:  # Keep last 50 controls
                self.recent_controls.pop(0)
                
        else:
            self.failed_mpc_solves += 1
            self.consecutive_mpc_failures += 1
            
            # Error logging
            error_msg = result.get('error', 'Unknown error')
            self.get_logger().error(
                f"[FAIL] MPC #{self.total_mpc_solves}: ❌ Failed in {solve_time:.4f}s, "
                f"Consecutive failures: {self.consecutive_mpc_failures}, Error: {error_msg}")
            
            # Log debugging information on failure
            if self.debug_logging_enabled:
                self.get_logger().error(f"[DEBUG] Failed state: {current_state}")
                self.get_logger().error(f"[DEBUG] Ref traj shape: {reference_traj.shape}")
                if hasattr(self, 'mpc_controller') and hasattr(self.mpc_controller, 'get_debug_info'):
                    debug_info = self.mpc_controller.get_debug_info()
                    self.get_logger().error(f"[DEBUG] Controller state: {debug_info}")

        return result

    def _validate_current_state(self):
        """Validate current vehicle state for numerical stability"""
        try:
            if self.current_pose is None:
                return False

            # Check position
            pos_x = self.current_pose.position.x
            pos_y = self.current_pose.position.y
            if not np.isfinite(pos_x) or not np.isfinite(pos_y):
                return False
            if abs(pos_x) > 1000 or abs(pos_y) > 1000:
                return False

            # Check velocity
            if not np.isfinite(self.current_velocity):
                return False
            if self.current_velocity < -10 or self.current_velocity > 50:
                return False

            # Check yaw
            if not np.isfinite(self.current_yaw):
                return False

            # Check additional states for dynamic model
            if self.mpc_type == MPCType.DYNAMIC:
                if not np.isfinite(self.current_beta) or not np.isfinite(self.current_yaw_rate):
                    return False
                if abs(self.current_beta) > np.pi / 2 or abs(self.current_yaw_rate) > 10:
                    return False

            return True
        except BaseException:
            return False

    def _publish_control_command(self, acceleration, steering_angle):
        """Publish control command with safety limits from parameters"""

        # Apply safety limits from parameters
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)

        # Convert acceleration to velocity command
        target_velocity = self.current_velocity + acceleration * (1.0 / self.control_hz)
        target_velocity = np.clip(target_velocity, self.min_speed, self.max_speed)

        # Create and publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = float(target_velocity)
        drive_msg.drive.steering_angle = float(steering_angle)

        self.control_publisher.publish(drive_msg)
        self.current_steering_angle = steering_angle

    def _publish_zero_control(self):
        """Publish zero control command"""
        # Create zero control message directly instead of using _publish_control_command
        # to avoid min_speed clipping
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0

        self.control_publisher.publish(drive_msg)

        if self.enable_logging:
            self.get_logger().debug("Published zero control command")

    def _publish_emergency_stop(self):
        """Publish emergency stop command"""

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = 0.0
        drive_msg.drive.steering_angle = 0.0

        self.control_publisher.publish(drive_msg)
        
        if not self.emergency_stop:  # Only log and set time on first activation
            self.emergency_stop = True
            self.emergency_stop_time = time.time()
            self.get_logger().warn("🚨 EMERGENCY STOP ACTIVATED")
        elif self.debug_logging_enabled and self.log_counter % (self.log_frequency_divider * 2) == 0:
            # Reduced frequency logging when already in emergency stop
            time_in_emergency = time.time() - (self.emergency_stop_time or 0)
            self.get_logger().warn(f"🚨 EMERGENCY STOP ACTIVE ({time_in_emergency:.1f}s)")

    def _publish_diagnostics(self):
        """Publish diagnostics information with all parameters"""

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # MPC Controller Status
        mpc_status = DiagnosticStatus()
        mpc_status.name = "Optimized MPC Controller"
        mpc_status.hardware_id = "optimized_mpc_node"

        if self.control_active and not self.emergency_stop:
            mpc_status.level = DiagnosticStatus.OK
            mpc_status.message = f"Optimized MPC ({self.mpc_type.value}) operating normally with all parameters"
        elif self.emergency_stop:
            mpc_status.level = DiagnosticStatus.ERROR
            mpc_status.message = "Emergency stop active"
        else:
            mpc_status.level = DiagnosticStatus.WARN
            mpc_status.message = "Optimized MPC controller inactive"

        # Add performance metrics
        perf_stats = self.mpc_controller.get_performance_stats()
        if perf_stats:
            mpc_status.values.extend([
                KeyValue(key="avg_solve_time", value=f"{perf_stats['avg_solve_time']:.4f}"),
                KeyValue(key="success_rate", value=f"{perf_stats['success_rate']:.3f}"),
                KeyValue(key="real_time_factor", value=f"{perf_stats['real_time_factor']:.1f}"),
                KeyValue(key="total_iterations", value=str(perf_stats['total_iterations']))
            ])

        # Add parameter status
        mpc_status.values.extend([
            KeyValue(key="mpc_type", value=self.mpc_type.value),
            KeyValue(key="horizon_N", value=str(self.horizon_N)),
            KeyValue(key="max_speed", value=f"{self.max_speed:.2f}"),
            KeyValue(key="max_steering_angle", value=f"{self.max_steering_angle:.3f}"),
            KeyValue(key="safety_checks", value=str(self.enable_safety_checks)),
            KeyValue(key="obstacle_avoidance", value=str(self.enable_obstacle_avoidance)),
            KeyValue(key="cost_weights_enabled", value=str(self.enable_cost_function_weights)),
            KeyValue(key="trajectory_tracking", value=str(self.enable_trajectory_tracking))
        ])

        # Add pose estimate diagnostic info
        if self.rviz_pose_estimate is not None and self.current_pose is not None:
            pos_diff_x = self.rviz_pose_estimate.position.x - self.current_pose.position.x
            pos_diff_y = self.rviz_pose_estimate.position.y - self.current_pose.position.y
            pos_diff_magnitude = np.sqrt(pos_diff_x**2 + pos_diff_y**2)

            rviz_orientation = self.rviz_pose_estimate.orientation
            _, _, rviz_yaw = euler_from_quaternion([
                rviz_orientation.x, rviz_orientation.y, rviz_orientation.z, rviz_orientation.w
            ])
            yaw_diff = np.abs(rviz_yaw - self.current_yaw)
            if yaw_diff > np.pi:
                yaw_diff = 2 * np.pi - yaw_diff

            mpc_status.values.extend([
                KeyValue(key="pose_diff_magnitude", value=f"{pos_diff_magnitude:.3f}"),
                KeyValue(key="heading_diff_deg", value=f"{np.degrees(yaw_diff):.1f}"),
                KeyValue(key="rviz_pose_available", value="true")
            ])
        else:
            mpc_status.values.append(KeyValue(key="rviz_pose_available", value="false"))

        diag_array.status = [mpc_status]
        self.diagnostics_publisher.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MPCNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🏁 Optimized MPC Node shutting down...")
    except Exception as e:
        print(f"❌ Fatal error in Optimized MPC Node: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
