#!/usr/bin/env python3

"""
F1TENTH Adaptive MPC Controlle        super().__init__('adaptive_mpc_node')

        self.get_logger().info("🚗 Starting Adaptive MPC Node...")

        # Callback groups for concurrent processing
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize node
        self._declare_all_parameters()
        self._load_all_parameters()
        self._initialize_adaptive_mpc()
        self._initialize_state()
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()
        self._setup_tf()

        self.get_logger().info("🚗 Adaptive MPC Node has been started! 🏎️") node implements an intelligent, self-adapting Model Predictive Controller
for F1TENTH autonomous racing that dynamically adjusts parameters based on real-time
conditions including tracking error, obstacle proximity, and track characteristics.

Key Features:
    - Dynamic parameter adaptation based on real-time metrics
    - LiDAR-based obstacle awareness
    - Path/waypoint following with adaptive horizon
    - Real-time parameter updates via ROS2 parameter callbacks
    - Comprehensive diagnostics and monitoring

Author: Mohammed Azab <mohammed@azab.io>
License: MIT
Version: 3.0.0
"""

import rclpy
import numpy as np
import time
import traceback
from typing import Dict, List, Optional, Any
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Message types
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, Float32, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from visualization_msgs.msg import Marker, MarkerArray
from giu_f1t_interfaces.msg import VehicleStateArray

# TF
from tf_transformations import euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs

try:
    from .adaptive_mpc_controller import AdaptiveMPCController, AdaptiveParameters, TrackingMetrics, EnvironmentMetrics
    from .kinematic_bicycle_model import MPCType
except ImportError:
    from adaptive_mpc_controller import AdaptiveMPCController, AdaptiveParameters, TrackingMetrics, EnvironmentMetrics
    from kinematic_bicycle_model import MPCType


class AdaptiveMPCNode(Node):
    """ROS2 Node for Adaptive MPC Controller"""
    
    def __init__(self):
        super().__init__('adaptive_mpc_node')

        # Callback groups for concurrent processing
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize node
        self._declare_all_parameters()
        self._load_all_parameters()
        self._initialize_adaptive_mpc()
        self._initialize_state()
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()
        self._setup_tf()

        self.get_logger().info("Adaptive MPC Node has been started! 🏎️")

    def _declare_all_parameters(self):
        """Declare all ROS2 parameters with adaptive extensions"""

        self.get_logger().info("Declaring adaptive MPC parameters")

        # Base parameters from original controller
        self.declare_parameter('wheelbase', 0.33)
        self.declare_parameter('horizon_N', 25)
        self.declare_parameter('horizon_T', 0.3)
        self.declare_parameter('lookahead_distance', 0.3)
        
        # Vehicle limits
        self.declare_parameter('max_steering_angle', 0.5)
        self.declare_parameter('max_acceleration', 5.0)
        self.declare_parameter('max_deceleration', 7.0)
        self.declare_parameter('min_speed', 0.1)
        self.declare_parameter('max_speed', 8.0)

        # Cost function weights
        self.declare_parameter('enable_cost_function_weights', True)
        self.declare_parameter('cost_function_weights.steering_weight', 0.7)
        self.declare_parameter('cost_function_weights.acceleration_weight', 0.5)
        self.declare_parameter('cost_function_weights.jerk_weight', 0.1)
        self.declare_parameter('cost_function_weights.heading_weight', 2.0)
        self.declare_parameter('cost_function_weights.position_weight', 5.0)
        self.declare_parameter('cost_function_weights.velocity_weight', 1.0)

        # Adaptive parameters
        self.declare_parameter('enable_adaptation', True)
        self.declare_parameter('adaptation_rate', 0.1)
        self.declare_parameter('adaptation_interval', 0.1)
        
        # Adaptation bounds
        self.declare_parameter('adaptive_bounds.horizon_N_min', 5)
        self.declare_parameter('adaptive_bounds.horizon_N_max', 50)
        self.declare_parameter('adaptive_bounds.horizon_T_min', 0.1)
        self.declare_parameter('adaptive_bounds.horizon_T_max', 1.0)
        
        # Weight adaptation bounds
        self.declare_parameter('adaptive_bounds.position_weight_min', 1.0)
        self.declare_parameter('adaptive_bounds.position_weight_max', 20.0)
        self.declare_parameter('adaptive_bounds.heading_weight_min', 0.5)
        self.declare_parameter('adaptive_bounds.heading_weight_max', 10.0)
        self.declare_parameter('adaptive_bounds.velocity_weight_min', 0.1)
        self.declare_parameter('adaptive_bounds.velocity_weight_max', 5.0)
        self.declare_parameter('adaptive_bounds.steering_weight_min', 0.1)
        self.declare_parameter('adaptive_bounds.steering_weight_max', 2.0)
        self.declare_parameter('adaptive_bounds.acceleration_weight_min', 0.1)
        self.declare_parameter('adaptive_bounds.acceleration_weight_max', 2.0)

        # MPC parameters
        self.declare_parameter('mpc_type', 'kinematic')
        self.declare_parameter('solver_type', 'ipopt')
        self.declare_parameter('control_hz', 15.0)

        # Safety parameters
        self.declare_parameter('enable_safety_checks', True)
        self.declare_parameter('safety_timeout', 1.0)
        self.declare_parameter('emergency_brake_threshold', 2.0)
        self.declare_parameter('obstacle_safety_distance', 0.5)

        # Topics
        self.declare_parameter('odom_topic', '/car_state/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('waypoints_topic', '/waypoints')
        self.declare_parameter('global_plan_topic', '/global_plan')
        self.declare_parameter('reference_topic', '/horizon_mapper/reference_trajectory')
        self.declare_parameter('status_topic', '/horizon_mapper/path_ready')
        self.declare_parameter('control_topic', '/drive')

        # Publishing options
        self.declare_parameter('publish_diagnostics', True)
        self.declare_parameter('publish_adaptation_status', True)
        self.declare_parameter('publish_predicted_path', True)
        
        # Debug and logging
        self.declare_parameter('debug_logging_enabled', False)
        self.declare_parameter('performance_logging_enabled', True)
        self.declare_parameter('adaptation_logging_enabled', True)

    def _load_all_parameters(self):
        """Load all parameters from ROS2 parameter server"""
        
        # Base vehicle parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.horizon_N = self.get_parameter('horizon_N').value
        self.horizon_T = self.get_parameter('horizon_T').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value

        # Vehicle limits
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_acceleration = self.get_parameter('max_acceleration').value
        self.max_deceleration = self.get_parameter('max_deceleration').value
        self.min_speed = self.get_parameter('min_speed').value
        self.max_speed = self.get_parameter('max_speed').value

        # Cost weights
        self.enable_cost_function_weights = self.get_parameter('enable_cost_function_weights').value
        self.cost_function_weights = {
            'steering_weight': self.get_parameter('cost_function_weights.steering_weight').value,
            'acceleration_weight': self.get_parameter('cost_function_weights.acceleration_weight').value,
            'jerk_weight': self.get_parameter('cost_function_weights.jerk_weight').value,
            'heading_weight': self.get_parameter('cost_function_weights.heading_weight').value,
            'position_weight': self.get_parameter('cost_function_weights.position_weight').value,
            'velocity_weight': self.get_parameter('cost_function_weights.velocity_weight').value,
        }

        # Adaptive parameters
        self.enable_adaptation = self.get_parameter('enable_adaptation').value
        self.adaptation_rate = self.get_parameter('adaptation_rate').value
        self.adaptation_interval = self.get_parameter('adaptation_interval').value

        # MPC settings
        self.mpc_type = self.get_parameter('mpc_type').value
        self.solver_type = self.get_parameter('solver_type').value
        self.control_hz = self.get_parameter('control_hz').value

        # Safety
        self.enable_safety_checks = self.get_parameter('enable_safety_checks').value
        self.safety_timeout = self.get_parameter('safety_timeout').value
        self.emergency_brake_threshold = self.get_parameter('emergency_brake_threshold').value
        self.obstacle_safety_distance = self.get_parameter('obstacle_safety_distance').value

        # Topics
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.waypoints_topic = self.get_parameter('waypoints_topic').value
        self.global_plan_topic = self.get_parameter('global_plan_topic').value
        self.reference_topic = self.get_parameter('reference_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.control_topic = self.get_parameter('control_topic').value

        # Publishing flags
        self.publish_diagnostics = self.get_parameter('publish_diagnostics').value
        self.publish_adaptation_status = self.get_parameter('publish_adaptation_status').value
        self.publish_predicted_path = self.get_parameter('publish_predicted_path').value

        # Debug settings
        self.debug_logging_enabled = self.get_parameter('debug_logging_enabled').value
        self.performance_logging_enabled = self.get_parameter('performance_logging_enabled').value
        self.adaptation_logging_enabled = self.get_parameter('adaptation_logging_enabled').value

    def _initialize_adaptive_mpc(self):
        """Initialize the adaptive MPC controller"""
        
        # Prepare initial parameters
        initial_params = {
            'N': self.horizon_N,
            'T': self.horizon_T,
            'wheelbase': self.wheelbase,
            'max_speed': self.max_speed,
            'max_steering_angle': self.max_steering_angle,
            'max_acceleration': self.max_acceleration,
            'max_deceleration': self.max_deceleration,
            'min_speed': self.min_speed,
            'mpc_type': MPCType.KINEMATIC if self.mpc_type == 'kinematic' else MPCType.DYNAMIC,
            'solver_type': self.solver_type,
            'enable_safety_checks': self.enable_safety_checks,
            'cost_function_weights': self.cost_function_weights
        }

        # Initialize adaptive MPC
        self.adaptive_mpc = AdaptiveMPCController(
            initial_params=initial_params,
            adaptation_rate=self.adaptation_rate,
            enable_adaptation=self.enable_adaptation
        )

        # Set up adaptive parameter bounds
        self._setup_adaptive_bounds()

        self.get_logger().info("✅ Adaptive MPC controller initialized")
        self.get_logger().info(f"📡 Subscribing to:")
        self.get_logger().info(f"   - Odometry: {self.odom_topic}")
        self.get_logger().info(f"   - LiDAR: {self.scan_topic}")
        self.get_logger().info(f"   - Reference: {self.reference_topic}")
        self.get_logger().info(f"   - Path status: {self.status_topic}")
        self.get_logger().info(f"📤 Publishing to: {self.control_topic}")

    def _setup_adaptive_bounds(self):
        """Set up adaptive parameter bounds from ROS parameters"""
        bounds = self.adaptive_mpc.adaptive_params
        
        # Horizon bounds
        bounds.horizon_N_range = (
            self.get_parameter('adaptive_bounds.horizon_N_min').value,
            self.get_parameter('adaptive_bounds.horizon_N_max').value
        )
        bounds.horizon_T_range = (
            self.get_parameter('adaptive_bounds.horizon_T_min').value,
            self.get_parameter('adaptive_bounds.horizon_T_max').value
        )
        
        # Weight bounds
        bounds.position_weight_range = (
            self.get_parameter('adaptive_bounds.position_weight_min').value,
            self.get_parameter('adaptive_bounds.position_weight_max').value
        )
        bounds.heading_weight_range = (
            self.get_parameter('adaptive_bounds.heading_weight_min').value,
            self.get_parameter('adaptive_bounds.heading_weight_max').value
        )
        bounds.velocity_weight_range = (
            self.get_parameter('adaptive_bounds.velocity_weight_min').value,
            self.get_parameter('adaptive_bounds.velocity_weight_max').value
        )
        bounds.steering_weight_range = (
            self.get_parameter('adaptive_bounds.steering_weight_min').value,
            self.get_parameter('adaptive_bounds.steering_weight_max').value
        )
        bounds.acceleration_weight_range = (
            self.get_parameter('adaptive_bounds.acceleration_weight_min').value,
            self.get_parameter('adaptive_bounds.acceleration_weight_max').value
        )

    def _initialize_state(self):
        """Initialize state variables"""
        
        # Current vehicle state
        self.current_state = {
            'x': 0.0,
            'y': 0.0,
            'v': 0.0,
            'theta': 0.0,
            'timestamp': 0.0
        }
        
        # Reference trajectory
        self.reference_trajectory = np.array([])
        self.waypoints = []
        self.global_plan = []
        
        # Sensor data
        self.lidar_data = []
        self.last_scan_time = 0.0
        
        # Track information
        self.track_info = {
            'curvature': 0.0,
            'width': 1.0,
            'speed': 0.0
        }
        
        # Status flags
        self.path_ready = False
        self.odometry_received = False
        self.first_control_command = True
        
        # Performance tracking
        self.last_control_time = 0.0
        self.control_computation_times = []
        self.adaptation_events = []

    def _setup_subscriptions(self):
        """Set up ROS2 subscriptions"""
        
        # Odometry subscription
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
            callback_group=self.sensor_callback_group
        )
        
        # LiDAR subscription
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
            callback_group=self.sensor_callback_group
        )
        
        # Waypoints subscription (disabled - no publisher available)
        # self.waypoints_sub = self.create_subscription(
        #     PoseArray,
        #     self.waypoints_topic,
        #     self.waypoints_callback,
        #     10,
        #     callback_group=self.sensor_callback_group
        # )
        
        # Global plan subscription (nav_msgs/Path from horizon_mapper)
        self.global_plan_sub = self.create_subscription(
            Path,
            self.global_plan_topic,
            self.global_plan_callback,
            10,
            callback_group=self.sensor_callback_group
        )
        
        # Reference trajectory subscription (VehicleStateArray from horizon mapper)
        self.reference_sub = self.create_subscription(
            VehicleStateArray,
            self.reference_topic,
            self.reference_trajectory_callback,
            10,
            callback_group=self.sensor_callback_group
        )
        
        # Path status subscription
        self.status_sub = self.create_subscription(
            Bool,
            self.status_topic,
            self.status_callback,
            10
        )

    def _setup_publishers(self):
        """Set up ROS2 publishers"""
        
        # Control command publisher
        self.control_pub = self.create_publisher(
            AckermannDriveStamped,
            self.control_topic,
            10
        )
        
        # Diagnostics publisher
        if self.publish_diagnostics:
            self.diagnostics_pub = self.create_publisher(
                DiagnosticArray,
                '/diagnostics',
                10
            )
        
        # Adaptation status publisher
        if self.publish_adaptation_status:
            self.adaptation_status_pub = self.create_publisher(
                DiagnosticArray,
                '/adaptive_mpc/status',
                10
            )
        
        # Predicted path publisher
        if self.publish_predicted_path:
            self.predicted_path_pub = self.create_publisher(
                Path,
                '/adaptive_mpc/predicted_path',
                10
            )
            
        # Visualization publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/adaptive_mpc/visualization',
            10
        )

    def _setup_timers(self):
        """Set up periodic timers"""
        
        # Main control timer
        control_period = 1.0 / self.control_hz
        self.control_timer = self.create_timer(
            control_period,
            self.control_callback,
            callback_group=self.control_callback_group
        )
        
        # Diagnostics timer
        if self.publish_diagnostics:
            self.diagnostics_timer = self.create_timer(
                0.5,  # 2 Hz
                self.publish_diagnostics_callback
            )
        
        # Adaptation status timer
        if self.publish_adaptation_status:
            self.adaptation_status_timer = self.create_timer(
                0.2,  # 5 Hz
                self.publish_adaptation_status_callback
            )

    def _setup_tf(self):
        """Set up TF buffer and listener"""
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg: Odometry):
        """Handle odometry messages"""
        try:
            # Extract position
            self.current_state['x'] = msg.pose.pose.position.x
            self.current_state['y'] = msg.pose.pose.position.y
            
            # Extract orientation
            orientation = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
            self.current_state['theta'] = yaw
            
            # Extract velocity
            linear_vel = msg.twist.twist.linear
            self.current_state['v'] = np.sqrt(linear_vel.x**2 + linear_vel.y**2)
            
            # Update timestamp
            self.current_state['timestamp'] = time.time()
            
            # Update track info with current speed
            self.track_info['speed'] = self.current_state['v']
            
            self.odometry_received = True
            
        except Exception as e:
            self.get_logger().error(f"Error in odometry callback: {e}")

    def scan_callback(self, msg: LaserScan):
        """Handle LiDAR scan messages"""
        try:
            # Store raw LiDAR data
            self.lidar_data = list(msg.ranges)
            self.last_scan_time = time.time()
            
            # Update track width estimation from LiDAR
            valid_ranges = [r for r in self.lidar_data if not np.isinf(r) and not np.isnan(r) and r > 0.1]
            if len(valid_ranges) > 10:
                # Simple track width estimation
                left_ranges = valid_ranges[:len(valid_ranges)//4]
                right_ranges = valid_ranges[-len(valid_ranges)//4:]
                
                if left_ranges and right_ranges:
                    avg_left = np.mean(left_ranges)
                    avg_right = np.mean(right_ranges)
                    self.track_info['width'] = avg_left + avg_right
            
        except Exception as e:
            self.get_logger().error(f"Error in scan callback: {e}")

    def waypoints_callback(self, msg: PoseArray):
        """Handle waypoints messages"""
        try:
            self.waypoints = []
            for pose in msg.poses:
                waypoint = [
                    pose.position.x,
                    pose.position.y,
                    0.0,  # Default speed, will be set by planner
                    euler_from_quaternion([
                        pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w
                    ])[2]
                ]
                self.waypoints.append(waypoint)
            
            if self.waypoints:
                self.reference_trajectory = np.array(self.waypoints)
                self._compute_track_curvature()
                
        except Exception as e:
            self.get_logger().error(f"Error in waypoints callback: {e}")

    def global_plan_callback(self, msg: Path):
        """Handle global plan messages"""
        try:
            self.global_plan = []
            for pose_stamped in msg.poses:
                pose = pose_stamped.pose
                waypoint = [
                    pose.position.x,
                    pose.position.y,
                    0.0,  # Default speed
                    euler_from_quaternion([
                        pose.orientation.x, pose.orientation.y,
                        pose.orientation.z, pose.orientation.w
                    ])[2]
                ]
                self.global_plan.append(waypoint)
            
            if self.global_plan:
                self.reference_trajectory = np.array(self.global_plan)
                self._compute_track_curvature()
                
        except Exception as e:
            self.get_logger().error(f"Error in global plan callback: {e}")

    def reference_trajectory_callback(self, msg: VehicleStateArray):
        """Handle VehicleStateArray reference trajectory messages from horizon mapper"""
        try:
            reference_points = []
            for vehicle_state in msg.states:  # Changed from msg.vehicle_states to msg.states
                point = [
                    vehicle_state.x,
                    vehicle_state.y,
                    vehicle_state.v,  # Velocity
                    vehicle_state.theta  # Heading
                ]
                reference_points.append(point)
            
            if reference_points:
                self.reference_trajectory = np.array(reference_points)
                self._compute_track_curvature()
                if len(reference_points) > 0:
                    self.get_logger().debug(f"Received reference trajectory with {len(reference_points)} points")
                
        except Exception as e:
            self.get_logger().error(f"Error in reference trajectory callback: {e}")

    def status_callback(self, msg: Bool):
        """Handle path status messages"""
        self.path_ready = msg.data

    def _compute_track_curvature(self):
        """Compute track curvature from reference trajectory"""
        if len(self.reference_trajectory) < 3:
            self.track_info['curvature'] = 0.0
            return
        
        try:
            # Simple curvature estimation using three points
            points = self.reference_trajectory[:3]
            
            # Vectors
            v1 = points[1] - points[0]
            v2 = points[2] - points[1]
            
            # Cross product magnitude gives curvature approximation
            cross_prod = abs(v1[0] * v2[1] - v1[1] * v2[0])
            v1_mag = np.linalg.norm(v1[:2])
            v2_mag = np.linalg.norm(v2[:2])
            
            if v1_mag > 0 and v2_mag > 0:
                curvature = cross_prod / (v1_mag * v2_mag)
                self.track_info['curvature'] = curvature
            else:
                self.track_info['curvature'] = 0.0
                
        except Exception as e:
            self.track_info['curvature'] = 0.0

    def control_callback(self):
        """Main control loop callback"""
        if not self.odometry_received:
            self.get_logger().debug("Waiting for odometry...")
            return
            
        if len(self.reference_trajectory) == 0:
            self.get_logger().debug("Waiting for reference trajectory...")
            return
        
        start_time = time.time()
        
        try:
            # Solve adaptive MPC
            result = self.adaptive_mpc.solve_mpc(
                current_state=self.current_state,
                reference_trajectory=self.reference_trajectory,
                lidar_data=self.lidar_data,
                track_info=self.track_info
            )
            
            if result.get('success', False):
                # Create and publish control command
                control_msg = AckermannDriveStamped()
                control_msg.header.stamp = self.get_clock().now().to_msg()
                control_msg.header.frame_id = 'base_link'
                
                control_msg.drive.steering_angle = float(result.get('steering', 0.0))
                control_msg.drive.acceleration = float(result.get('acceleration', 0.0))
                control_msg.drive.speed = float(result.get('speed', 0.0))
                
                self.control_pub.publish(control_msg)
                
                # Log adaptation events
                if result.get('adapted', False) and self.adaptation_logging_enabled:
                    self.get_logger().info(
                        f"🔄 Adapted: N={result.get('current_horizon_N', 0)}, "
                        f"T={result.get('current_horizon_T', 0.0):.3f}, "
                        f"Error={result.get('tracking_error', 0.0):.3f}"
                    )
                
                # Track performance
                solve_time = time.time() - start_time
                self.control_computation_times.append(solve_time)
                if len(self.control_computation_times) > 100:
                    self.control_computation_times.pop(0)
                
            else:
                # Emergency stop
                self._publish_emergency_stop()
                self.get_logger().warn("⚠️ MPC solve failed, emergency stop activated")
            
        except Exception as e:
            self.get_logger().error(f"Error in control callback: {e}")
            self._publish_emergency_stop()

    def _publish_emergency_stop(self):
        """Publish emergency stop command"""
        control_msg = AckermannDriveStamped()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = 'base_link'
        control_msg.drive.steering_angle = 0.0
        control_msg.drive.acceleration = -self.max_deceleration
        control_msg.drive.speed = 0.0
        
        self.control_pub.publish(control_msg)

    def publish_diagnostics_callback(self):
        """Publish diagnostics information"""
        if not self.publish_diagnostics:
            return
            
        try:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Main MPC status
            mpc_status = DiagnosticStatus()
            mpc_status.name = "Adaptive MPC Controller"
            mpc_status.level = DiagnosticStatus.OK
            mpc_status.message = "Running"
            
            # Add performance metrics
            if self.control_computation_times:
                avg_time = np.mean(self.control_computation_times)
                max_time = np.max(self.control_computation_times)
                
                mpc_status.values.append(KeyValue(key="avg_solve_time", value=f"{avg_time:.4f}"))
                mpc_status.values.append(KeyValue(key="max_solve_time", value=f"{max_time:.4f}"))
                mpc_status.values.append(KeyValue(key="control_frequency", value=f"{1.0/avg_time:.1f}"))
            
            mpc_status.values.append(KeyValue(key="adaptation_enabled", value=str(self.enable_adaptation)))
            mpc_status.values.append(KeyValue(key="odometry_received", value=str(self.odometry_received)))
            mpc_status.values.append(KeyValue(key="path_ready", value=str(self.path_ready)))
            
            diag_array.status.append(mpc_status)
            self.diagnostics_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing diagnostics: {e}")

    def publish_adaptation_status_callback(self):
        """Publish adaptation status information"""
        if not self.publish_adaptation_status:
            return
            
        try:
            adaptation_status = self.adaptive_mpc.get_adaptation_status()
            
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Adaptation status
            adapt_status = DiagnosticStatus()
            adapt_status.name = "Adaptive Parameters"
            adapt_status.level = DiagnosticStatus.OK
            adapt_status.message = "Active" if self.enable_adaptation else "Disabled"
            
            # Current adaptive parameters
            params = adaptation_status['adaptive_params']
            for key, value in params.items():
                adapt_status.values.append(KeyValue(key=key, value=f"{value:.3f}"))
            
            # Current metrics
            metrics = adaptation_status['metrics']
            for key, value in metrics.items():
                adapt_status.values.append(KeyValue(key=f"metric_{key}", value=f"{value:.3f}"))
            
            # Performance stats
            performance = adaptation_status['performance']
            for key, value in performance.items():
                adapt_status.values.append(KeyValue(key=f"perf_{key}", value=str(value)))
            
            diag_array.status.append(adapt_status)
            self.adaptation_status_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing adaptation status: {e}")

    def parameter_callback(self, params):
        """Handle parameter updates"""
        successful_params = []
        
        for param in params:
            try:
                if param.name == 'enable_adaptation':
                    self.enable_adaptation = param.value
                    self.adaptive_mpc.set_adaptation_enabled(param.value)
                    successful_params.append(param)
                    
                elif param.name == 'adaptation_rate':
                    self.adaptation_rate = param.value
                    self.adaptive_mpc.adaptation_rate = param.value
                    successful_params.append(param)
                    
                # Add more parameter handling as needed
                    
            except Exception as e:
                self.get_logger().error(f"Failed to update parameter {param.name}: {e}")
        
        return successful_params


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        # Create node
        adaptive_mpc_node = AdaptiveMPCNode()
        
        # Use MultiThreadedExecutor for concurrent processing
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(adaptive_mpc_node)
        
        # Add parameter callback
        adaptive_mpc_node.add_on_set_parameters_callback(adaptive_mpc_node.parameter_callback)
        
        adaptive_mpc_node.get_logger().info("🏁 Adaptive MPC Node ready for racing!")
        
        # Spin
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
        traceback.print_exc()
    finally:
        # Cleanup
        if 'adaptive_mpc_node' in locals():
            adaptive_mpc_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
