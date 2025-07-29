from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Bool
from giu_f1t_interfaces.msg import VehicleState, VehicleStateArray

from .preprocess_trajectory import preprocess_trajectory

import rclpy
from rclpy.node import Node
import csv
import math
import os
import numpy as np
from tf_transformations import euler_from_quaternion


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



class TrajectoryPublisherNode(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')

        # Declare ROS2 parameters
        self.declare_parameter('enable_logging', default_config.enable_logging)
        self.declare_parameter('optimal_trajectory_path', default_config.optimal_trajectory_path)
        self.declare_parameter('reference_trajectory_path', default_config.reference_trajectory_path)
        self.declare_parameter('horizon_N', default_config.horizon_N)
        self.declare_parameter('wheelbase', default_config.wheelbase)
        self.declare_parameter('max_steering_angle', default_config.max_steering_angle)
        self.declare_parameter('min_speed', default_config.min_speed)

        # Load parameters from ROS2 parameter server
        self.enable_logging = self.get_parameter('enable_logging').value
        self.input_path = self.get_parameter('optimal_trajectory_path').value
        self.output_path = self.get_parameter('reference_trajectory_path').value
        self.horizon = self.get_parameter('horizon_N').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_steering = self.get_parameter('max_steering_angle').value
        self.min_speed = self.get_parameter('min_speed').value

        if self.enable_logging:
            self.get_logger().info("Detailed logging enabled")
        else:
            self.get_logger().info("Detailed logging disabled - only warnings and errors will be shown")

        if not self.input_path or not self.output_path:
            self.get_logger().error("Missing trajectory paths in parameters!")
            return

        self.get_logger().info(f"Input trajectory: {self.input_path}")
        self.get_logger().info(f"Output trajectory: {self.output_path}")
        self.get_logger().info(f"Horizon: {self.horizon} steps")

        # State variables
        self.reference_trajectory = []
        self.path_ready = False
        self.current_vehicle_state = VehicleState()
        self.current_pose_estimate = None
        self.trajectory_index = 0

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/car_state_odom',
            self.odometry_callback,
            10)

        # Subscribe to RViz 2D Pose Estimate
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_estimate_callback,
            10)

        # Publishers
        self.path_pub = self.create_publisher(
            Path,
            '/mpc/reference_path',
            10)

        self.trajectory_pub = self.create_publisher(
            VehicleStateArray,
            '/mpc/reference_trajectory',
            10)

        self.status_pub = self.create_publisher(
            Bool,
            '/mpc/path_ready',
            10)

        # Timers
        self.publish_timer = self.create_timer(0.1, self.publish_predictive_trajectory)  # 10 Hz for MPC

        # Run preprocessing on startup
        self.preprocess_and_load_trajectory()

    def log_info(self, message):
        """Log info messages only if logging is enabled"""
        if self.enable_logging:
            self.get_logger().info(message)

    def log_debug(self, message):
        """Log debug messages only if logging is enabled"""
        if self.enable_logging:
            self.get_logger().debug(message)

    def odometry_callback(self, msg):
        """Update current vehicle state from odometry"""
        try:
            # Extract position
            self.current_vehicle_state.x = msg.pose.pose.position.x
            self.current_vehicle_state.y = msg.pose.pose.position.y

            # Extract velocity (assuming this is in body frame)
            linear_velocity = msg.twist.twist.linear
            self.current_vehicle_state.v = np.sqrt(linear_velocity.x**2 + linear_velocity.y**2)

            # Extract yaw from quaternion (for future steering calculations)
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])

            # For now, set delta to 0 (we'll calculate this from trajectory tracking)
            self.current_vehicle_state.delta = 0.0

            self.log_debug(f"Odometry update: x={self.current_vehicle_state.x:.2f}, "
                           f"y={self.current_vehicle_state.y:.2f}, "
                           f"v={self.current_vehicle_state.v:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {str(e)}")

    def pose_estimate_callback(self, msg):
        """Update pose estimate from RViz 2D Pose Estimate tool"""
        try:
            self.current_pose_estimate = msg

            # Use pose estimate to correct vehicle state
            self.current_vehicle_state.x = msg.pose.pose.position.x
            self.current_vehicle_state.y = msg.pose.pose.position.y

            # Extract yaw from quaternion for heading
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])

            self.log_info(f"Pose estimate from RViz: x={self.current_vehicle_state.x:.2f}, "
                          f"y={self.current_vehicle_state.y:.2f}, yaw={yaw:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error processing pose estimate: {str(e)}")

    def preprocess_and_load_trajectory(self):
        """Run preprocessing script and load trajectory"""
        try:
            self.log_info("Starting trajectory preprocessing...")

            # Run the preprocessing script with parameters
            success = preprocess_trajectory(
                self.input_path,
                self.output_path,
                wheelbase=self.wheelbase,
                max_steering=self.max_steering,
                min_velocity=self.min_speed
            )

            if success:
                self.log_info("Preprocessing completed successfully")
                self.reference_trajectory = self.load_trajectory_from_csv(self.output_path)
                self.path_ready = True
                self.log_info(f"Reference trajectory loaded: {len(self.reference_trajectory)} points")
            else:
                self.get_logger().error("Preprocessing failed")
                self.path_ready = False

        except Exception as e:
            self.get_logger().error(f"Error during preprocessing: {str(e)}")
            self.path_ready = False

    def load_trajectory_from_csv(self, path):
        """Load processed trajectory from CSV file with theta support"""
        data = []
        try:
            with open(path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    state = VehicleState()
                    state.x = float(row['x'])
                    state.y = float(row['y'])
                    state.v = float(row['v'])

                    # Handle theta (heading angle) - this is what MPC needs
                    if 'theta' in row:
                        state.theta = float(row['theta'])
                    else:
                        # Calculate theta from the data if not provided
                        state.theta = 0.0  # Will be calculated later

                    # Handle both 'delta' and 'δ' column names for steering angle
                    if 'delta' in row:
                        state.delta = float(row['delta'])
                    elif 'δ' in row:
                        state.delta = float(row['δ'])
                    else:
                        state.delta = 0.0
                    data.append(state)

            # If theta was not provided, calculate it from consecutive points
            if len(data) > 1 and data[0].theta == 0.0:
                for i in range(len(data)):
                    if i < len(data) - 1:
                        dx = data[i + 1].x - data[i].x
                        dy = data[i + 1].y - data[i].y
                        data[i].theta = math.atan2(dy, dx)
                    else:
                        # For the last point, use the same theta as the previous point
                        data[i].theta = data[i - 1].theta

            self.log_info(f"Loaded {len(data)} trajectory points")
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")
        return data

    def find_closest_trajectory_point(self):
        """Find the closest point on the reference trajectory to current position"""
        if not self.reference_trajectory:
            return 0

        min_distance = float('inf')
        closest_index = 0

        current_x = self.current_vehicle_state.x
        current_y = self.current_vehicle_state.y

        for i, point in enumerate(self.reference_trajectory):
            distance = np.sqrt((point.x - current_x)**2 + (point.y - current_y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index

    def create_predictive_trajectory(self):
        """Create predictive trajectory horizon starting from closest reference point"""
        if not self.reference_trajectory:
            return VehicleStateArray()

        # Find closest point on reference trajectory
        closest_index = self.find_closest_trajectory_point()

        # Create trajectory array for MPC horizon
        trajectory_msg = VehicleStateArray()

        # Start with current vehicle state as first point
        trajectory_msg.states.append(self.current_vehicle_state)

        # Add reference trajectory points for the prediction horizon
        for i in range(1, self.horizon):
            ref_index = (closest_index + i) % len(self.reference_trajectory)
            ref_point = self.reference_trajectory[ref_index]

            # Create predicted state
            predicted_state = VehicleState()
            predicted_state.x = ref_point.x
            predicted_state.y = ref_point.y
            predicted_state.v = ref_point.v
            predicted_state.delta = ref_point.delta

            trajectory_msg.states.append(predicted_state)

        return trajectory_msg

    def create_path_msg(self):
        """Create Path message for RViz visualization"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for state in self.reference_trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation for simplicity
            path_msg.poses.append(pose)

        return path_msg

    def publish_predictive_trajectory(self):
        """Publish predictive trajectory for MPC and status"""
        # Always publish status
        status_msg = Bool()
        status_msg.data = self.path_ready
        self.status_pub.publish(status_msg)

        # Only publish trajectory data if ready and we have current state
        if self.path_ready and len(self.reference_trajectory) > 0:
            # Publish full reference path for RViz (less frequent)
            if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # ~10% of the time
                path_msg = self.create_path_msg()
                self.path_pub.publish(path_msg)
                self.log_debug("Published reference path for RViz visualization")

            # Always publish predictive trajectory for MPC
            trajectory_msg = self.create_predictive_trajectory()
            if len(trajectory_msg.states) > 0:
                self.trajectory_pub.publish(trajectory_msg)

                self.log_debug(
                    f"Published predictive trajectory: {len(trajectory_msg.states)} points, "
                    f"current pos: ({self.current_vehicle_state.x:.2f}, {self.current_vehicle_state.y:.2f}), "
                    f"closest ref idx: {self.find_closest_trajectory_point()}")

    def publish_data(self):
        """Legacy method - kept for compatibility"""
        self.publish_predictive_trajectory()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.log_info("Trajectory publisher node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
