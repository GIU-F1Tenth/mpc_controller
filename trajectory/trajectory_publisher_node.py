from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from giu_f1t_interfaces.msg import VehicleState, VehicleStateArray

from preprocess_trajectory import preprocess_trajectory, find_config_file, load_ros2_params

import rclpy
from rclpy.node import Node
import csv
import os


class TrajectoryPublisherNode(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')

        # Load configuration from params.yaml
        config_path = find_config_file()
        if not config_path:
            self.get_logger().error("Could not find config/params.yaml file!")
            return

        self.get_logger().info(f"Loading config from: {config_path}")
        self.params = load_ros2_params(config_path)

        # Get paths from config
        self.input_path = self.params.get('optimal_trajectory_path')
        self.output_path = self.params.get('reference_trajectory_path')
        self.horizon = self.params.get('horizon_N', 10)

        if not self.input_path or not self.output_path:
            self.get_logger().error("Missing trajectory paths in config file!")
            return

        # State variables
        self.trajectory = []
        self.path_ready = False

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
        self.publish_timer = self.create_timer(0.3, self.publish_data)  # 1 Hz

        # Run preprocessing on startup
        self.preprocess_and_load_trajectory()

    def preprocess_and_load_trajectory(self):
        """Run preprocessing script and load trajectory"""
        try:
            self.get_logger().info("Starting trajectory preprocessing...")

            # Run the preprocessing script
            success = preprocess_trajectory(self.input_path, self.output_path)

            if success:
                self.get_logger().info("Preprocessing completed successfully")
                self.trajectory = self.load_trajectory_from_csv(self.output_path)
                self.path_ready = True
                self.get_logger().info(f"Trajectory loaded: {len(self.trajectory)} points")
            else:
                self.get_logger().error("Preprocessing failed")
                self.path_ready = False

        except Exception as e:
            self.get_logger().error(f"Error during preprocessing: {str(e)}")
            self.path_ready = False

    def load_trajectory_from_csv(self, path):
        """Load processed trajectory from CSV file"""
        data = []
        try:
            with open(path, 'r') as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    state = VehicleState()
                    state.x = float(row['x'])
                    state.y = float(row['y'])
                    state.v = float(row['v'])
                    # Handle both 'delta' and 'δ' column names
                    if 'delta' in row:
                        state.delta = float(row['delta'])
                    elif 'δ' in row:
                        state.delta = float(row['δ'])
                    else:
                        state.delta = 0.0
                    data.append(state)

            self.get_logger().info(f"Loaded {len(data)} trajectory points")
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")
        return data

    def create_path_msg(self):
        """Create Path message for RViz visualization"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for state in self.trajectory:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # No rotation for simplicity
            path_msg.poses.append(pose)

        return path_msg

    def create_trajectory_msg(self):
        """Create VehicleStateArray message for MPC"""
        trajectory_msg = VehicleStateArray()
        # Send full trajectory or horizon-limited trajectory
        trajectory_msg.states = self.trajectory[:self.horizon] if len(
            self.trajectory) > self.horizon else self.trajectory
        return trajectory_msg

    def publish_data(self):
        """Publish all trajectory data"""
        # Always publish status
        status_msg = Bool()
        status_msg.data = self.path_ready
        self.status_pub.publish(status_msg)

        # Only publish trajectory data if ready
        if self.path_ready and len(self.trajectory) > 0:
            # Publish path for RViz
            path_msg = self.create_path_msg()
            self.path_pub.publish(path_msg)

            # Publish trajectory for MPC
            trajectory_msg = self.create_trajectory_msg()
            self.trajectory_pub.publish(trajectory_msg)

            self.get_logger().debug(f"Published trajectory data: {len(self.trajectory)} points")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
