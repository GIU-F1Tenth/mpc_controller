#!/usr/bin/env python3

"""
Simple Trajectory Publisher Node for MPC Controller

This node publishes a simple trajectory for testing the MPC controller.
"""

import rclpy
from rclpy.node import Node
from giu_f1t_interfaces.msg import VehicleState, VehicleStateArray
from std_msgs.msg import Bool
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import numpy as np
import math


class SimpleTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('simple_trajectory_publisher')

        # Declare parameters
        self.declare_parameter('horizon_N', 10)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('trajectory_type', 'straight')  # 'straight', 'circle', 'infinity'
        self.declare_parameter('reference_speed', 1.0)

        # Load parameters
        self.horizon_N = self.get_parameter('horizon_N').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.reference_speed = self.get_parameter('reference_speed').value

        # Publishers
        self.trajectory_pub = self.create_publisher(
            VehicleStateArray,
            '/mpc/reference_trajectory',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/mpc/reference_path',
            10
        )

        self.status_pub = self.create_publisher(
            Bool,
            '/mpc/path_ready',
            10
        )

        # Subscriber to get current car position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/car_state/odom',
            self.odometry_callback,
            10
        )

        # State
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.path_ready = True

        # Create timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_trajectory)

        self.get_logger().info(f"📍 Simple Trajectory Publisher started")
        self.get_logger().info(f"   - Trajectory type: {self.trajectory_type}")
        self.get_logger().info(f"   - Reference speed: {self.reference_speed} m/s")
        self.get_logger().info(f"   - Horizon: {self.horizon_N} steps")

    def odometry_callback(self, msg):
        """Update current car position from odometry"""
        try:
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y

            # Extract yaw from quaternion
            orientation_q = msg.pose.pose.orientation
            _, _, self.current_yaw = euler_from_quaternion([
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            ])

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {str(e)}")

    def generate_trajectory(self):
        """Generate trajectory based on type"""
        states = []

        if self.trajectory_type == 'straight':
            # Simple straight line trajectory
            for i in range(self.horizon_N + 1):
                state = VehicleState()
                state.x = self.current_x + i * 0.5  # 0.5m spacing
                state.y = self.current_y
                state.v = self.reference_speed
                state.delta = 0.0
                states.append(state)

        elif self.trajectory_type == 'circle':
            # Circular trajectory
            radius = 2.0
            center_x = self.current_x + radius
            center_y = self.current_y

            for i in range(self.horizon_N + 1):
                angle = i * 0.1  # 0.1 rad spacing
                state = VehicleState()
                state.x = center_x + radius * math.cos(angle)
                state.y = center_y + radius * math.sin(angle)
                state.v = self.reference_speed
                # Calculate steering for circular path
                state.delta = math.atan(0.33 / radius)  # wheelbase / radius
                states.append(state)

        elif self.trajectory_type == 'infinity':
            # Infinity sign (figure-8) trajectory
            for i in range(self.horizon_N + 1):
                t = i * 0.1
                scale = 2.0
                state = VehicleState()
                state.x = self.current_x + scale * math.sin(t)
                state.y = self.current_y + scale * math.sin(t) * math.cos(t)
                state.v = self.reference_speed
                state.delta = 0.1 * math.sin(2 * t)  # Varying steering
                states.append(state)

        return states

    def publish_trajectory(self):
        """Publish trajectory and status"""
        # Always publish status
        status_msg = Bool()
        status_msg.data = self.path_ready
        self.status_pub.publish(status_msg)

        # Generate and publish trajectory
        states = self.generate_trajectory()

        # Publish trajectory for MPC
        trajectory_msg = VehicleStateArray()
        trajectory_msg.states = states
        self.trajectory_pub.publish(trajectory_msg)

        # Publish path for visualization (less frequently)
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:  # ~10% of time
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for state in states:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = path_msg.header.stamp
                pose.pose.position.x = state.x
                pose.pose.position.y = state.y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)

        # Update current position (simulate movement)
        self.current_x += 0.01

        self.get_logger().debug(f"Published trajectory with {len(states)} states")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimpleTrajectoryPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Simple trajectory publisher shutting down...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
