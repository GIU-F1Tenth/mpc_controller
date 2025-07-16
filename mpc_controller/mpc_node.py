#!/usr/bin/env python3

"""
F1TENTH MPC Controller Node

This ROS2 node implements a Model Predictive Controller for trajectory tracking
in F1TENTH autonomous racing cars. It listens to odometry and reference trajectory,
computes control inputs (steering angle and velocity), and publishes them as drive commands.

Author: Mohammed Azab <mohammed@azab.io>
License: MIT
Version: 1.0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
import numpy as np

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        self._declare_parameters()
        self._load_parameters()
        self._initialize_state()
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()

        self.get_logger().info("F1TENTH MPC Node has been started successfully")

    def _declare_parameters(self):
        # Control limits
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_steering_angle', 0.4)
        self.declare_parameter('control_hz', 20.0)

        # Topics
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reference_topic', '/mpc/reference_path')
        self.declare_parameter('control_topic', '/drive')

        # QoS
        self.declare_parameter('qos_depth', 10)

    def _load_parameters(self):
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.control_hz = self.get_parameter('control_hz').value

        self.odom_topic = self.get_parameter('odom_topic').value
        self.reference_topic = self.get_parameter('reference_topic').value
        self.control_topic = self.get_parameter('control_topic').value

        self.qos_depth = self.get_parameter('qos_depth').value

    def _initialize_state(self):
        self.current_pose = None
        self.current_velocity = 0.0
        self.reference_path = []  # List of (x, y)

    def _setup_subscriptions(self):
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            self.qos_depth
        )

        self.create_subscription(
            PoseStamped,  # Could be custom msg depending on your trajectory
            self.reference_topic,
            self._reference_callback,
            self.qos_depth
        )

    def _setup_publishers(self):
        self.control_publisher = self.create_publisher(
            AckermannDriveStamped,
            self.control_topic,
            self.qos_depth
        )

    def _setup_timers(self):
        self.create_timer(1.0 / self.control_hz, self._control_loop)

    def _odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist.linear.x

    def _reference_callback(self, msg: PoseStamped):
        # You may want a custom reference trajectory msg in practice
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.reference_path = [(x, y)]

    def _control_loop(self):
        if self.current_pose is None or not self.reference_path:
            return

        # Placeholder MPC logic (replace with real QP-based MPC)
        x_current = self.current_pose.position.x
        y_current = self.current_pose.position.y

        x_ref, y_ref = self.reference_path[-1]  # Target last pose

        dx = x_ref - x_current
        dy = y_ref - y_current
        distance = np.hypot(dx, dy)

        # Simple P-controller (placeholder for actual MPC output)
        speed = min(self.max_speed, distance * 1.0)
        steering_angle = np.clip(np.arctan2(dy, dx), -self.max_steering_angle, self.max_steering_angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steering_angle

        self.control_publisher.publish(drive_msg)
        self.get_logger().debug(f"Publishing control: speed={speed:.2f}, angle={steering_angle:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
