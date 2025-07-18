"""
ROS 2 Node for Model Predictive Control of F1Tenth vehicle.

This module implements a ROS 2 node that uses MPC for autonomous vehicle control,
subscribing to odometry and publishing Ackermann drive commands.
"""

import logging
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import transforms3d.euler as euler
from mpc_controller.MPC_Controller import MPCController

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MPCControllerNode(Node):
    """
    ROS 2 Node for MPC-based vehicle control.

    This node subscribes to odometry messages and publishes Ackermann drive
    commands using Model Predictive Control.
    """

    def __init__(self):
        """Initialize the MPC controller node."""
        super().__init__('mpc_controller')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheelbase', 0.33),
                ('mpc_horizon', 10),
                ('mpc_time_horizon', 1.0),
                ('max_velocity', 5.0),
                ('control_frequency', 10.0),
                ('reference_trajectory_file', ''),
            ]
        )

        # Get parameters
        self.wheelbase = self.get_parameter(
            'wheelbase').get_parameter_value().double_value
        mpc_horizon = self.get_parameter(
            'mpc_horizon').get_parameter_value().integer_value
        mpc_time = self.get_parameter(
            'mpc_time_horizon').get_parameter_value().double_value
        self.max_velocity = self.get_parameter(
            'max_velocity').get_parameter_value().double_value

        # Initialize MPC controller
        self.mpc = MPCController(N=mpc_horizon, T=mpc_time, L=self.wheelbase)

        # State variables
        self.current_state = np.zeros(4)  # [x, y, v, theta]
        self.state_initialized = False

        # Reference trajectory
        self.reference_trajectory = self._generate_simple_reference()

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odometry_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        control_period = 1.0 / \
            self.get_parameter(
                'control_frequency').get_parameter_value().double_value
        self.control_timer = self.create_timer(
            control_period, self.control_callback)
        self.get_logger().info(f"Control timer set to {control_period:.2f}s")

        self.get_logger().info(
            f"MPC Controller Node initialized with wheelbase={self.wheelbase}m, "
            f"horizon={mpc_horizon}, time_horizon={mpc_time}s"
        )

    def _generate_simple_reference(self) -> np.ndarray:
        """
        Generate a simple circular reference trajectory.

        Returns:
            Reference trajectory (4 x N)
        """
        N = 50  # Number of reference points
        radius = 5.0
        center_x, center_y = 0.0, -10.0

        angles = np.linspace(0, 2 * np.pi, N)
        x_ref = center_x + radius * np.cos(angles)
        y_ref = center_y + radius * np.sin(angles)
        v_ref = np.full(N, 3.0)  # Constant reference velocity
        theta_ref = angles + np.pi / 2  # Tangent to circle

        return np.vstack([x_ref, y_ref, v_ref, theta_ref])

    def _find_closest_reference_point(self, current_pos: np.ndarray) -> int:
        """
        Find the closest point on the reference trajectory.

        Args:
            current_pos: Current position [x, y]

        Returns:
            Index of closest reference point
        """
        ref_positions = self.reference_trajectory[:2, :]  # [x, y] positions
        distances = np.linalg.norm(
            ref_positions - current_pos.reshape(-1, 1), axis=0)
        return np.argmin(distances)

    def _extract_reference_segment(self, start_idx: int) -> np.ndarray:
        """
        Extract a reference trajectory segment for MPC.

        Args:
            start_idx: Starting index in reference trajectory

        Returns:
            Reference segment (4 x N+1)
        """
        N = self.mpc.N
        ref_length = self.reference_trajectory.shape[1]

        # Extract segment with wraparound
        indices = [(start_idx + i) % ref_length for i in range(N + 1)]
        return self.reference_trajectory[:, indices]

    def odometry_callback(self, msg: Odometry) -> None:
        """
        Process odometry messages and update current state.

        Args:
            msg: Odometry message
        """
        try:
            # Extract position and velocity
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            v = np.sqrt(
                msg.twist.twist.linear.x**2 +
                msg.twist.twist.linear.y**2
            )

            # Extract orientation
            quat = msg.pose.pose.orientation
            _, _, theta = euler.quat2euler([quat.w, quat.x, quat.y, quat.z])

            # Update current state
            self.current_state = np.array([x, y, v, theta])
            self.state_initialized = True

            self.get_logger().debug(
                f"State updated: x={x:.2f}, y={y:.2f}, v={v:.2f}, θ={theta:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing odometry: {e}")

    def control_callback(self) -> None:
        """Execute MPC control loop."""
        if not self.state_initialized:
            self.get_logger().debug("Waiting for odometry data...")
            return

        try:
            # Find closest reference point
            current_pos = self.current_state[:2]
            ref_idx = self._find_closest_reference_point(current_pos)

            # Extract reference segment
            X_ref = self._extract_reference_segment(ref_idx)

            # Solve MPC
            acceleration, steering_angle = self.mpc.solve_mpc(
                self.current_state[0], self.current_state[1],
                self.current_state[2], self.current_state[3],
                X_ref
            )

            # Publish control command
            self._publish_control_command(acceleration, steering_angle)

            self.get_logger().debug(
                f"MPC output: a={acceleration:.3f}, δ={steering_angle:.3f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
            # Publish emergency stop
            self._publish_control_command(0.0, 0.0)

    def _publish_control_command(self, acceleration: float, steering_angle: float) -> None:
        """
        Publish Ackermann drive command.

        Args:
            acceleration: Acceleration command
            steering_angle: Steering angle command
        """
        self.get_logger().debug(
            f"Publishing control command: a={acceleration:.3f}, δ={steering_angle:.3f}")
        try:
            # Calculate target velocity (simple integration)
            current_velocity = self.current_state[2]
            dt = 1.0 / \
                self.get_parameter(
                    'control_frequency').get_parameter_value().double_value
            target_velocity = max(0.0, min(
                self.max_velocity,
                current_velocity + acceleration * dt
            ))

            # Create and publish message
            cmd_msg = AckermannDriveStamped()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.header.frame_id = "base_link"
            cmd_msg.drive.speed = target_velocity
            cmd_msg.drive.steering_angle = steering_angle
            cmd_msg.drive.acceleration = acceleration

            self.cmd_publisher.publish(cmd_msg)
            self.get_logger().debug(
                f"Published command: speed={target_velocity:.2f}, "
                f"steering_angle={steering_angle:.2f}, acceleration={acceleration:.2f}"
            )

        except Exception as e:
            self.get_logger().error(f"Error publishing control command: {e}")


def main(args=None):
    """Main function to run the MPC controller node."""
    try:
        rclpy.init(args=args)
        node = MPCControllerNode()

        logger.info("MPC Controller Node started")
        rclpy.spin(node)

    except KeyboardInterrupt:
        logger.info("Shutting down MPC Controller Node")
    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
