import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import transforms3d.euler as euler
from mpc_controller.MPC_Controller import MPC_Controller



class MPCCtrlNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')

        self.mpc = MPC_Controller(N = 10, T = 1.0, L = 0.33)

        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        self.get_logger().info("MPC Controller has been started")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x
        quat = msg.pose.pose.orientation
        _, _, theta = euler.quat2euler([quat.x, quat.y, quat.z, quat.w])
        
        self.get_logger().info(f"Odometry - X: {x}, Y: {y}, Velocity: {v}, Theta: {theta}")

        # Create reference trajectory 
        X_ref = np.tile([x, y, v, theta], (11, 1)).T  # (4, N+1) shape

        # Solve for optimal control
        optimal_accel, optimal_steer = self.mpc.solve_mpc(x, y, v, theta, X_ref)

        self.get_logger().info(f"Optimal Control - Acceleration: {optimal_accel}, Steering Angle: {optimal_steer}")

        # Publish Ackermann Drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = v + optimal_accel * 0.1  
        drive_msg.drive.steering_angle = optimal_steer
        self.publisher_.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MPCCtrlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
