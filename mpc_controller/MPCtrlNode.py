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

        self.mpc = MPC_Controller(N=10, T=1.0, L=0.33)

        self.subscription = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.publisher_ = self.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)
        self.timer = self.create_timer(0.005, self.get_pose)  # 50 Hz
        self.get_logger().info("MPC Controller has been started")
          
        #self.trajectory_type = 'straight'
        # self.trajectory_type = 'circular'  
        self.trajectory_type = 'pure_pursuit'
        
        self.v = 0.0
        self.get_logger().info(self.trajectory_type)

    def get_pose(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map',      # target_frame
                'laser',    # source_frame (your base_frame)
                now,
                timeout=rclpy.duration.Duration(seconds=0.5)
            )

            trans = transform.transform.translation
            rot = transform.transform.rotation

            # Convert quaternion to yaw
            orientation_list = [rot.x, rot.y, rot.z, rot.w]

            # self.get_logger().info(f"Robot Pose - x: {trans.x:.2f}, y: {trans.y:.2f}, yaw: {yaw:.2f}")
            x, y = trans.x, trans.y
            _, _, theta = euler.quat2euler(orientation_list)
        
            self.get_logger().info(f"Odometry - X: {x}, Y: {y}, Velocity: {self.v}, Theta: {theta}")
            
            current_state = (x, y, self.v, theta)

            if self.trajectory_type == 'circular':
                X_ref = self.mpc.create_circular_trajectory(center_x=0, center_y=0, radius=1.5, start_theta=theta, N=10)
                optimal_accel, optimal_steer = self.mpc.solve_mpc(x, y, self.v, theta, X_ref)
            elif self.trajectory_type == 'straight':
                X_ref = self.mpc.create_straight_line_trajectory(start_x=x, start_y=y, start_theta=theta, end_x=x+5, end_y=y, N=10)
                optimal_accel, optimal_steer = self.mpc.solve_mpc(x, y, self.v, theta, X_ref)
            elif self.trajectory_type == 'pure_pursuit':
                optimal_accel, optimal_steer = self.mpc.followPurePursuit(current_state, lookahead_distance=0.7)

        
        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

        self.get_logger().info(f"Optimal Control - Acceleration: {optimal_accel}, Steering Angle: {optimal_steer}")

        # Publish Ackermann Drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.7 
        drive_msg.drive.steering_angle = optimal_steer
        self.publisher_.publish(drive_msg)

    def odom_callback(self, msg):
        # pass
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y
        self.v = msg.twist.twist.linear.x
        # quat = msg.pose.pose.orientation
        # _, _, theta = euler.quat2euler([quat.x, quat.y, quat.z, quat.w])
        
        # self.get_logger().info(f"Odometry - X: {x}, Y: {y}, Velocity: {v}, Theta: {theta}")
        
        # current_state = (x, y, v, theta)

        # if self.trajectory_type == 'circular':
        #     X_ref = self.mpc.create_circular_trajectory(center_x=0, center_y=0, radius=1.5, start_theta=theta, N=10)
        #     optimal_accel, optimal_steer = self.mpc.solve_mpc(x, y, v, theta, X_ref)
        # elif self.trajectory_type == 'straight':
        #     X_ref = self.mpc.create_straight_line_trajectory(start_x=x, start_y=y, start_theta=theta, end_x=x+5, end_y=y, N=10)
        #     optimal_accel, optimal_steer = self.mpc.solve_mpc(x, y, v, theta, X_ref)
        # elif self.trajectory_type == 'pure_pursuit':
        #     optimal_accel, optimal_steer = self.mpc.followPurePursuit(current_state, lookahead_distance=0.7)

        

        # self.get_logger().info(f"Optimal Control - Acceleration: {optimal_accel}, Steering Angle: {optimal_steer}")

        # # Publish Ackermann Drive command
        # drive_msg = AckermannDriveStamped()
        # drive_msg.drive.speed = 0.7 
        # drive_msg.drive.steering_angle = optimal_steer
        # self.publisher_.publish(drive_msg)

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
