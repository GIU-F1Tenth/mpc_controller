import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import pandas as pd
from mpc_controller.MPC_Controller import MPC_Controller as MPC
from visualization_msgs.msg import Marker
from pynput import keyboard
import math

def read_trajectory_csv(file_path):
    df = pd.read_csv(file_path)
    return {
        "x": df["x"].values,
        "y": df["y"].values,
        "speed": df["v"].values,
    }

def euler_from_quaternion(quat):
    x, y, z, w = quat
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.declare_parameter("trajectory_file", "/home/mohammedazab/GIU_F1Tenth/mpc_controller/mpc_controller/mansour_3_out.csv")
        self.declare_parameter("rate", 20.0)
        self.declare_parameter('max_lookahead_distance', 2.0)
        self.declare_parameter('min_lookahead_distance', 0.8)
        self.declare_parameter('max_velocity', 7.0) 
        self.declare_parameter('min_velocity', 1.0)

        traj_path = self.get_parameter("trajectory_file").get_parameter_value().string_value
        self.rate = self.get_parameter("rate").get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
        self.min_velocity = self.get_parameter('min_velocity').get_parameter_value().double_value
        self.min_lad = self.get_parameter('min_lookahead_distance').get_parameter_value().double_value
        self.max_lad = self.get_parameter('max_lookahead_distance').get_parameter_value().double_value

        self.trajectory = read_trajectory_csv(traj_path)
        self.path_pub = self.create_publisher(Path, "/pp_path", 10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 10)
        self.lookahead_pub = self.create_publisher(Marker, "/lookahead_point", 10)

        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)
        self.mpc = MPC()

        self.current_state = np.zeros(4)  # [x, y, yaw, v]
        self.state_received = False
        self.activate_autonomous_vel = False 
        self.lookahead_dist = 1.5

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

    def on_press(self, key):
        try:
            if key.char == 'a':
                self.activate_autonomous_vel = True 
        except AttributeError:
            self.get_logger().warn("Error with keyboard input")

    def on_release(self, key):
        self.activate_autonomous_vel = False
        if key == keyboard.Key.esc:
            return False

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        self.current_state = np.array([x, y, yaw, v])
        self.state_received = True
        self.get_logger().info(f"Odometry - X: {x:.2f}, Y: {y:.2f}, V: {v:.2f}, Yaw: {yaw:.2f}")

    def control_loop(self):
        if not self.state_received:
            self.get_logger().warn("Waiting for odometry...")
            return

        self.publish_path()
        idx = self.find_lookahead_index(self.current_state[0], self.current_state[1])
        if idx is None:
            self.get_logger().warn("No valid lookahead point found.")
            return

        x_lh = self.trajectory["x"][idx]
        y_lh = self.trajectory["y"][idx]
        self.publish_lookahead_marker(x_lh, y_lh)

        # Reference trajectory for MPC (e.g., a window of N points ahead)
        horizon = self.mpc.N if hasattr(self.mpc, 'N') else 10
        x_ref = self.trajectory["x"][idx:idx + horizon]
        y_ref = self.trajectory["y"][idx:idx + horizon]
        v_ref = self.trajectory["speed"][idx:idx + horizon]
        
        if len(x_ref) < horizon:
            self.get_logger().warn("End of trajectory reached.")
            return

        X_ref = np.vstack((x_ref, y_ref, v_ref))

        try:
            steer, accel = self.mpc.solve(self.current_state, X_ref)
            self.publish_cmd([steer, accel])
        except Exception as e:
            self.get_logger().error(f"MPC solve failed: {e}")

    def get_lad_thresh(self, v):
        m = (self.max_lad - self.min_lad) / (self.max_velocity - self.min_velocity)
        c = self.max_lad - m * self.max_velocity
        lad = m * v + c
        return np.clip(lad, self.min_lad, self.max_lad)

    def find_lookahead_index(self, x, y):
        min_dist = float('inf')
        closest_idx = 0
        for i in range(len(self.trajectory["x"])):
            dist = np.hypot(self.trajectory["x"][i] - x, self.trajectory["y"][i] - y)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        for i in range(closest_idx, len(self.trajectory["x"])):
            dist = np.hypot(self.trajectory["x"][i] - x, self.trajectory["y"][i] - y)
            if dist >= self.lookahead_dist:
                return i
        return None

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in zip(self.trajectory["x"], self.trajectory["y"]):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def publish_lookahead_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.2
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.lookahead_pub.publish(marker)

    def publish_cmd(self, u):
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.steering_angle = float(u[0])
        cmd.drive.speed = float(u[1]) / 1.3 if self.activate_autonomous_vel else 0.0
        self.get_logger().info(f"Command -> Speed: {cmd.drive.speed:.2f}, Steering: {cmd.drive.steering_angle:.2f}")
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()