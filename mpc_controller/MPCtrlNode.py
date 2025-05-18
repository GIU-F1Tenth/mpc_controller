import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import pandas as pd
from .MPC_Controller import MPC
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

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class MPCControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.declare_parameter("trajectory_file", "/my_ws/src/mpc_controller/mpc_controller/sansibarstr_out.csv")
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

        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()

    def on_press(self, key):
        try:
            if key.char == 'a':
                self.activate_autonomous_vel = True 
        except AttributeError:
            self.get_logger().warn("error while sending.. :(")

    def on_release(self, key):
        # Stop the robot when the key is released
        # self.start_algorithm = False
        self.activate_autonomous_vel = False
        if key == keyboard.Key.esc:
            # Stop listener
            return False
        
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        v = msg.twist.twist.linear.x

        self.current_state = np.array([x, y, yaw, v])
        self.state_received = True
        self.lookahead_dist = self.get_lad_thresh(v)

    def get_lad_thresh(self, v):
        # lad = m*v + c
        m = (self.max_lad - self.min_lad)/(self.max_velocity - self.min_velocity)
        c = self.max_lad - m * self.max_velocity
        lad = m * v + c
        if lad < self.min_lad:
            lad = self.min_lad
        if lad > self.max_lad:
            lad = self.max_lad
        return lad

    def control_loop(self):
        if not self.state_received:
            self.get_logger().warn("Waiting for odometry...")
            return

        self.publish_path()
        idx = self.find_lookahead_index(self.current_state[0], self.current_state[1])
        x_lh = self.trajectory["x"][idx]
        y_lh = self.trajectory["y"][idx]
        self.publish_lookahead_marker(x_lh, y_lh)

        u = self.mpc.solve(self.current_state, self.trajectory, idx)
        self.publish_cmd(u)

    def find_lookahead_index(self, x, y):
        # find the closest point on the path from the car's position
        closest_idx = 0
        min_distance = float('inf')
        for i in range(len(self.trajectory["x"])):
            dx = self.trajectory["x"][i] - x
            dy = self.trajectory["y"][i] - y
            dist = np.hypot(dx, dy)
            if dist < min_distance:
                min_distance = dist
                closest_idx = i
        # find the first point from the lookahead distance
        for i in range(closest_idx, len(self.trajectory["x"])):
            dx = self.trajectory["x"][i] - x
            dy = self.trajectory["y"][i] - y
            dist = np.hypot(dx, dy)
            if dist >= self.lookahead_dist:
                return i
        # if no point is found search from the begining
        closest_idx = 0
        for i in range(closest_idx, len(self.trajectory["x"])):
            dx = self.trajectory["x"][i] - x
            dy = self.trajectory["y"][i] - y
            dist = np.hypot(dx, dy)
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
        marker.pose.position.z = 0.2  # slightly above ground
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
        if self.activate_autonomous_vel:
            cmd.drive.speed = float(u[1])/1.3
        else:
            cmd.drive.speed = 0.0
        self.get_logger().info(f"Sending cmd: speed={u[1]:.2f}, steering={u[0]:.2f}")
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
