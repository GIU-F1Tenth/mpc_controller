import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped

class MPCCtrlNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Timer to publish messages
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, This MPC Node!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
    
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x
        theta = msg.pose.pose.orientation.z 


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
