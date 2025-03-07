import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class MPCCtrlNode(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        
        # Publisher
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Timer to publish messages
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')


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
