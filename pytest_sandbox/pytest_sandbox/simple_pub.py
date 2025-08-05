import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_pub_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = "Hello, ROS 2!"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
