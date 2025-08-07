#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class ChatterPublisher(Node):
    def __init__(self):
        super().__init__('chatter_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0
        
    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    chatter_publisher = ChatterPublisher()
    
    try:
        rclpy.spin(chatter_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        chatter_publisher.destroy_node()
        if rclpy.ok():  # <-- Only shutdown if context is still valid
            rclpy.shutdown()


if __name__ == '__main__':
    main()