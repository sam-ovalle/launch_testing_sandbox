#!/usr/bin/env python3

import unittest
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_testing.actions import ReadyToTest
import launch_testing
import threading
import time


# Test node to subscribe and verify messages
class ChatterTestSubscriber(Node):
    def __init__(self):
        super().__init__('chatter_test_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.chatter_callback,
            10
        )
        self.received_messages = []
        self.message_count = 0
        
    def chatter_callback(self, msg):
        self.received_messages.append(msg.data)
        self.message_count += 1
        self.get_logger().info(f'Received: "{msg.data}"')


@pytest.mark.launch_test
def generate_test_description():
    """Generate the launch description for the test."""
    return LaunchDescription([
        LaunchNode(
            package='pytest_sandbox',
            executable='chatter_publisher',
            name='chatter_publisher',
            output='screen',
        ),
        ReadyToTest(),
    ])


class TestChatterPublisher(unittest.TestCase):
    """Test cases for the chatter publisher."""
    
    @classmethod
    def setUpClass(cls):
        """Set up the ROS2 context for testing."""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Clean up the ROS2 context."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up for each test method."""
        self.test_node = ChatterTestSubscriber()
        
    def tearDown(self):
        """Clean up after each test method."""
        self.test_node.destroy_node()
    
    def test_chatter_publisher_messages(self):
        """Test that the chatter publisher is sending messages."""
        # Spin the test node in a separate thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.test_node)
        
        def spin_node():
            try:
                executor.spin()
            except Exception as e:
                print(f"Executor error: {e}")
        
        spin_thread = threading.Thread(target=spin_node, daemon=True)
        spin_thread.start()
        
        # Wait for a few messages
        timeout = 10.0  # seconds
        start_time = time.time()
        
        while (self.test_node.message_count < 3 and 
               time.time() - start_time < timeout):
            time.sleep(0.1)
        
        # Stop the executor
        try:
            executor.shutdown()
        except Exception:
            pass
        
        # Verify we received at least some messages
        self.assertGreater(self.test_node.message_count, 0, 
                          "No messages received from chatter publisher")
        self.assertGreaterEqual(self.test_node.message_count, 3,
                               f"Expected at least 3 messages, got {self.test_node.message_count}")
        
        # Verify message format
        for msg in self.test_node.received_messages:
            self.assertIn("Hello World:", msg, 
                         f"Message format incorrect: {msg}")
    
    def test_chatter_publisher_topic_exists(self):
        """Test that the chatter topic exists and has the right type."""
        # Wait a bit for the node to start publishing
        time.sleep(2.0)
        
        # Get topic info
        topic_names_and_types = self.test_node.get_topic_names_and_types()
        
        # Find the chatter topic
        chatter_found = False
        for topic_name, topic_types in topic_names_and_types:
            if topic_name == '/chatter':
                chatter_found = True
                self.assertIn('std_msgs/msg/String', topic_types,
                             f"Wrong message type for /chatter topic: {topic_types}")
                break
        
        self.assertTrue(chatter_found, 
                       f"The /chatter topic was not found. Available topics: {[name for name, _ in topic_names_and_types]}")


# Post-shutdown tests (optional)
@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_output, proc_info):
        """Check that the process exited cleanly."""
        # Allow for normal shutdown (exit code 0) or SIGINT (exit code -2)
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
