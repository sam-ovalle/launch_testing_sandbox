#!/usr/bin/env python3

import os
import sys
import unittest
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import launch
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch_testing.actions import ReadyToTest
import launch_testing
import threading
import time


class ChatterTestNode(Node):
    def __init__(self):
        super().__init__('chatter_test_node')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.chatter_callback,
            10
        )
        self.received_messages = []
        self.msg_count = 0
        
    def chatter_callback(self, msg):
        self.received_messages.append(msg.data)
        self.msg_count += 1


def generate_test_description():
    """Generate the launch description for the test."""
    return LaunchDescription([
        LaunchNode(
            package='pytest_sandbox',
            executable='chatter_publisher',
            name='chatter_publisher',
        ),
        ReadyToTest(),
    ])


class TestChatter(unittest.TestCase):
    
    def setUp(self):
        rclpy.init()
        self.node = ChatterTestNode()
        
    def tearDown(self):
        self.node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            # May already be shutdown
            pass
    
    def test_receives_messages(self):
        """Test that we can receive messages from the chatter publisher."""
        
        # Create executor and spin in background
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.node)
        
        def spin():
            executor.spin()
        
        spin_thread = threading.Thread(target=spin, daemon=True)
        spin_thread.start()
        
        # Wait for messages
        timeout = 10.0
        start_time = time.time()
        
        while self.node.msg_count < 2 and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        executor.shutdown()
        
        # Assertions
        self.assertGreater(self.node.msg_count, 0, "Should receive at least one message")
        
        for msg in self.node.received_messages:
            self.assertIn("Hello World:", msg)


@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_output, proc_info):
        """Test that processes exit cleanly."""
        # Allow exit code 1 due to RCL shutdown race condition
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, 1])