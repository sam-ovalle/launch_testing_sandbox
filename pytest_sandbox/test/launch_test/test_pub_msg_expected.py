#!/usr/bin/env python3

import unittest
import time

import rclpy
from std_msgs.msg import String

import launch
import launch_ros
import launch_testing
import launch_testing.actions


def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package='pytest_sandbox',
                    namespace='',
                    executable='chatter_publisher',
                    name='chatter_publisher',
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )

class TestChatter(unittest.TestCase):
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_chatter_simple')

    def tearDown(self):
        self.node.destroy_node()
    
    def test_receives_messages(self, proc_output):
        msgs_rx = []
        sub = self.node.create_subscription(
            String,
            'chatter',
            lambda msg: msgs_rx.append(msg.data),
            10
        )

        try:
            end_time = time.time() + 1
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1)

            assert len(msgs_rx) >= 1, "Should receive at least one messages"

            for msg in msgs_rx:
                assert "Hello World:" in msg, f"Unexpected message content: {msg}"
                # assert "xJRHwbweqj" in msg, f"Unexpected message content: {msg}" # intentionally fail

        finally:
            self.node.destroy_subscription(sub)

"""only allow 
        success case exit [0] 
        graceful shutdown [-2]
   else raise a failure"""
@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_output, proc_info):
        
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])