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

    def test_chatter_publisher_topic_exists(self, proc_output):
        """Test that the chatter topic exists and has the right type."""
        time.sleep(2.0)  # Allow time for the topic to be registered

        topic_names_and_types = self.node.get_topic_names_and_types()
        chatter_found = False

        for topic_name, topic_types in topic_names_and_types:
            if topic_name == '/chatter':
                chatter_found = True
                self.assertIn('std_msgs/msg/String', topic_types,
                              f"Wrong message type for /chatter topic: {topic_types}")
                break

        self.assertTrue(chatter_found,
                        f"The /chatter topic was not found. Available topics: {[name for name, _ in topic_names_and_types]}")

"""only allow 
        success case exit [0] 
        graceful shutdown [-2]
   else raise a failure"""
@launch_testing.post_shutdown_test()
class TestProcessExitCodes(unittest.TestCase):
    def test_exit_codes(self, proc_output, proc_info):
        
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
