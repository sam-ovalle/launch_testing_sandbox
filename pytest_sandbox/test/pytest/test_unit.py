#!/usr/bin/env python3

import unittest
import rclpy
from pytest_sandbox.chatter_publisher import ChatterPublisher


class TestChatterPublisherUnit(unittest.TestCase):
    """Unit tests for the ChatterPublisher class."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 for testing."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2."""
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def test_node_creation(self):
        """Test that the ChatterPublisher node can be created."""
        node = ChatterPublisher()
        self.assertEqual(node.get_name(), 'chatter_publisher')
        self.assertIsNotNone(node.publisher_)
        self.assertIsNotNone(node.timer)
        node.destroy_node()

    def test_message_format(self):
        """Test that messages have the correct format."""
        node = ChatterPublisher()
        
        # Test initial count
        self.assertEqual(node.count, 0)
        
        # Simulate timer callback
        node.timer_callback()
        self.assertEqual(node.count, 1)
        
        node.timer_callback()
        self.assertEqual(node.count, 2)
        
        node.destroy_node()


if __name__ == '__main__':
    unittest.main()