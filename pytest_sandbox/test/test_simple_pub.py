import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
import launch_testing

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        LaunchNode(
            package='pytest_sandbox',
            executable='simple_pub',
            name='simple_pub'
        ),
        launch_testing.actions.ReadyToTest()
    ])

@pytest.fixture
def rclpy_node():
    rclpy.init()
    node = Node("test_simple_pub_node")
    yield node
    node.destroy_node()
    rclpy.shutdown()

def test_published_message(rclpy_node):
    msgs = []

    def callback(msg):
        msgs.append(msg.data)

    sub = rclpy_node.create_subscription(String, 'chatter', callback, 10)

    # Spin for 2 seconds to collect messages
    import time
    start = time.time()
    while time.time() - start < 2:
        rclpy.spin_once(rclpy_node, timeout_sec=0.1)

    assert len(msgs) > 0
    assert "Hello, ROS 2!" in msgs
    assert msgs[0].data == "Hello, ROS 2!"

