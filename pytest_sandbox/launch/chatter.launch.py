from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pytest_sandbox',  # Replace with your actual package name
            executable='chatter_publisher',
            name='chatter_publisher',
            output='screen',
            parameters=[{
                # Add any parameters here if needed
            }]
        ),
    ])
