from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            parameters=[],
            respawn=False
        ),
        Node(
            package='ros2_teleop',
            executable='ros2_teleop_node',
            name='ros2_teleop',
            output='screen',
            parameters=[],
            respawn=False
        )
    ])