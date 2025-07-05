from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol_system',
            executable='ultrasonic_publisher',
            name='ultrasonic_publisher',
            output='screen'
        ),
        Node(
            package='patrol_system',
            executable='camera_monitor',
            name='camera_monitor',
            output='screen'
        ),
    ])