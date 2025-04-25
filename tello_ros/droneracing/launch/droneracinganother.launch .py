from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='droneracing',
            executable='gate_detector',
            name='gate_detector',
            output='screen'
        ),
        Node(
            package='droneracing',
            executable='tello_controller',
            name='tello_controller',
            output='screen'
        )
    ])