"""Simulate N Tello drones in a grid formation"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    NUM_DRONES = 20
    GRID_SIZE = int(NUM_DRONES ** 0.5) + 1
    SPACING = 2.0

    world_path = os.path.join(
        get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
    tello_description_path = get_package_share_directory('tello_description')
    tello_xml_path = os.path.join(tello_description_path, 'urdf', 'tello.xml')
    urdf_output_dir = os.path.join(tello_description_path, 'urdf')  # 输出路径

    launch_actions = []

    launch_actions.append(
        ExecuteProcess(cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_path
        ], output='screen')
    )


    for i in range(NUM_DRONES):
        drone_num = i + 1
        row = i // GRID_SIZE
        col = i % GRID_SIZE
        x = col * SPACING
        y = row * SPACING
        z = 0.1
        suffix = f'_{drone_num}'
        topic_ns = f'drone{drone_num}'
        urdf_filename = f'tello{suffix}.urdf'
        urdf_path = os.path.join(urdf_output_dir, urdf_filename)

        generate_cmd = [
            'python3',
            os.path.join(tello_description_path, 'src', 'replace.py'),
            tello_xml_path,
            f'suffix={suffix}',
            f'topic_ns={topic_ns}'
        ]
        with open(urdf_path, 'w') as f:
            subprocess.run(generate_cmd, stdout=f, check=True)

        launch_actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'tello_gazebo', 'inject_entity.py',
                    urdf_path, str(x), str(y), str(z), '0'
                ],
                output='screen'
            )
        )

        launch_actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'robot_state_publisher', 'robot_state_publisher',
                    urdf_path
                ],
                output='screen'
            )
        )

    launch_actions.append(
        Node(
            package='open_project',
            executable='tello_controller',
            name='tello_controller',
            output='screen'
        )
    )

    return LaunchDescription(launch_actions)
