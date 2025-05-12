"""Simulate N Tello drones in a grid formation"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
     NUM_DRONES = 3
     GRID_SIZE = int(NUM_DRONES ** 0.5) + 1
     SPACING = 2.0

     world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'simple.world')
     tello_description_path = get_package_share_directory('tello_description')

     launch_actions = [
          # Launch Gazebo with the world
          ExecuteProcess(cmd=[
               'gazebo',
               '--verbose',
               '-s', 'libgazebo_ros_init.so',
               '-s', 'libgazebo_ros_factory.so',
               world_path
          ], output='screen'),
     ]

     for i in range(NUM_DRONES):
          row = i // GRID_SIZE
          col = i % GRID_SIZE
          x = col * SPACING
          y = row * SPACING
          z = 1.0
          suffix = '_' + str(i + 1)
          urdf_path = os.path.join(tello_description_path, 'urdf', f'tello{suffix}.urdf')

          # Spawn the drone
          launch_actions.append(
               Node(
                    package='tello_gazebo',
                    executable='inject_entity.py',
                    output='screen',
                    arguments=[urdf_path, str(x), str(y), str(z), '0']
               )
          )
        # Static transform publisher
          launch_actions.append(
               Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    output='screen',
                    arguments=[urdf_path]
               )
          )
     return LaunchDescription(launch_actions)