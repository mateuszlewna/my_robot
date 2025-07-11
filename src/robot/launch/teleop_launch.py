#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot', # Nazwa Twojego pakietu
            executable='teleop_keyboard_node',
            name='keyboard_teleop_node',
            output='screen',
            prefix='sudo -E /usr/bin/python3'
        )
    ])
