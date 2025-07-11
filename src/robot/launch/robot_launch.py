#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from xacro import process_file


def generate_launch_description():
    pkg_name = 'robot' # <--- Używamy obecnej nazwy pakietu!
    pkg_share_dir = get_package_share_directory(pkg_name)
    urdf_path = os.path.join(pkg_share_dir, 'description', 'robot.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    slam_toolbox_params_file = os.path.join(
        get_package_share_directory('robot'), # Upewnij się, że 'robot' to nazwa pakietu, gdzie masz config
        'config',
        'slam_toolbox_params.yaml'
    )

    return LaunchDescription([
        declare_use_sim_time_arg,

        # 1. Węzeł odometrii - TAK JAK DZIAŁAŁ NA SCREENIE (z małą poprawką)
        Node(
            package=pkg_name, # <--- Używamy package=pkg_name
            executable='odometry_publisher', # <--- Używamy nazwy wykonywalnego z setup.py
            name='robot_odometry_publisher', # <--- Nazwa węzła, którą ustawiliśmy
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            #prefix=['/usr/bin/sudo', '-E', '/usr/bin/python3'], # <-- Usunięty prefix SUDO
        ),

        # 2. Robot State Publisher (publikuje transformacje z URDF/XACRO)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': process_file(urdf_path).toxml()},
                {'use_sim_time': use_sim_time}
            ],
        ),

        # 3. Odpalenie lidara
        Node(
            package='rplidar_ros',
            executable='rplidar_composition', # To jest nazwa wykonywalnego węzła
            name='rplidar_node', # Możesz nadać inną nazwę, np. 'rplidar_driver'
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'}, # Upewnij się, że to poprawny port
                {'serial_baudrate': 256000}, # Upewnij się, że to poprawny baudrate
                {'scan_mode': 'Standard'}
            ]
        ),

        # 4. SLAM Toolbox
         Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                slam_toolbox_params_file # --- Zastąpiono poprzednie parametry tym wczytywaniem! ---
            ],
        ),
    ])

