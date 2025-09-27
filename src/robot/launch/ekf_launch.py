#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_name = 'robot'
    
    ekf_params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ekf_parameters.yaml'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        
        # Węzeł EKF, który laczy dane z akcelerometru i enkoderów
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}]
        )
    ])