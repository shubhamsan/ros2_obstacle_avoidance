#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get config file path
    pkg_share = get_package_share_directory('ros2_obstacle_avoidance')
    config_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
    return LaunchDescription([
        # Obstacle avoider node
        Node(
            package='ros2_obstacle_avoidance',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            parameters=[config_file],
            output='screen',
        ),
        
        # Status logger node
        Node(
            package='ros2_obstacle_avoidance',
            executable='status_logger',
            name='status_logger',
            parameters=[config_file],
            output='screen'
        ),
    ])