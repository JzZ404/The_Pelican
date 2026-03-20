#!/usr/bin/env python3
"""
RViz visualization launch file for The Pelican.
Launches robot_state_publisher (URDF) and RViz2 with the project config.

Usage:
    ros2 launch final display.launch.py

The robot body, TF tree, LaserScan, and camera feed will be visible in RViz.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('final')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'the_pelican.urdf')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'the_pelican.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])
