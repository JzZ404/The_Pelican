#!/usr/bin/env python3
"""
Teleoperation launch file for The Pelican.
Launches teleop_twist_keyboard for manual control of the TurtleBot3.

Usage:
    ros2 launch final teleop.launch.py

Controls:
    u / i / o     - forward-left / forward / forward-right
    j / k / l     - turn-left / stop / turn-right
    m / , / .     - back-left / backward / back-right
    q/z           - increase/decrease max speeds
    Ctrl+C        - quit
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
                '--ros-args', '-r', '/cmd_vel:=/cmd_vel'
            ],
            output='screen',
            prefix='xterm -e',
        ),
    ])
