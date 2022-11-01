"""
      This launch file is intended to launch the launcher so it can launch everything for our
      Simulation.
      Cannot record data and does not launch sensor drivers.
"""

import os
from os import getenv

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch the Launcher GUI
        Node(
            name='eufs_launcher',
            package='eufs_launcher',
            executable='eufs_launcher',
            output='screen',
            arguments=['--force-discover'],
            parameters=[
                {'config': 'config/eufs_launcher'},
                {'config_loc': 'eufs_launcher'},
                {'gui': True}
            ]
        ),
    ])
