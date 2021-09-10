#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

def generate_launch_description():

    gps = launch_ros.actions.Node(
            package='gps_utm',
            executable='gps_node',
            output='screen',
        )

    return LaunchDescription([
        gps,
    ])

