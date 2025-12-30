#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Daichi Utsugi
# SPDX-License-Indentifier: BSD-3-Clause

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    target_path_arg = DeclareLaunchArgument(
        'target_path', default_value='/mnt/c', description='Path to monitor'
    )

    return LaunchDescription([
        target_path_arg,
        Node(
            package='kadai',
            executable='disk_monitor',
            arguments=[LaunchConfiguration('target_path')],
        ),
        Node(
            package='kadai',
            executable='disk_sub',
            output='screen',
        )
    ])
