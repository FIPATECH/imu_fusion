#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_fusion',
            executable='imu_fusion_node',
            name='imu_fusion',
            output='screen'
        )
    ])
