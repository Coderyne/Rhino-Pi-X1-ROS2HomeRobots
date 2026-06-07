#!/usr/bin/env python3
"""人体跟随启动 — person_follower 节点"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    person_follower = Node(
        package='perception',
        executable='person_follower',
        name='person_follower',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('perception'),
                'config', 'params.yaml'
            ])
        ],
    )

    return LaunchDescription([person_follower])
