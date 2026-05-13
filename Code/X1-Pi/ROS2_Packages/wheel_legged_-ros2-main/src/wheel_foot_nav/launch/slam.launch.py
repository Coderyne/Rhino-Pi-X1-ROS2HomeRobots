#!/usr/bin/env python3
"""SLAM 建图启动 — slam_toolbox + 雷达 (雷达由 start.sh 统一启动)"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('wheel_foot_nav'),
                'config', 'slam_toolbox.yaml'
            ])
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        slam_node,
        rviz_node,
    ])
