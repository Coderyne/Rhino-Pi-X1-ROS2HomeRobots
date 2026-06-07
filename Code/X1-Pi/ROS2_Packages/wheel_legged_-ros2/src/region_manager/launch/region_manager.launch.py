#!/usr/bin/env python3
"""区域管理节点启动"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    regions_file_arg = DeclareLaunchArgument(
        'regions_file', default_value='',
        description='区域数据 JSON 文件路径 (默认 ~/robot_regions.json)')

    map_frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='地图坐标系')

    node = Node(
        package='region_manager',
        executable='region_manager',
        name='region_manager',
        output='screen',
        parameters=[{
            'regions_file': LaunchConfiguration('regions_file'),
            'map_frame': LaunchConfiguration('map_frame'),
        }],
    )

    return LaunchDescription([
        regions_file_arg,
        map_frame_arg,
        node,
    ])
