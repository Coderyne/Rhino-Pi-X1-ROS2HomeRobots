#!/usr/bin/env python3
"""完整 bringup — 底盘桥接 + SLAM/Nav2 (雷达由 start.sh 统一启动)"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    map_name = LaunchConfiguration('map')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='slam',
        description='slam (建图) | nav (导航)')
    map_arg = DeclareLaunchArgument(
        'map', default_value='map',
        description='导航时使用的地图名称 (不包含路径和 .yaml 后缀)')

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('stm32_bridge'), 'launch', 'bridge.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        mode_arg,
        map_arg,
        bridge_launch,
    ])
