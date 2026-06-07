#!/usr/bin/env python3
"""启动 wheel_foot_bridge 节点"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyACM0',
        description='STM32 USB CDC 串口设备路径')

    tlm_rate_arg = DeclareLaunchArgument(
        'telemetry_rate', default_value='100.0',
        description='遥测接收频率 (Hz)')

    cmd_rate_arg = DeclareLaunchArgument(
        'cmd_rate', default_value='50.0',
        description='指令下发频率 (Hz)')

    timeout_arg = DeclareLaunchArgument(
        'timeout_ms', default_value='100',
        description='指令超时时间 (ms)')

    bridge_node = Node(
        package='stm32_bridge',
        executable='wheel_foot_bridge',
        name='wheel_foot_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'telemetry_rate': LaunchConfiguration('telemetry_rate'),
            'cmd_rate': LaunchConfiguration('cmd_rate'),
            'timeout_ms': LaunchConfiguration('timeout_ms'),
        }],
    )

    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )

    return LaunchDescription([
        port_arg,
        tlm_rate_arg,
        cmd_rate_arg,
        timeout_arg,
        bridge_node,
        base_footprint_tf,
    ])
