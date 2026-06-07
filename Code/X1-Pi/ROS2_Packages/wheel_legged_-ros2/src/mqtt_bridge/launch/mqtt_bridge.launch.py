#!/usr/bin/env python3
"""启动 MQTT ↔ ROS2 双向桥接节点"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mqtt_host_arg = DeclareLaunchArgument(
        'mqtt_host', default_value='127.0.0.1',
        description='MQTT Broker 地址')

    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port', default_value='1883',
        description='MQTT Broker 端口')

    prefix_arg = DeclareLaunchArgument(
        'topic_prefix', default_value='wheel_robot',
        description='MQTT 主题前缀')

    bridge_node = Node(
        package='mqtt_bridge',
        executable='mqtt_bridge',
        name='mqtt_bridge',
        output='screen',
        parameters=[{
            'mqtt_host': LaunchConfiguration('mqtt_host'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
            'topic_prefix': LaunchConfiguration('topic_prefix'),
        }],
    )

    return LaunchDescription([
        mqtt_host_arg,
        mqtt_port_arg,
        prefix_arg,
        bridge_node,
    ])
