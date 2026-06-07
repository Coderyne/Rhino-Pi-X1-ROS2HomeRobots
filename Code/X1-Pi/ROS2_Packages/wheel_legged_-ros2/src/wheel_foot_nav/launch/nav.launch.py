#!/usr/bin/env python3
"""导航启动 — map_server + AMCL + Nav2 规划控制 (雷达由 start.sh 统一启动)"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='地图名称, 位于 maps/xxx.yaml (必传)')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='自动使能 Nav2 lifecycle 节点')

    pkg_share = FindPackageShare('wheel_foot_nav')
    map_dir = PathJoinSubstitution([pkg_share, 'maps'])
    map_file = LaunchConfiguration('map')
    map_yaml = PathJoinSubstitution([map_dir, map_file])
    nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'nav.rviz'])

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_params, {'yaml_filename': [map_yaml, TextSubstitution(text='.yaml')]}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params],
    )

    # 相机静态 TF — base_link → camera_depth_optical_frame (上方 10cm)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_depth_optical',
        arguments=['0', '0', '0.1', '-1.5708', '0', '-1.5708',
                   'base_link', 'camera_depth_optical_frame'],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'False',
        }.items(),
    )

    lifecycle_boot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='wheel_foot_nav',
                executable='lifecycle_boot.py',
                name='lifecycle_boot',
                output='screen',
            )
        ]
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('perception'), 'launch', 'person_follower.launch.py'
            ])
        ]),
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        autostart_arg,
        map_server,
        amcl,
        camera_tf,
        nav2_launch,
        lifecycle_boot,
        perception_launch,
        rviz,
    ])
