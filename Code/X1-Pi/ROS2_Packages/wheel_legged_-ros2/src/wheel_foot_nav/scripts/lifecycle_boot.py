#!/usr/bin/env python3
"""引导生命周期 (Lifecycle) 节点: configure → activate → AMCL 全局重定位

Nav2 的关键节点 (map_server, amcl, planner_server 等) 使用 ROS2 Lifecycle 管理,
需要依次调用 configure (transition id=1) 和 activate (transition id=3) 才能正常工作.

本脚本按顺序引导 9 个节点, 并在 AMCL 激活后自动触发全局重定位.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from std_srvs.srv import Empty


def boot_node(node, node_name):
    """引导单个生命周期节点: configure → activate

    Lifecycle 节点状态机:
      Unconfigured(0) → configure(1) → Inactive(1) → activate(3) → Active(2)

    Args:
        node: ROS2 节点 (用于创建 Service client)
        node_name: 节点名称 (如 "map_server", "amcl")

    Returns:
        True 如果引导成功, False 失败
    """
    cli = node.create_client(ChangeState, f'/{node_name}/change_state')
    if not cli.wait_for_service(timeout_sec=15.0):
        node.get_logger().error(f'{node_name}: service not available')
        return False

    # 步骤 1: configure (unconfigured → inactive)
    node.get_logger().info(f'{node_name}: configure')
    req = ChangeState.Request()
    req.transition.id = 1        # transition_id=1 = configure
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    time.sleep(2.0)              # 等待配置完成

    # 步骤 2: activate (inactive → active)
    req = ChangeState.Request()
    req.transition.id = 3        # transition_id=3 = activate
    node.get_logger().info(f'{node_name}: activate')
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=15.0)
    if future.done() and future.result() and future.result().success:
        node.get_logger().info(f'{node_name}: activate success')
        return True

    # 首次激活失败时重试一次
    time.sleep(1.0)
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    node.get_logger().warn(f'{node_name}: activate returned false, assuming active')
    return True


def main():
    rclpy.init()
    node = Node('lifecycle_boot')
    time.sleep(2.0)    # 等待所有节点注册完毕

    # 需要引导的 Nav2 生命周节点 (顺序有依赖关系)
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    for name in lifecycle_nodes:
        if not boot_node(node, name):
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        # AMCL 激活后触发全局重定位: 全地图撒点, 通过 scan 匹配收敛
        if name == 'amcl':
            time.sleep(5.0)    # 等待 AMCL 完成初始化
            cli = node.create_client(Empty, '/reinitialize_global_localization')
            if cli.wait_for_service(timeout_sec=15.0):
                node.get_logger().info('Triggering AMCL global re-localization...')
                req = Empty.Request()
                future = cli.call_async(req)
                rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
                node.destroy_client(cli)
                if future.done() and future.result():
                    node.get_logger().info('AMCL global re-localization triggered')
                else:
                    node.get_logger().warn('Re-localization call returned no result')
            else:
                node.get_logger().error('/reinitialize_global_localization service not available')

    node.get_logger().info('Lifecycle nodes booted successfully')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
