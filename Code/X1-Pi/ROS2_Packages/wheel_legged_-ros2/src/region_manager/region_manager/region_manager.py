#!/usr/bin/env python3
"""区域管理 ROS2 节点

在地图上划分命名区域, 支持:
- Web 端交互: 框选 → 命名 → 持久化 (JSON)
- 点击区域矩形 → 自动导航到区域中心
- RViz / Web 可视化 (MarkerArray)

接口:
  Service:
    /region_manager/list_regions (Trigger) → 返回所有区域 JSON

  Topic Subscribers:
    /region_manager/save     (String) — JSON {name, cx, cy, width, height, rotation, color}
    /region_manager/delete   (String) — JSON {name}
    /region_manager/navigate (String) — JSON {name}

  Topic Publishers:
    /region_manager/regions  (String)     — 全量区域数据, 每次变更后发布
    /region_manager/response (String)     — 操作结果通知
    /region_markers          (MarkerArray) — 地图可视化 (CUBE + TEXT)
"""

import json
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .region_store import RegionStore, DEFAULT_COLOR


class RegionManager(Node):
    """区域管理节点

    使用 RegionStore 进行数据持久化 (JSON 文件),
    发布 MarkerArray 用于 RViz / Web 地图可视化.
    """

    def __init__(self):
        super().__init__('region_manager')

        # ── 参数 ──
        self.declare_parameter('regions_file', '')
        self.declare_parameter('map_frame', 'map')

        regions_file = self.get_parameter('regions_file').value
        if not regions_file:
            regions_file = os.path.join(os.path.expanduser('~'), 'robot_regions.json')

        self._map_frame = self.get_parameter('map_frame').value
        self._store = RegionStore(regions_file)

        # ── 订阅者 ──
        self._sub_save = self.create_subscription(
            String, '/region_manager/save', self._on_save, 10)
        self._sub_delete = self.create_subscription(
            String, '/region_manager/delete', self._on_delete, 10)
        self._sub_navigate = self.create_subscription(
            String, '/region_manager/navigate', self._on_navigate, 10)

        # ── Service ──
        self._srv_list = self.create_service(
            Trigger, '/region_manager/list_regions', self._on_list)

        # ── 发布者 ──
        self._pub_regions = self.create_publisher(
            String, '/region_manager/regions', 10)
        self._pub_response = self.create_publisher(
            String, '/region_manager/response', 10)
        self._pub_goal = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self._pub_markers = self.create_publisher(
            MarkerArray, '/region_markers', 10)

        # 启动时发布已有区域
        self._publish_regions()
        self._publish_markers()

        self.get_logger().info(
            'Region manager ready, {} regions loaded'.format(len(self._store.all())))

    # ── 工具: 发送响应消息 ────────────────────────────────

    def _respond(self, success, message, operation):
        """发布操作结果到 /region_manager/response (JSON 格式)"""
        resp = json.dumps({
            'success': success,
            'message': message,
            'operation': operation,
        }, ensure_ascii=False)
        self._pub_response.publish(String(data=resp))

    # ── 订阅回调 ──────────────────────────────────────────

    def _on_save(self, msg):
        """保存区域: 解析 JSON → 持久化 → 更新可视化

        期望格式:
        {
            "name": "客厅",
            "cx": -1.5, "cy": 2.0,
            "width": 2.0, "height": 2.0,
            "rotation": 0.0,
            "color": "#FF6B6B"
        }
        """
        try:
            data = json.loads(msg.data)
            name = data.get('name', '').strip()
            if not name:
                self._respond(False, '区域名称不能为空', 'save')
                return
            cx = float(data['cx'])
            cy = float(data['cy'])
            width = float(data['width'])
            height = float(data['height'])
            rotation = float(data.get('rotation', 0.0))
            color = data.get('color', DEFAULT_COLOR)
            self._store.add(name, cx, cy, width, height, rotation, color)
            self._publish_regions()
            self._publish_markers()
            self._respond(True, "区域 '{}' 已保存".format(name), 'save')
            self.get_logger().info("Saved region '{}'".format(name))
        except Exception as e:
            self._respond(False, '保存失败: {}'.format(str(e)), 'save')
            self.get_logger().error('Save region error: {}'.format(e))

    def _on_delete(self, msg):
        """删除区域: 按名称删除并更新可视化"""
        try:
            data = json.loads(msg.data)
            name = data.get('name', '').strip()
            if self._store.delete(name):
                self._publish_regions()
                self._publish_markers()
                self._respond(True, "区域 '{}' 已删除".format(name), 'delete')
                self.get_logger().info("Deleted region '{}'".format(name))
            else:
                self._respond(False, "区域 '{}' 不存在".format(name), 'delete')
        except Exception as e:
            self._respond(False, '删除失败: {}'.format(str(e)), 'delete')

    def _on_navigate(self, msg):
        """导航到区域: 查找区域中心 → 发布 PoseStamped goal

        发布到 /goal_pose 即可被 Nav2 接收并规划路径.
        """
        try:
            data = json.loads(msg.data)
            name = data.get('name', '').strip()
            region = self._store.get(name)
            if not region:
                self._respond(False, "区域 '{}' 不存在".format(name), 'navigate')
                return
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self._map_frame
            goal.pose.position.x = region['cx']
            goal.pose.position.y = region['cy']
            goal.pose.orientation.w = 1.0
            self._pub_goal.publish(goal)
            self._respond(True, "正在导航到 '{}'".format(name), 'navigate')
            self.get_logger().info("Navigating to '{}'".format(name))
        except Exception as e:
            self._respond(False, '导航失败: {}'.format(str(e)), 'navigate')

    # ── Service 回调 ──────────────────────────────────────

    def _on_list(self, request, response):
        """列出所有区域: 返回 JSON 数组"""
        regions = self._store.all()
        response.success = True
        response.message = json.dumps(regions, ensure_ascii=False)
        return response

    # ── 发布 ──────────────────────────────────────────────

    def _publish_regions(self):
        """发布全量区域数据到 /region_manager/regions (String JSON)"""
        regions = self._store.all()
        self._pub_regions.publish(
            String(data=json.dumps(regions, ensure_ascii=False)))

    def _publish_markers(self):
        """发布 MarkerArray 可视化到 /region_markers

        每个区域渲染两个 Marker:
        1. CUBE — 半透明彩色矩形 (表示区域位置和大小)
        2. TEXT_VIEW_FACING — 白色文字标签 (显示区域名称)

        用于 RViz 和 Web 仪表盘的地图面板.
        """
        import math
        markers = MarkerArray()
        regions = self._store.all()

        # 先清除所有旧标记
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        for i, region in enumerate(regions):
            # 解析 HEX 颜色 → RGB
            color_hex = region.get('color', DEFAULT_COLOR).lstrip('#')
            r_ = int(color_hex[0:2], 16) / 255.0
            g_ = int(color_hex[2:4], 16) / 255.0
            b_ = int(color_hex[4:6], 16) / 255.0

            w = region['width']
            h = region['height']
            rot = region.get('rotation', 0.0)
            half_yaw = rot / 2.0
            qz = math.sin(half_yaw)
            qw = math.cos(half_yaw)

            # CUBE: 半透明矩形
            cube = Marker()
            cube.header.frame_id = self._map_frame
            cube.header.stamp = self.get_clock().now().to_msg()
            cube.ns = 'region'
            cube.id = i
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = region['cx']
            cube.pose.position.y = region['cy']
            cube.pose.position.z = 0.01
            cube.pose.orientation.z = qz
            cube.pose.orientation.w = qw
            cube.scale.x = w
            cube.scale.y = h
            cube.scale.z = 0.02
            cube.color.r = r_
            cube.color.g = g_
            cube.color.b = b_
            cube.color.a = 0.3           # 透明度
            markers.markers.append(cube)

            # TEXT: 区域名称标签
            label = Marker()
            label.header.frame_id = self._map_frame
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = 'region_label'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING   # 始终面向相机
            label.action = Marker.ADD
            label.pose.position.x = region['cx']
            label.pose.position.y = region['cy']
            label.pose.position.z = 0.06
            label.scale.z = 0.3                     # 文字高度
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            label.text = region['name']
            markers.markers.append(label)

        self._pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = RegionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
