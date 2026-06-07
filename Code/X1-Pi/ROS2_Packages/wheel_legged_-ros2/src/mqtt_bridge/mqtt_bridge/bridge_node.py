"""
MQTT ↔ ROS2 双向桥接节点

订阅 MQTT 指令 → 转发为 ROS2 Topic
订阅 ROS2 Topic → 转发为 MQTT 消息
支持: 速度控制 / 导航 / 使能 / 急停 / 跳跃 / 人体跟随 / 巡逻
"""

import json
import math
from collections import deque
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import paho.mqtt.client as mqtt

from nav_msgs.msg import Odometry
from .topic_mapping import (
    COMMAND_MAPPINGS, STATUS_MAPPINGS, THROTTLED_MAPPINGS,
    dict_to_pose_stamped, yaw_from_quat,
)


class ThrottledSub:
    """限流订阅器 — 以固定频率转发 ROS Topic → MQTT

    相比直接订阅回调, 这个类以 rate_hz 频率采样最新消息,
    避免高频 Topic 打爆 MQTT 链路。
    """

    def __init__(self, node, ros_topic, msg_type, mqtt_topic, converter, rate_hz):
        """
        Args:
            node: ROS2 节点
            ros_topic: 要订阅的 ROS Topic 名
            msg_type: 消息类型
            mqtt_topic: 转发到的 MQTT Topic 后缀
            converter: 消息转换函数 ROS→dict
            rate_hz: 转发频率 (0=不限流, 直接回调)
        """
        self.node = node
        self.mqtt_topic = mqtt_topic
        self.converter = converter
        self.last_msg = None

        # 订阅 ROS Topic, 每次收到消息更新缓存
        node.create_subscription(msg_type, ros_topic, self._cb, 10)

        # 以固定频率从缓存中取最新消息转发
        if rate_hz > 0:
            period = 1.0 / rate_hz
            node.create_timer(period, self._timer_cb)

    def _cb(self, msg):
        """ROS 回调: 缓存最新消息"""
        self.last_msg = msg

    def _timer_cb(self):
        """定时器回调: 将缓存的消息转换后发布到 MQTT"""
        if self.last_msg is not None:
            try:
                data = self.converter(self.last_msg)
                self.node._publish_mqtt(self.mqtt_topic, data)
            except Exception as e:
                self.node.get_logger().error(f"Throttled {self.mqtt_topic}: {e}")


class MqttBridge(Node):
    """MQTT ↔ ROS2 桥接主节点

    工作流程:
        MQTT 指令 → _on_message → _process_cmd_queue → _handle_cmd → ROS Publisher
        ROS Topic  → _status_cb / ThrottledSub → _publish_mqtt → MQTT Broker

    支持 MQTT 接入: Home Assistant / WebApp / 手机 App 等
    """

    def __init__(self):
        super().__init__("mqtt_bridge")

        # ── 参数声明 ─────────────────────────────────────
        self.declare_parameter("mqtt_host", "127.0.0.1")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_username", "")
        self.declare_parameter("mqtt_password", "")
        self.declare_parameter("topic_prefix", "wheel_robot")
        self.declare_parameter("location_map", {})
        self.declare_parameter("battery_min_voltage", 21.0)
        self.declare_parameter("battery_max_voltage", 25.2)
        self.declare_parameter("goal_tolerance", 0.2)
        self.declare_parameter("goal_reached_seconds", 2.0)
        self.declare_parameter("patrol_interval_seconds", 3.0)
        self.mqtt_host = self.get_parameter("mqtt_host").value
        self.mqtt_port = self.get_parameter("mqtt_port").value
        self.mqtt_user = self.get_parameter("mqtt_username").value
        self.mqtt_pass = self.get_parameter("mqtt_password").value
        self.topic_prefix = self.get_parameter("topic_prefix").value
        self.location_map = self.get_parameter("location_map").value or {}
        self.goal_tolerance = self.get_parameter("goal_tolerance").value
        self.goal_reached_sec = self.get_parameter("goal_reached_seconds").value
        self.patrol_interval = self.get_parameter("patrol_interval_seconds").value

        # ── 状态变量 ─────────────────────────────────────
        self.cmd_queue = deque()         # MQTT 消息队列 (FIFO)
        self.mqtt_connected = False      # MQTT 连接状态
        self.ros_publishers = {}         # 缓存的 ROS Publisher 对象
        self.throttled = []              # ThrottledSub 列表

        # 巡逻模式状态
        self.patrol_active = False       # 巡逻是否激活
        self.patrol_waypoints = []       # 巡逻航点列表
        self.patrol_index = 0            # 当前航点索引
        self.patrol_timer = None         # 巡逻检查定时器

        # 导航目标追踪状态
        self.last_goal_xy = None         # 上次设置的导航目标 (x, y)
        self.goal_close_count = 0        # 连续 close 计次 (用于去抖)
        self.nav_status = "idle"         # idle / navigating / reached
        self.current_odom = None         # 最近一次里程计数据

        self._setup_mqtt()
        self._setup_ros()

        # 命令队列处理定时器 10Hz
        self.create_timer(0.1, self._process_cmd_queue)
        self.get_logger().info(f"bridge started: {self.mqtt_host}:{self.mqtt_port} prefix={self.topic_prefix}")

    # ── MQTT 初始化 ────────────────────────────────────

    def _setup_mqtt(self):
        """初始化 MQTT 客户端, 注册回调并连接 Broker"""
        self.mc = mqtt.Client()
        self.mc.on_connect = self._on_connect
        self.mc.on_disconnect = self._on_disconnect
        self.mc.on_message = self._on_message

        if self.mqtt_user:
            self.mc.username_pw_set(self.mqtt_user, self.mqtt_pass)
        self.mc.will_set(f"{self.topic_prefix}/status/bridge", "offline", retain=True)

        try:
            self.mc.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            self.mc.loop_start()
        except Exception as e:
            self.get_logger().error(f"MQTT connect: {e}")

    def _on_connect(self, client, userdata, flags, rc):
        """MQTT 连接成功回调 — 订阅所有指令 Topic"""
        if rc == 0:
            self.mqtt_connected = True
            self.get_logger().info("MQTT connected")
            client.publish(f"{self.topic_prefix}/status/bridge", "online", retain=True)
            # 批量订阅命令 Topic: cmd/velocity, cmd/goto, cmd/enable ...
            for t, _, _, _ in COMMAND_MAPPINGS:
                client.subscribe(f"{self.topic_prefix}/{t}")
            client.subscribe(f"{self.topic_prefix}/patrol/start")
        else:
            self.get_logger().error(f"MQTT rc={rc}")

    def _on_disconnect(self, client, userdata, rc):
        """MQTT 断开回调"""
        self.mqtt_connected = False

    def _on_message(self, client, userdata, msg):
        """MQTT 消息接收回调 — 入队待处理"""
        self.cmd_queue.append(msg)

    # ── ROS 初始化 ─────────────────────────────────────

    def _setup_ros(self):
        """初始化 ROS 发布者与订阅者

        根据 COMMAND_MAPPINGS 创建 ROS Publisher (MQTT → ROS)
        根据 STATUS_MAPPINGS / THROTTLED_MAPPINGS 创建 ROS Subscription (ROS → MQTT)
        """
        # 指令发布者: MQTT 过来的命令发给对应 ROS Topic
        for t, topic, msg_type, conv in COMMAND_MAPPINGS:
            self.ros_publishers[topic] = self.create_publisher(msg_type, topic, 10)

        # 状态订阅者: ROS Topic 转 MQTT (含限流)
        for topic, msg_type, mqtt_t, conv, rate in STATUS_MAPPINGS:
            if rate > 0:
                # 高频 Topic 使用 ThrottledSub 限流
                self.throttled.append(ThrottledSub(self, topic, msg_type, mqtt_t, conv, rate))
            else:
                # 低频 Topic 直接回调
                self.create_subscription(msg_type, topic, partial(self._status_cb, mqtt_t=mqtt_t, conv=conv), 10)

        # 大体积 Topic 全部使用 ThrottledSub (如地图)
        for topic, msg_type, mqtt_t, conv, rate in THROTTLED_MAPPINGS:
            self.throttled.append(ThrottledSub(self, topic, msg_type, mqtt_t, conv, rate))

        # 里程计单独订阅 (用于目标到达检测)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    # ── ROS 回调 ───────────────────────────────────────

    def _status_cb(self, msg, mqtt_t, conv):
        """ROS 状态 Topic 回调 — 直接转发到 MQTT"""
        if self.mqtt_connected:
            try:
                data = conv(msg)
                self._publish_mqtt(mqtt_t, data)
            except Exception as e:
                self.get_logger().error(f"status {mqtt_t}: {e}")

    def _on_odom(self, msg):
        """里程计回调 — 更新位置并检查目标到达"""
        self.current_odom = msg
        if self.mqtt_connected:
            q = msg.pose.pose.orientation
            data = {
                "x": round(msg.pose.pose.position.x, 3),
                "y": round(msg.pose.pose.position.y, 3),
                "yaw": round(yaw_from_quat(q), 3),
                "linear_x": round(msg.twist.twist.linear.x, 3),
                "angular_z": round(msg.twist.twist.angular.z, 3),
            }
            self._publish_mqtt("status/odometry", data)
            self._check_goal()

    # ── MQTT 发布 ──────────────────────────────────────

    def _publish_mqtt(self, sub, data):
        """发布 JSON 消息到 MQTT (自动拼接 topic prefix)"""
        self.mc.publish(f"{self.topic_prefix}/{sub}", json.dumps(data), qos=1)

    # ── 命令队列处理 ───────────────────────────────────

    def _process_cmd_queue(self):
        """10Hz 轮询: 依次处理 MQTT 消息队列中的指令"""
        while self.cmd_queue:
            m = self.cmd_queue.popleft()
            self._handle_cmd(m.topic, m.payload)

    def _handle_cmd(self, topic, payload):
        """解析并执行 MQTT 指令

        匹配 COMMAND_MAPPINGS 中的指令, 转换为 ROS 消息并发布.
        非标准指令如 patrol/start 单独处理.

        Args:
            topic: MQTT 完整 Topic
            payload: JSON 字符串
        """
        short = topic.replace(f"{self.topic_prefix}/", "", 1)
        try:
            data = json.loads(payload)
        except Exception:
            return

        # 匹配标准命令 (cmd/velocity, cmd/goto, cmd/enable ...)
        for t, ros_topic, _, conv in COMMAND_MAPPINGS:
            if short == t:
                try:
                    self.ros_publishers[ros_topic].publish(conv(data))
                    # 导航目标需要额外跟踪到达状态
                    if ros_topic == "/goal_pose":
                        self._track_goal(data)
                except Exception as e:
                    self.get_logger().error(f"cmd {short}: {e}")
                return

        # 非标准命令: 启动巡逻
        if short == "patrol/start":
            self._handle_patrol(data)

    # ── 导航目标追踪 ──────────────────────────────────

    def _track_goal(self, data):
        """记录导航目标, 用于后续到达检测"""
        self.last_goal_xy = (float(data.get("x", 0)), float(data.get("y", 0)))
        self.goal_close_count = 0
        self.nav_status = "navigating"
        self._publish_mqtt("status/nav_state", {"state": "navigating"})

    def _check_goal(self):
        """检查机器人是否到达目标

        使用 tolerance 距离判断 + 连续计次去抖.
        到达后自动触发巡逻下一站 (如果处于巡逻模式).
        """
        if self.last_goal_xy is None or self.current_odom is None:
            return
        gx, gy = self.last_goal_xy
        p = self.current_odom.pose.pose.position
        dist = math.hypot(p.x - gx, p.y - gy)
        # 连续多帧在 tolerance 内才算到达 (防抖动)
        self.goal_close_count = self.goal_close_count + 1 if dist < self.goal_tolerance else 0
        if self.goal_close_count >= int(self.goal_reached_sec / 0.1):
            self.nav_status = "reached"
            self.last_goal_xy = None
            self._publish_mqtt("status/nav_state", {"state": "reached"})
            self._advance_patrol()

    # ── 巡逻模式 ───────────────────────────────────────

    def _handle_patrol(self, data):
        """启动巡逻模式: 解析航点列表, 发送第一个航点

        巡逻工作流程:
        _handle_patrol → _send_patrol → 到达 → _advance_patrol → _send_patrol → ... → _done_patrol
        """
        wps = data.get("waypoints", [])
        if not wps:
            return
        self.patrol_waypoints = wps
        self.patrol_index = 0
        self.patrol_active = True
        self._send_patrol()
        if self.patrol_timer:
            self.patrol_timer.cancel()
        # 定期检查是否到达, 自动切换到下一站
        self.patrol_timer = self.create_timer(self.patrol_interval, self._check_patrol)
        self._publish_mqtt("status/patrol", {"active": True, "current": 1, "total": len(wps)})

    def _send_patrol(self):
        """发送当前索引的巡逻航点到导航系统"""
        if self.patrol_index >= len(self.patrol_waypoints):
            self._done_patrol()
            return
        wp = self.patrol_waypoints[self.patrol_index]
        goal = {"x": float(wp[0]), "y": float(wp[1]), "yaw": float(wp[2]) if len(wp) > 2 else 0.0}
        self.ros_publishers["/goal_pose"].publish(dict_to_pose_stamped(goal))
        self._track_goal(goal)

    def _check_patrol(self):
        """巡逻检查定时器回调: 到达后自动前进到下一站"""
        if self.patrol_active and self.nav_status == "reached":
            self._advance_patrol()

    def _advance_patrol(self):
        """前进到下一个巡逻航点, 全部完成则结束巡逻"""
        if not self.patrol_active:
            return
        self.patrol_index += 1
        if self.patrol_index >= len(self.patrol_waypoints):
            self._done_patrol()
            return
        self._send_patrol()
        self._publish_mqtt("status/patrol", {
            "active": True, "current": self.patrol_index + 1,
            "total": len(self.patrol_waypoints),
        })

    def _done_patrol(self):
        """结束巡逻模式, 清理定时器"""
        self.patrol_active = False
        if self.patrol_timer:
            self.patrol_timer.cancel()
            self.patrol_timer = None
        self._publish_mqtt("status/patrol", {"active": False})


def main():
    rclpy.init()
    node = MqttBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
