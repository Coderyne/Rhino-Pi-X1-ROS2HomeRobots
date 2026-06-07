#!/usr/bin/env python3
"""
人体跟随节点 — LiDAR 质心漂移 + Kalman + 混合控制

近距离 (≤1.5m): 比例控制器直接 /cmd_vel → 响应快, 无避障
远距离 (>1.5m): /goal_pose 交给 Nav2 全局规划 + 避障

工作流程:
    1. Web 端框选目标 → /follow_target (PointStamped, map 坐标系)
    2. 每次激光扫描 → 在目标半径内找点云质心 → Kalman 平滑
    3. 根据距离选择控制策略: 直接速度控制 or Nav2 导航
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, Header
from visualization_msgs.msg import Marker
import tf2_ros


class KalmanFilter2D:
    """恒速模型 Kalman 滤波器 — 2D 位置 + 速度

    状态向量: [x, y, vx, vy]
    观测向量: [zx, zy]
    状态转移: 恒速模型 F = [[1,0,dt,0], [0,1,0,dt], [0,0,1,0], [0,0,0,1]]
    观测矩阵: H = [[1,0,0,0], [0,1,0,0]]

    用于平滑激光雷达检测到的人体位置, 减少抖动.
    """

    def __init__(self, process_noise=0.1, measurement_noise=0.05):
        """
        Args:
            process_noise: 过程噪声方差 q (状态转移不确定性)
            measurement_noise: 观测噪声方差 r (传感器测量不确定性)
        """
        self._dt = 0.05
        # 状态: [x, y, vx, vy]
        self._x = [0.0, 0.0, 0.0, 0.0]
        # 协方差矩阵 P (4x4, 展平为一维数组, 行主序)
        # 初始化: 位置协方差 100, 速度协方差 1
        self._P = [100.0]*4 + [0.0]*12
        for i in range(4):
            self._P[i*5] = [100.0, 100.0, 1.0, 1.0][i]
        self._q = process_noise
        self._r = measurement_noise

    def predict(self, dt=None):
        """预测步骤: 根据恒速模型推算下一时刻状态

        使用 dt 更新状态转移矩阵, 执行 x' = F·x, P' = F·P·F^T + Q
        """
        if dt is not None:
            self._dt = max(0.001, min(dt, 1.0))
        # 状态转移矩阵 F (4x4, 行主序)
        F = [1, 0, self._dt, 0, 0, 1, 0, self._dt, 0, 0, 1, 0, 0, 0, 0, 1]
        # x' = F @ x
        nx = [sum(F[i*4+k] * self._x[k] for k in range(4)) for i in range(4)]
        # tmp = F @ P
        tmp = [sum(F[i*4+k] * self._P[k*4+j] for k in range(4)) for i in range(4) for j in range(4)]
        # P' = tmp @ F^T + Q
        nP = [sum(tmp[i*4+k] * F[j*4+k] for k in range(4)) for i in range(4) for j in range(4)]
        # 添加过程噪声 Q (对角矩阵)
        for i in range(4):
            nP[i*5] += self._q
        self._x, self._P = nx, nP

    def update(self, zx, zy, dt=None):
        """更新步骤: 融合观测值修正状态

        使用标准 Kalman gain 公式: K = P·H^T·(H·P·H^T + R)^(-1)
        然后 x = x + K·(z - H·x), P = (I - K·H)·P

        Args:
            zx: 观测 x 坐标
            zy: 观测 y 坐标
            dt: 时间步长 (可选)
        """
        if dt is not None:
            self._dt = max(0.001, min(dt, 1.0))
        self.predict(dt)
        # 观测残差: y = z - H·x
        yx, yy = zx - self._x[0], zy - self._x[1]
        # S = H·P·H^T + R (2x2 观测协方差矩阵)
        s00, s01 = self._P[0] + self._r, self._P[1]
        s10, s11 = self._P[4], self._P[5] + self._r
        det = s00 * s11 - s01 * s10

        if abs(det) < 1e-12:
            return

        # Kalman gain K = P·H^T·S^(-1) (4x2 矩阵, 展平)
        K = [
            (self._P[0]*s11 - self._P[1]*s10)/det, (self._P[1]*s00 - self._P[0]*s01)/det,
            (self._P[4]*s11 - self._P[5]*s10)/det, (self._P[5]*s00 - self._P[4]*s01)/det,
            (self._P[8]*s11 - self._P[9]*s10)/det, (self._P[9]*s00 - self._P[8]*s01)/det,
            (self._P[12]*s11 - self._P[13]*s10)/det, (self._P[13]*s00 - self._P[12]*s01)/det,
        ]
        # 状态更新: x = x + K·y
        self._x[0] += K[0]*yx + K[1]*yy; self._x[1] += K[2]*yx + K[3]*yy
        self._x[2] += K[4]*yx + K[5]*yy; self._x[3] += K[6]*yx + K[7]*yy
        # 协方差更新: P = (I - K·H)·P
        ikh = [1-K[0], -K[1], 0, 0, -K[2], 1-K[3], 0, 0, -K[4], -K[5], 1, 0, -K[6], -K[7], 0, 1]
        nP = [sum(ikh[i*4+k] * self._P[k*4+j] for k in range(4)) for i in range(4) for j in range(4)]
        self._P = nP

    def state(self):
        """返回当前状态估计: (x, y, vx, vy)"""
        return self._x[0], self._x[1], self._x[2], self._x[3]

    def set_state(self, x, y):
        """重置状态到指定位置 (速度清零)"""
        self._x = [x, y, 0.0, 0.0]
        self._P = [1.0]*4 + [0.0]*12
        for i in range(4):
            self._P[i*5] = 1.0


class PersonFollower(Node):
    """人体跟随 ROS2 节点

    核心逻辑:
    1. 从激光扫描点云中提取目标半径内的质心
    2. Kalman 滤波平滑位置估计
    3. 混合控制:
       - dist ≤ 1.5m: 比例控制器直接发 /cmd_vel (快速响应)
       - dist > 1.5m: 通过 /goal_pose 委托 Nav2 规划和避障
    """

    def __init__(self):
        super().__init__('person_follower')

        # ── 参数声明 ─────────────────────────────────────
        self.declare_parameter('target_radius', 0.3)       # 目标搜索半径 (m)
        self.declare_parameter('follow_distance', 0.8)      # 期望跟随距离 (m)
        self.declare_parameter('robot_frame_front', 0.15)   # 机器人前方自遮挡 (m)
        self.declare_parameter('robot_frame_back', 0.35)    # 机器人后方自遮挡 (m)
        self.declare_parameter('robot_frame_side', 0.15)    # 机器人侧方自遮挡 (m)
        self.declare_parameter('goal_min_distance', 0.025)  # 最小位移阈值 (m, 低于此不发新 goal)
        self.declare_parameter('goal_min_interval', 0.5)    # 两次 goal 最小间隔 (s)
        self.declare_parameter('enable_kalman', True)       # 是否启用 Kalman 滤波
        self.declare_parameter('process_noise', 0.1)        # Kalman 过程噪声
        self.declare_parameter('measurement_noise', 0.05)   # Kalman 观测噪声

        self._target_radius = self.get_parameter('target_radius').value
        self._follow_distance = self.get_parameter('follow_distance').value
        self._robot_front = self.get_parameter('robot_frame_front').value
        self._robot_back = self.get_parameter('robot_frame_back').value
        self._robot_side = self.get_parameter('robot_frame_side').value
        self._goal_min_dist = self.get_parameter('goal_min_distance').value
        self._goal_min_interval = self.get_parameter('goal_min_interval').value

        # ── TF 缓冲区 ───────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── 跟踪状态 (base_link 坐标系下) ────────────────
        self._target_bl_x = None          # 目标在 base_link 下的 x
        self._target_bl_y = None          # 目标在 base_link 下的 y
        self._kf = KalmanFilter2D(
            process_noise=self.get_parameter('process_noise').value,
            measurement_noise=self.get_parameter('measurement_noise').value)
        self._follow_active = False       # 跟随激活状态

        # ── 限流状态 (避免高频重复发送 goal) ─────────────
        self._last_goal_map_x = None
        self._last_goal_map_y = None
        self._last_goal_time = None

        # ── 发布者 ───────────────────────────────────────
        self._pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_marker = self.create_publisher(Marker, '/follow_target_marker', 10)

        # ── 订阅者 ───────────────────────────────────────
        self._sub_scan = self.create_subscription(
            LaserScan, '/scan', self._on_scan, 10)
        self._sub_target = self.create_subscription(
            PointStamped, '/follow_target', self._on_target, 10)
        self._sub_active = self.create_subscription(
            Bool, '/follow_active', self._on_active, 10)
        self._sub_radius = self.create_subscription(
            Float32, '/follow_radius', self._on_radius, 10)

        self.get_logger().info('Person follower ready — click "跟随" on web to start')

    # ── TF 坐标转换 ─────────────────────────────────────

    def _bl_to_map(self, x, y):
        """将 base_link 坐标系下的点转换到 map 坐标系

        Args:
            x, y: base_link 坐标

        Returns:
            (map_x, map_y) 或 (None, None) 当 TF 不可用时
        """
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.5))
            # 从四元数解算旋转角 (绕 Z 轴)
            cos_t = math.cos(2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w))
            sin_t = math.sin(2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w))
            return (t.transform.translation.x + x * cos_t - y * sin_t,
                    t.transform.translation.y + x * sin_t + y * cos_t)
        except Exception:
            return None, None

    def _map_to_bl(self, x, y):
        """将 map 坐标系下的点转换到 base_link 坐标系

        Args:
            x, y: map 坐标

        Returns:
            (bl_x, bl_y) 或 (None, None) 当 TF 不可用时
        """
        try:
            t = self._tf_buffer.lookup_transform(
                'base_link', 'map', rclpy.time.Time(), timeout=Duration(seconds=0.5))
            cos_t = math.cos(2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w))
            sin_t = math.sin(2 * math.atan2(t.transform.rotation.z, t.transform.rotation.w))
            return (t.transform.translation.x + x * cos_t - y * sin_t,
                    t.transform.translation.y + x * sin_t + y * cos_t)
        except Exception:
            return None, None

    # ── 订阅回调 ───────────────────────────────────────

    def _on_active(self, msg: Bool):
        """跟随激活开关回调"""
        self._follow_active = msg.data
        if self._follow_active:
            # 激活时重置限流状态
            self._last_goal_map_x = None
            self._last_goal_map_y = None
            self._last_goal_time = None
        elif not self._follow_active:
            # 停用时立即发送零速停止机器人
            self._pub_cmd.publish(Twist())
        self.get_logger().info(f'Follow active: {self._follow_active}')

    def _on_radius(self, msg: Float32):
        """设置目标搜索半径 (来自 Web 端或 MQTT)"""
        self._target_radius = max(0.1, min(msg.data, 2.0))
        self.get_logger().info(f'Target radius set to: {self._target_radius:.2f}m')

    def _on_target(self, msg: PointStamped):
        """接收新目标 (Web 端框选) — 设置跟随目标位置

        将 map 坐标系下的目标转换到 base_link, 初始化 Kalman 状态.
        """
        bx, by = self._map_to_bl(msg.point.x, msg.point.y)
        if bx is None:
            self.get_logger().warn('Cannot transform /follow_target: map→base_link unavailable')
            return
        self._target_bl_x, self._target_bl_y = bx, by
        self._kf.set_state(bx, by)
        self._follow_active = True
        # 重置限流, 确保立即发布第一个 goal
        self._last_goal_map_x = None
        self._last_goal_map_y = None
        self._last_goal_time = None
        self.get_logger().info(
            f'Follow target: map({msg.point.x:.2f},{msg.point.y:.2f}) -> bl({bx:.2f},{by:.2f})')

    def _on_scan(self, msg: LaserScan):
        """激光扫描回调 — 实时更新目标位置

        核心算法: 质心漂移 (Centroid Drift)
        1. 过滤机器人自身遮挡的点
        2. 在目标半径内查找所有激光点
        3. 计算质心作为目标新位置
        4. 可选 Kalman 滤波平滑
        """
        if self._target_bl_x is None or self._target_bl_y is None:
            return

        # 步骤 1: 将激光扫描转为 (x, y) 点集, 排除机器人自身
        points = []
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                # 排除落在机器人矩形轮廓内的点 (自遮挡)
                if not (-self._robot_back < x < self._robot_front and
                        -self._robot_side < y < self._robot_side):
                    points.append((x, y))
            angle += msg.angle_increment

        if not points:
            return

        # 步骤 2: 在目标半径内找点云质心
        sum_x, sum_y, count = 0.0, 0.0, 0
        t2 = self._target_radius ** 2
        for x, y in points:
            dx = x - self._target_bl_x
            dy = y - self._target_bl_y
            if dx * dx + dy * dy < t2:
                sum_x += x; sum_y += y; count += 1

        # 点数过少时放弃更新 (防止误跟踪)
        if count <= 1:
            return

        new_x, new_y = sum_x / count, sum_y / count

        # 步骤 3: Kalman 滤波或直接使用质心
        if self.get_parameter('enable_kalman').value:
            self._kf.update(new_x, new_y)
            self._target_bl_x, self._target_bl_y, _, _ = self._kf.state()
        else:
            self._target_bl_x, self._target_bl_y = new_x, new_y

        # 步骤 4: 尝试发布控制指令
        self._try_publish()

    # ── 混合控制 — 近距离直控 / 远距离 Nav2 ────────────

    def _try_publish(self):
        """根据目标距离选择控制策略并发布指令

        策略:
        - dist ≤ 1.5m: 比例控制器 → /cmd_vel (快速响应)
        - dist > 1.5m:  导航目标 → /goal_pose (Nav2 避障)
        """
        if not self._follow_active:
            return
        if self._target_bl_x is None:
            return

        tx, ty = self._target_bl_x, self._target_bl_y
        dist = math.hypot(tx, ty)
        if dist < 0.01:
            return

        # 将目标位置转换到 map 坐标系 (用于可视化 / Nav2)
        target_mx, target_my = self._bl_to_map(tx, ty)
        if target_mx is None:
            return

        # ── 发布可视化标记 (绿色球体) ──────────────────
        marker = Marker()
        marker.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        marker.ns = 'follow_target'
        marker.id = 0
        marker.type = Marker.SPHERE; marker.action = Marker.ADD
        marker.pose.position.x = target_mx
        marker.pose.position.y = target_my
        marker.pose.position.z = 0.15
        marker.scale.x = 0.2; marker.scale.y = 0.2; marker.scale.z = 0.2
        marker.color.r = 0.25; marker.color.g = 0.88; marker.color.b = 0.35; marker.color.a = 0.8
        self._pub_marker.publish(marker)

        # ── 近距离: 比例控制器直接发 /cmd_vel ──────────
        if dist <= 1.5:
            err = dist - self._follow_distance     # 距离误差 (正=太远, 负=太近)
            angle = math.atan2(ty, tx)              # 目标方向角

            # P 控制器: 速度与误差成正比 (限幅)
            vx = max(-0.25, min(0.4, err * 0.5))
            wz = max(-0.6, min(0.6, angle * 1.0))

            cmd = Twist()
            cmd.linear.x = vx
            cmd.angular.z = wz
            self._pub_cmd.publish(cmd)
            return

        # ── 远距离: 委托 Nav2 规划 + 避障 ──────────────
        # 计算跟随点: 在目标方向上的 follow_distance 处
        ratio = max(0.0, 1.0 - self._follow_distance / dist)
        goal_mx, goal_my = self._bl_to_map(tx * ratio, ty * ratio)
        if goal_mx is None:
            return

        # 限流: 距离变化过小或时间间隔过短则跳过
        now = self.get_clock().now()
        if self._last_goal_map_x is not None and self._last_goal_time is not None:
            d = math.hypot(goal_mx - self._last_goal_map_x, goal_my - self._last_goal_map_y)
            dt = (now - self._last_goal_time).nanoseconds * 1e-9
            if d < self._goal_min_dist or dt < self._goal_min_interval:
                return

        self._last_goal_map_x = goal_mx
        self._last_goal_map_y = goal_my
        self._last_goal_time = now

        # 发布导航目标
        goal = PoseStamped()
        goal.header = Header(stamp=now.to_msg(), frame_id='map')
        goal.pose.position.x = goal_mx
        goal.pose.position.y = goal_my
        goal.pose.position.z = 0.0
        # 朝向: 面向目标方向
        rx, ry = self._bl_to_map(0.0, 0.0)
        if rx is not None:
            yaw = math.atan2(target_my - ry, target_mx - rx)
            goal.pose.orientation.z = math.sin(yaw * 0.5)
            goal.pose.orientation.w = math.cos(yaw * 0.5)
        else:
            goal.pose.orientation.w = 1.0
        self._pub_goal.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
