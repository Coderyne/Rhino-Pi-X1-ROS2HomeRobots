#!/usr/bin/env python3
"""
轮足机器人 STM32 USB CDC 串口通讯桥接节点 (WheelFootBridge)

职责:
  - 物理层: /dev/ttyACM0 (USB CDC VCP)
  - 遥测接收 100Hz (Type=0x01): 解析 IMU/Odom/Joint/Battery/State
  - 指令下发 50Hz  (Type=0x02): 速度 / 姿态 / 腿长 / 控制标志
  - TF 广播: odom → base_footprint (基于里程计积分)
  - 超时保护: 100ms 未收到 ENABLE 指令则自动停车
  - 逐字节状态机解析, 兼容 CDC 拆包

TF 树:
  odom → base_footprint   (本节点发布, 通过积分 vel_n 和 yaw 获得)
  base_footprint → base_link  (静态)
  base_link → base_laser      (雷达驱动发布)
"""

import glob
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.timer import Timer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration

import serial

from geometry_msgs.msg import TransformStamped, Twist, Vector3, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, BatteryState
from std_msgs.msg import Header, Float32MultiArray, Bool, Int8MultiArray
from tf2_ros import TransformBroadcaster

from .protocol import (
    FrameParser, parse_telemetry, pack_command,
    TYPE_TELEMETRY, TYPE_COMMAND,
    CMD_ENABLE, CMD_JUMP, CMD_ESTOP, CMD_RECOVER,
)


class WheelFootBridge(Node):
    """STM32 串口通讯桥接主节点

    整体数据流:
      STM32 串口 → FrameParser (状态机) → parse_telemetry → ROS Topics
      ROS Topics → 回调更新状态 → pack_command → 串口 → STM32

    两个独立定时器:
      - 遥测定时器 (100Hz): 读取串口数据
      - 指令定时器 (50Hz):  下发速度/姿态/标志位
    """

    def __init__(self):
        super().__init__('wheel_foot_bridge')

        # ── 参数声明 ─────────────────────────────────────────
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('telemetry_rate', 100.0)    # 遥测读取频率
        self.declare_parameter('cmd_rate', 50.0)            # 指令下发频率
        self.declare_parameter('timeout_ms', 100)            # 使能超时保护 (ms)
        self.declare_parameter('enable_at_start', False)     # 启动时是否自动使能
        self.declare_parameter('keep_alive', True)           # 持续发送指令帧

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._tlm_rate = self.get_parameter('telemetry_rate').value
        self._cmd_rate = self.get_parameter('cmd_rate').value
        self._timeout_ms = self.get_parameter('timeout_ms').value

        # ── 串口 ─────────────────────────────────────────────
        self._ser = None
        self._parser = FrameParser()
        self._try_open_serial()

        # ── 控制状态 ─────────────────────────────────────────
        self._v_set = 0.0            # 目标线速度 (m/s)
        self._yaw_rate_set = 0.0     # 目标角速度 (rad/s)
        self._roll_set = 0.0         # 目标横滚角 (rad)
        self._leg_set = 0.14         # 目标腿长 (m), 默认 0.14m
        self._pitch_set = 0.0        # 目标俯仰角 (rad)
        self._enable = self.get_parameter('enable_at_start').value   # 使能状态
        self._jump_pending = False   # 跳跃请求 (一次性)
        self._estop_active = False   # 急停激活
        self._recover_active = False # 自起请求 (一次性)
        self._last_enable_time = time.time()   # 上次收到 enable 的时间
        self._keep_alive = self.get_parameter('keep_alive').value    # 持续发送模式
        self._last_user_flags = 0    # 记忆的用户按钮状态
        self._cmd_dirty = False      # 用户触发了操作, 需要至少发送一帧

        # ── 遥测缓存 ─────────────────────────────────────────
        self._last_tlm = None

        # ── 里程计积分 (odom → base_footprint) ───────────────
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._odom_last_tlm_stamp = None  # 上次遥测时间戳

        # ── 发布者 ───────────────────────────────────────────
        self._pub_imu      = self.create_publisher(Imu, '/imu/data', 10)
        self._pub_odom     = self.create_publisher(Odometry, '/odom', 10)
        self._pub_joints   = self.create_publisher(JointState, '/joint_states', 10)
        self._pub_battery  = self.create_publisher(BatteryState, '/battery', 10)
        self._pub_state    = self.create_publisher(Int8MultiArray, '/chassis_state', 10)
        self._pub_cmd_debug = self.create_publisher(Float32MultiArray, '/cmd_frame_debug', 10)

        # ── TF 广播 ──────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)

        # ── 订阅者 ───────────────────────────────────────────
        self._sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._sub_attitude = self.create_subscription(
            Float32MultiArray, '/cmd_attitude', self._on_cmd_attitude, 10)
        self._sub_estop = self.create_subscription(
            Bool, '/cmd_estop', self._on_estop, 10)
        self._sub_jump = self.create_subscription(
            Bool, '/cmd_jump', self._on_jump, 10)
        self._sub_recover = self.create_subscription(
            Bool, '/cmd_recover', self._on_recover, 10)
        self._sub_enable = self.create_subscription(
            Bool, '/cmd_enable', self._on_enable, 10)
        self._sub_keep_alive = self.create_subscription(
            Bool, '/cmd_keep_alive', self._on_keep_alive, 10)

        # ── 定时器 ───────────────────────────────────────────
        # 遥测读取定时器 (独立回调组, 避免与指令下发互相阻塞)
        cbg = MutuallyExclusiveCallbackGroup()
        self._tlm_timer = self.create_timer(
            1.0 / self._tlm_rate, self._on_telemetry_tick, callback_group=cbg)

        cmd_cbg = MutuallyExclusiveCallbackGroup()
        self._cmd_timer = self.create_timer(
            1.0 / self._cmd_rate, self._on_command_tick, callback_group=cmd_cbg)

        self.get_logger().info(f'Bridge initialized, port={self._port}')

    # ── 串口管理 ─────────────────────────────────────────────

    def _try_open_serial(self) -> bool:
        """尝试打开串口, 支持自动探测 /dev/ttyACM*

        如果配置的端口打不开, 自动尝试 /dev/ttyACM0, /dev/ttyACM1 ...
        用于 Jetson 等设备上 USB 端口号不固定的场景.
        """
        ports = [self._port] if self._port else []

        if not ports or self._port == '/dev/ttyACM0':
            acm_ports = sorted(glob.glob('/dev/ttyACM*'))
            ports = acm_ports if acm_ports else [self._port]

        for p in ports:
            try:
                self._ser = serial.Serial(
                    p, baudrate=self._baudrate, timeout=0.005,
                    write_timeout=0.01)
                self._port = p
                self._parser.reset()
                self.get_logger().info(f'Serial port opened: {p}')
                return True
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to open {p}: {e}')

        self.get_logger().error('No serial port available')
        return False

    def _read_serial(self):
        """非阻塞读取串口所有可用数据, 喂入帧解析器

        使用 in_waiting 查询缓冲区大小, 一次性读取所有字节,
        逐字节喂给 FrameParser 状态机. 当收到完整帧时,
        _handle_frame 被自动调用.
        """
        if self._ser is None or not self._ser.is_open:
            return

        try:
            n = self._ser.in_waiting
            if n == 0:
                return
            data = self._ser.read(n)
        except serial.SerialException:
            return

        for byte in data:
            result = self._parser.feed(byte)
            if result is not None:
                self._handle_frame(result)

    def _write_serial(self, frame: bytes):
        """写入完整帧到串口, 失败时尝试重连"""
        if self._ser is None or not self._ser.is_open:
            return
        try:
            self._ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self._try_open_serial()

    # ── 帧处理 ───────────────────────────────────────────────

    def _handle_frame(self, result: tuple):
        """帧解析完成回调 — 根据帧类型分派"""
        frame_type, payload = result
        if frame_type == TYPE_TELEMETRY:
            self._handle_telemetry(payload)

    def _handle_telemetry(self, payload: bytes):
        """处理遥测帧: 更新里程计积分 + 发布 ROS Topics

        从 STM32 下位机收到的遥测数据包含:
          - IMU: roll / pitch / yaw / 陀螺仪 / 加速度
          - 里程: vel_n (线速度)
          - 关节: 左右腿摆角 / 腿长 / 力矩
          - 电池: 电压
          - 状态: 起步 / 跳跃 / 触地标志

        里程计积分公式:
          x += vel_n * cos(yaw) * dt
          y += vel_n * sin(yaw) * dt
        """
        tlm = parse_telemetry(payload)
        if tlm is None:
            return

        self._last_tlm = tlm
        now = self.get_clock().now()
        stamp = now.to_msg()

        # ── 里程计积分 (基于 STM32 时间戳) ──────────────
        if self._odom_last_tlm_stamp is not None:
            dt = tlm['timestamp'] - self._odom_last_tlm_stamp
            if dt > 0.0 and dt < 1.0:   # 有效时间差, 防跳变
                yaw = tlm['yaw']
                vel_n = tlm['vel_n']
                self._odom_x += vel_n * math.cos(yaw) * dt
                self._odom_y += vel_n * math.sin(yaw) * dt
                self._odom_yaw = yaw
        self._odom_last_tlm_stamp = tlm['timestamp']

        # ── TF: odom → base_footprint ───────────────────
        t = TransformStamped()
        t.header = Header(stamp=stamp, frame_id='odom')
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self._odom_x
        t.transform.translation.y = self._odom_y
        t.transform.translation.z = 0.0
        q = self._rpy_to_quat(0.0, 0.0, self._odom_yaw)
        t.transform.rotation = q
        self._tf_broadcaster.sendTransform(t)

        # ── IMU ──────────────────────────────────────────
        imu = Imu()
        imu.header = Header(stamp=stamp, frame_id='imu_link')
        imu.orientation = self._rpy_to_quat(tlm['roll'], tlm['pitch'], tlm['yaw'])
        imu.angular_velocity = Vector3(
            x=tlm['gyro_x'], y=tlm['gyro_y'], z=tlm['gyro_z'])
        imu.linear_acceleration = Vector3(
            x=tlm['accel_x'], y=tlm['accel_y'], z=tlm['accel_z'])
        self._pub_imu.publish(imu)

        # ── Odometry ─────────────────────────────────────
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id='odom')
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation = self._rpy_to_quat(0.0, 0.0, self._odom_yaw)
        odom.twist.twist.linear.x = tlm['vel_n']
        odom.twist.twist.angular.z = tlm['gyro_z']
        self._pub_odom.publish(odom)

        # ── JointState (左右腿关节) ──────────────────────
        joints = JointState()
        joints.header = Header(stamp=stamp, frame_id='')
        joints.name = [
            'L_hip', 'L_wheel',
            'R_hip', 'R_wheel',
        ]
        joints.position = [
            tlm['theta_L'], tlm['L0_L'],    # 左髋摆角 / 左腿长
            tlm['theta_R'], tlm['L0_R'],    # 右髋摆角 / 右腿长
        ]
        joints.velocity = [
            tlm['d_theta_L'], 0.0,           # 左髋角速度
            tlm['d_theta_R'], 0.0,           # 右髋角速度
        ]
        joints.effort = [
            tlm['Tp_L'], tlm['wheel_T_L'],   # 左髋力矩 / 左轮力矩
            tlm['Tp_R'], tlm['wheel_T_R'],   # 右髋力矩 / 右轮力矩
        ]
        self._pub_joints.publish(joints)

        # ── Battery ──────────────────────────────────────
        bat = BatteryState()
        bat.header = Header(stamp=stamp, frame_id='')
        bat.voltage = tlm['battery_voltage']
        bat.present = True
        self._pub_battery.publish(bat)

        # ── State flags ──────────────────────────────────
        state = Int8MultiArray()
        state.data = [
            tlm['start_flag'],
            tlm['jump_flag'],
            tlm['contact_L'],
            tlm['contact_R'],
        ]
        self._pub_state.publish(state)

    # ── 定时器回调 ───────────────────────────────────────────

    def _on_telemetry_tick(self):
        """遥测定时器回调 (100Hz): 读取串口数据"""
        self._read_serial()

    def _on_command_tick(self):
        """指令定时器回调 (50Hz): 打包并下发控制指令帧

        逻辑:
        1. 超时检测: 使能状态下超过 100ms 未收到任何指令 → 自动停使能
        2. keep_alive 关闭时, 仅有 cmd_dirty 时发送一帧 (省带宽)
        3. 一次性指令 (跳跃/自起) 发送一次即清除
        4. 调试消息发布到 /cmd_frame_debug
        """
        now = time.time()

        # 超时保护: 使能后 100ms 内无新指令则自动停使能
        if self._enable and (now - self._last_enable_time) > self._timeout_ms / 1000.0:
            self.get_logger().warn('Enable timeout, auto-disabling')
            self._enable = False

        # 非 keep-alive 模式: 仅在有实际变化时发送一帧
        if not self._keep_alive and not self._cmd_dirty:
            return

        # 组装 flags: 用户状态 + 一次性指令
        flags = self._last_user_flags
        if self._jump_pending:
            flags |= CMD_JUMP
            self._jump_pending = False       # 跳跃是一次性指令, 发送后清除
        if self._recover_active:
            flags |= CMD_RECOVER
            self._recover_active = False     # 自起也是一次性指令

        timestamp = self.get_clock().now().nanoseconds / 1e9

        frame = pack_command(
            timestamp=timestamp,
            v_set=self._v_set,
            yaw_rate_set=self._yaw_rate_set,
            roll_set=self._roll_set,
            leg_set=self._leg_set,
            pitch_set=self._pitch_set,
            flags=flags,
        )

        self._write_serial(frame)

        # 发布调试消息 (Web 端 /cmd_frame_debug 可查看当前下发值)
        msg = Float32MultiArray()
        msg.data = [float(timestamp), float(self._v_set), float(self._yaw_rate_set),
                    float(self._roll_set), float(self._leg_set), float(self._pitch_set),
                    float(flags)]
        self._pub_cmd_debug.publish(msg)
        self._cmd_dirty = False

    # ── 订阅回调 ─────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        """速度指令回调: 更新目标速度, 自动使能

        收到任意速度指令时自动使能底盘 (安全考虑: 需配合超时保护).
        """
        self._v_set = msg.linear.x
        self._yaw_rate_set = msg.angular.z
        self._enable = True
        self._last_enable_time = time.time()
        self._cmd_dirty = True

    def _on_cmd_attitude(self, msg: Float32MultiArray):
        """姿态指令回调: 更新目标 roll / pitch / leg

        msg.data[0]: roll  (横滚角)
        msg.data[1]: pitch (俯仰角)
        msg.data[2]: leg   (腿长)
        """
        if len(msg.data) >= 1:
            self._roll_set = msg.data[0]
        if len(msg.data) >= 2:
            self._pitch_set = msg.data[1]
        if len(msg.data) >= 3:
            self._leg_set = msg.data[2]
        self._enable = True
        self._last_enable_time = time.time()
        self._cmd_dirty = True

    def _on_enable(self, msg: Bool):
        """使能指令回调"""
        self._enable = msg.data
        if self._enable:
            self._last_enable_time = time.time()
            self._last_user_flags = CMD_ENABLE
        self._cmd_dirty = True

    def _on_keep_alive(self, msg: Bool):
        """持续心跳模式开关

        开启: 50Hz 持续下发指令帧 (延迟小但带宽大)
        关闭: 仅在有操作时发送 (省带宽, 适合无线/慢速链路)
        """
        self._keep_alive = msg.data
        self.get_logger().info(f'Keep-alive: {self._keep_alive}')

    def _on_estop(self, msg: Bool):
        """急停指令: 设置急停标志 + 清除使能"""
        if msg.data:
            self._estop_active = True
            self._enable = False
            self._last_user_flags = CMD_ESTOP
            self.get_logger().warn('ESTOP triggered')
            self._cmd_dirty = True

    def _on_jump(self, msg: Bool):
        """跳跃指令 (一次性): 标记 pending, 下次指令帧附带"""
        if msg.data:
            self._jump_pending = True
            self.get_logger().info('Jump command received')
            self._cmd_dirty = True

    def _on_recover(self, msg: Bool):
        """自起指令 (一次性): 标记 pending, 下次指令帧附带"""
        if msg.data:
            self._recover_active = True
            self.get_logger().info('Recover command received')
            self._cmd_dirty = True

    # ── 工具方法 ─────────────────────────────────────────────

    @staticmethod
    def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> Quaternion:
        """欧拉角 (RPY) → 四元数

        使用标准公式:
          w = cr·cp·cy + sr·sp·sy
          x = sr·cp·cy - cr·sp·sy
          y = cr·sp·cy + sr·cp·sy
          z = cr·cp·sy - sr·sp·cy
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q


def main(args=None):
    rclpy.init(args=args)
    node = WheelFootBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭串口
        if node._ser and node._ser.is_open:
            node._ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
