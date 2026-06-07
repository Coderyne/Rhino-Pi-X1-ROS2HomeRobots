"""
MQTT ↔ ROS2 消息映射与转换工具

定义:
- 数据类型转换函数: ROS msg ↔ dict
- 命令/状态映射表: 描述 MQTT Topic ↔ ROS Topic 的对应关系

本文件是 bridge_node.py 的路由依据, 修改映射表即可增减支持的命令.
"""

import math
import io
import base64

from std_msgs.msg import Bool, Float32, Float32MultiArray, Int8MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState, Imu
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker


# ═══════════════════════════════════════════════════════════════
#  四元数 ↔ 欧拉角
# ═══════════════════════════════════════════════════════════════

def yaw_from_quat(q):
    """从四元数提取偏航角 (yaw / heading)

    ROS 中 q 是 geometry_msgs/Quaternion, 使用标准公式:
        yaw = atan2(2*(w*z + x*y), 1 - 2*(y² + z²))
    """
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw):
    """偏航角 → 四元数 (仅绕 Z 轴旋转, 用于导航目标)

    返回 (x, y, z, w) 四元数元组, ROS 中 Z 轴朝上.
    """
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


# ═══════════════════════════════════════════════════════════════
#  ROS → dict 转换 (用于 -> MQTT 发布)
# ═══════════════════════════════════════════════════════════════

def occupancy_grid_to_png(msg, max_w=800):
    """ROS OccupancyGrid → PNG base64 图片

    用于 Web 仪表盘在地图上显示实时 SLAM 地图.
    - -1 (未知) → 灰色 (200,200,200)
    - 0 (空闲)  → 白色 (255,255,255)
    - 100 (障碍) → 黑色 (0,0,0)
    - 中间值 → 灰度渐变

    Args:
        msg: nav_msgs/OccupancyGrid
        max_w: 最大宽度, 超过则等比缩放

    Returns:
        dict: {width, height, resolution, origin_x, origin_y, image(base64)}
    """
    from PIL import Image

    w, h = msg.info.width, msg.info.height
    img = Image.new("RGB", (w, h))
    px = img.load()

    for y in range(h):
        for x in range(w):
            v = msg.data[y * w + x]
            if v < 0:
                px[x, h - 1 - y] = (200, 200, 200)   # 未知: 灰色
            elif v == 0:
                px[x, h - 1 - y] = (255, 255, 255)   # 空闲: 白色
            elif v >= 100:
                px[x, h - 1 - y] = (0, 0, 0)          # 障碍: 黑色
            else:
                g = int(255 - v * 2.55)
                px[x, h - 1 - y] = (g, g, g)          # 灰度渐变

    if w > max_w:
        r = max_w / w
        img = img.resize((max_w, int(h * r)), Image.NEAREST)

    buf = io.BytesIO()
    img.save(buf, format="PNG")
    return {
        "width": img.width,
        "height": img.height,
        "resolution": msg.info.resolution,
        "origin_x": msg.info.origin.position.x,
        "origin_y": msg.info.origin.position.y,
        "image": base64.b64encode(buf.getvalue()).decode(),
    }


def battery_conv(msg):
    """BatteryState → dict (电压 + 在位)
    用于 MQTT 上报电池状态.
    """
    return {"voltage": round(msg.voltage, 2), "present": msg.present}


def imu_conv(msg):
    """IMU → dict (欧拉角 度为单位)
    从四元数解算 roll / pitch / yaw.
    """
    q = msg.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return {
        "roll": round(math.degrees(roll), 1),
        "pitch": round(math.degrees(pitch), 1),
        "yaw": round(math.degrees(yaw_from_quat(q)), 1),
    }


def chassis_conv(msg):
    """底盘状态 (Int8MultiArray) → dict

    data[0]: start_flag  起步标志
    data[1]: jump_flag   跳跃标志
    data[2]: contact_L   左腿触地
    data[3]: contact_R   右腿触地
    """
    d = msg.data
    return {
        "start_flag": int(d[0]) if len(d) > 0 else 0,
        "jump_flag": int(d[1]) if len(d) > 1 else 0,
        "contact_L": int(d[2]) if len(d) > 2 else 0,
        "contact_R": int(d[3]) if len(d) > 3 else 0,
    }


def marker_conv(msg):
    """可视化标记 (Marker) → {x, y} 位置

    用于将 follow_target 标记位置转发到 MQTT.
    """
    return {"x": round(msg.pose.position.x, 3), "y": round(msg.pose.position.y, 3)}


# ═══════════════════════════════════════════════════════════════
#  dict → ROS 转换 (用于 <- MQTT 接收)
# ═══════════════════════════════════════════════════════════════

def dict_to_twist(data):
    """MQTT dict → geometry_msgs/Twist (速度指令)

    期望格式: {"linear": x, "angular": z}
    """
    msg = Twist()
    msg.linear.x = float(data.get("linear", 0))
    msg.angular.z = float(data.get("angular", 0))
    return msg


def dict_to_pose_stamped(data, frame="map"):
    """MQTT dict → geometry_msgs/PoseStamped (导航目标)

    期望格式: {"x": ..., "y": ..., "yaw": ...}
    内部调用 _quat_from_yaw 将偏航转为四元数.
    """
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = float(data.get("x", 0))
    msg.pose.position.y = float(data.get("y", 0))
    yaw = float(data.get("yaw", 0))
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = _quat_from_yaw(yaw)
    return msg


def dict_to_bool(data):
    """MQTT dict → std_msgs/Bool

    支持多种输入格式: 纯布尔值 / {"value": bool}
    """
    msg = Bool()
    msg.data = bool(data) if isinstance(data, bool) else bool(data.get("value", True)) if isinstance(data, dict) else True
    return msg


def dict_to_float32(data):
    """MQTT dict → std_msgs/Float32

    支持多种输入格式: 数字 / {"value": 数字}
    """
    msg = Float32()
    if isinstance(data, (int, float)):
        msg.data = float(data)
    elif isinstance(data, dict):
        msg.data = float(data.get("value", 0))
    else:
        msg.data = 0.0
    return msg


def dict_to_float32_multi(data):
    """MQTT dict → std_msgs/Float32MultiArray (姿态指令)

    期望格式: {"roll": r, "pitch": p, "leg": l}
    顺序: [roll, pitch, leg] 对应 /cmd_attitude 接口.
    """
    msg = Float32MultiArray()
    msg.data = [float(data.get(k, 0)) for k in ("roll", "pitch", "leg")]
    return msg


# ═══════════════════════════════════════════════════════════════
#  映射表 (bridge_node.py 依赖此处定义)
# ═══════════════════════════════════════════════════════════════

COMMAND_MAPPINGS = [
    # (MQTT子Topic, ROS Topic, ROS消息类型, 转换函数)
    ("cmd/velocity", "/cmd_vel", Twist, dict_to_twist),           # 速度控制
    ("cmd/goto", "/goal_pose", PoseStamped, dict_to_pose_stamped),  # 导航目标
    ("cmd/enable", "/cmd_enable", Bool, dict_to_bool),            # 使能
    ("cmd/estop", "/cmd_estop", Bool, dict_to_bool),              # 急停
    ("cmd/jump", "/cmd_jump", Bool, dict_to_bool),                # 跳跃
    ("cmd/recover", "/cmd_recover", Bool, dict_to_bool),          # 自起
    ("cmd/attitude", "/cmd_attitude", Float32MultiArray, dict_to_float32_multi),  # 姿态/腿长
    ("cmd/keep_alive", "/cmd_keep_alive", Bool, dict_to_bool),    # 持续心跳
    ("follow/active", "/follow_active", Bool, dict_to_bool),      # 人体跟随开关
    ("follow/radius", "/follow_radius", Float32, dict_to_float32),  # 跟随搜索半径
]

STATUS_MAPPINGS = [
    # (ROS Topic, ROS消息类型, MQTT子Topic, 转换函数, 限流频率Hz)
    ("/battery", BatteryState, "status/battery", battery_conv, 1.0),        # 电池状态 1Hz
    ("/imu/data", Imu, "status/imu", imu_conv, 5.0),                        # IMU姿态 5Hz
    ("/chassis_state", Int8MultiArray, "status/chassis", chassis_conv, 5.0), # 底盘状态 5Hz
    ("/follow_target_marker", Marker, "status/follow_target", marker_conv, 2.0),  # 跟随目标位置 2Hz
]

THROTTLED_MAPPINGS = [
    # 大体积数据: 全部使用限流, 避免 MQTT 链路拥塞
    ("/map", OccupancyGrid, "status/map", occupancy_grid_to_png, 1.0),      # SLAM地图 1Hz
]
