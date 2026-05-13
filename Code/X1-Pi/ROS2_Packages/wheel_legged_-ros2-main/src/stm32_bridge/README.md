# stm32_bridge — 轮足机器人 STM32 USB CDC 通讯桥接节点

> ROS2 Humble | Python | 物理层 `/dev/ttyACM0` (USB CDC VCP)

---

## 目录

- [1. 概述](#1-概述)
- [2. 硬件连接](#2-硬件连接)
- [3. 通讯协议](#3-通讯协议)
- [4. ROS2 接口](#4-ros2-接口)
- [5. 节点架构](#5-节点架构)
- [6. 安装与编译](#6-安装与编译)
- [7. 运行](#7-运行)
- [8. 测试](#8-测试)
- [9. 文件说明](#9-文件说明)

---

## 1. 概述

`stm32_bridge` 是轮足机器人上位机与 STM32H723VGT6 底盘主控之间的 ROS2 通讯桥接节点，通过 USB CDC 虚拟串口实现双向数据交换：

| 方向 | 频率 | 帧类型 | 功能 |
|------|------|--------|------|
| STM32 → ROS2 (遥测) | 100 Hz | Type 0x01 | IMU / 里程计 / 关节 / 电池 / 状态标志 |
| ROS2 → STM32 (指令) | 50 Hz | Type 0x02 | 速度 / 姿态 / 腿长 / 控制标志 |

**核心特性**：

- **逐字节状态机解析** — 兼容 USB CDC 拆包，不假设单次 `read()` 返回完整帧
- **CRC16-CCITT 校验** — 严格匹配下位机多项式 `0x1021`，初始值 `0xFFFF`
- **超时保护** — 连续 100 ms 未收到 ENABLE 指令时自动停车
- **串口自动探测** — 自动匹配 `/dev/ttyACM*` 设备

---

## 2. 硬件连接

```
┌───────────────┐     USB OTG HS (CDC VCP)      ┌────────────────┐
│  上位机        │ ◄────────────────────────────► │  STM32H723VGT6 │
│  (Jetson/PC)  │     /dev/ttyACM0               │  (底盘主控)     │
└───────────────┘                                └────────────────┘
```

- USB CDC 为虚拟串口，`baudrate` 参数对物理层无实际影响，保留为 `115200`
- 设备路径默认 `/dev/ttyACM0`，若插入多个 CDC 设备会自动探测 `/dev/ttyACM*`

---

## 3. 通讯协议

### 3.1 帧格式（统一）

```
┌─────────┬─────────┬────────┬────────┬──────────────────┬──────────┐
│ Header0 │ Header1 │ Type   │ Len    │ Payload          │ CRC16    │
│ 0xAA    │ 0x55    │ 1 byte │ 1 byte │ Len bytes         │ 2 bytes  │
└─────────┴─────────┴────────┴────────┴──────────────────┴──────────┘
                                            │                      │
                                            └─ CRC16 over ────────┘
                                 CRC 计算范围: Type + Len + Payload
```

- **CRC16-CCITT**: `poly=0x1021`, `init=0xFFFF`, 小端序 (LSB first)

### 3.2 遥测帧 (Type 0x01, 96 bytes payload)

Python struct: `<23f4B` (23 × float + 4 × uint8, 96 bytes)

| 序号 | 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|------|
| 0 | timestamp | f | s | 下位机启动秒数 |
| 1 | roll | f | rad | IMU 横滚角 |
| 2 | pitch | f | rad | IMU 俯仰角 |
| 3 | yaw | f | rad | IMU 偏航角 |
| 4-6 | gyro_x/y/z | f | rad/s | 机体角速度 (X前 Y左 Z上) |
| 7-9 | accel_x/y/z | f | m/s² | 世界系加速度 |
| 10 | vel_n | f | m/s | 世界系水平速度 |
| 11 | pos_n | f | m | 世界系水平位移 |
| 12 | theta_L | f | rad | 左腿摆角 |
| 13 | L0_L | f | m | 左腿等效腿长 |
| 14 | wheel_T_L | f | Nm | 左轮毂力矩 |
| 15 | Tp_L | f | Nm | 左髋关节力矩 |
| 16 | d_theta_L | f | rad/s | 左腿摆角速度 |
| 17-21 | theta_R... | f | — | 右腿对应字段 |
| 22 | battery_voltage | f | V | 电池电压 |
| 23 | start_flag | B | — | 0=停止, 1=运行 |
| 24 | jump_flag | B | — | 跳跃阶段 (>0 表示跳跃中) |
| 25 | contact_L | B | — | 左腿着地 (0=离地, 1=着地) |
| 26 | contact_R | B | — | 右腿着地 |

### 3.3 指令帧 (Type 0x02, 25 bytes payload)

Python struct: `<6fB` (6 × float + 1 × uint8, 25 bytes)

| 序号 | 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|------|
| 0 | timestamp | f | s | ROS2 时间戳 |
| 1 | v_set | f | m/s | 目标前进速度 (>0 前进) |
| 2 | yaw_rate_set | f | rad/s | 目标偏航角速度 (>0 左转) |
| 3 | roll_set | f | rad | 目标横滚角 (±0.4) |
| 4 | leg_set | f | m | 目标腿长 (0.072~0.165) |
| 5 | pitch_set | f | rad | 俯仰偏移量 |
| 6 | cmd_flags | B | — | 控制标志位 |

### 3.4 cmd_flags 位定义

| Bit | 常量 | 值 | 说明 |
|-----|------|-----|------|
| 0 | `CMD_ENABLE` | 0x01 | 使能底盘控制 (必须置 1 下位机才响应) |
| 1 | `CMD_JUMP` | 0x02 | 触发跳跃序列 (上升沿) |
| 2 | `CMD_ESTOP` | 0x04 | 紧急停止 (最高优先级) |
| 3 | `CMD_RECOVER` | 0x08 | 倒地自起 |

---

## 4. ROS2 接口

### 4.1 发布的话题 (Publisher)

| Topic | 消息类型 | 频率 | 内容 |
|-------|----------|------|------|
| `/imu/data` | `sensor_msgs/Imu` | 100 Hz | 四元数方向 + 角速度 + 加速度 |
| `/odom` | `nav_msgs/Odometry` | 100 Hz | 线速度 `vel_n` + 角速度 + 偏航角 |
| `/joint_states` | `sensor_msgs/JointState` | 100 Hz | L_hip, L_wheel, R_hip, R_wheel 位置/速度/力矩 |
| `/battery` | `sensor_msgs/BatteryState` | 100 Hz | 电池电压 |
| `/chassis_state` | `std_msgs/Int8MultiArray` | 100 Hz | `[start_flag, jump_flag, contact_L, contact_R]` |

### 4.2 订阅的话题 (Subscriber)

| Topic | 消息类型 | 映射字段 | 说明 |
|-------|----------|----------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | `linear.x → v_set`<br>`angular.z → yaw_rate_set` | 速度控制, 收到后自动使能 |
| `/cmd_attitude` | `std_msgs/Float32MultiArray` | `data[0] → roll_set`<br>`data[1] → pitch_set`<br>`data[2] → leg_set` | 姿态/腿长控制 |
| `/cmd_enable` | `std_msgs/Bool` | `True → flags \|= 0x01` | 手动使能 |
| `/cmd_estop` | `std_msgs/Bool` | `True → flags \|= 0x04` | 紧急停止 (单次触发) |
| `/cmd_jump` | `std_msgs/Bool` | `True → flags \|= 0x02` | 跳跃触发 (单次触发) |
| `/cmd_recover` | `std_msgs/Bool` | `True → flags \|= 0x08` | 倒地恢复 (单次触发) |

### 4.3 参数 (Parameters)

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `port` | string | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | int | `115200` | 波特率 (CDC 无效, 保留) |
| `telemetry_rate` | double | `100.0` | 遥测读取频率 (Hz) |
| `cmd_rate` | double | `50.0` | 指令下发频率 (Hz) |
| `timeout_ms` | int | `100` | ENABLE 超时时间 (ms) |
| `enable_at_start` | bool | `false` | 启动时是否自动使能 |

---

## 5. 节点架构

```
┌─────────────────────────────────────────────────────────┐
│                     WheelFootBridge                      │
│                                                          │
│  ┌──────────────────┐     ┌─────────────────────────┐   │
│  │  100Hz Timer      │────►│  _read_serial()         │   │
│  │  (telemetry_tick) │     │  非阻塞读取全部可用字节  │   │
│  └──────────────────┘     │  逐字节喂入 FrameParser   │   │
│                            │  完整帧 → _handle_frame() │   │
│                            └──────────┬───────────────┘   │
│                                       │                   │
│                            ┌──────────▼───────────────┐   │
│                            │  _handle_telemetry()      │   │
│                            │  struct.unpack <23f4B     │   │
│                            │  → /imu /odom /joints     │   │
│                            │    /battery /state        │   │
│                            └───────────────────────────┘   │
│                                                            │
│  ┌──────────────────┐     ┌─────────────────────────┐     │
│  │  /cmd_vel          │────►│  _on_cmd_vel()          │     │
│  │  /cmd_attitude     │────►│  _on_cmd_attitude()     │     │
│  │  /cmd_enable       │────►│  _on_enable()           │     │
│  │  /cmd_estop        │────►│  _on_estop()            │     │
│  │  /cmd_jump         │────►│  _on_jump()             │     │
│  │  /cmd_recover      │────►│  _on_recover()          │     │
│  └──────────────────┘     └──────────┬───────────────┘     │
│                                       │                    │
│  ┌──────────────────┐     ┌──────────▼───────────────┐     │
│  │  50Hz Timer       │────►│  _on_command_tick()      │     │
│  │  (command_tick)   │     │  构建 cmd_flags          │     │
│  └──────────────────┘     │  pack_command() → 串口   │     │
│                            └──────────────────────────┘     │
└─────────────────────────────────────────────────────────┘
```

**超时保护逻辑**：

```
每个 cmd_tick 检查:
   if enable AND (now - last_enable_time) > timeout_ms:
       enable = False
       log warning
```

---

## 6. 安装与编译

### 6.1 依赖

```bash
pip3 install pyserial
```

系统已依赖（`package.xml` 声明）：
- `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`

### 6.2 编译

```bash
cd ~/ros2_dev
source /opt/ros/humble/setup.bash
colcon build --packages-select stm32_bridge
source install/setup.bash
```

---

## 7. 运行

### 7.1 launch 启动

```bash
ros2 launch stm32_bridge bridge.launch.py
```

带参数覆盖：

```bash
ros2 launch stm32_bridge bridge.launch.py \
    port:=/dev/ttyACM1 \
    cmd_rate:=50.0 \
    timeout_ms:=200
```

### 7.2 直接启动节点

```bash
ros2 run stm32_bridge wheel_foot_bridge --ros-args \
    -p port:=/dev/ttyACM0 \
    -p cmd_rate:=50.0
```

### 7.3 控制示例

```bash
# 使能底盘
ros2 topic pub /cmd_enable std_msgs/msg/Bool "data: true" -1

# 前进 0.3 m/s, 左转 0.1 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.1}}"

# 设置腿长 0.12 m
ros2 topic pub /cmd_attitude std_msgs/msg/Float32MultiArray \
    "{data: [0.0, 0.0, 0.12]}"

# 跳跃
ros2 topic pub /cmd_jump std_msgs/msg/Bool "data: true" -1

# 急停
ros2 topic pub /cmd_estop std_msgs/msg/Bool "data: true" -1
```

---

## 8. 测试

### 8.1 脱离硬件测试 — 串口自环

使用虚拟串口（`socat`）测试协议帧的打包/解包：

```bash
# 创建虚拟串口对
socat -d -d pty,raw,echo=0 pty,raw,echo=0

# 终端1: 运行 bridge 连接 /dev/pts/X
ros2 run stm32_bridge wheel_foot_bridge --ros-args -p port:=/dev/pts/X

# 终端2: 发送模拟遥测帧
python3 -c "
from stm32_bridge.protocol import pack_frame, TYPE_TELEMETRY
import struct, serial

tlm = struct.pack('<23f4B',
    1.0, 0.0, 0.1, 0.5,          # ts, roll, pitch, yaw
    0.01, -0.02, 0.03,           # gyro
    0.0, 0.0, 9.8,               # accel
    0.0, 0.0,                    # vel, pos
    0.0, 0.1, 0.0, 0.0, 0.0,    # left leg
    0.0, 0.1, 0.0, 0.0, 0.0,    # right leg
    12.6,                        # battery
    1, 0, 1, 1)                  # flags

frame = pack_frame(TYPE_TELEMETRY, tlm)
ser = serial.Serial('/dev/pts/Y', 115200)
ser.write(frame)
ser.close()
"
```

### 8.2 导入自检

```bash
source install/setup.bash
python3 -c "from stm32_bridge.protocol import *; print('protocol OK')"
python3 -c "from stm32_bridge.wheel_foot_bridge import WheelFootBridge; print('bridge OK')"
```

---

## 9. 文件说明

| 文件 | 说明 |
|------|------|
| `protocol.py` | CRC16-CCITT, 帧打包/解包, 逐字节状态机 FrameParser, 遥测/指令编解码 |
| `wheel_foot_bridge.py` | ROS2 节点主逻辑 (`WheelFootBridge`) |
| `bridge.launch.py` | 启动文件 |
| `params.yaml` | 默认参数文件 |
| `package.xml` | 依赖声明 (`rclpy`, `sensor_msgs`, `nav_msgs` 等) |
| `setup.py` | 安装注册和 `console_scripts` 入口点 |
