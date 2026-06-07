# 轮足机器人 STM32–ROS2 通信协议

> 下位机：STM32H723VGT6 + FreeRTOS + USB CDC (VCP)  
> 物理层：USB OTG HS Full-Speed (12 Mbps), 虚拟串口 `/dev/ttyACM0`  
> 遥测频率：100 Hz  
> 指令频率：上限 100 Hz（建议 ≥50 Hz 以维持使能状态）

---

## 一、帧格式（统一格式）

```
┌──────────┬──────────┬────────┬────────┬──────────────────┬──────────┐
│ Header0  │ Header1  │ Type   │ Len    │ Payload          │ CRC16    │
│ 0xAA     │ 0x55     │ 1 byte │ 1 byte │ Len bytes         │ 2 bytes  │
└──────────┴──────────┴────────┴────────┴──────────────────┴──────────┘
                                                │                      │
                                                └─ CRC16 over ────────┘
                                     CRC 计算范围: Type + Len + Payload
                                     CRC: CRC16-CCITT, poly=0x1021, init=0xFFFF
```

| 偏移 | 大小 | 字段 | 说明 |
|------|------|------|------|
| 0 | 2 | Header | `0xAA 0x55` |
| 2 | 1 | Type | `0x01`=遥测, `0x02`=指令 |
| 3 | 1 | Len | Payload 字节数 |
| 4 | Len | Payload | 见下文结构体定义 |
| 4+Len | 2 | CRC16 | LSB first, 覆盖 Offset 2 到 4+Len-1 |

### 帧长速查

| 帧类型 | Payload | 总帧长 |
|--------|---------|--------|
| 遥测 (0x01) | **96 bytes** | **102 bytes** |
| 指令 (0x02) | **25 bytes** | **31 bytes** |

---

## 二、遥测帧（STM32 → ROS2, Type=0x01, 100Hz）

### Python struct 解包格式

```python
TLM_FORMAT = '<23f4B'       # 23个float + 4个uint8, little-endian, packed
TLM_SIZE   = 23*4 + 4       # = 96 bytes
```

### 字段表

| 序号 | 字段 | Python struct | 单位 | 说明 |
|------|------|---------------|------|------|
| 0 | timestamp | f | s | 下位机启动后秒数 (DWT) |
| 1 | roll | f | rad | IMU 横滚角 |
| 2 | pitch | f | rad | IMU 俯仰角 |
| 3 | yaw | f | rad | IMU 偏航角 |
| 4 | gyro_x | f | rad/s | 机体 X 轴角速度 |
| 5 | gyro_y | f | rad/s | 机体 Y 轴角速度 |
| 6 | gyro_z | f | rad/s | 机体 Z 轴角速度 |
| 7 | accel_x | f | m/s² | 世界系 X 轴加速度 |
| 8 | accel_y | f | m/s² | 世界系 Y 轴加速度 |
| 9 | accel_z | f | m/s² | 世界系 Z 轴加速度 |
| 10 | vel_n | f | m/s | 世界系水平速度 |
| 11 | pos_n | f | m | 世界系水平位移 |
| 12 | theta_L | f | rad | 左腿摆角 |
| 13 | L0_L | f | m | 左腿等效腿长 |
| 14 | wheel_T_L | f | Nm | 左轮毂输出力矩 |
| 15 | Tp_L | f | Nm | 左髋关节输出力矩 |
| 16 | d_theta_L | f | rad/s | 左腿摆角速度 |
| 17 | theta_R | f | rad | 右腿摆角 |
| 18 | L0_R | f | m | 右腿等效腿长 |
| 19 | wheel_T_R | f | Nm | 右轮毂输出力矩 |
| 20 | Tp_R | f | Nm | 右髋关节输出力矩 |
| 21 | d_theta_R | f | rad/s | 右腿摆角速度 |
| 22 | battery_voltage | f | V | 电池电压 |
| 23 | start_flag | B | — | 0=停止, 1=运行 |
| 24 | jump_flag | B | — | 跳跃状态机阶段 (0=无跳跃) |
| 25 | contact_L | B | — | 左腿着地 (0=离地, 1=着地) |
| 26 | contact_R | B | — | 右腿着地 (0=离地, 1=着地) |

### 坐标系说明

- **gyro / body frame**: X=前, Y=左, Z=上 (ENU-like body)
- **accel / world frame**: 绝对水平坐标系 (MotionAccel_n)
- **roll/pitch/yaw**: 经 EKF 融合后的欧拉角
- **vel_n / pos_n**: 世界系水平面内沿运动方向的速度和位移（由 INS 积分）

---

## 三、指令帧（ROS2 → STM32, Type=0x02）

### Python struct 打包格式

```python
CMD_FORMAT = '<6fB'         # 6个float + 1个uint8, little-endian, packed
CMD_SIZE   = 6*4 + 1        # = 25 bytes
```

### 字段表

| 序号 | 字段 | Python struct | 单位 | 说明 |
|------|------|---------------|------|------|
| 0 | timestamp | f | s | ROS2 时间戳 (ros::Time::now().toSec()) |
| 1 | v_set | f | m/s | 目标前进速度 (>0 前进) |
| 2 | yaw_rate_set | f | rad/s | 目标偏航角速度 (>0 左转) |
| 3 | roll_set | f | rad | 目标横滚角 (范围建议 ±0.4) |
| 4 | leg_set | f | m | 目标腿长 (范围 0.072~0.165) |
| 5 | pitch_set | f | rad | 俯仰偏移量 |
| 6 | cmd_flags | B | — | 控制标志位 |

### cmd_flags 位定义

| Bit | 宏 | 值 | 说明 |
|-----|-----|-----|------|
| 0 | `ROS2_CMD_ENABLE` | 0x01 | **使能底盘控制** — 必须置1下位机才响应 |
| 1 | `ROS2_CMD_JUMP` | 0x02 | 触发跳跃序列 (上升沿, 两腿同时起跳) |
| 2 | `ROS2_CMD_ESTOP` | 0x04 | 紧急停止 (最高优先级, 忽略其他位) |
| 3 | `ROS2_CMD_RECOVER` | 0x08 | 倒地自起 |

### 下位机行为细节

- **正常控制**: `cmd_flags=0x01` + 有效运动参数 → 下位机积分 `yaw_rate_set` 和 `v_set` 更新目标位姿
- **跳跃**: `cmd_flags=0x03` (ENABLE+JUMP) → 仅在 `jump_flag==0` 时触发一次
- **急停**: `cmd_flags=0x04` → 立即关闭电机, 回零所有目标
- **自起**: `cmd_flags=0x08` → 进入倒地恢复模式 (leg_set 强制 0.08m)
- **超时保护**: 下位机连续 **100 ms** 未收到有效 `ENABLE` 指令时, 自动停车

---

## 四、上位机实现要点

### 4.1 串口打开

```python
import serial
ser = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0.01)
# baudrate 对 USB CDC 是虚拟的, 设为任意值均可
```

### 4.2 帧同步与解析

> ⚠️ CDC 传输可能将一帧拆成多个 USB 包, 必须用字节级状态机拼帧，**不能假设一次 read 就是一帧**。

```python
HEADER = b'\xaa\x55'

class FrameParser:
    def __init__(self):
        self.buf = bytearray()
        self.state = 'WAIT_H0'

    def feed(self, byte: int) -> bytes | None:
        """每次喂 1 个字节, 收到完整帧时返回 payload bytes, 否则返回 None"""
        self.buf.append(byte)
        if self.state == 'WAIT_H0':
            if byte == 0xAA:
                self.state = 'WAIT_H1'
            else:
                self.buf.clear()
        elif self.state == 'WAIT_H1':
            if byte == 0x55:
                self.state = 'WAIT_TYPE'
            elif byte != 0xAA:
                self.state = 'WAIT_H0'
                self.buf.clear()
            else:
                self.buf = bytearray([0xAA])
        elif self.state == 'WAIT_TYPE':
            self.state = 'WAIT_LEN'
        elif self.state == 'WAIT_LEN':
            self.payload_len = byte
            self.state = 'WAIT_DATA'
        elif self.state == 'WAIT_DATA':
            hdr_len = 4
            needed = hdr_len + self.payload_len + 2  # +2 for CRC
            if len(self.buf) >= needed:
                frame = bytes(self.buf[:needed])
                self.buf.clear()
                self.state = 'WAIT_H0'
                # verify CRC
                crc_calc = crc16_ccitt(frame[2:hdr_len + self.payload_len + 2])
                if crc_calc == 0:
                    return frame[hdr_len:hdr_len + self.payload_len]
        return None
```

### 4.3 CRC16-CCITT

```python
def crc16_ccitt(data: bytes) -> int:
    """匹配下位机的 CRC16-CCITT, poly=0x1021, init=0xFFFF"""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc
```

### 4.4 遥测解析

```python
import struct

TLM_FMT = '<23f4B'
TLM_SIZE = struct.calcsize(TLM_FMT)  # 96

def parse_telemetry(payload: bytes) -> dict:
    fields = struct.unpack(TLM_FMT, payload)
    return {
        'timestamp':       fields[0],
        'roll':            fields[1],
        'pitch':           fields[2],
        'yaw':             fields[3],
        'gyro_x':          fields[4],
        'gyro_y':          fields[5],
        'gyro_z':          fields[6],
        'accel_x':         fields[7],
        'accel_y':         fields[8],
        'accel_z':         fields[9],
        'vel_n':           fields[10],
        'pos_n':           fields[11],
        'theta_L':         fields[12],
        'L0_L':            fields[13],
        'wheel_T_L':       fields[14],
        'Tp_L':            fields[15],
        'd_theta_L':       fields[16],
        'theta_R':         fields[17],
        'L0_R':            fields[18],
        'wheel_T_R':       fields[19],
        'Tp_R':            fields[20],
        'd_theta_R':       fields[21],
        'battery_voltage': fields[22],
        'start_flag':      fields[23],
        'jump_flag':       fields[24],
        'contact_L':       fields[25],
        'contact_R':       fields[26],
    }
```

### 4.5 指令打包

```python
import struct

CMD_FMT = '<6fB'

def pack_command(timestamp: float, v_set: float, yaw_rate: float,
                 roll: float, leg: float, pitch: float,
                 enable: bool, jump: bool = False) -> bytes:
    flags = 0
    if enable:
        flags |= 0x01
    if jump:
        flags |= 0x02
    payload = struct.pack(CMD_FMT, timestamp, v_set, yaw_rate,
                          roll, leg, pitch, flags)
    # 组帧
    frame = bytearray([0xAA, 0x55, 0x02, len(payload)])
    frame += payload
    crc_data = frame[2:]
    crc = crc16_ccitt(crc_data + b'\x00\x00')  # placeholder
    # 实际需要计算后填入
    crc2 = crc16_ccitt(crc_data)
    frame += struct.pack('<H', crc2)
    return bytes(frame)
```

### 4.6 ROS2 话题映射建议

| STM32 数据 | ROS2 Topic | ROS2 Msg Type |
|------------|------------|---------------|
| roll/pitch/yaw | `/imu/data` | `sensor_msgs/Imu` |
| gyro_x/y/z | (同上) | |
| accel_x/y/z | (同上) | |
| vel_n, pos_n | `/odom` | `nav_msgs/Odometry` |
| theta_L/R, L0_L/R, d_theta_L/R | `/joint_states` | `sensor_msgs/JointState` |
| wheel_T_L/R, Tp_L/R | `/effort_states` | 自定义或 `/joint_states` effort 字段 |
| battery_voltage | `/battery` | `sensor_msgs/BatteryState` |
| contact_L/R | `/contact` | 自定义或 `std_msgs/Bool` |
| start_flag, jump_flag | `/state` | 自定义 |

| ROS2 控制源 | STM32 字段 | 说明 |
|-------------|------------|------|
| `cmd_vel.linear.x` | `v_set` | 前进速度 |
| `cmd_vel.angular.z` | `yaw_rate_set` | 偏航角速度 |
| 自定义 topic `/cmd_attitude` | `roll_set` | 横滚角 |
| 自定义 topic `/cmd_attitude` | `leg_set` | 腿长 |
| `/cmd_estop` (Bool) | `cmd_flags | 0x04` | 急停 |
| `/cmd_jump` (Bool) | `cmd_flags | 0x02` | 跳跃 |

---

## 五、AI 提示词模板

以下提示词可直接发给 AI 助手以生成 ROS2 上位机 Python 代码:

---

### 基础版提示词

```
我有一台通过 USB CDC 虚拟串口与 STM32 通信的轮足机器人, 需要编写
ROS2 (Humble) Python 节点, 实现双向通信。请根据以下协议完成代码:

1. 物理连接: /dev/ttyACM0, 波特率对 CDC 无效可忽略
2. 帧格式: 0xAA 0x55 | Type(1B) | Len(1B) | Payload(Len B) | CRC16(2B LSB)
   - CRC16-CCITT: poly=0x1021, init=0xFFFF, 覆盖 Type+Len+Payload
3. 遥测帧 (Type=0x01): 96 bytes payload, Python struct 格式 '<23f4B'
   字段: timestamp, roll, pitch, yaw, gyro_x, gyro_y, gyro_z,
         accel_x, accel_y, accel_z, vel_n, pos_n,
         theta_L, L0_L, wheel_T_L, Tp_L, d_theta_L,
         theta_R, L0_R, wheel_T_R, Tp_R, d_theta_R,
         battery_voltage, start_flag, jump_flag, contact_L, contact_R
4. 指令帧 (Type=0x02): 25 bytes payload, Python struct 格式 '<6fB'
   字段: timestamp, v_set, yaw_rate_set, roll_set, leg_set, pitch_set, cmd_flags
   cmd_flags: bit0=使能, bit1=跳跃, bit2=急停, bit3=倒地自起
5. 遥测 100Hz 持续发送; 需要字节级状态机拼帧 (CDC 可能拆包)
6. 发布 topic: /imu (sensor_msgs/Imu), /odom (nav_msgs/Odometry),
   /joint_states (sensor_msgs/JointState), /battery (sensor_msgs/BatteryState)
7. 订阅 topic: /cmd_vel (geometry_msgs/Twist) 映射到 v_set + yaw_rate_set
8. 每 100ms 无指令自动超时停止 (需要节点持续发心跳指令)
9. 用 rclpy.spin_once 或 timer 驱动主循环, 不要阻塞在串口读
```

### 进阶版提示词（含更多上下文）

```
[在此粘贴本 .md 文档全文]

请在 ROS2 Humble + Python 中实现完整的上位机通信节点, 要求:
- 类名: WheelFootBridge
- 自动检测 /dev/ttyACM* 并连接
- 100Hz 遥测解析 + 50Hz 指令下发 (两个独立定时器)
- 订阅 /cmd_vel 映射为 v_set/yaw_rate_set
- 订阅 /cmd_attitude (自定义 Float32MultiArray: [roll, pitch, leg])
- 服务 /enable 和 /estop 控制启停
- 参数: port, telemetry_rate, cmd_rate, timeout_ms
- 线程安全的串口帧解析
- launch 文件
```

---

## 六、下位机源码位置

| 文件 | 内容 |
|------|------|
| `User/APP/ROS/ros2_task.h` | 协议常量、结构体定义 |
| `User/APP/ROS/ros2_task.c` | FreeRTOS 任务、CRC、帧打包/解包、数据采集、指令执行 |
| `USB_DEVICE/App/usbd_cdc_if.c` | CDC 接收通知 (ros2_cdc_rx_ready) |
| `Core/Src/freertos.c` | ROS_Task 注册 (优先级 Normal, 栈 1024 words) |
