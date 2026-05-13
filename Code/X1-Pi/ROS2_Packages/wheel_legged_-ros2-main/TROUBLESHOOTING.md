# 异常排查记录

> 2026-05-13，启动 `./start.sh slam` 建图模式后 Web 仪表盘实时地图始终空白，显示“等待地图...”。

---

## 一、配置与环境

| 项 | 值 |
|----|-----|
| ROS2 版本 | Humble |
| 建图算法 | slam_toolbox (online async) |
| 激光雷达 | LD06 (`/dev/ttyUSB0`) |
| STM32 串口 | USB CDC `/dev/ttyACM0` |
| Web 仪表盘 | roslibjs + rosbridge WebSocket (9091) |

TF 树（修复后）：
```
map ← odom ← base_footprint ← base_link ← base_laser
 ↑slam    ↑bridge    ↑static TF(0,0,0)    ↑lidar(0,0,0.18)
```

---

## 二、已发现的异常

### 异常 1：STM32 遥测 `vel_n` 值严重异常

**现象**：`vel_n` 持续输出 ~81.6 m/s，机器人实际静止。

```
$ ros2 topic echo /odom --once
twist:
  twist:
    linear:
      x: 81.67325592041016    ← 异常，正常应 ≤1.5 m/s
```

**影响**：bridge 积分后里程计漂移到 -39 km。

**根因推测**：
- STM32 端 INS / EKF 未正确初始化或校准
- IMU 传感器 (BMI088 / ICM42688 等) 零偏未消除
- 机器人静止时 INS 未输出零速

### 异常 2：里程计位置漂移

**现象**：odom position 持续增长

```
$ ros2 topic echo /odom --once
position:
  x: -39008.66       ← 约 -39 km
  y:  12762.14       ← 约 12.7 km
```

**影响**：slam_toolbox 用 odom 做扫描匹配初始位姿猜测。相关搜索窗口仅 0.5m，39km 的偏移导致匹配永远无法收敛 → 无地图生成。

### 异常 3：STM32 串口写入超时

**现象**：bridge 日志反复出现 `Write timeout`

```
[ERROR] [wheel_foot_bridge]: Serial write error: Write timeout
[INFO]  [wheel_foot_bridge]: Serial port opened: /dev/ttyACM0
[ERROR] [wheel_foot_bridge]: Serial write error: Write timeout
... (每 5~10 秒重复)
```

**影响**：指令帧（`/cmd_vel`、`/cmd_enable` 等）间歇性无法发送到 STM32。

**根因推测**：
- USB CDC 物理连接不稳定
- STM32 端 CDC 接收缓冲区溢出
- 波特率或流控不匹配（CDC 虚拟串口，波特率理论上无效）

### 异常 4：Enable 超时警告

**现象**：bridge 反复输出 `Enable timeout, auto-disabling`

```
[WARN] [wheel_foot_bridge]: Enable timeout, auto-disabling
```

**原因**：`keep_alive=False` 时不发送心跳帧，但 `_enable=True`（由 `cmd_vel` 回调触发），100ms 超时触发自动禁用。

### 异常 5（已修复）：TF 链缺失 `base_footprint → base_link`

**现象**：slam_toolbox 无法将 `/scan`（frame_id=`base_laser`）变换到 `odom`。

**根因**：仅 `odom → base_footprint`（bridge）和 `base_link → base_laser`（lidar）存在，中间跳缺失。

**修复**：`bridge.launch.py` 新增 `static_transform_publisher` 发布零偏移 `base_footprint → base_link`。

### 异常 6（已修复）：bridge 启动失败

**现象**：`bridge.launch.py` 编译的旧版本含有 `from tf2_ros import StaticTransformPublisher`，此 API 在 Humble 中不存在。

**修复**：`colcon build --packages-select stm32_bridge` 重新编译。

---

## 三、关键数据诊断

| 检查项 | 状态 | 说明 |
|--------|------|------|
| `/scan` (LaserScan) | ✓ 正常 | RViz2 可显示实时点云 |
| `/imu/data` | ✓ 正常 | 50Hz，四元数 / 角速度值合理 |
| `/joint_states` | ✓ 正常 | 关节位置和力矩值合理 |
| `/odom` | ✗ 异常 | vel_n=81.6 m/s，position=-39km |
| TF `odom → base_laser` | ✓ 完整 | 修复后链正常，但含巨大偏移 |
| `/map` (OccupancyGrid) | ✗ 无数据 | topic 已注册但无消息发布 |
| bridge 串口写 | ✗ 间歇超时 | Write timeout 每 5~10s |

---

## 四、下一步排查方向

| 优先级 | 项目 | 操作 |
|--------|------|------|
| **P0** | 检查 STM32 IMU 传感器 | 确认 IMU 芯片物理连接、I2C/SPI 通信正常 |
| **P0** | STM32 INS/EKF 标定 | 静止状态下执行零速校准和零偏标定 |
| **P1** | 遥测 `vel_n` 字段格式 | 验证字节偏移、struct 格式与协议文档一致 |
| **P1** | USB CDC 稳定性 | 检查 USB 线材、接触、供电；STM32 端 CDC 缓存大小 |
| **P2** | bridge keep_alive 逻辑 | 关闭 keep_alive 时 enable timeout 是否需要静默 |

---

*文件生成时间: 2026-05-13 22:xx*
