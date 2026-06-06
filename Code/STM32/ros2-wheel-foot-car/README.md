# ros2-wheel-foot-car

基于 **STM32H723** + **FreeRTOS** 的轮足机器人底层控制固件，包含姿态解算、VMC 腿部控制、CAN 电机驱动、遥控输入（PS2/Xbox）、ROS2 通信等核心模块，实现轮足底盘的平衡、运动与跳跃控制。

## 硬件平台

| 组件 | 型号 |
|------|------|
| MCU | STM32H723VGT6 (Cortex-M7, 550MHz) |
| IMU | BMI088 (SPI) |
| 关节电机 | DM4310 × 2 (CAN) |
| 轮毂电机 | GIM6010 × 2 (CAN) |
| 遥控 | PS2 无线手柄 / Xbox 无线手柄 |
| 调试输出 | USART1 → VOFA+ |
| 上位机通信 | USART → ROS2 (串口协议) |

## 目录结构

```
ros2-wheel-foot-car/
├── Core/                      # CubeMX 生成代码 (HAL / FreeRTOS / 启动)
│   ├── Inc/                   #   头文件 (FreeRTOSConfig, main, stm32h7xx_it 等)
│   └── Src/                   #   源文件 (main, freertos, 中断服务)
├── Drivers/                   # CMSIS + STM32H7 HAL 驱动
├── Middlewares/               # 中间件 (ST / Third_Party)
├── USB_DEVICE/                # USB 设备栈 (App / Target)
├── User/                      # 🔑 核心业务代码
│   ├── APP/                   # 任务层 (FreeRTOS 任务入口)
│   │   ├── INS_task.c/h       #   姿态估计任务 (IMU 数据融合)
│   │   ├── chassisL_task.c/h  #   左腿底盘控制任务
│   │   ├── chassisR_task.c/h  #   右腿底盘控制任务
│   │   ├── observe_task.c/h   #   观测/监控任务
│   │   ├── ps2_task.c/h       #   PS2 手柄控制任务
│   │   ├── xbox_task.c/h      #   Xbox 手柄控制任务
│   │   ├── Power/             #   电源管理 (ADC 电压检测 / 蜂鸣器告警)
│   │   ├── ROS/               #   ROS2 通信任务 (串口协议收发)
│   │   └── Vofa/              #   VOFA+ 调试输出 (浮点流可视化)
│   ├── Algorithm/             # 算法层
│   │   ├── EKF/               #   四元数扩展卡尔曼滤波 (QuaternionEKF)
│   │   ├── PID/               #   PID 控制器
│   │   ├── VMC/               #   虚拟模型控制 (VMC 腿部运动学/动力学)
│   │   ├── kalman/            #   卡尔曼滤波器
│   │   └── mahony/            #   Mahony AHRS + 互补滤波
│   ├── Devices/               # 设备驱动
│   │   ├── BMI088/            #   BMI088 IMU 驱动 + 中间件 (SPI)
│   │   ├── DM_Motor/          #   DM4310 关节电机驱动 (CAN)
│   │   └── SW_Motor/          #   GIM6010 轮毂电机驱动 (CAN)
│   ├── Controller/            # 顶层控制器 (controller.c/h)
│   ├── Bsp/                   # 板级支持包 (PWM / DWT 定时 / CAN)
│   └── Lib/                   # 用户工具库 (user_lib)
├── MDK-ARM/                   # Keil MDK 工程
│   ├── CtrlBoard-H7_IMU.uvprojx   # 主工程文件
│   └── startup_stm32h723xx.s      # 启动汇编
├── build/                     # 编译输出
├── CtrlBoard-H7_IMU.ioc       # CubeMX 硬件配置
├── 通信协议-STM32与ROS2.md     # STM32 ↔ ROS2 串口通信协议
└── Xbox键位.md                # Xbox 手柄键位与控制映射
```

## 控制架构

```
┌─────────────────────────────────────────────────────┐
│                    传感器层                           │
│   BMI088 IMU ──► SPI ──► 姿态解算 (EKF/Mahony)       │
│   电机编码器 ◄──► CAN ──► 速度/位置反馈               │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│                    算法层                             │
│   Kalman 滤波 → QuaternionEKF → Mahony AHRS          │
│   PID 控制器 → VMC 腿部运动学/动力学计算              │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│                    控制层                             │
│   controller.c  ──  平衡 / 速度 / 位置 / 转向 / 跳跃  │
│   chassisL/R  ──  左右腿独立控制 (离地检测 / 相位)    │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│                    执行层                             │
│   DM4310 关节电机 (CAN)  │  GIM6010 轮毂电机 (CAN)   │
└─────────────────────────────────────────────────────┘
```

## 任务系统 (FreeRTOS)

| 任务 | 文件 | 周期 | 功能 |
|------|------|------|------|
| INS | `INS_task.c` | ~1ms | IMU 数据读取 + 姿态融合 |
| ChassisL | `chassisL_task.c` | ~1ms | 左腿 VMC 计算 + 电机控制 |
| ChassisR | `chassisR_task.c` | ~1ms | 右腿 VMC 计算 + 电机控制 |
| Observe | `observe_task.c` | — | 状态监控 |
| PS2 | `ps2_task.c` | — | PS2 手柄数据接收与解析 |
| Xbox | `xbox_task.c` | ~55ms | Xbox 手柄数据接收与解析 |
| ROS2 | `ROS/ros2_task.c` | — | 串口协议收发 (上位机通信) |
| Power | `Power/power_task.c` | 2s | 电池电压检测 / 低压蜂鸣器告警 |
| Vofa | `Vofa/vofa_task.c` | 100ms | 调试数据发送 (USART1) |

## 遥控输入

支持双遥控方案，详见：

- **PS2 手柄**：[`User/APP/ps2_task.c`](User/APP/ps2_task.c) — 无线接收器接入，解析按键 + 摇杆数据
- **Xbox 手柄**：[`User/APP/xbox_task.c`](User/APP/xbox_task.c) — 键位与控制映射见 [`Xbox键位.md`](Xbox键位.md)

### Xbox 控制摘要

| 输入 | 功能 |
|------|------|
| A 键 (上升沿) | 底盘启停切换 |
| LB + RB (同时按下) | 触发跳跃 |
| B 键 (电平) | 横滚复位 |
| 左摇杆 Y/X | 前后速度 / 转向 |
| 右摇杆 X/Y | 横滚 / 腿长 |

## 构建与下载

### Keil MDK-ARM (推荐)

1. 打开 `MDK-ARM/CtrlBoard-H7_IMU.uvprojx`
2. 选择目标 `CtrlBoard-H7_IMU`
3. 编译 (F7) → 下载 (F8)

### CubeMX 配置

硬件配置工程 `CtrlBoard-H7_IMU.ioc`，可在 STM32CubeMX 中打开修改引脚/外设配置，重新生成 `Core/` 代码。

## 调试

### VOFA+ 实时波形

通过 USART1 发送浮点数据到 [VOFA+](https://www.vofa-plus.com/) 进行可视化调试：

- 帧格式：float 数组 + `00 00 80 7F` 帧尾 (JustFloat)
- 当前发送 9 路数据，替换 `data[]` 为实际调试变量即可

> 详见 [`User/APP/Vofa/README.md`](User/APP/Vofa/README.md)

### 电源管理

- 三路电源输出控制 (DC24_0 / DC24_1 / DC5)
- 电池电压 ADC 采样，低压 (<24V) 蜂鸣器告警

> 详见 [`User/APP/Power/README.md`](User/APP/Power/README.md)

## 相关文档

| 文档 | 说明 |
|------|------|
| [`通信协议-STM32与ROS2.md`](通信协议-STM32与ROS2.md) | STM32 ↔ ROS2 串口通信协议定义 |
| [`Xbox键位.md`](Xbox键位.md) | Xbox 手柄键位与控制映射 |
| [`User/APP/Power/README.md`](User/APP/Power/README.md) | 电源管理模块说明 |
| [`User/APP/Vofa/README.md`](User/APP/Vofa/README.md) | VOFA+ 调试模块说明 |

## 开发须知

- 阅读代码入口建议从 `Core/Src/main.c` 的任务创建开始，再到 `User/APP/` 各任务文件
- 修改遥控映射或控制参数后，请同步更新对应说明文档 ([`Xbox键位.md`](Xbox键位.md)、[`通信协议-STM32与ROS2.md`](通信协议-STM32与ROS2.md))
- `Core/` 由 CubeMX 生成，手动修改可能被覆盖；业务逻辑请写在 `User/` 下
- 代码格式化使用 `.clang-format`，支持 VS Code 和 Keil

