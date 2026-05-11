# ros2-wheel-foot-car

基于 **STM32H7** + **FreeRTOS** 的轮足机器人控制工程，包含姿态解算、VMC 腿部控制、轮毂/关节电机控制、遥控输入（PS2/Xbox）等核心模块，用于实现轮足底盘的平衡、运动与跳跃控制。

## 项目简介

本项目面向轮足底盘控制板固件开发，主要能力包括：

- 传感器融合与姿态估计（`INS`）
- 双腿 VMC 运动学/动力学计算与离地检测
- 底盘平衡、速度/位置/转向控制
- CAN 电机驱动（关节 + 轮毂）
- 遥控输入任务（`PS2`、`Xbox`）

## 目录说明（核心）

- `Core/`：CubeMX 生成的 HAL、FreeRTOS、启动与中断代码
- `Drivers/`：CMSIS 与 STM32H7 HAL 驱动
- `Middlewares/`：中间件（含第三方库）
- `User/`：项目核心业务代码
	- `APP/`：任务层（`chassis*_task`、`INS_task`、`observe_task`、`ps2_task`、`xbox_task` 等）
	- `Algorithm/`：算法层（如 `VMC`、滤波、控制相关算法）
	- `Devices/`、`Bsp/`、`Controller/`：设备驱动与控制模块
- `MDK-ARM/`：Keil 工程文件与编译输出

## 控制输入说明

- `User/APP/ps2_task.c`：PS2 手柄控制逻辑
- `User/APP/xbox_task.c`：Xbox 手柄控制逻辑
- `Xbox键位.md`：Xbox 键位与控制映射说明文档

## 构建与下载

当前仓库包含 Keil 工程（`MDK-ARM/CtrlBoard-H7_IMU.uvprojx`），可直接在 **Keil MDK-ARM** 中打开并编译下载到目标板。

## 备注

- 本仓库以嵌入式实时控制为主，建议先从 `User/APP` 的任务入口阅读系统行为。
- 若修改手柄映射或控制参数，请同步更新对应说明文档，避免调试与实际逻辑不一致。

