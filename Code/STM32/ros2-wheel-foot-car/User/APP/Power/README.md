# 电源管理模块

本模块用于电源输出控制与电池低压检测，低压时通过蜂鸣器报警。  
核心实现位于 `User/APP/Power/power_task.c`。

---

## 1. 模块功能

- 控制三路电源输出：`DC24_0`、`DC24_1`、`DC5`
- 读取电池电压（ADC）
- 低压检测（阈值默认 `24.0V`）
- 低压时驱动蜂鸣器（TIM12 PWM）告警

---

## 2. 对外接口

头文件：`User/APP/Power/power_task.h`

- `void Power_Init(bool DC24_0_Status, bool DC24_1_Status, bool DC5_Status);`
	- 配置三路电源输出开关状态
- `float GetBatteryVoltage(void);`
	- 获取当前电池电压（单位：V）
- `void StartPowerTask(void);`
	- 启动电源任务（内部为无限循环）

---

## 3. 工作流程

`StartPowerTask()` 执行流程：

1. 启动 ADC：`HAL_ADC_Start(&hadc1)`
2. 启动蜂鸣器 PWM：`HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2)`
3. 初始化电源输出：`Power_Init(false, false, true)`（默认仅打开 `DC5`）
4. 每 `2000ms` 执行一次低压检查

低压判定逻辑：

- 当 `voltage < Battery_Low_Threshold`（默认 `24.0V`）
- 蜂鸣器执行 3 组鸣叫（100ms 响 + 100ms 停）

---

## 4. 电压换算说明

当前换算公式：

- `voltage = (adc / 65536) * 3.3 * 11 + offset`

其中：

- `11`：分压比相关系数
- `offset`：软件校准偏移（当前默认 `1.0f`）

如实测电压与显示值有偏差，可调 `offset`。

---

## 5. 可调参数

在 `power_task.c` 中：

- `enable_low_power`：是否启用低压告警（`1` 开启，`0` 关闭）
- `Battery_Low_Threshold`：低压阈值（默认 `24.0f`）
- `offset`：电压校准偏移

---

## 6. 接入说明

### 6.1 FreeRTOS

建议将 `StartPowerTask()` 作为独立任务入口调用一次。  
注意：`StartPowerTask()` 内部已是 `for(;;)` 无限循环，不要在外层再套无限循环重复调用。

### 6.2 外设依赖

本模块依赖：

- `ADC1`：电池电压采样
- `TIM12 CH2`：蜂鸣器 PWM 输出
- 对应 GPIO：`DC24_0`、`DC24_1`、`DC5` 控制引脚

---

## 7. 常见问题

- 电压一直不变：检查 ADC 通道与采样引脚配置是否正确。
- 蜂鸣器不响：检查 `TIM12 CH2` 和蜂鸣器引脚复用配置。
- 电压偏差较大：先确认分压电阻比，再调 `offset`。
