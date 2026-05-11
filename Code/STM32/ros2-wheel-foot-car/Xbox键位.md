# Xbox 手柄控制逻辑说明

本文档对应当前工程中的 `User/APP/xbox_task.c` 实现，用于说明 Xbox 手柄各按键/摇杆与底盘控制行为的映射关系。

## 1. 任务周期与数据更新

- 控制任务函数：`Xbox_task()`
- 任务周期：约 `55ms`（`osDelay(55)`）
- 每周期执行：
	1. 检查手柄是否超时断连
	2. 执行 `Xbox_data_process(&chassis_move, 0.055f)` 更新底盘目标

### 1.1 断连保护

- 若超过约 `51ms` 未收到新数据（`cur_time - last_time > 51`），会将 `xboxController` 全部清零。
- 清零后摇杆与按键等效于“无输入”。

---

## 2. 按键逻辑

当前底盘控制实际使用按键：`A`、`B`、`LB`、`RB`。

### 2.1 A 键：底盘启停切换（边沿触发）

- 触发条件：`A` 从未按下变为按下（上升沿）
- 逻辑：
	- 若 `start_flag == 0`：置 `start_flag = 1`（启动）
	- 若 `start_flag == 1`：置 `start_flag = 0`（关闭），并清 `recover_flag = 0`

> 说明：使用边沿触发，长按不会重复切换。

### 2.2 LB + RB 组合键：触发跳跃（边沿触发）

- 触发条件：`LB` 与 `RB` 同时按下，并且组合键由“未触发”变为“触发”（上升沿）
- 限制条件：`jump_flag == 0 && jump_flag2 == 0`
- 动作：
	- `jump_flag = 1`
	- `jump_flag2 = 1`

### 2.3 B 键：横滚复位（电平触发）

- 触发条件：`B` 为按下状态
- 动作：`roll_set = -0.03f`

> 说明：按住 `B` 时每个控制周期都会执行复位。

---

## 3. 摇杆逻辑

### 3.1 左摇杆：速度与转向

- `leftStickY` → 前后速度 `v_set`
	- 计算：`v_set = (-leftStickY) * 0.010f`
- `leftStickX` → 转向角目标增量 `turn_set`
	- 计算：`turn_set += (-leftStickX) * 0.00064f`
- 位置积分：`x_set += v_set * dt`，其中 `dt = 0.055f`

### 3.2 右摇杆：横滚与腿长

- `rightStickX` → 横滚目标 `roll_set`
	- 计算：`roll_set += (-rightStickX) * 0.00009f`
	- 限幅：`roll_set ∈ [-0.40, 0.40]`
- `rightStickY` → 腿长目标 `leg_set`
	- 计算：`leg_set += (-rightStickY) * 0.00002f`
	- 限幅：`leg_set ∈ [0.072, 0.21]`

### 3.3 腿长主动控制标志

- 若 `fabsf(last_leg_set - leg_set) > 0.0001f`：
	- `right.leg_flag = 1`
	- `left.leg_flag = 1`

---

## 4. 启动/关闭状态行为

### 4.1 启动状态（`start_flag == 1`）

- 按摇杆输入实时更新：速度、位置、转向、横滚、腿长。

### 4.2 关闭状态（`start_flag == 0`）

- 强制设置为安全默认目标：
	- `v_set = 0.0f`
	- `x_set = x_filter`
	- `turn_set = total_yaw`
	- `leg_set = 0.08f`
	- `roll_set = -0.03f`

---

## 5. 协议按键位映射（`p[8]`）

`Xbox_ProcessData()` 中当前按键位定义如下：

- `0x01`：`A`
- `0x02`：`LB`
- `0x04`：`RB`
- `0x08`：`Start`（当前底盘逻辑未使用）
- `0x10`：`B`

其余按键（`X/Y/Back/方向键`）当前解析后固定为 `false`，未参与控制。

---

## 6. 备注

- 本文档描述的是“当前代码实际行为”。
- 若后续修改了 `xbox_task.c` 中系数、限幅或按键位定义，请同步更新本文件。
