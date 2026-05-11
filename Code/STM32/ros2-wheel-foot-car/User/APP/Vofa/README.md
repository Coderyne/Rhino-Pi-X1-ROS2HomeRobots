# VOFA 调试模块

本模块用于将 MCU 内部变量通过串口发送到 VOFA+ 进行实时波形观察。  
当前实现绑定在 `USART1`（即 `huart1`）。

---

## 1. 模块位置

- 头文件：`User/APP/Vofa/vofa_task.h`
- 源文件：`User/APP/Vofa/vofa_task.c`

---

## 2. 数据帧格式

发送格式为 **float 数组 + 4 字节帧尾**：

- 数据区：`count * sizeof(float)` 字节
- 帧尾：`00 00 80 7F`

对应代码中的宏：

- `VOFA_FRAME_TAIL_0 = 0x00`
- `VOFA_FRAME_TAIL_1 = 0x00`
- `VOFA_FRAME_TAIL_2 = 0x80`
- `VOFA_FRAME_TAIL_3 = 0x7F`

> 该格式适用于 VOFA+ 的浮点流解析方式（JustFloat 风格）。

---

## 3. 可用接口

### 3.1 通用发送

- `Vofa_SendFloatArray(UART_HandleTypeDef *huart, const float *data, uint16_t count, uint16_t timeout)`

说明：

- `huart`：串口句柄（本工程通常传 `&huart1`）
- `data`：浮点数组首地址
- `count`：通道数
- `timeout`：串口阻塞发送超时（ms）

### 3.2 快捷接口

- `Vofa_Send1(...)`
- `Vofa_Send2(...)`
- `Vofa_Send3(...)`
- `Vofa_Send4(...)`

用于快速发送 1~4 路浮点数据。

---

## 4. 任务用法（当前实现）

`vofa_task.c` 中提供了 `StartVofaTask(void *argument)`：

- 周期：`osDelay(100)`（约 10Hz）
- 每次发送 9 路数据到 `huart1`
- 当前 `data[0..8]` 仍为占位值，需要替换为实际调试变量

建议按以下方式接入：

1. 将 `data[]` 替换为你要观察的变量（如角度、速度、控制量等）
2. 在 FreeRTOS 任务创建处注册 `StartVofaTask`
3. 确认 `USART1` 波特率与 VOFA+ 端一致

---

## 5. 快速示例

在控制循环中发送 3 路数据（目标值、测量值、误差）：

- `Vofa_Send3(&huart1, target, feedback, target - feedback, 100);`

---

## 6. 注意事项

- 串口发送使用 `HAL_UART_Transmit`（阻塞），发送频率过高可能影响实时性。
- 建议优先发送关键变量，减少通道数与发送频率。
- 若波形显示异常，优先检查：
	- 串口号是否正确（`USART1`）
	- 波特率是否一致
	- 解析格式是否匹配 float + `00 00 80 7F` 帧尾

