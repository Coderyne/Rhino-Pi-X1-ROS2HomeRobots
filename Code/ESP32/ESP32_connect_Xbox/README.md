# Xbox Series X Controller ESP32 示例

本项目基于 PlatformIO 平台，通过蓝牙 BLE 实现 ESP32-C3 与 Xbox Series X 手柄的通信交互。

## 项目简介

该项目用于 ESP32-C3 开发板与 Xbox Series X / Xbox One 手柄之间的蓝牙通信，可解析手柄的按键、摇杆、扳机等输入数据，通过串口输出，并支持振动反馈功能。

## 硬件要求

- ESP32-C3 开发板（默认配置 [weactstudio_esp32c3coreboard](https://github.com/WeActStudio/WeActStudio.ESP32C3CoreBoard)）
- Xbox Series X / Xbox One 手柄（支持蓝牙的型号）

## 软件依赖

- [PlatformIO](https://platformio.org/)
- Arduino 框架
- [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)（蓝牙 BLE 协议栈）

## 项目结构

```
├── platformio.ini                               # PlatformIO 配置文件
├── src/
│   ├── main.cpp                                 # 主程序入口
│   ├── XboxControllerNotificationParser.h       # 手柄通知数据解析器
│   ├── XboxControllerNotificationParser.cpp     # 解析器实现
│   ├── XboxSeriesXControllerESP32_asukiaaa.hpp  # 手柄 BLE 驱动
│   └── XboxSeriesXHIDReportBuilder_asukiaaa.hpp  # HID 振动报告构建器
└── README.md
```

## 功能特性

- 解析 Xbox Series X 手柄的全部按键输入（Y/X/B/A/LB/RB/Select/Start/Xbox/Share/LS/RS/十字键）
- 读取双摇杆模拟值（归一化到 -100~100）
- 读取双扳机键程值（归一化到 0~100）
- 支持四种振动反馈模式（左/右/高频/低频），可调力度、时长和循环次数
- 自定义 11 字节二进制串口协议输出（通过 Serial1，GPIO 20/21），含包头和校验和
- Serial1 接收包头 `0x07 0x21` 的帧时自动触发 250ms 振动（带 300ms 防抖冷却）
- 蓝牙断连后自动重连，连接失败超 2 次自动重启

## 使用方法

### 1. 配置手柄 MAC 地址

编辑 `src/main.cpp` 第 7 行，替换为自己的手柄蓝牙 MAC 地址：

```cpp
XboxSeriesXControllerESP32_asukiaaa::Core
    xboxController("58:d0:05:0e:85:8d"); // 替换为你的手柄地址
```

> **获取手柄 MAC 地址的方法**：Windows 设置 → 蓝牙和其他设备 → 设备 → Xbox Wireless Controller → 属性。或连接后查看串口日志。

### 2. 编译上传

```bash
# 编译项目
pio run

# 上传到 ESP32-C3
pio run --target upload

# 查看串口输出
pio device monitor
```

> PlatformIO 会根据 `platformio.ini` 自动安装依赖，无需手动安装。

如果使用其他 ESP32 板子，请修改 `platformio.ini` 中的 `board` 配置。

### 3. 串口监视器

连接 ESP32 后，串口将输出手柄连接状态。连接成功后，通过 `Serial1` (GPIO 20/21, 115200) 持续输出 11 字节的二进制数据帧。

## 串口数据协议

连接成功后，每 10ms 通过 Serial1 输出一帧 11 字节的二进制数据：

| 字节 | 内容 |
|------|------|
| 0-1 | 包头 `0x07 0x21` |
| 2 | 左摇杆 X（-100~100 映射到 28~228） |
| 3 | 左摇杆 Y（-100~100 映射到 28~228） |
| 4 | 右摇杆 X（-100~100 映射到 28~228） |
| 5 | 右摇杆 Y（-100~100 映射到 28~228） |
| 6 | 左扳机 LT（0~100） |
| 7 | 右扳机 RT（0~100） |
| 8 | 按钮组 1（bit0:A, bit1:LB, bit2:RB, bit3:Xbox） |
| 9 | 按钮组 2（bit0:B, bit1:X, bit2:Y） |
| 10 | 校验和（前 10 字节求和取低 8 位） |

## 振动控制

支持四种振动模式，可随意搭配：

| 模式 | 说明 |
|------|------|
| `left` | 上左电机 |
| `right` | 上右电机 |
| `center` | 下双电机高频低力量 |
| `shake` | 下双电机低频大力量 |

### 自动振动

当 Serial1 接收到包头 `0x07 0x21` 的完整帧时，手柄自动触发 250ms 振动（shake + center 模式，50% 力度），带 300ms 冷却防抖。可用于对端设备的力反馈通知。

## 注意事项

1. 使用前必须在代码中填入自己手柄的蓝牙 MAC 地址
2. 手柄需要处于配对模式（长按手柄顶部配对按钮，Xbox 灯快闪）
3. 使用其他 ESP32 板子时需调整 `platformio.ini` 中的 `board` 和串口引脚配置
4. 下左电机和下右电机是绑定的，只能同时动作
