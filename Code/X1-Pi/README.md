# X1-Pi 代码

Rhino-Pi X1 机器人上位机代码，运行于 Aidlux 平台，涵盖 ROS2 底层控制、语音交互、表情显示、LLM 推理等模块。

## 系统架构

```
┌──────────────────────────────────────────────────────────┐
│                      X1-Pi (Aidlux)                       │
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────┐   │
│  │ ROS2_Packages│  │HA_Voice      │  │ emotion_display│   │
│  │ 轮足机器人控制 │  │Assistant     │  │ GIF 表情播放器  │   │
│  │ SLAM/导航     │  │ASR/TTS/KWS   │  │ FIFO 命令切换  │   │
│  └──────┬───────┘  └──────┬───────┘  └───────┬───────┘   │
│         │                 │                   │           │
│         ▼                 ▼                   ▼           │
│  ┌──────────────────────────────────────────────────┐    │
│  │              共享通信层 (FIFO / ROS2)              │    │
│  └──────────────────────────────────────────────────┘    │
│         │                 │                   │           │
│  ┌──────┴───────┐  ┌──────┴───────┐  ┌───────┴───────┐   │
│  │   Vision     │  │   Aid_LLM    │  │  Home Assistant│   │
│  │  (规划中)     │  │ Qwen3-4B NPU │  │  Conversation │   │
│  └──────────────┘  └──────────────┘  └───────────────┘   │
└──────────────────────────────────────────────────────────┘
```

## 模块说明

### [`ROS2_Packages`](ROS2_Packages/)
ROS2 底层功能包，包含轮足机器人核心控制代码：
- 机器人运动控制与状态管理
- LD06/LD19/STL27L 激光雷达驱动
- STM32 与 ROS2 通信协议（串口）
- SLAM 建图与导航（含 Web 可视化）
- 启动脚本与地图文件

> 详见 [wheel_legged_-ros2/README.md](ROS2_Packages/wheel_legged_-ros2/README.md) 和 [通信协议文档](ROS2_Packages/wheel_legged_-ros2/通信协议-STM32与ROS2.md)

### [`HA_VoiceAssistant`](HA_VoiceAssistant/)
Home Assistant 智能语音助手，本地部署 ASR/TTS/KWS 模型：
- **KWS 唤醒词检测** — sherpa-onnx Zipformer，本地离线唤醒
- **ASR 语音识别** — AidVoice NPU SenseVoice，NPU 加速推理
- **TTS 语音合成** — VITS-Piper，流式合成与播放
- **Wyoming STT 服务器** — 将 ASR 注册为 HA 语音转文字引擎
- **外部管道模式** — 独立 KWS + ASR 管道，通过 HA Conversation API 交互

> 详见 [HA_VoiceAssistant/README.md](HA_VoiceAssistant/README.md)

### [`emotion_display`](emotion_display/)
全屏 GIF 表情播放器，用于机器人情感化交互：
- PySide6 全屏窗口 + QMovie 播放
- 支持 14 种表情 GIF（待机/开心/难过/生气/睡眠等）
- 运行时通过命名管道 `/tmp/gif_cmd` 动态切换表情
- 支持 GNOME + Wayland 开机自启动

> 详见 [emotion_display/README.md](emotion_display/README.md) 和 [emotion_display/DEPLOY.md](emotion_display/DEPLOY.md)

### [`Aid_LLM`](Aid_LLM/)
LLM 推理模块 — 通过 AidGenSE 在 NPU 上部署 Qwen3-4B-Instruct 实现本地对话推理。

### [`Vision`](Vision/)
视觉交互模块 — 体感交互、手势/姿态识别等功能。

---

## 硬件依赖

| 模块 | 硬件要求 |
|------|---------|
| ROS2_Packages | X1 轮足机器人底盘、STM32 主控、激光雷达 (LD06/19/STL27L) |
| HA_VoiceAssistant | 麦克风阵列、扬声器、NPU (AidVoice) |
| emotion_display | HDMI 显示器 |
| Aid_LLM | NPU (Aidlux) |