# HA 语音助手 - AidVoice NPU 语音转文字 + HA Conversation API

基于 sherpa-onnx KWS、AidVoice NPU SenseVoice ASR 和 Home Assistant Conversation API 的智能语音助手。

## 系统架构

```
                         ┌──────────────────────────────────────┐
                         │      Home Assistant (Docker)         │
                         │                                      │
  ┌──────┐   Wyoming     │  ┌─────────────┐  ┌───────────────┐  │
  │ STT  │◄── TCP ───────┼──┤ Assist 管道  │  │ Conversation  │  │
  │Server│   :10300      │  │ (STT 客户端) │──┤ Agent         │  │
  └──┬───┘              │  └─────────────┘  │(extended_openai│  │
     │                  │                    └───────┬───────┘  │
  ┌──┴──────────┐       │                            │          │
  │ AidVoiceASR │       │                    ┌───────┴───────┐  │
  │(NPU SenseVoice)     │                    │  TTS (可选)   │  │
  └─────────────┘       └────────────────────┴───────────────┘  │

  ┌─────────────────────────────────────────────────────────┐
  │  外部语音管道（可选，与 Wyoming STT 互补）                │
  │                                                         │
  │  麦克风 → KWS(唤醒词) → ASR(AidVoice NPU) → HA API → TTS│
  └─────────────────────────────────────────────────────────┘
```

本仓库提供两种工作模式：
1. **Wyoming STT 模式** — 将 AidVoice 注册为 HA 的语音转文字引擎，完全融入 Assist 管道
2. **外部管道模式** — 外部 KWS + ASR 检测唤醒词后，通过 HA REST API 调用对话助手

## 目录结构

```
HA_VoiceAssistant/
├── __init__.py              # 包入口
├── config.py                # 配置管理（KWS/TTS/HA）
├── state.py                 # 线程安全状态机
├── audio.py                 # ALSA 麦克风采集
├── wakeword.py              # sherpa-onnx 关键词唤醒检测
├── asr_aidvoice.py          # AidVoice NPU SenseVoice 语音识别
├── asr_bridge.cpp           # NPU ASR C 桥接源码
├── asr_bridge.h             # NPU ASR C 桥接头文件
├── asr_bridge.so            # NPU ASR C 动态库（预编译）
├── tts.py                   # VITS-Piper 语音合成 + 流式播放
├── ha_client.py             # HA Conversation API 客户端
├── pipeline.py              # 流水线编排（KWS → ASR → HA API → TTS）
├── start.py                 # 外部管道模式入口
├── start.sh                 # 外部管道模式启动脚本
├── keys.txt                 # HA 长期访问令牌（用户自行填写）
├── migrate/                 # 迁移部署指南
│   └── README.md
├── stt_server/              # Wyoming STT 服务器
│   ├── __init__.py
│   ├── server.py            # AidVoice Wyoming STT 服务端
│   └── start.sh             # STT 服务器启动脚本
└── models/                  # 模型文件
    ├── sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01/
    │   ├── encoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx
    │   ├── decoder-epoch-12-avg-2-chunk-16-left-64.int8.onnx
    │   ├── joiner-epoch-12-avg-2-chunk-16-left-64.int8.onnx
    │   ├── tokens.txt
    │   ├── keywords.txt
    │   └── keywords_raw.txt
    └── vits-piper-zh_CN-xiao_ya-medium-fp16/
        ├── zh_CN-xiao_ya-medium.fp32.onnx
        ├── tokens.txt
        ├── lexicon.txt
        ├── date.fst
        ├── number.fst
        └── phone.fst
```

## 前置条件

### 安装依赖

```bash
pip install sherpa-onnx sounddevice requests numpy wyoming
```

### HA 长期访问令牌

在 HA 中创建长期访问令牌（设置 → 长期访问令牌 → 创建），保存到 `keys.txt`：

```
HA_TOKEN=你的令牌内容
```

## Wyoming STT 服务器（推荐）

将 AidVoice NPU SenseVoice 注册为 HA 的语音转文字引擎。

### 启动服务器

```bash
cd /home/aidlux/MIC_ASR/HA_VoiceAssistant
bash stt_server/start.sh
```

默认监听 `tcp://0.0.0.0:10300`。

### HA 配置

在 `configuration.yaml` 中添加：

```yaml
wyoming:
  - server: localhost
    port: 10300
```

重启 HA，进入 设置 → 语音助手 → Assist Pipeline：
- 新建或编辑管道
- **语音转文字** 选择 "AidVoice SenseVoice"
- **对话代理** 选择你的助手（如 Qwen3-4B-8550）
- **文字转语音** 按需选择

## 外部管道模式（外部 KWS + TTS）

此模式下，外部脚本负责唤醒词检测和 ASR，识别结果通过 HA API 发送。

### 启动

```bash
cd /home/aidlux/MIC_ASR/HA_VoiceAssistant
bash start.sh
```

### 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--kws-encoder` | KWS encoder ONNX 路径 | 必填 |
| `--kws-decoder` | KWS decoder ONNX 路径 | 必填 |
| `--kws-joiner` | KWS joiner ONNX 路径 | 必填 |
| `--kws-tokens` | KWS tokens.txt 路径 | 必填 |
| `--kws-keywords` | keywords.txt 路径 | 必填 |
| `--kws-num-threads` | KWS 推理线程数 | `1` |
| `--kws-threshold` | 唤醒灵敏度阈值 | `0.10` |
| `--kws-score` | 关键词加分值 | `1.5` |
| `--tts-model` | TTS 模型 ONNX 路径 | 必填 |
| `--tts-tokens` | TTS tokens.txt 路径 | 必填 |
| `--tts-lexicon` | TTS lexicon.txt 路径 | `""` |
| `--tts-rule-fsts` | 文本归一化 FST 文件（逗号分隔） | `""` |
| `--tts-speed` | 语速 (>1 更快) | `1.0` |
| `--tts-mode` | TTS 播报模式，`sentence` 逐句 / `full` 整段 | `sentence` |
| `--volume-gain` | 音量增益 | `1.0` |
| `--asr-cooldown-ms` | TTS 回复后的冷却期（毫秒） | `1200` |
| `--ha-url` | HA 服务器地址 | `http://localhost:8123` |
| `--ha-agent-id` | 对话助手 agent_id | `conversation.qwen3_4b_8550` |
| `--ha-timeout` | HTTP 请求超时（秒） | `30` |
| `--device-name` | ALSA 录音设备 | `plughw:2,0` |
| `--playback-device` | ALSA 播放设备名，留空自动检测 | `""` |

### 唤醒词列表

创建 `keywords_raw.txt`，每行写中文唤醒词，空格用下划线代替：

```
小爱同学 @小爱同学
你好军哥 @你好军哥
```

使用 `sherpa-onnx-cli text2token` 工具转为拼音 token 格式：

```bash
sherpa-onnx-cli text2token \
  --tokens models/sherpa-onnx-kws-zipformer-wenetspeech-3.3M-2024-01-01/tokens.txt \
  --tokens-type ppinyin \
  keywords_raw.txt keywords.txt
```

生成的文件格式如下：

```
x iǎo ài t óng x ué @小爱同学
n ǐ h ǎo j ūn g ē @你好军哥
```

放入模型目录覆盖原文件，重启助手即可生效。

## TTS 播报模式

支持两种播报方式，通过 `--tts-mode` 参数切换：

| 模式 | 参数 | 行为 |
|------|------|------|
| **逐句播报** | `--tts-mode sentence`（默认） | 按标点拆分为句子，逐句合成播放，延迟低 |
| **整段播报** | `--tts-mode full` | 整段文本一次合成后播放 |

```bash
# 整段播报
./start.sh --tts-mode full
```

## 迁移到其他机器

同型号设备迁移部署请参考 [migrate/README.md](migrate/README.md)。

## Wyoming 协议说明

STT 服务基于 [Wyoming 协议](https://www.home-assistant.io/integrations/wyoming/) 实现：

1. **Describe/Info** — 服务发现，HA 查询可用的 ASR 模型
2. **Transcribe** — 启动转录会话
3. **AudioStart/AudioChunk/AudioStop** — 流式传输音频（16kHz, 16-bit PCM, 单声道）
4. **Transcript** — 返回识别文本

音频格式转换：PCM s16le 字节流 → float32 numpy 数组（归一化到 [-1, 1]）
