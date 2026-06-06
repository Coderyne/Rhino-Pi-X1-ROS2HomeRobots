# Emotion Display

全屏 GIF 表情播放器，支持通过命名管道在运行时动态切换表情、调整循环间隔。

## 目录结构

```
emotion_display/
├── main.py          # 入口
├── gif_player.py    # 全屏窗口 + QMovie + 状态机
├── fifo_handler.py  # FIFO 命令监听与解析
└── gif_source/      # 14 个表情 GIF
```

## 环境要求

- Python >= 3.8
- PySide6

```bash
pip install PySide6
```

## 启动

```bash
cd emotion_display
python3 main.py
```

启动后全屏播放 `standby.gif`，无限循环，间隔 2 秒。

按 **Esc** 或 **Q** 退出。

## 运行时命令

从另一个终端通过命名管道 `/tmp/gif_cmd` 发送命令：

### 切换表情

```bash
echo "play happy"         > /tmp/gif_cmd   # 无限循环，保持当前间隔
echo "play sad 3"         > /tmp/gif_cmd   # 播放 3 次后停止，保持当前间隔
echo "play sad 3 0"       > /tmp/gif_cmd   # 播放 3 次，间隔 0s（连播）
echo "play enjoy 0 0.5"   > /tmp/gif_cmd   # 无限循环，间隔 0.5s
```

`play` 命令完整格式：`play <表情名> [次数] [间隔秒数]`，次数默认 0（无限），间隔不指定则保持当前值。

### 调整间隔

```bash
echo "interval 1.5" > /tmp/gif_cmd    # 循环间隔改为 1.5 秒
```

### 暂停/恢复

```bash
echo "stop" > /tmp/gif_cmd            # 暂停
echo "resume" > /tmp/gif_cmd          # 恢复播放
```

### 退出

```bash
echo "quit" > /tmp/gif_cmd            # 退出程序
```

## 可用表情

| 文件名 | 对应命令 |
|--------|---------|
| afraid.gif | `play afraid` |
| angery.gif | `play angery` |
| charging\&like.gif | `play charging&like` |
| cold.gif | `play cold` |
| dizz.gif | `play dizz` |
| enjoy.gif | `play enjoy` |
| happy.gif | `play happy` |
| kick.gif | `play kick` |
| sad.gif | `play sad` |
| sad\&tear.gif | `play sad&tear` |
| shutdown.gif | `play shutdown` |
| sleep.gif | `play sleep` |
| standby.gif | `play standby` |
| voice.gif | `play voice` |
