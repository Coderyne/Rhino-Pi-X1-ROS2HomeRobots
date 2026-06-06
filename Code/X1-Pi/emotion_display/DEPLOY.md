# Emotion Display — 部署指南

## 目录

- [环境要求](#环境要求)
- [安装依赖](#安装依赖)
- [项目结构](#项目结构)
- [手动启动](#手动启动)
- [开机自启动（GNOME + Wayland）](#开机自启动gnome--wayland)
- [运行时命令](#运行时命令)
- [常见问题](#常见问题)

---

## 环境要求

| 组件 | 要求 |
|------|------|
| OS | Ubuntu 22.04+ (arm64) |
| Python | >= 3.8 |
| PySide6 | >= 6.5 |
| 显示器 | HDMI 单屏（默认全屏到主显示器） |

## 安装依赖

```bash
# 1. 安装 PySide6
pip3 install PySide6

# 2. 安装系统依赖（Qt XCB 平台插件）
sudo apt install -y libxcb-cursor0
```

## 项目结构

```
emotion_display/
├── main.py           # 入口：组装 GifPlayer + FifoHandler
├── gif_player.py     # 全屏窗口 + QMovie 播放 + 状态机
├── fifo_handler.py   # FIFO 命令监听与解析
├── start.sh          # 启动脚本（封装环境变量）
├── DEPLOY.md         # 本部署文档
├── README.md         # 项目说明与命令参考
└── gif_source/       # 14 个表情 GIF
    ├── standby.gif   # 待机（默认启动显示）
    ├── happy.gif
    ├── sad.gif
    ├── sad&tear.gif
    ├── angery.gif
    ├── afraid.gif
    ├── cold.gif
    ├── dizz.gif
    ├── enjoy.gif
    ├── kick.gif
    ├── voice.gif
    ├── sleep.gif
    ├── charging&like.gif
    └── shutdown.gif
```

## 手动启动

```bash
cd /home/aidlux/qt_-dev/emotion_display
python3 main.py
```

启动后全屏播放 `standby.gif`，无限循环，循环间隔 2 秒。

按 **Esc** 或 **Q** 退出。

## 开机自启动（GNOME + Wayland）

### 背景说明

本系统使用 GNOME + Wayland。PySide6 在 Wayland 原生模式下全屏窗口（`showFullScreen()`）存在已知兼容问题，因此启动脚本通过 `QT_QPA_PLATFORM=xcb` 强制使用 XCB 后端。

### 步骤

#### 1. 启动脚本 `start.sh`

已创建于项目根目录：

```bash
#!/bin/bash
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
cd /home/aidlux/qt_-dev/emotion_display
python3 main.py
```

#### 2. GNOME 自启动项

已创建于 `~/.config/autostart/emotion-display.desktop`：

```ini
[Desktop Entry]
Type=Application
Name=Emotion Display
Exec=/home/aidlux/qt_-dev/emotion_display/start.sh
X-GNOME-Autostart-enabled=true
X-GNOME-Autostart-Delay=5
```

- **延迟 5 秒**：等桌面环境完全就绪后再启动。
- **启用状态**：`X-GNOME-Autostart-enabled=true`。

#### 3. 生效

重启 GNOME 会话（注销重登或重启系统）后自动生效。

如需临时禁用，删除或重命名 desktop 文件：

```bash
mv ~/.config/autostart/emotion-display.desktop ~/.config/autostart/emotion-display.desktop.bak
```

## 运行时命令

通过命名管道 `/tmp/gif_cmd` 发送命令（从另一终端）：

### 切换表情

```bash
echo "play happy"         > /tmp/gif_cmd   # 无限循环，保持当前间隔
echo "play sad 3"         > /tmp/gif_cmd   # 播放 3 次后停止
echo "play sad 3 0"       > /tmp/gif_cmd   # 播放 3 次，间隔 0s（连播）
echo "play enjoy 0 0.5"   > /tmp/gif_cmd   # 无限循环，间隔 0.5s
```

格式：`play <表情名> [循环次数] [间隔秒数]`

- 表情名对应 `gif_source/` 下的文件名（不含 `.gif`）
- 循环次数：`0` = 无限循环，`3` = 播放 3 次后暂停
- 间隔秒数：不指定则保持当前值

### 调整循环间隔

```bash
echo "interval 1.5" > /tmp/gif_cmd    # 改为 1.5 秒
```

### 暂停 / 恢复

```bash
echo "stop"   > /tmp/gif_cmd           # 暂停
echo "resume" > /tmp/gif_cmd           # 恢复播放
```

### 退出

```bash
echo "quit" > /tmp/gif_cmd
```

### 可用表情

| 命令 | 对应文件 |
|------|---------|
| `play afraid` | afraid.gif |
| `play angery` | angery.gif |
| `play charging&like` | charging&like.gif |
| `play cold` | cold.gif |
| `play dizz` | dizz.gif |
| `play enjoy` | enjoy.gif |
| `play happy` | happy.gif |
| `play kick` | kick.gif |
| `play sad` | sad.gif |
| `play sad&tear` | sad&tear.gif |
| `play shutdown` | shutdown.gif |
| `play sleep` | sleep.gif |
| `play standby` | standby.gif |
| `play voice` | voice.gif |

## 常见问题

### Qt 报错 "xcb-cursor0 or libxcb-cursor0 is needed"

```bash
sudo apt install -y libxcb-cursor0
```

### PySide6 未安装

```bash
pip3 install PySide6
```

确认安装成功：

```bash
python3 -c "from PySide6.QtWidgets import QApplication; print('OK')"
```

### 启动后窗口不显示

检查是否已设置 `DISPLAY`：

```bash
echo $DISPLAY    # 应为 :0
```

若使用 Wayland 但未强制 XCB，可先尝试：

```bash
export QT_QPA_PLATFORM=xcb
python3 main.py
```

### 自启动后程序未运行

检查 desktop 文件：

```bash
ls -la ~/.config/autostart/emotion-display.desktop
```

检查启动日志（如果有输出重定向）：

```bash
cat /tmp/emotion_display.log
```

尝试手动执行 `start.sh` 确认无误：

```bash
/home/aidlux/qt_-dev/emotion_display/start.sh
```

### 多显示器问题

当前仅支持单显示器（HDMI）。如需多显示器选屏，需修改 `gif_player.py`，使用 `QScreen` 指定目标屏幕。
