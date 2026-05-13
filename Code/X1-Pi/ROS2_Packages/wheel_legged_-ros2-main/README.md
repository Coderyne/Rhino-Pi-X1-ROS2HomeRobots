# Wheel Legged Robot — ROS2 开发工作空间

> 轮足机器人上位机 ROS2 Humble 开发工作空间，支持底盘通讯、Web 可视化、激光 SLAM 建图、自主导航与 NPU 大模型对话。

## 项目结构

```
wheel_legged_-ros2/
├── src/
│   ├── stm32_bridge/              # STM32 USB CDC 通讯桥接包
│   │   ├── stm32_bridge/
│   │   │   ├── protocol.py        # CRC16 / 帧打包解包 / 逐字节状态机解析
│   │   │   └── wheel_foot_bridge.py  # 主节点 (遥测/指令/TF广播)
│   │   ├── launch/
│   │   ├── config/
│   │   └── README.md
│   ├── wheel_foot_nav/            # SLAM & Nav2 导航包
│   │   ├── launch/
│   │   │   ├── slam.launch.py     # 建图 (slam_toolbox + 雷达 + RViz)
│   │   │   ├── nav.launch.py      # 导航 (AMCL + Nav2 + 地图)
│   │   │   └── bringup.launch.py  # 基础 bringup
│   │   ├── config/
│   │   │   ├── slam_toolbox.yaml  # SLAM 参数
│   │   │   └── nav2_params.yaml   # Nav2 全套参数
│   │   └── maps/                  # 保存的地图文件
│   ├── llm_interfaces/            # LLM 自定义接口定义
│   │   ├── srv/Chat.srv           # Service: 请求-响应
│   │   └── action/ChatStream.action  # Action: 异步流式
├── ldlidar_ros2_ws/               # LD19/LD06 激光雷达工作空间
│   └── src/ldlidar_stl_ros2/
├── web/                           # Web 可视化仪表盘
│   ├── index.html                 # 实时数值/曲线/关节/控制 + 地图导航面板
│   ├── css/dashboard.css
│   └── js/dashboard.js            # roslibjs 连接 + 地图渲染 + 导航交互
├── start.sh                       # 一键启动脚本
└── 通信协议-STM32与ROS2.md       # 下位机通讯协议参考
```

## 功能包

| 包名 | 工作空间 | 说明 |
|------|----------|------|
| `stm32_bridge` | wheel_legged_-ros2 | STM32 USB CDC 通讯桥接（遥测/指令/TF广播） |
| `wheel_foot_nav` | wheel_legged_-ros2 | SLAM 建图 & Nav2 导航启动与配置 |
| `llm_interfaces` | wheel_legged_-ros2 | 大模型 ROS2 接口定义 (Chat.srv, ChatStream.action) |
| `ldlidar_stl_ros2` | ldlidar_ros2_ws | LD19/LD06 激光雷达驱动 |

## 依赖

```bash
# 系统包
sudo apt install ros-humble-rosbridge-suite \
               ros-humble-slam-toolbox \
               ros-humble-navigation2 \
               ros-humble-nav2-bringup \
               ros-humble-tf2-ros

# Python 包
pip3 install pyserial
```

## 编译

```bash
# 主工作空间
cd ~/ROS2_Dev/wheel_legged_-ros2
source /opt/ros/humble/setup.bash

# 全部编译
colcon build

# 或按需编译
colcon build --packages-select stm32_bridge wheel_foot_nav llm_interfaces llm_bridge

source install/setup.bash

# 雷达驱动 (如未编译)
cd ~/ROS2_Dev/wheel_legged_-ros2/ldlidar_ros2_ws
colcon build
```

## TF 坐标树

```
map → odom → base_footprint → base_link → base_laser
 ↑      ↑         ↑              ↑            ↑
slam   bridge新增  bridge发布    静态         lidar驱动
```

- `odom → base_footprint` 由 `wheel_foot_bridge` 发布，通过积分 `vel_n` 和 `yaw` 获得
- `base_link → base_laser` 由 `ldlidar_stl_ros2` 静态广播
- `map → odom` 由 `slam_toolbox` / `AMCL` 动态修正

## Web 仪表盘功能

浏览器打开 `http://<IP地址>:8192`，通过 rosbridge WebSocket 连接机器人。

### 数据监控
| 区域 | 内容 |
|------|------|
| 实时数值 | Yaw / Roll / Pitch / 速度 / 电池 / 运行状态 |
| 时序曲线 | 速度曲线 (vel_n + 角速度) / 姿态曲线 (roll + pitch) |
| 关节状态 | 左/右腿摆角条、腿长条、轮力矩条 |
| 触地状态 | 左/右着地指示、跳跃阶段 |

### 地图与导航
| 功能 | 说明 |
|------|------|
| 实时地图 | 订阅 `/map`，Canvas 渲染 OccupancyGrid（未知/空闲/障碍） |
| 激光扫描叠加 | 订阅 `/scan`，绿色半透明点云 |
| 路径显示 | 订阅 `/plan`，蓝色折线显示 Nav2 规划路径 |
| 机器人位置 | 订阅 `/tf`，红色三角箭头标示 map→base_footprint |
| 导航目标 | 点击按钮 → 地图选点 → 发布 `/goal_pose` |
| 初始位姿 | 点击按钮 → 地图选点+朝向 → 发布 `/initialpose` |
| 保存地图 | 调用 `/map_saver/save_map` 服务 |
| 取消导航 | 发送零速 `cmd_vel` |

### 控制面板
`/cmd_vel` 滑条、使能/停止/急停/跳跃/自起按钮、`/cmd_attitude` 腿长/姿态滑条

## 运行

### 一键启动

```bash
./start.sh              # 基础模式: rosbridge + 底盘桥接 + Web 仪表盘
./start.sh slam         # 建图模式: 基础 + 雷达 + slam_toolbox + RViz
./start.sh nav <地图>   # 导航模式: 基础 + 雷达 + Nav2 + 地图
./start.sh attach       # 启动并进入 tmux 查看日志
./start.sh stop         # 停止所有服务
```

启动后访问 `http://<IP地址>:8192` 打开仪表盘。自定义端口：

```bash
ROSBRIDGE_PORT=9092 WEB_PORT=9000 ./start.sh
```

### SLAM 建图流程

```bash
# 1. 启动建图
./start.sh slam

# 2. 遥控机器人移动覆盖环境
#    浏览器仪表盘可实时查看地图和扫描数据

# 3. 保存地图 (两种方式)
#    方式A: Web 仪表盘点击 "保存地图" 按钮
#    方式B: 命令行
ros2 run nav2_map_server map_saver_cli -f src/wheel_foot_nav/maps/map
```

### Nav2 导航流程

```bash
# 1. 启动导航
./start.sh nav map

# 2. 在 Web 仪表盘中:
#    - 点击 "设初始位姿" → 地图上点选位置 → 点选朝向
#    - 点击 "导航目标" → 地图上点选目标点
#    - 机器人自动规划并执行, 地图面板实时显示路径和位置
#
#    也可在 RViz 中使用 2D Pose Estimate / Nav2 Goal 工具
```

### 手动启动

```bash
# 终端 1: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091

# 终端 2: 底盘桥接
ros2 launch stm32_bridge bridge.launch.py

# 终端 3: Web 仪表盘
python3 -m http.server 8192 --directory ~/ROS2_Dev/wheel_legged_-ros2/web

# 终端 4: 雷达
ros2 launch ldlidar_stl_ros2 ld19.launch.py

# 终端 5: SLAM 建图
ros2 launch wheel_foot_nav slam.launch.py

## NPU 大模型对话

`llm_bridge` 将大语言模型封装为 ROS2 Service 和 Action，支持两种推理后端：

| 后端 | 启动参数 | 说明 |
|------|----------|------|
| **本地 NPU** | `backend:=npu`（默认） | 高通 NPU Qwen3-4B，离线推理 |
| **云端 API** | `backend:=openai` | DeepSeek / Qwen / GPT OpenAI 兼容接口 |

### 启动

```bash
# 本地 NPU
ros2 launch llm_bridge llm_bridge.launch.py

# 云端 DeepSeek
ros2 launch llm_bridge llm_bridge.launch.py \
    backend:=openai \
    openai_api_key:=sk-xxx

# 内存紧张时降低上下文
ros2 launch llm_bridge llm_bridge.launch.py \
    max_context_len:=2048 \
    max_new_tokens:=512
```

### 调用

```bash
# Service 方式: 同步问答
ros2 service call /llm_bridge/chat llm_interfaces/srv/Chat \
    "{prompt: '用一句话解释ROS2是什么'}"

# Action 方式: 流式输出 (实时逐字显示)
ros2 action send_goal /llm_bridge/chat_stream \
    llm_interfaces/action/ChatStream \
    "{prompt: '什么是SLAM?'}" --feedback
```

### 从 Python 代码调用

```python
from llm_interfaces.srv import Chat
# 同步
client = node.create_client(Chat, '/llm_bridge/chat')
resp = client.call(Chat.Request(prompt="你好"))

from llm_interfaces.action import ChatStream
# 流式
client = ActionClient(node, ChatStream, '/llm_bridge/chat_stream')
goal = client.send_goal(ChatStream.Goal(prompt="写一段Python代码"),
                         feedback_callback=lambda fb: print(fb.partial_text))
```

### 独立 CLI 工具

不启动 ROS2 节点，直接命令行对话：

```bash
cd ~/ROS2_Dev/llm
python3 infer.py                     # 交互式对话
python3 infer.py --prompt "你好"     # 单次问答
```
