# Wheel Legged Robot — ROS2 开发工作空间

> 轮足机器人上位机 ROS2 Humble 开发工作空间，支持底盘通讯、Web 可视化、激光 SLAM 建图、自主导航、区域导航、MQTT 桥接。

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
│   │   │   ├── slam.launch.py     # 建图 (slam_toolbox + RViz)
│   │   │   ├── nav.launch.py      # 导航 (map_server + AMCL + Nav2 + RViz + 人跟随)
│   │   │   └── bringup.launch.py  # 基础 bringup
│   │   ├── config/
│   │   │   ├── slam_toolbox.yaml  # SLAM 参数
│   │   │   └── nav2_params.yaml   # Nav2 全套参数
│   │   ├── rviz/
│   │   │   ├── slam.rviz          # 建图 RViz 配置 (自动加载)
│   │   │   └── nav.rviz           # 导航 RViz 配置 (自动加载)
│   │   ├── scripts/
│   │   │   └── lifecycle_boot.py  # map_server/AMCL 等 9 节点生命周期引导
│   │   └── maps/                  # 保存的地图文件
│   ├── perception/                 # 人体跟随 (LiDAR 质心漂移 + Kalman)
│   │   ├── perception/
│   │   │   └── person_follower.py # 跟随节点 (质心漂移 + Kalman + TF → /goal_pose)
│   │   ├── config/
│   │   │   └── params.yaml        # 跟随参数 (搜索半径/跟随距离/限流阈值)
│   │   └── launch/
│   │       └── person_follower.launch.py
│   ├── mqtt_bridge/               # MQTT ↔ ROS2 桥接包
│   │   ├── mqtt_bridge/
│   │   │   └── mqtt_node.py       # MQTT 协议转 ROS2 节点
│   │   ├── launch/
│   │   └── config/
│   └── region_manager/            # 区域导航管理
│       ├── region_manager/
│       │   ├── region_manager.py  # ROS2 节点 (Topic+Service, MarkerArray 可视化)
│       │   └── region_store.py    # JSON 文件持久化
│       ├── launch/
│       └── config/
├── ldlidar_ros2_ws/               # LD19/LD06 激光雷达工作空间
│   └── src/ldlidar_stl_ros2/
├── reloc                           # AMCL 全局重定位独立脚本
├── web/                           # Web 可视化仪表盘
│   ├── index.html                 # 实时数值/关节/控制 + 地图导航面板 + 跟随 + 区域管理
│   ├── css/dashboard.css
│   └── js/dashboard.js            # roslibjs 连接 + 地图渲染(缩放/平移) + 导航交互 + 跟随 + 区域划分
├── start.sh                       # 一键启动脚本
└── 通信协议-STM32与ROS2.md       # 下位机通讯协议参考
```

## 功能包

| 包名 | 工作空间 | 说明 |
|------|----------|------|
| `stm32_bridge` | wheel_legged_-ros2 | STM32 USB CDC 通讯桥接（遥测/指令/TF广播） |
| `wheel_foot_nav` | wheel_legged_-ros2 | SLAM 建图 & Nav2 导航启动与配置 |
| `perception` | wheel_legged_-ros2 | LiDAR 人体跟随（质心漂移 + Kalman + Nav2 目标发布） |
| `mqtt_bridge` | wheel_legged_-ros2 | MQTT ↔ ROS2 双向桥接 (Web/MQTT 控制) |
| `region_manager` | wheel_legged_-ros2 | 地图区域划分、持久化存储、区域导航 (Topic + Service) |
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
cd ~/ROS2_Dev/wheel_legged_-ros2          # 实际路径替换为你的工作空间
source /opt/ros/humble/setup.bash

# 全部编译
colcon build

# 或按需编译
colcon build --packages-select stm32_bridge wheel_foot_nav perception mqtt_bridge region_manager

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
| 缩放/平移 | 鼠标滚轮缩放(以光标为中心) + 拖拽平移 + +/- 按钮 |
| 激光扫描叠加 | 订阅 `/scan`，绿色半透明点云 |
| 路径显示 | 订阅 `/plan`，蓝色折线显示 Nav2 规划路径 |
| 机器人位置 | 订阅 `/tf`，红色三角箭头标示 map→base_footprint |
| 导航目标 | 点击按钮 → 两阶段选点(位置+朝向) → 发布 `/goal_pose` |
| 初始位姿 | 点击按钮 → 两阶段选点(位置+朝向) → 发布 `/initialpose` |
| 人体跟随 | 点击按钮 → 矩形框选目标 → 发布 `/follow_target`，机器人自动跟随 |
| 全局重定位 | 点击按钮 → 调用 `/reinitialize_global_localization` 撒粒子收敛 |
| 保存地图 | 调用 `/map_saver/save_map` 服务 |
| 取消导航 | 发送零速 `cmd_vel` + 当前位置 goal 终止导航 |
| 区域管理 | 点击按钮 → 矩形框选区域 → 命名+选色 → 持久化保存 |
| 区域导航 | 点击地图上的区域矩形 → 自动发布 `/goal_pose` 导航到区域中心 |

### 控制面板
`/cmd_vel` 滑条、使能/停止/急停/跳跃/自起按钮、`/cmd_attitude` 腿长/姿态滑条

## 运行

### 一键启动

```bash
./start.sh              # 基础模式: rosbridge + 底盘桥接 + MQTT + 区域管理 + Web 仪表盘
./start.sh slam         # 建图模式: 基础 + 雷达 + slam_toolbox + RViz
./start.sh nav <地图>   # 导航模式: 基础 + 雷达 + Nav2 + 人跟随 + RViz
./start.sh reloc        # 触发 AMCL 全局重定位 (导航模式运行时)
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
# 1. 启动导航 (RViz 自动加载 nav.rviz 配置, 人跟随节点自动启动)
./start.sh nav map

# 2. AMCL 启动后自动调用 /reinitialize_global_localization 全局重定位，
#    粒子全地图随机撒播，通过 /scan 扫描匹配自动收敛到正确位姿。

# 3. 设置导航目标:
#    - Web 仪表盘: 点击 "导航目标" → 两阶段选点(位置+朝向)
#    - Web 仪表盘: 点击地图上的彩色区域矩形 → 自动导航到区域中心
#    - RViz: 使用 Nav2 Goal 工具
#    - 机器人自动规划并执行, 地图面板实时显示路径和位置

# 4. 人体跟随:
#    - Web 仪表盘: 点击 "跟随" → 矩形框选目标区域 → 机器人自动跟随
#    - 点击 "取消" 停止跟随

# 5. 手动重定位 (机器人被搬动后):
./start.sh reloc
```

### 人体跟随参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `target_radius` | 0.3m | 目标搜索半径 (框选时由矩形动态覆盖) |
| `follow_distance` | 0.8m | 期望跟随距离 |
| `goal_min_distance` | 0.025m | 目标位移阈值 (低于此不发新goal) |
| `goal_min_interval` | 0.5s | 两次 goal 最小间隔 |
| `enable_kalman` | true | Kalman 滤波平滑 |

配置文件: `src/perception/config/params.yaml`

### 区域导航

`region_manager` 将地图划分为可命名的矩形区域，支持 Web 端交互和外部程序调用。

#### 区域管理

1. **创建区域** — Web 仪表盘点击 "区域管理" → 地图上矩形框选 → 输入名称和颜色
2. **导航到区域** — 直接点击地图上的彩色区域矩形，机器人自动规划到区域中心点
3. **侧边栏管理** — 区域列表显示所有已划分区域，支持点击导航和删除

#### 接口

```bash
# 命令行导航到区域
ros2 topic pub /region_manager/navigate std_msgs/String \
    "data: '{\"name\":\"客厅\"}'" --once

# 列出所有区域
ros2 service call /region_manager/list_regions std_srvs/srv/Trigger

# 保存区域
ros2 topic pub /region_manager/save std_msgs/String \
    "data: '{\"name\":\"客厅\",\"x1\":-2.5,\"y1\":1.0,\"x2\":-0.5,\"y2\":3.0,\"color\":\"#FF6B6B\"}'" --once

# 删除区域
ros2 topic pub /region_manager/delete std_msgs/String \
    "data: '{\"name\":\"客厅\"}'" --once
```

#### Python 调用

```python
import json
from std_msgs.msg import String

# 导航到区域
pub = node.create_publisher(String, '/region_manager/navigate', 10)
pub.publish(String(data=json.dumps({"name": "客厅"})))

# LLM 集成示例: "去客厅" → 解析 → 区域导航
```

#### 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `regions_file` | `~/robot_regions.json` | 区域数据持久化文件路径 |
| `map_frame` | `map` | 地图坐标系 |

配置文件: `src/region_manager/config/params.yaml`

区域数据存储在 JSON 文件中，格式如下：

```json
{
  "regions": [
    {
      "name": "客厅",
      "x1": -2.5, "y1": 1.0,
      "x2": -0.5, "y2": 3.0,
      "center_x": -1.5, "center_y": 2.0,
      "color": "#FF6B6B"
    }
  ]
}
```

### 手动启动

```bash
# 终端 1: rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9091

# 终端 2: 底盘桥接
ros2 launch stm32_bridge bridge.launch.py

# 终端 3: Web 仪表盘
python3 -m http.server 8192 --directory web

# 终端 4: 雷达
ros2 launch ldlidar_stl_ros2 ld19.launch.py

# 终端 5: SLAM 建图
ros2 launch wheel_foot_nav slam.launch.py

# 终端 6: 区域管理 (可选)
ros2 launch region_manager region_manager.launch.py
```


