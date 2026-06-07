#!/usr/bin/env bash
#
# 轮足机器人 — 一键启动脚本
#
# 用法:
#   ./start.sh            # 基础服务: rosbridge + stm32_bridge + Web 仪表盘
#   ./start.sh slam       # 建图模式: 基础 + 雷达 + slam_toolbox + RViz
#   ./start.sh nav MAP    # 导航模式: 基础 + 雷达 + Nav2 (MAP=地图名称)
#   ./start.sh attach     # 启动并进入 tmux 会话
#   ./start.sh stop       # 停止所有服务

set -e

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
SESSION="ros2"
ROS_SETUP="/opt/ros/humble/setup.bash"
LOCAL_SETUP="$ROOT_DIR/install/setup.bash"
LIDAR_SETUP="$ROOT_DIR/ldlidar_ros2_ws/install/setup.bash"
WEB_DIR="$ROOT_DIR/web"

ROSBRIDGE_PORT=${ROSBRIDGE_PORT:-9091}
WEB_PORT=${WEB_PORT:-8192}
CAMERA_PORT=${CAMERA_PORT:-8193}

# ── 颜色输出 ───────────────────────────────────────
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

log()  { echo -e "${GREEN}[INFO]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
err()  { echo -e "${RED}[ERROR]${NC} $1"; }

# ═════════════════════════════════════════════════════
#  stop — 停止服务
# ═════════════════════════════════════════════════════
do_stop() {
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux kill-session -t "$SESSION"
        log "已停止所有服务"
    else
        warn "服务未在运行"
    fi
}

# ═════════════════════════════════════════════════════
#  do_start_base — 启动基础服务 (共 3 个窗口)
# ═════════════════════════════════════════════════════
do_start_base() {
    if [ ! -f "$LOCAL_SETUP" ]; then
        warn "工作空间未编译, 跳过 stm32_bridge"
        HAS_BRIDGE=false
    else
        HAS_BRIDGE=true
    fi
    if [ ! -d "$WEB_DIR" ]; then
        warn "Web 目录不存在, 跳过仪表盘"
        HAS_WEB=false
    else
        HAS_WEB=true
    fi

    # 窗口 0: rosbridge
    tmux new-session -d -s "$SESSION" -n rosbridge \
        "bash -c 'source $ROS_SETUP && ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=$ROSBRIDGE_PORT'; exec bash"

    # 窗口 1: stm32_bridge
    if $HAS_BRIDGE; then
        tmux new-window -t "$SESSION" -n stm32 \
            "bash -c 'source $ROS_SETUP && source $LOCAL_SETUP && ros2 launch stm32_bridge bridge.launch.py'; exec bash"
    fi

    # 窗口 2: mqtt_bridge (MQTT ↔ ROS2)
    tmux new-window -t "$SESSION" -n mqtt \
        "bash -c 'source $ROS_SETUP && source $LOCAL_SETUP && ros2 launch mqtt_bridge mqtt_bridge.launch.py'; exec bash"

    # 窗口 3: region_manager (区域导航)
    if $HAS_BRIDGE; then
        tmux new-window -t "$SESSION" -n region \
            "bash -c 'source $ROS_SETUP && source $LOCAL_SETUP && ros2 launch region_manager region_manager.launch.py'; exec bash"
    fi

    # 窗口 4: Web server
    if $HAS_WEB; then
        tmux new-window -t "$SESSION" -n web \
            "python3 -m http.server $WEB_PORT --directory $WEB_DIR; exec bash"
    fi

    # 窗口 5: Camera stream (MJPEG)
    CAMERA_STREAM_SCRIPT="$ROOT_DIR/scripts/camera_stream.py"
    if [ -f "$CAMERA_STREAM_SCRIPT" ]; then
        tmux new-window -t "$SESSION" -n camera \
            "python3 $CAMERA_STREAM_SCRIPT --port $CAMERA_PORT --device /dev/video2 --width 1280 --height 720 --max-fps 30 --quality 60; exec bash"
    fi
}

# ═════════════════════════════════════════════════════
#  start — 基础模式
# ═════════════════════════════════════════════════════
do_start() {
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        warn "服务已在运行中, 请先执行 $0 stop 停止"
        exit 1
    fi

    if [ ! -f "$ROS_SETUP" ]; then
        err "ROS2 未安装: $ROS_SETUP"
        exit 1
    fi

    log "启动 rosbridge (WebSocket: $ROSBRIDGE_PORT) ..."
    log "启动 stm32_bridge ..."
    log "启动 mqtt_bridge (MQTT ↔ ROS2) ..."
    log "启动 region_manager (区域导航) ..."
    log "启动 Web 仪表盘 (HTTP: $WEB_PORT) ..."
    echo ""

    do_start_base

    sleep 2
    print_summary basic
}

# ═════════════════════════════════════════════════════
#  slam — 建图模式 (基础 + 雷达 + slam_toolbox + rviz)
# ═════════════════════════════════════════════════════
do_slam() {
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        warn "服务已在运行中, 请先执行 $0 stop 停止"
        exit 1
    fi

    if [ ! -f "$ROS_SETUP" ]; then
        err "ROS2 未安装: $ROS_SETUP"
        exit 1
    fi
    if [ ! -f "$LIDAR_SETUP" ]; then
        warn "雷达工作空间未编译, 尝试跳过雷达"
        HAS_LIDAR=false
    else
        HAS_LIDAR=true
    fi

    log "═══════════════════════════════════════"
    log "  SLAM 建图模式"
    log "═══════════════════════════════════════"
    echo ""

    do_start_base

    # 窗口 4: 激光雷达
    if $HAS_LIDAR; then
        tmux new-window -t "$SESSION" -n lidar \
            "bash -c 'source $ROS_SETUP && source $LIDAR_SETUP && ros2 launch ldlidar_stl_ros2 ld06.launch.py'; exec bash"
    fi

    # 窗口 5: slam_toolbox + RViz2
    tmux new-window -t "$SESSION" -n slam \
        "bash -c 'source $ROS_SETUP && source $LOCAL_SETUP && ros2 launch wheel_foot_nav slam.launch.py'; exec bash"

    sleep 3
    print_summary slam
}

# ═════════════════════════════════════════════════════
#  nav — 导航模式 (基础 + 雷达 + Nav2 + 地图)
# ═════════════════════════════════════════════════════
do_nav() {
    MAP_NAME="${1}"
    if [ -z "$MAP_NAME" ]; then
        err "缺少地图名称参数"
        echo "用法: $0 nav <地图名称>"
        echo "  地图文件应位于: src/wheel_foot_nav/maps/<名称>.yaml"
        exit 1
    fi

    if tmux has-session -t "$SESSION" 2>/dev/null; then
        warn "服务已在运行中, 请先执行 $0 stop 停止"
        exit 1
    fi

    if [ ! -f "$ROS_SETUP" ]; then
        err "ROS2 未安装: $ROS_SETUP"
        exit 1
    fi
    if [ ! -f "$LIDAR_SETUP" ]; then
        warn "雷达工作空间未编译, 尝试跳过雷达"
        HAS_LIDAR=false
    else
        HAS_LIDAR=true
    fi

    log "═══════════════════════════════════════"
    log "  Nav2 导航模式  地图: $MAP_NAME"
    log "═══════════════════════════════════════"
    echo ""

    do_start_base

    # 窗口 4: 激光雷达
    if $HAS_LIDAR; then
        tmux new-window -t "$SESSION" -n lidar \
            "bash -c 'source $ROS_SETUP && source $LIDAR_SETUP && ros2 launch ldlidar_stl_ros2 ld06.launch.py'; exec bash"
    fi

    # 窗口 5: Nav2 + RViz2 (map_server + AMCL + planner + controller)
    tmux new-window -t "$SESSION" -n nav2 \
        "bash -c 'source $ROS_SETUP && source $LOCAL_SETUP && ros2 launch wheel_foot_nav nav.launch.py map:=${MAP_NAME}'; exec bash"

    sleep 3
    print_summary nav
}

# ═════════════════════════════════════════════════════
#  print_summary
# ═════════════════════════════════════════════════════
print_summary() {
    local MODE="$1"
    echo ""
    log "═══════════════════════════════════════"
    log "  所有服务已启动 (tmux session: $SESSION)"
    log "═══════════════════════════════════════"
    log "  rosbridge  : ws://localhost:$ROSBRIDGE_PORT"
    log "  stm32_bridge: /dev/ttyACM0"
    log "  mqtt_bridge : ws://localhost:1883"
    if [ "$MODE" = "slam" ] || [ "$MODE" = "nav" ]; then
        log "  激光雷达   : /dev/ttyUSB0 (LD06)"
    fi
    if [ "$MODE" = "slam" ]; then
        log "  slam_toolbox: 在线异步建图"
    fi
    if [ "$MODE" = "nav" ]; then
        log "  Nav2       : AMCL + 路径规划 + 控制"
    fi
    log "  仪表盘     : http://$(hostname -I 2>/dev/null | awk '{print $1}' || echo 'localhost'):$WEB_PORT"
    log "  摄像头流   : http://$(hostname -I 2>/dev/null | awk '{print $1}' || echo 'localhost'):$CAMERA_PORT/stream"
    log ""
    log "  管理命令:"
    log "    $0 attach  进入 tmux 查看日志"
    log "    $0 stop    停止所有服务"
    if [ "$MODE" = "slam" ]; then
        log ""
        log "  保存地图:"
        log "    ros2 run nav2_map_server map_saver_cli -f src/wheel_foot_nav/maps/map"
    fi
}

# ═════════════════════════════════════════════════════
#  main
# ═════════════════════════════════════════════════════
case "${1:-start}" in
    start)
        do_start
        ;;
    slam)
        do_slam
        ;;
    nav)
        do_nav "${2}"
        ;;
    attach)
        do_start
        sleep 1
        tmux attach -t "$SESSION"
        ;;
    stop)
        do_stop
        ;;
    *)
        echo "用法: $0 {start|slam|nav <map>|attach|stop}"
        echo ""
        echo "  start      启动基础服务 (rosbridge + 桥接 + MQTT + Web)"
        echo "  slam       建图模式 (基础 + 雷达 + slam_toolbox)"
        echo "  nav <map>  导航模式 (基础 + 雷达 + Nav2, 指定地图名)"
        echo "  attach     启动并进入 tmux"
        echo "  stop       停止所有服务"
        exit 1
        ;;
esac
