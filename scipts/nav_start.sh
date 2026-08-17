#!/bin/bash
# 哨兵机器人一键导航启动脚本
#
# 启动链路:
#   1. livox_ros_driver2          雷达驱动 (MID360)
#   2. sentry_description         静态 TF: base_link -> livox_frame
#   3. livox_to_laserscan         CustomMsg -> PointCloud2 -> /scan (局部代价地图输入)
#   4. localizer_launch           fastlio2 (odom->base_link) + localizer (map->odom)
#   5. relocalize 服务            加载 PCD 地图并完成初始定位
#   6. navigation2.launch         Nav2 (map_server + 规划/控制, 全局代价地图输入 pgm 地图)
#
# 用法:
#   ./nav_start.sh                      # 默认初始位姿 (0, 0, 0, yaw=0)
#   ./nav_start.sh 1.5 -2.0 0 1.57      # 指定初始位姿 x y z yaw(rad)

# 禁止以 source 方式运行 (会污染当前 shell 并导致 trap/wait 行为异常)
if (return 0 2>/dev/null); then
    echo "错误: 请勿用 source 运行本脚本, 请使用: ./scipts/nav_start.sh"
    return 1 2>/dev/null || exit 1
fi

# 脚本位于 <工作空间>/scipts/ 下, 工作空间根为脚本目录的上级
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
    echo "错误: 未找到 $WS_DIR/install/setup.bash, 请先在工作空间根目录执行 colcon build"
    exit 1
fi
source "$WS_DIR/install/setup.bash"

# ---------- 0. 清理残留进程 (避免重复启动导致节点成对/话题冲突) ----------
if pgrep -f 'lio_node|localizer_node|controller_server|map_server|rviz2' >/dev/null 2>&1; then
    echo "[nav_start] 检测到残留导航进程, 自动清理..."
    bash "$SCRIPT_DIR/stop_all.sh"
    sleep 1
fi

# ---------- 参数 ----------
PCD_MAP="$WS_DIR/maps/floor/pcd/map.pcd"
INIT_X=${1:-0.0}
INIT_Y=${2:-0.0}
INIT_Z=${3:-0.0}
INIT_YAW=${4:-0.0}

PIDS=()
cleanup() {
    echo ""
    echo "[nav_start] 正在停止所有节点..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    # 清理各 launch 拉起的子进程
    pkill -f 'ros2 launch' 2>/dev/null
    pkill -f 'lio_node|localizer_node|map_server|controller_server|planner_server|rviz2' 2>/dev/null
    echo "[nav_start] 已退出"
    exit 0
}
trap cleanup INT TERM

start() {
    echo "[nav_start] 启动: $*"
    "$@" &
    PIDS+=($!)
    sleep 1
}

# ---------- 1. 雷达驱动 ----------
start ros2 launch livox_ros_driver2 msg_MID360s_launch.py

# ---------- 2. 静态 TF (base_link -> livox_frame) ----------
start ros2 launch sentry_description description_launch.py

# ---------- 3. 点云转激光扫描 (/scan, 局部代价地图输入) ----------
start ros2 launch livox_to_laserscan livox_scan.launch.py

# ---------- 4. 定位: fastlio2 + localizer ----------
start ros2 launch localizer localizer_launch.py

# ---------- 5. 加载 PCD 地图并初始定位 ----------
echo "[nav_start] 等待 /localizer/relocalize 服务就绪..."
for i in $(seq 1 30); do
    if ros2 service list 2>/dev/null | grep -q '/localizer/relocalize'; then
        break
    fi
    if [ "$i" -eq 30 ]; then
        echo "[nav_start] 警告: 等待服务超时, 跳过初始定位, 请手动调用 relocalize 服务"
    fi
    sleep 1
done

if ros2 service list 2>/dev/null | grep -q '/localizer/relocalize'; then
    echo "[nav_start] 加载地图 $PCD_MAP, 初始位姿 ($INIT_X, $INIT_Y, $INIT_Z, yaw=$INIT_YAW)"
    ros2 service call /localizer/relocalize interface/srv/Relocalize \
        "{pcd_path: '$PCD_MAP', x: $INIT_X, y: $INIT_Y, z: $INIT_Z, yaw: $INIT_YAW, pitch: 0.0, roll: 0.0}"
fi

# ---------- 6. Nav2 导航栈 ----------
start ros2 launch sentry_nav2_bringup navigation2.launch.py

echo ""
echo "[nav_start] 全部启动完成, Ctrl+C 一键停止"
echo "[nav_start] RViz 中使用 2D Goal Pose 下发导航目标"

# 保持前台, 等待任意子进程退出
wait
