#!/bin/bash
# 强制停止所有导航相关进程 (用于清理残留, 解决重复启动导致的节点成对问题)

if (return 0 2>/dev/null); then
    echo "错误: 请勿用 source 运行, 请使用: ./scipts/stop_all.sh"
    return 1 2>/dev/null || exit 1
fi

echo "[stop_all] 正在停止所有导航相关进程..."

# launch 进程
pkill -9 -f 'ros2 launch' 2>/dev/null
pkill -9 -f 'ros2 run' 2>/dev/null

# 定位/建图
pkill -9 -f 'lio_node' 2>/dev/null
pkill -9 -f 'localizer_node' 2>/dev/null

# nav2 全套
pkill -9 -f 'controller_server' 2>/dev/null
pkill -9 -f 'planner_server' 2>/dev/null
pkill -9 -f 'smoother_server' 2>/dev/null
pkill -9 -f 'behavior_server' 2>/dev/null
pkill -9 -f 'bt_navigator' 2>/dev/null
pkill -9 -f 'waypoint_follower' 2>/dev/null
pkill -9 -f 'velocity_smoother' 2>/dev/null
pkill -9 -f 'collision_monitor' 2>/dev/null
pkill -9 -f 'docking_server' 2>/dev/null
pkill -9 -f 'route_server' 2>/dev/null
pkill -9 -f 'map_server' 2>/dev/null
pkill -9 -f 'map_saver' 2>/dev/null
pkill -9 -f 'lifecycle_manager' 2>/dev/null
pkill -9 -f 'component_container' 2>/dev/null

# 传感器与转换
pkill -9 -f 'livox_lidar' 2>/dev/null
pkill -9 -f 'livox_to_pointcloud' 2>/dev/null
pkill -9 -f 'cloud_to_scan' 2>/dev/null

# 模型与可视化
pkill -9 -f 'robot_state_publisher' 2>/dev/null
pkill -9 -f 'rviz2' 2>/dev/null

sleep 2

# 检查残留
REMAIN=$(pgrep -f 'lio_node|localizer_node|controller_server|planner_server|bt_navigator|map_server|rviz2|component_container|livox' 2>/dev/null | wc -l)
if [ "$REMAIN" -gt 0 ]; then
    echo "[stop_all] 仍有 $REMAIN 个进程残留:"
    pgrep -af 'lio_node|localizer_node|controller_server|planner_server|bt_navigator|map_server|rviz2|component_container|livox' 2>/dev/null
    echo "[stop_all] 请将上面列出的 PID 手动 kill: kill -9 <PID>"
    exit 1
else
    echo "[stop_all] 已全部停止"
    exit 0
fi
