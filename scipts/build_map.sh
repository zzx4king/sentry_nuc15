#!/usr/bin/env bash
#
# build_map.sh
#
# 建图脚本: 依次启动 livox 雷达驱动 (msg_MID360s_launch.py)、fastlio2
# (lio_node) 与 pgo (pgo_node) 进行激光建图, 运行指定时间后调用
# /pgo/save_maps 服务将点云地图保存为 map.pcd, 并自动停止全部节点。
#
# 输出目录结构:
#   /home/robomaster/project/sentry_ws/maps/<map_name>/
#     ├── pcd/map.pcd    # 建图生成的点云地图
#     └── pgm/           # 预留目录 (可配合 pcd2pgm.sh 生成栅格地图)
#
# 用法:
#   ./build_map.sh <map_name> <duration_seconds>
#
# 示例:
#   ./build_map.sh test 120
#

set -euo pipefail

# ==================== 基础配置 ====================
WORKSPACE_DIR="/home/robomaster/project/sentry_ws"
MAPS_ROOT_DIR="${WORKSPACE_DIR}/maps"

ROS_SETUP="/opt/ros/jazzy/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

LIO_NODE_EXEC="${WORKSPACE_DIR}/install/fastlio2/lib/fastlio2/lio_node"
PGO_NODE_EXEC="${WORKSPACE_DIR}/install/pgo/lib/pgo/pgo_node"
LIO_CONFIG="${WORKSPACE_DIR}/install/fastlio2/share/fastlio2/config/lio.yaml"
PGO_CONFIG="${WORKSPACE_DIR}/install/pgo/share/pgo/config/pgo.yaml"

SAVE_MAP_SRV="/pgo/save_maps"
SAVE_MAP_SRV_TYPE="interface/srv/SaveMaps"

# livox 雷达驱动 launch (包名 + launch 文件名)
LIVOX_LAUNCH_PKG="livox_ros_driver2"
LIVOX_LAUNCH_FILE="msg_MID360s_launch.py"
LIDAR_TOPIC="/livox/lidar"

# 建图服务就绪的最大等待时间（秒）
SERVICE_READY_TIMEOUT=30
# 等待雷达点云首条数据的最大时间（秒）
# 注意: livox 驱动收到首帧点云后才创建 /livox/lidar 话题(惰性创建),
# 雷达重连后首帧数据可能较慢, 请保持足够的等待时间
LIDAR_READY_TIMEOUT=30
# 等待激光里程计首条数据的最大时间（秒）
DATA_READY_TIMEOUT=15
# 节点启动自检等待时间（秒）
NODE_STARTUP_CHECK_DELAY=2
# 保存地图服务调用的超时时间（秒）
SAVE_MAP_TIMEOUT=120

# ==================== 颜色输出 ====================
IS_TTY=0
if [[ -t 1 ]]; then
    IS_TTY=1
    C_RED='\033[0;31m'
    C_GREEN='\033[0;32m'
    C_YELLOW='\033[0;33m'
    C_BLUE='\033[0;34m'
    C_RESET='\033[0m'
else
    C_RED=''; C_GREEN=''; C_YELLOW=''; C_BLUE=''; C_RESET=''
fi

log_info()  { echo -e "${C_BLUE}[INFO]${C_RESET} $*"; }
log_ok()    { echo -e "${C_GREEN}[ OK ]${C_RESET} $*"; }
log_warn()  { echo -e "${C_YELLOW}[WARN]${C_RESET} $*"; }
log_error() { echo -e "${C_RED}[ERR ]${C_RESET} $*" >&2; }

# ==================== 运行时状态变量 ====================
LIVOX_PID=""
LIO_PID=""
PGO_PID=""
LIVOX_LOG=""
LIO_LOG=""
PGO_LOG=""
SAVER_LOG=""
ROS_LOG_DIR=""
INTERRUPTED=0
MAP_SAVED=0

# ==================== 清理函数 ====================
stop_node() {
    local name="$1" pid="$2"
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
        log_info "停止 ${name} (PID: ${pid}) ..."
        kill "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    fi
}

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM
    stop_node "pgo_node" "${PGO_PID}"
    stop_node "lio_node" "${LIO_PID}"
    stop_node "livox 驱动" "${LIVOX_PID}"
    [[ -n "${LIVOX_LOG}" && -f "${LIVOX_LOG}" ]] && rm -f "${LIVOX_LOG}" || true
    [[ -n "${LIO_LOG}" && -f "${LIO_LOG}" ]] && rm -f "${LIO_LOG}" || true
    [[ -n "${PGO_LOG}" && -f "${PGO_LOG}" ]] && rm -f "${PGO_LOG}" || true
    [[ -n "${ECHO_LOG}" && -f "${ECHO_LOG}" ]] && rm -f "${ECHO_LOG}" || true
    [[ -n "${SAVER_LOG}" && -f "${SAVER_LOG}" ]] && rm -f "${SAVER_LOG}" || true
    [[ -n "${ROS_LOG_DIR}" && -d "${ROS_LOG_DIR}" ]] && rm -rf "${ROS_LOG_DIR}" || true
    exit "${exit_code}"
}
trap cleanup EXIT

on_interrupt() {
    INTERRUPTED=1
}
trap on_interrupt INT TERM

# ==================== 帮助信息 ====================
usage() {
    cat <<EOF
用法: $(basename "$0") <map_name> <duration_seconds>

参数:
  map_name          建图目录名称（仅允许字母/数字/下划线/连字符）
  duration_seconds  建图运行时间（秒，正整数）

示例:
  $(basename "$0") test 120
EOF
}

# ==================== 参数解析 ====================
if [[ $# -ne 2 ]]; then
    log_error "参数数量错误: 期望 2 个, 实际传入 $# 个"
    usage
    exit 1
fi

MAP_NAME="$1"
DURATION="$2"

if [[ ! "${MAP_NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    log_error "无效的建图目录名称: ${MAP_NAME} (仅允许字母/数字/下划线/连字符)"
    exit 1
fi

if ! [[ "${DURATION}" =~ ^[0-9]+$ ]] || [[ "${DURATION}" -le 0 ]]; then
    log_error "无效的建图时间: ${DURATION} (必须为正整数, 单位: 秒)"
    exit 1
fi

log_ok "参数解析成功:"
log_ok "  地图名称: ${MAP_NAME}"
log_ok "  建图时长: ${DURATION}s"

# ==================== 输出目录创建 ====================
MAP_DIR="${MAPS_ROOT_DIR}/${MAP_NAME}"
PCD_DIR="${MAP_DIR}/pcd"
PGM_DIR="${MAP_DIR}/pgm"

log_info "创建地图目录 ..."

if [[ -e "${MAP_DIR}" && ! -d "${MAP_DIR}" ]]; then
    log_error "路径已存在且不是目录: ${MAP_DIR}"
    exit 1
fi

if [[ -f "${PCD_DIR}/map.pcd" ]]; then
    log_warn "已存在地图文件, 将被覆盖: ${PCD_DIR}/map.pcd"
fi

if ! mkdir -p "${PCD_DIR}" "${PGM_DIR}"; then
    log_error "创建目录失败: ${MAP_DIR}"
    exit 1
fi

if [[ ! -d "${PCD_DIR}" || ! -d "${PGM_DIR}" ]]; then
    log_error "目录创建后校验失败: ${MAP_DIR}"
    exit 1
fi

log_ok "目录创建成功:"
log_ok "  ${PCD_DIR}"
log_ok "  ${PGM_DIR}"

# ==================== 环境源化与依赖检查 ====================
log_info "源化 ROS 2 环境 ..."
if [[ ! -f "${ROS_SETUP}" ]]; then
    log_error "未找到 ROS 2 setup.bash: ${ROS_SETUP}"
    exit 1
fi
if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    log_error "未找到工作空间 install/setup.bash: ${WORKSPACE_SETUP}"
    log_error "请先构建工作空间: colcon build"
    exit 1
fi

# ROS setup 脚本会引用未定义变量（如 AMENT_TRACE_SETUP_FILES），临时关闭 -u
set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1091
source "${WORKSPACE_SETUP}"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
    log_error "ros2 命令不可用，请检查 ROS 2 环境"
    exit 1
fi

if [[ ! -x "${LIO_NODE_EXEC}" ]]; then
    log_error "lio_node 可执行文件不存在: ${LIO_NODE_EXEC}"
    log_error "请先构建 fastlio2 包: colcon build --packages-select fastlio2"
    exit 1
fi
if [[ ! -x "${PGO_NODE_EXEC}" ]]; then
    log_error "pgo_node 可执行文件不存在: ${PGO_NODE_EXEC}"
    log_error "请先构建 pgo 包: colcon build --packages-select pgo"
    exit 1
fi
if [[ ! -f "${LIO_CONFIG}" ]]; then
    log_error "未找到 lio 配置文件: ${LIO_CONFIG}"
    exit 1
fi
if [[ ! -f "${PGO_CONFIG}" ]]; then
    log_error "未找到 pgo 配置文件: ${PGO_CONFIG}"
    exit 1
fi
if ! ros2 interface show "${SAVE_MAP_SRV_TYPE}" >/dev/null 2>&1; then
    log_error "未找到服务类型 ${SAVE_MAP_SRV_TYPE}, 请先构建 interface 包"
    exit 1
fi

LIVOX_LAUNCH_FILE_PATH="${WORKSPACE_DIR}/install/${LIVOX_LAUNCH_PKG}/share/${LIVOX_LAUNCH_PKG}/launch_ROS2/${LIVOX_LAUNCH_FILE}"
if [[ ! -f "${LIVOX_LAUNCH_FILE_PATH}" ]]; then
    log_error "未找到 livox 驱动 launch 文件: ${LIVOX_LAUNCH_FILE_PATH}"
    log_error "请先构建 livox_ros_driver2 包: colcon build --packages-select livox_ros_driver2"
    exit 1
fi

log_ok "环境与依赖检查通过"

# 将 ROS 日志重定向到可写临时目录，避免 ~/.ros/log 权限问题
ROS_LOG_DIR="$(mktemp -d)"
export ROS_LOG_DIR
export RCUTILS_LOGGING_USE_STDOUT=1

# ==================== 清理残留进程 ====================
for prog in '[l]ivox_ros_driver2_node' '[l]io_node' '[p]go_node'; do
    if pgrep -f "${prog}" >/dev/null 2>&1; then
        log_warn "发现残留进程, 正在清理: ${prog}"
        pkill -f "${prog}" 2>/dev/null || true
    fi
done
sleep 1

# ==================== 启动建图节点 ====================
LIVOX_LOG="$(mktemp --suffix=.log)"
LIO_LOG="$(mktemp --suffix=.log)"
PGO_LOG="$(mktemp --suffix=.log)"
ECHO_LOG="$(mktemp --suffix=.log)"

dump_log_tail() {
    local log_file="$1" name="$2"
    log_error "${name} 日志末尾:"
    tail -n 20 "${log_file}" 2>/dev/null | sed 's/^/    /' >&2 || true
}

check_nodes_alive() {
    # 尚未启动的进程 (PID 为空) 跳过检查
    if [[ -n "${LIVOX_PID}" ]] && ! kill -0 "${LIVOX_PID}" 2>/dev/null; then
        log_error "livox 驱动异常退出"
        dump_log_tail "${LIVOX_LOG}" "livox 驱动"
        exit 1
    fi
    if [[ -n "${LIO_PID}" ]] && ! kill -0 "${LIO_PID}" 2>/dev/null; then
        log_error "lio_node 异常退出"
        dump_log_tail "${LIO_LOG}" "lio_node"
        exit 1
    fi
    if [[ -n "${PGO_PID}" ]] && ! kill -0 "${PGO_PID}" 2>/dev/null; then
        log_error "pgo_node 异常退出"
        dump_log_tail "${PGO_LOG}" "pgo_node"
        exit 1
    fi
}

# 等待话题出现并接收到首条数据
# 注意: ros2 topic echo 在话题不存在时会立即报错退出("Could not determine
# the type")而不会等待, 因此必须先轮询话题出现, 再用 echo 等待首条消息
# 用法: wait_for_topic_data <topic> <timeout_s>
wait_for_topic_data() {
    local topic="$1" timeout_s="$2"
    local elapsed=0 found=0

    # 阶段1: 等待话题出现 (livox 驱动收到首帧数据后才创建话题)
    while [[ ${elapsed} -lt ${timeout_s} ]]; do
        check_nodes_alive
        if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
            found=1
            break
        fi
        printf "  ... 等待话题 %s 出现 (%ds)\r" "${topic}" "${elapsed}"
        sleep 1 || true
        elapsed=$((elapsed + 1))
    done
    echo ""
    if [[ ${found} -ne 1 ]]; then
        log_error "话题 ${topic} 在 ${timeout_s}s 内未出现"
        return 1
    fi

    # 阶段2: 等待首条数据
    if ! timeout "${timeout_s}" ros2 topic echo --once "${topic}" >/dev/null 2>"${ECHO_LOG}"; then
        log_error "话题 ${topic} 已存在, 但未能收到首条数据"
        log_error "ros2 topic echo 输出:"
        sed 's/^/    /' "${ECHO_LOG}" >&2 || true
        return 1
    fi
    return 0
}

# ---------- 1. 启动 livox 雷达驱动 ----------
log_info "启动 livox 雷达驱动 (${LIVOX_LAUNCH_PKG} ${LIVOX_LAUNCH_FILE}) ..."

# ros2 launch 收到 SIGTERM 会向其子节点转发关闭, cleanup 可正常回收
ros2 launch "${LIVOX_LAUNCH_PKG}" "${LIVOX_LAUNCH_FILE}" \
    > "${LIVOX_LOG}" 2>&1 &
LIVOX_PID=$!
log_ok "livox 驱动已启动 (PID: ${LIVOX_PID})"

sleep "${NODE_STARTUP_CHECK_DELAY}"
check_nodes_alive

log_info "等待雷达点云数据 (${LIDAR_TOPIC}) ..."
if ! wait_for_topic_data "${LIDAR_TOPIC}" "${LIDAR_READY_TIMEOUT}"; then
    log_error "请检查 MID360 雷达连接与 MID360s_config.json 中的 IP 配置"
    dump_log_tail "${LIVOX_LOG}" "livox 驱动"
    exit 1
fi
log_ok "雷达数据接收正常"

# ---------- 2. 启动建图节点 ----------
log_info "启动建图节点 ..."

# 直接执行节点二进制（而非 ros2 launch），确保 PID 就是节点进程本身，
# cleanup 时 kill 才能真正终止节点，避免孤儿进程
"${LIO_NODE_EXEC}" --ros-args \
    -r __ns:=/fastlio2 \
    -p config_path:="${LIO_CONFIG}" \
    > "${LIO_LOG}" 2>&1 &
LIO_PID=$!

"${PGO_NODE_EXEC}" --ros-args \
    -p config_path:="${PGO_CONFIG}" \
    > "${PGO_LOG}" 2>&1 &
PGO_PID=$!

log_ok "lio_node 已启动 (PID: ${LIO_PID})"
log_ok "pgo_node 已启动 (PID: ${PGO_PID})"

# 启动自检: 短暂等待后确认节点仍在运行
sleep "${NODE_STARTUP_CHECK_DELAY}"
check_nodes_alive
log_ok "建图节点运行正常"

# ==================== 等待建图服务就绪 ====================
log_info "等待 ${SAVE_MAP_SRV} 服务就绪 ..."
elapsed=0
ready=0
while [[ ${elapsed} -lt ${SERVICE_READY_TIMEOUT} ]]; do
    check_nodes_alive
    if ros2 service list 2>/dev/null | grep -qx "${SAVE_MAP_SRV}"; then
        ready=1
        break
    fi
    printf "  ... 等待服务 (%ds)\r" "${elapsed}"
    sleep 1 || true
    elapsed=$((elapsed + 1))
done
echo ""

if [[ ${ready} -ne 1 ]]; then
    log_error "建图服务在 ${SERVICE_READY_TIMEOUT}s 内未就绪"
    exit 1
fi
log_ok "建图服务已就绪"

# ==================== 等待激光里程计数据 ====================
# 服务就绪仅代表 pgo_node 已构造, 并不代表传感器数据在流动;
# 若无数据, 建图结束后保存会因 "NO POSES!" 失败, 因此先验证数据流
log_info "等待激光里程计数据 (/fastlio2/lio_odom) ..."
if ! wait_for_topic_data "/fastlio2/lio_odom" "${DATA_READY_TIMEOUT}"; then
    log_error "雷达数据正常但 lio_node 无输出, 请检查 IMU 初始化 (/livox/imu) 与 lio.yaml 配置"
    dump_log_tail "${LIO_LOG}" "lio_node"
    exit 1
fi
log_ok "已接收到激光里程计数据"

# ==================== 建图倒计时 ====================
log_info "开始建图, 总时长 ${DURATION}s (Ctrl+C 可提前结束并保存) ..."

remaining=${DURATION}
while [[ ${remaining} -gt 0 ]]; do
    if [[ ${INTERRUPTED} -eq 1 ]]; then
        echo ""
        log_warn "收到中断信号, 提前结束建图 ..."
        break
    fi
    check_nodes_alive
    if [[ ${IS_TTY} -eq 1 ]]; then
        printf "  建图中 ... 剩余时间: %4ds  \r" "${remaining}"
    elif (( remaining % 30 == 0 )); then
        log_info "建图中 ... 剩余时间: ${remaining}s"
    fi
    sleep 1 || true
    remaining=$((remaining - 1))
done
echo ""

# ==================== 保存点云地图 ====================
PCD_FILE="${PCD_DIR}/map.pcd"
SAVER_LOG="$(mktemp --suffix=.log)"

save_map() {
    log_info "调用 ${SAVE_MAP_SRV} 保存点云地图到: ${PCD_DIR}"
    if ! timeout "${SAVE_MAP_TIMEOUT}" ros2 service call \
        "${SAVE_MAP_SRV}" "${SAVE_MAP_SRV_TYPE}" \
        "{file_path: '${PCD_DIR}', save_patches: false}" \
        > "${SAVER_LOG}" 2>&1; then
        rc=$?
        log_error "保存地图服务调用失败 (退出码 ${rc})"
        cat "${SAVER_LOG}" >&2 || true
        return 1
    fi
    # 兼容两种响应格式: "success: true" (yaml) 与 "success=True" (python repr)
    if ! grep -Eqi 'success[=:-][[:space:]]*true' "${SAVER_LOG}"; then
        log_error "保存地图服务返回失败"
        if grep -qi "NO POSES" "${SAVER_LOG}"; then
            log_error "pgo 未积累任何位姿: 建图期间未收到有效传感器数据"
            log_error "请检查雷达/IMU 数据流 (/livox/lidar, /livox/imu) 是否正常"
        fi
        cat "${SAVER_LOG}" >&2 || true
        return 1
    fi
    return 0
}

# 保存期间屏蔽中断信号, 防止重复 Ctrl+C 破坏保存流程
trap '' INT TERM
if save_map; then
    MAP_SAVED=1
fi
trap on_interrupt INT TERM

if [[ ${MAP_SAVED} -ne 1 ]]; then
    exit 1
fi
log_ok "点云地图保存成功"

# ==================== 验证输出文件 ====================
log_info "验证输出文件 ..."

if [[ ! -f "${PCD_FILE}" ]]; then
    log_error "未生成点云文件: ${PCD_FILE}"
    exit 1
fi
if [[ ! -s "${PCD_FILE}" ]]; then
    log_error "点云文件为空: ${PCD_FILE}"
    exit 1
fi

PCD_SIZE=$(stat -c%s "${PCD_FILE}")

# ==================== 完成 ====================
log_ok "建图完成!"
echo ""
echo -e "${C_GREEN}========================================${C_RESET}"
echo -e "${C_GREEN} 建图完成${C_RESET}"
echo -e "${C_GREEN}========================================${C_RESET}"
echo -e " 地图名称: ${MAP_NAME}"
echo -e " 点云文件: ${PCD_FILE} (${PCD_SIZE} bytes)"
echo -e " PGM 目录: ${PGM_DIR} (可运行 pcd2pgm.sh 生成栅格地图)"
echo -e "${C_GREEN}========================================${C_RESET}"

exit 0
