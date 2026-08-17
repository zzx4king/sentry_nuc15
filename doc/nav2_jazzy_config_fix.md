# Nav2 路径规划失效修复记录（Jazzy 兼容性）

- 日期：2026-08-17
- 现象：`./scipts/nav_start.sh` 启动后，RViz 中通过 Nav2_Goal 发布目标点无法生成全局路径；膨胀区域（Inflation Layer）不显示，疑似机器人半径/膨胀参数未加载
- 环境：ROS 2 Jazzy + Nav2（Jazzy 版）+ fastlio2/localizer 定位 + Livox Mid360s 雷达
- 修改文件：`src/sentry_nav2_bringup/config/nav2_params.yaml`（共 4 类修改，均已 `colcon build` 同步至 install）

---

## 1. 排查思路

症状是"膨胀层/机器人半径未加载"，但最终证明这些参数**一直是正确的**，真正的问题是
**lifecycle_manager 的 bringup 链中途失败，整个导航栈从未激活**（bt_navigator 不 active →
NavigateToPose action server 不存在 → 发目标无响应；costmap 不 active → 无膨胀层显示）。

关键排查手段：

1. 查看节点日志 `~/.ros/log/*.log`（每个 lifecycle 节点一个文件，configure/activate 失败会留下明确报错）
2. 确认 `install/` 与 `src/` 的配置一致（launch 从 install 共享目录加载参数）
3. 阅读 Jazzy 实际安装的 `navigation_launch.py`，确认 lifecycle_manager 托管节点列表
4. 用独立 `ROS_DOMAIN_ID` + 静态 TF 做离线全链路验证，不干扰实车

## 2. 根因链（按 lifecycle 顺序逐环暴露）

Jazzy 版 `navigation_launch.py` 托管 **10 个节点**（比旧版多出 `route_server`、
`collision_monitor`、`docking_server`）：

```
controller_server → smoother_server → planner_server → route_server → behavior_server
→ velocity_smoother → collision_monitor → bt_navigator → waypoint_follower → docking_server
```

链条顺序执行 configure + activate，**任何一环失败即中止**，其后所有节点（含 bt_navigator）
永远无法激活。本次连续修复了 4 个坑：

### 坑 1：planner 插件名旧格式（Humble 风格）

```
[FATAL] [planner_server]: Failed to create global planner. Exception: ... the class
nav2_navfn_planner/NavfnPlanner ... does not exist.
Declared types are nav2_navfn_planner::NavfnPlanner ...
```

修复：`/` 分隔 → `::` 分隔

```yaml
GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"   # 原 nav2_navfn_planner/NavfnPlanner
```

behavior_server 的 5 个插件同理：`nav2_behaviors::Spin` 等。

### 坑 2：collision_monitor 无配置节

```
[ERROR] [collision_monitor]: Error while getting parameters: parameter 'observation_sources' is not initialized
```

Jazzy 托管该节点但我们的 yaml 中没有对应节。新增配置时又连续踩了 4 个参数格式变化：

| 参数 | Humble 旧写法 | Jazzy 正确写法 |
|---|---|---|
| 数据源列表 | `observation_sources: "scan"`（字符串） | `observation_sources: ["scan"]`（**字符串数组**） |
| 多边形列表 | `polygon_names: [...]` | `polygons: [...]` |
| 多边形顶点 | `points: "0.45,0.45, -0.45,0.45, ..."` | `points: "[[0.45,0.45], [-0.45,0.45], ...]"`（**二维数组**） |
| 减速动作 | `action_type: "slow"` | `action_type: "slowdown"` |

最终新增配置节（STOP 区 0.45m / 减速区 0.75m，匹配 0.36m 机器人半径）：

```yaml
collision_monitor:
  ros__parameters:
    use_sim_time: False
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    # 速度链路: controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> 本节点 -> /cmd_vel
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    transform_tolerance: 0.2
    source_timeout: 5.0
    base_shift_correction: False
    polygons: ["PolygonStop", "PolygonSlow"]
    PolygonStop:
      type: "polygon"
      points: "[[0.45, 0.45], [-0.45, 0.45], [-0.45, -0.45], [0.45, -0.45]]"
      action_type: "stop"
      min_points: 4
      visualize: False
      polygon_pub_topic: "polygon_stop"
    PolygonSlow:
      type: "polygon"
      points: "[[0.75, 0.75], [-0.75, 0.75], [-0.75, -0.75], [0.75, -0.75]]"
      action_type: "slowdown"
      min_points: 4
      slowdown_ratio: 0.3
      visualize: False
      polygon_pub_topic: "polygon_slowdown"
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "/scan"
      min_height: 0.05
      max_height: 1.2
```

### 坑 3：docking_server 无配置节

```
[ERROR] [docking_server]: Charging dock plugins not given!
```

按 nav2_bringup 官方默认值新增 `docking_server` 节（含 `dock_plugins:
["simple_charging_dock"]` 与 `controller` 参数），仅用于让链条通过，见 nav2_params.yaml
中该节完整内容。

### 坑 4：bt_navigator 的 plugin_lib_names 与默认注册冲突

```
[FATAL] [bt_navigator]: Failed to create navigator id navigate_to_pose.
Exception: ID [ComputePathToPose] already registered
```

Jazzy 的 bt_navigator 默认已注册全套 BT 节点，显式提供旧版 `plugin_lib_names` 列表
会导致节点 ID 重复注册而 FATAL（并伴随段错误）。修复：**删除整个 plugin_lib_names 列表**。

## 3. 离线验证方法（不依赖雷达/实车）

在独立 domain 中用静态 TF 模拟定位链，启动全部节点 + lifecycle_manager：

```bash
export ROS_DOMAIN_ID=99
source install/setup.bash
P=install/sentry_nav2_bringup/share/sentry_nav2_bringup/config/nav2_params.yaml

# 静态 TF 模拟 map->odom->base_link (替代 fastlio2+localizer)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &

# 11 个节点 (map_server + navigation_launch.py 的 10 个)
ros2 run nav2_map_server map_server --ros-args --params-file $P \
  -p yaml_filename:=/home/robomaster/project/sentry_ws/maps/floor/pgm/map.yaml &
ros2 run nav2_controller controller_server --ros-args --params-file $P &
# ... smoother/planner/route/behavior/velocity_smoother/collision_monitor/
#     bt_navigator/waypoint_follower/opennav_docking 同理 &

ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p node_names:="[map_server,controller_server,smoother_server,planner_server,route_server,behavior_server,velocity_smoother,collision_monitor,bt_navigator,waypoint_follower,docking_server]" \
  -p autostart:=true &
```

验证结果：

```
Configuring map_server → ... → docking_server    （11 个全部配置成功）
Activating  map_server → ... → docking_server    （11 个全部激活，bond 全部连接）
[lifecycle_manager]: Managed nodes are active    ← 关键成功标志
```

端到端路径规划实测（起点/终点需在自由空间，见第 5 节注意事项）：

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{use_start: true, start: {...(-2.39,-7.7)...}, goal: {...(6.95,-29.4)...}}"
# 结果: error_code: 0, SUCCEEDED, 2958 个路径点(约 59 米)
```

costmap 采样确认膨胀层生效：`robot_radius=0.36 / inflation_radius=0.55`，
代价值分布完整（自由 0 / 膨胀渐变 57~96 / 致死 99~100）。

> 注意：costmap 激活要求 TF（map->odom->base_link）可用，否则激活挂起直至
> lifecycle_manager 60s 超时。离线测试必须先起静态 TF。

## 4. 历史遗留问题（本次一并确认）

- `use_sim_time` 必须全为 `False`：残留 `True` 会导致实车时钟冻结、生命周期激活失败（前一轮已修复）
- 重复节点：`nav_start.sh` 启动前已做残留进程清理（stop_all.sh）
- 同时运行两个 RViz（localizer 链 + navigation2.launch.py）时日志会出现
  `Publisher already registered for node name: 'rviz2'` 警告，注意在正确的 RViz 窗口观察 costmap

## 5. 使用注意事项

1. **目标点必须落在自由空间**：落在墙体/障碍膨胀区（inflation_radius=0.55m）内的目标会返回
   `NO_VALID_PATH(208)`，属于正常行为。地图信息：origin(-15.085, -29.518)，1272x2172 @0.02m。
   已验证可行的测试点对：(-2.39, -7.7) → (6.95, -29.4)
2. **目标点不能贴地图边界**：低于地图 origin 会返回 `GOAL_OUTSIDE_MAP(204)`
3. **底盘速度话题**：当前链路最终输出在 **`/cmd_vel`**（collision_monitor 的
   `cmd_vel_out_topic`）。若底盘订阅其他话题，需修改该参数
4. **改完 src 配置必须重新构建**：`colcon build --packages-select sentry_nav2_bringup`，
   否则 launch 仍读取 install 中的旧配置

## 6. 相关文件

| 文件 | 说明 |
|---|---|
| `src/sentry_nav2_bringup/config/nav2_params.yaml` | 本次全部修复所在 |
| `/opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py` | Jazzy 托管节点列表来源（只读参考） |
| `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` | 官方默认参数（docking_server 等参考来源） |
| `scipts/nav_start.sh` | 一键启动脚本（含残留进程清理） |
