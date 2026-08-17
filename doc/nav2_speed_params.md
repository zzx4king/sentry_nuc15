# Nav2 导航速度参数说明与修改指南

本文说明本项目 `sentry_nav2_bringup` 导航栈中各项速度参数的**含义、当前值、修改位置和修改方法**，适用于单点导航测试中的巡航速度、到点速度、角速度、加减速度等调整。

## 1. 速度指令链路

```text
controller_server (DWB)
        │  输出: /cmd_vel_nav
        ▼
velocity_smoother (速度平滑)
        │  输出: /cmd_vel_smoothed
        ├──────────────► sentry_control (底盘实际执行，当前订阅此话题)
        ▼
collision_monitor (碰撞监控)
        │  输出: /cmd_vel
        ▼
     其他底盘节点（当前 sentry_control 未订阅此话题，见文末注意事项）
```

- 当前速度来源：`/fastlio2/lio_odom`
  - `controller_server.odom_topic = /fastlio2/lio_odom`
  - `velocity_smoother.odom_topic = /fastlio2/lio_odom`
- 到达目标点判定：`controller_server` 中的 `general_goal_checker`，当前使用 `StoppedGoalChecker`，同时检查位置、朝向和实际速度。

## 2. 配置文件位置

| 文件 | 作用 |
|---|---|
| `src/sentry_nav2_bringup/config/nav2_params.yaml` | Nav2 全部速度参数 |
| `src/sentry_control/config/control.yaml` | 底盘串口执行层的速度话题、角速度缩放、发送频率 |
| `install/sentry_nav2_bringup/share/sentry_nav2_bringup/config/nav2_params.yaml` | 实际 launch 读取的配置，需通过 `colcon build` 同步 |

## 3. 当前速度参数一览

### 3.1 DWB 本地规划器速度限制

节点：`controller_server`，配置节：`FollowPath`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `FollowPath.max_vel_x` | `0.52` m/s | 最大前进线速度（巡航速度上限） |
| `FollowPath.min_vel_x` | `0.0` m/s | 最小 X 线速度；当前为 0，表示 DWB 不生成倒车轨迹 |
| `FollowPath.max_vel_y` | `0.0` m/s | 最大 Y 线速度，差速底盘必须为 0 |
| `FollowPath.min_vel_y` | `0.0` m/s | 最小 Y 线速度 |
| `FollowPath.max_speed_xy` | `0.52` m/s | 平面合成速度上限 `sqrt(vx^2 + vy^2)` |
| `FollowPath.min_speed_xy` | `0.0` m/s | 平面合成速度下限 |
| `FollowPath.max_vel_theta` | `1.0` rad/s | 最大角速度 |
| `FollowPath.min_speed_theta` | `0.0` rad/s | 最小角速度阈值 |

### 3.2 DWB 加速度 / 减速度限制

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `FollowPath.acc_lim_x` | `2.5` m/s² | 最大前进加速度 |
| `FollowPath.decel_lim_x` | `-2.5` m/s² | 最大前进减速度（Nav2 约定负值） |
| `FollowPath.acc_lim_y` | `0.0` | Y 方向加速度 |
| `FollowPath.decel_lim_y` | `0.0` | Y 方向减速度 |
| `FollowPath.acc_lim_theta` | `3.2` rad/s² | 最大角加速度 |
| `FollowPath.decel_lim_theta` | `-3.2` rad/s² | 最大角减速度（负值） |

### 3.3 到达目标点速度判定

节点：`controller_server`，配置节：`general_goal_checker`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `general_goal_checker.trans_stopped_velocity` | `0.05` m/s | 到达时允许的最大线速度 |
| `general_goal_checker.rot_stopped_velocity` | `0.15` rad/s | 到达时允许的最大角速度 |
| `general_goal_checker.xy_goal_tolerance` | `0.25` m | 到达判定的位置容差 |
| `general_goal_checker.yaw_goal_tolerance` | `0.25` rad | 到达判定的朝向容差 |

当前 `plugin: "nav2_controller::StoppedGoalChecker"`，只有位姿进入容差且实际速度低于上述速度阈值，导航才会判定到达。

### 3.4 DWB 近目标停车切换参数

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `FollowPath.xy_goal_tolerance` | `0.25` m | 进入目标窗口的距离 |
| `FollowPath.trans_stopped_velocity` | `0.05` m/s | 进入目标窗口后，线速度低于该值才切换为原地转向对准 |
| `FollowPath.RotateToGoal.slowing_factor` | `5.0` | 目标窗口内减速权重，越大减速越积极 |
| `FollowPath.RotateToGoal.lookahead_time` | `-1.0` | 使用轨迹末端姿态评估朝向误差 |

### 3.5 速度平滑器

节点：`velocity_smoother`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `max_velocity` | `[0.52, 0.0, 1.0]` | 最终输出上限：`[vx, vy, wz]` |
| `min_velocity` | `[-0.52, 0.0, -1.0]` | 最终输出下限：`[vx, vy, wz]` |
| `max_accel` | `[2.5, 0.0, 3.2]` | 平滑器允许的最大加速度 |
| `max_decel` | `[-2.5, 0.0, -3.2]` | 平滑器允许的最大减速度（负值） |
| `deadband_velocity` | `[0.0, 0.0, 0.0]` | 速度死区 |
| `velocity_timeout` | `1.0` s | 超过该时间未收到新指令则输出零速 |
| `smoothing_frequency` | `20.0` Hz | 平滑器运行频率 |

### 3.6 行为恢复 / 原地旋转速度

节点：`behavior_server`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `max_rotational_vel` | `1.0` rad/s | 恢复行为旋转动作最大角速度 |
| `min_rotational_vel` | `0.4` rad/s | 恢复行为旋转动作最小角速度 |
| `rotational_acc_lim` | `3.2` rad/s² | 恢复行为旋转角加速度限制 |

### 3.7 碰撞监控减速 / 停车

节点：`collision_monitor`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `PolygonStop.action_type` | `stop` | 0.45 m 停车区内输出零速 |
| `PolygonSlow.action_type` | `slowdown` | 0.75 m 减速区内减速 |
| `PolygonSlow.slowdown_ratio` | `0.3` | 减速区内速度乘以 0.3 |

### 3.8 底盘执行层

配置文件：`src/sentry_control/config/control.yaml`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `cmd_vel_topic` | `/cmd_vel_smoothed` | 底盘实际订阅的速度话题 |
| `angular_divisor` | `1.42` | 下发 MCU 前角速度除以该系数 |
| `send_rate` | `50.0` Hz | 速度指令发送频率 |
| `cmd_vel_timeout` | `0.5` s | 超时未收到速度指令则自动发零速 |

## 4. 修改方法

### 4.1 修改源文件

修改：

```bash
src/sentry_nav2_bringup/config/nav2_params.yaml
```

必要时也修改：

```bash
src/sentry_control/config/control.yaml
```

### 4.2 重新构建并同步 install

```bash
cd /home/robomaster/project/sentry_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select sentry_nav2_bringup
```

如果只改了底盘层：

```bash
colcon build --packages-select sentry_control
```

> launch 读取的是 `install/` 下的配置，只改 `src/` 不构建不会生效。

### 4.3 重启导航

```bash
./scipts/nav_start.sh
```

或单独启动 Nav2：

```bash
ros2 launch sentry_nav2_bringup navigation2.launch.py
```

### 4.4 运行中查看参数

```bash
# DWB 巡航线速度
ros2 param get /controller_server FollowPath.max_vel_x

# DWB 平面合成速度
ros2 param get /controller_server FollowPath.max_speed_xy

# 到达目标点线速度阈值
ros2 param get /controller_server general_goal_checker.trans_stopped_velocity

# 到达目标点角速度阈值
ros2 param get /controller_server general_goal_checker.rot_stopped_velocity

# 平滑器速度上限
ros2 param get /velocity_smoother max_velocity

# 实时速度指令
ros2 topic echo /cmd_vel_smoothed

# 实时里程计速度
ros2 topic echo /fastlio2/lio_odom --field twist.twist
```

## 5. 常见修改示例

### 5.1 修改巡航线速度

例如当前 `0.52 m/s`，想改为 `0.80 m/s`，需要同时改四处，保持 DWB 与平滑器一致：

```yaml
FollowPath:
  max_vel_x: 0.80
  max_speed_xy: 0.80

velocity_smoother:
  ros__parameters:
    max_velocity: [0.80, 0.0, 1.0]
    min_velocity: [-0.80, 0.0, -1.0]
```

注意：
- `max_speed_xy` 不要小于 `max_vel_x`。
- `velocity_smoother.max_velocity` 与 `FollowPath.max_vel_x` 建议保持相同。
- 如需倒车，还要把 `FollowPath.min_vel_x` 设为负值。

### 5.2 修改最大角速度

```yaml
FollowPath:
  max_vel_theta: 1.0

velocity_smoother:
  ros__parameters:
    max_velocity: [0.52, 0.0, 1.0]
    min_velocity: [-0.52, 0.0, -1.0]
```

角速度在 DWB 中为 `max_vel_theta`，在平滑器数组中为第三个元素。

### 5.3 修改加减速度

```yaml
FollowPath:
  acc_lim_x: 2.5
  decel_lim_x: -2.5
  acc_lim_theta: 3.2
  decel_lim_theta: -3.2

velocity_smoother:
  ros__parameters:
    max_accel: [2.5, 0.0, 3.2]
    max_decel: [-2.5, 0.0, -3.2]
```

- 数值越大，起停越猛。
- Nav2 减速度参数通常写负值，表示减速能力。

### 5.4 修改到达目标点速度

想让到点更慢更稳，可同时减小以下两个位置的值：

```yaml
general_goal_checker:
  plugin: "nav2_controller::StoppedGoalChecker"
  trans_stopped_velocity: 0.05
  rot_stopped_velocity: 0.15

FollowPath:
  trans_stopped_velocity: 0.05
```

当前含义：
- `general_goal_checker.trans_stopped_velocity`：实际速度必须低于该值才判定到达。
- `FollowPath.trans_stopped_velocity`：DWB 低于该线速度后停止前进，转为原地转向对准。

如果到点后 yaw 对准太慢，可适当放宽 `rot_stopped_velocity`，例如 `0.20 ~ 0.25`。

### 5.5 修改恢复行为旋转速度

```yaml
behavior_server:
  ros__parameters:
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

### 5.6 修改碰撞减速比例

```yaml
collision_monitor:
  ros__parameters:
    PolygonSlow:
      action_type: "slowdown"
      slowdown_ratio: 0.3   # 改为 0.5 表示减速区速度乘以 0.5
```

### 5.7 修改底盘层角速度缩放与发送频率

`src/sentry_control/config/control.yaml`：

```yaml
sentry_control:
  ros__parameters:
    angular_divisor: 1.42
    send_rate: 50.0
    cmd_vel_timeout: 0.5
```

- 实际下发 MCU 的角速度 = `cmd_vel.angular.z / angular_divisor`。
- 不要随意降低 `send_rate`，MCU 有 500 ms 超时约束，当前 50 Hz 对应 20 ms 周期。

## 6. 注意事项

1. **DWB 与 velocity_smoother 要一起改**。只改 `max_vel_x` 不改 `velocity_smoother.max_velocity` 时，平滑器会继续把速度限制在旧值。
2. **`src` 修改后必须重新构建**，否则 `install/` 中仍是旧配置。
3. **当前底盘订阅 `/cmd_vel_smoothed`**，而 `collision_monitor` 输出的是 `/cmd_vel`。因此当前 `PolygonStop` / `PolygonSlow` 的减速参数不作用于底盘执行链路。如果需要碰撞减速生效，应把 `src/sentry_control/config/control.yaml` 中的 `cmd_vel_topic` 改为 `/cmd_vel`。
4. **到达速度判定依赖里程计**。`controller_server.odom_topic` 必须为 `/fastlio2/lio_odom`，否则 `StoppedGoalChecker` 拿不到真实速度。
5. **实际底盘速度以 MCU/里程计反馈为准**，Nav2 参数只是上位机指令限制。修改后建议先在空旷区域验证加速、刹车距离、到点停车和 yaw 对准。
