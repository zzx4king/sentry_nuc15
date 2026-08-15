# sentry_nuc15 — 哨兵机器人 NUC15 上位机

配合 `sentry_mcu` 使用的 RoboMaster 哨兵机器人上位机程序，运行于 NUC15。基于 Livox MID360s 激光雷达与 FAST-LIO2 激光惯性里程计，实现哨兵机器人的**激光建图**，并预留重定位与 Nav2 自主导航能力。

## 1. 项目简介

本项目为一套 ROS 2 colcon 工作空间，核心链路为：

```
Livox MID360 驱动 → fastlio2 激光惯性里程计 → pgo 位姿图优化 → 点云地图(map.pcd) → pcd2pgm 栅格地图(map.pgm)
```

通过一键脚本建图，产出可供 Nav2 使用的点云地图与 2D 栅格地图。

## 2. 运行环境

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 24.04.4 LTS (noble) |
| ROS 2 | Jazzy |
| 编译器 | GCC 13.3.0 |
| CMake | 3.28.3 |
| PCL | 1.14.0（apt） |
| Eigen | 3.4.0（apt） |
| Boost | 1.83.0 |
| Livox-SDK2 | 源码编译安装 |
| 激光雷达 | Livox MID360s（lidar_ip: 192.168.1.117, host_ip: 192.168.1.50） |

## 3. 目录结构与功能包说明

共包含 **9 个功能包**、**2 个一键脚本**，约 226 个源文件。

```
sentry_ws/
├── src/                          # 功能包
│   ├── FASTLIO2_ROS2/            # 建图定位链路（5 个包）
│   │   ├── fastlio2/             # 激光惯性里程计：实时里程计与局部建图（ikd-Tree + iEKF）
│   │   ├── hba/                  # 层次束调整（Hierarchical Bundle Adjustment）：回环检测与图优化
│   │   ├── pgo/                  # 位姿图优化：回环后全局优化，提供 /pgo/save_maps 保存点云地图
│   │   ├── localizer/            # 基于 ICP 的重定位（未完成）
│   │   └── interface/            # 自定义服务接口（SaveMaps / Relocalize / IsValid / RefineMap / SavePoses）
│   ├── livox_ros_driver2/        # Livox 雷达驱动（含 MID360s 配置与发布线程本地修改）
│   ├── livox_to_laserscan/       # 点云转 LaserScan（供 Nav2 避障用，含本地修改）
│   ├── pcd2pgm/                  # PCD 点云地图转 PGM 栅格地图（含本地修改）
│   └── sentry_nav2_bringup/      # Nav2 导航启动包（launch + nav2_params.yaml，未完成）
├── scipts/                       # 一键脚本
│   ├── build_map.sh              # 建图：启动驱动+lio+pgo，计时结束后自动保存 map.pcd
│   └── pcd2pgm.sh                # 地图转换：PCD → PGM（调用 map_saver_cli 保存）
├── maps/                         # 建图结果（pcd 点云 + pgm 栅格）
│   ├── 207/                      # 207 场地地图
│   ├── floor/                    # 楼层建图
│   └── test/                     # 测试地图
└── doc/
    └── fit.md                    # FASTLIO2_ROS2 适配 ROS 2 Jazzy 的完整记录
```

## 4. 项目完成度

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 激光建图 | ✅ 已完成 | `build_map.sh` 一键建图，已在 207 / floor 场地验证 |
| 地图转换（PCD→PGM） | ✅ 已完成 | `pcd2pgm.sh` 一键转换 |
| 重定位（localizer/hba） | ⛔ 未完成 | 代码已就位，未调试 |
| Nav2 自主导航（sentry_nav2_bringup） | ⛔ 未完成 | 仅搭好启动包框架，未联调 |

> **当前仅完成建图流程**，定位与导航部分待后续开发。

## 5. 借鉴的开源仓库

| 仓库 | 用途 |
| --- | --- |
| [liangheming/FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) | fastlio2 / hba / pgo / localizer / interface 建图定位链路 |
| [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) | Livox 雷达 ROS 2 驱动 |
| [Livox-SDK/Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) | Livox 雷达底层 SDK（驱动依赖） |
| [is-buiquocdoanh/livox_to_laserscan](https://github.com/is-buiquocdoanh/livox_to_laserscan) | 点云转 LaserScan |
| [LihanChen2004/pcd2pgm](https://github.com/LihanChen2004/pcd2pgm) | PCD 转 PGM 栅格地图 |

## 6. 依赖安装

### 6.1 ROS 2 Jazzy

参考[官方文档](https://docs.ros.org/en/jazzy/Installation.html)安装 `ros-jazzy-desktop`。

### 6.2 Livox-SDK2（源码编译）

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake .. && make -j
sudo make install
```

### 6.3 apt 依赖

```bash
sudo apt install \
    libpcl-dev \
    libeigen3-dev \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-message-filters \
    ros-jazzy-visualization-msgs
```

### 6.4 GTSAM 与 Sophus（hba / pgo / fastlio2 需要，源码编译）

```bash
# GTSAM
git clone --branch 4.2.0 https://github.com/borglab/gtsam.git
cd gtsam && mkdir build && cd build
cmake -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_WITH_TBB=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON ..
make -j && sudo make install

# Sophus
git clone https://github.com/strasdat/Sophus.git
cd Sophus && mkdir build && cd build
cmake -DBUILD_SOPHUS_TESTS=OFF ..
make -j && sudo make install
```

> 详细踩坑记录见 [doc/fit.md](doc/fit.md)。

### 6.5 构建工作空间

```bash
cd sentry_ws
colcon build --symlink-install
source install/setup.bash
```

### 6.6 快速使用

```bash
# 一键建图（名称 + 时长秒数）
./scipts/build_map.sh test 120

# 点云地图转栅格地图
./scipts/pcd2pgm.sh maps/test/pcd/map.pcd
```
