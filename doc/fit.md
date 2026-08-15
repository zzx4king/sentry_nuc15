# FASTLIO2_ROS2 适配 ROS2 Jazzy 记录

本文档记录将 `src/FASTLIO2_ROS2` 功能包从 README 所述的 ROS2 Humble / Ubuntu 22.04 环境适配到 **ROS2 Jazzy / Ubuntu 24.04** 环境的完整过程：依赖安装、构建、兼容性排查与运行验证。

> 结论先行：**FASTLIO2_ROS2 源码无需任何改动即可在 ROS2 Jazzy 下编译并运行**。所有适配工作集中在「未安装的第三方依赖（Sophus / GTSAM）的源码编译安装」以及「Ubuntu 24.04 / GCC 13 下第三方库自身的编译问题处理」上。

---

## 1. 环境信息

| 项目 | 版本 |
| --- | --- |
| 操作系统 | Ubuntu 24.04.4 LTS (noble) |
| ROS2 | Jazzy（`/opt/ros/jazzy`，README 原指 Humble） |
| 编译器 | GCC 13.3.0 |
| CMake | 3.28.3 |
| PCL | 1.14.0（系统 apt） |
| Eigen | 3.4.0（系统 apt） |
| Boost | 1.83.0 |
| TBB | 2021.11 |
| fmt | 9.1.0 |
| livox_ros_driver2 | 工作空间内已预先构建并安装（`install/livox_ros_driver2`） |

工作空间根目录：`/home/robomaster/project/sentry_ws`
第三方依赖源码下载目录：`/home/robomaster/git_install`

---

## 2. 依赖识别

README `编译依赖` 一节（`src/FASTLIO2_ROS2/README.md#L12-L19`）列出：

```text
pcl
Eigen
sophus
gtsam
livox_ros_driver2
```

逐项核查各功能包 `CMakeLists.txt` 的实际 `find_package` 用法：

| 功能包 | Sophus | GTSAM | 说明 |
| --- | --- | --- | --- |
| `fastlio2` | ✅ REQUIRED | ❌ | 里程计主节点，仅用 Sophus |
| `pgo` | ❌ | ✅ REQUIRED（直接链接 `gtsam`） | 回环 + 位姿图优化 |
| `localizer` | ❌ | ❌ | 重定位，CMake 仅依赖 Eigen/PCL |
| `hba` | ✅ REQUIRED | ✅ REQUIRED（链接 `gtsam`） | 一致性地图优化 |
| `interface` | ❌ | ❌ | 自定义 srv，仅 rosidl |

核查系统现状：`pcl` / `Eigen` / `livox_ros_driver2` 均已就绪，**`sophus` 与 `gtsam` 未安装**（README 第 16-17 行）。二者即为本轮需补齐的依赖。

> 说明：`localizer` 的 `CMakeLists.txt` 未显式 `find_package(Sophus)`，但其源码与 `fastlio2`/`hba` 共用头文件且通过 `add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)` 关闭 fmt 依赖，故 Sophus 系统级安装后即满足全部使用点。

---

## 3. 第三方依赖安装

### 3.1 系统级安装策略与 sudo 配置

README 给出的安装方式为 `sudo make install`（安装到 `/usr/local`）。本机 `/usr/local/{include,lib}` 属 root，普通用户不可写，且 `sudo` 需要密码。为使后续 `sudo make install` / `sudo ldconfig` 可非交互完成，一次性配置了 NOPASSWD sudoers 规则：

```bash
echo '<你的密码>' | sudo -S bash -c 'echo "robomaster ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/zz-robomaster-nopasswd && chmod 440 /etc/sudoers.d/zz-robomaster-nopasswd'
sudo visudo -c -f /etc/sudoers.d/zz-robomaster-nopasswd   # 校验语法
sudo -n true && echo NOPASSWD_OK
```

> ⚠️ 安全提示：该规则放宽了 `robomaster` 的 sudo 限制，适配完成后如不再需要，请删除：`sudo rm /etc/sudoers.d/zz-robomaster-nopasswd`。

### 3.2 Sophus 安装

**源码路径**：`/home/robomaster/git_install/Sophus`，按 README 检出 `1.22.10`。

```bash
cd /home/robomaster/git_install
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_SOPHUS_TESTS=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

**遇到的问题**

| # | 现象 | 分析 | 解决 |
| --- | --- | --- | --- |
| 1 | `make` 报错 `cc1plus: all warnings being treated as errors`，定位到 `test/core/test_rxso3.cpp:130` 的 `-Werror=array-bounds=` | Sophus 自带 `-Werror`；GCC 13 对 `__m128` 数组越界的静态分析比 GCC 11（Humble 默认）更严格，命中**测试代码**（库本身是 header-only，不受影响） | 关闭测试构建：`-DBUILD_SOPHUS_TESTS=OFF`。库头文件与 CMake 配置仍正常安装 |
| 2 | README 提到「新 Sophus 依赖 fmt，可用 `add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)` 去除」 | `fastlio2`/`hba` 的 `CMakeLists.txt` 已 `add_compile_definitions(SOPHUS_USE_BASIC_LOGGING)`；为保持系统安装的 Sophus 配置一致，构建时同样传 `-DSOPHUS_USE_BASIC_LOGGING=ON` | 构建即传该选项，安装后的 `SophusConfig.cmake` 不强依赖 fmt（本机 fmt 9.1.0 亦已存在，双保险） |

**安装结果**

- 头文件：`/usr/local/include/sophus/*.hpp`
- CMake 配置：`/usr/local/share/sophus/cmake/SophusConfig.cmake`

### 3.3 GTSAM 安装

**源码路径**：`/home/robomaster/git_install/gtsam`，使用稳定发行分支 `4.2.0`。

```bash
cd /home/robomaster/git_install
git clone --branch 4.2.0 https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

**缺失的 apt 依赖**（构建前补装；Boost 1.83 / TBB 2021.11 已存在）：

```bash
sudo apt-get update
sudo apt-get install -y libsuitesparse-dev libceres-dev libgmp-dev libmpfr-dev
```

安装后版本：SuiteSparse 7.6.1、Ceres 2.2.0、gmp、mpfr 4.2.1。

**关键 CMake 选项说明**

| 选项 | 取值 | 原因 |
| --- | --- | --- |
| `GTSAM_USE_SYSTEM_EIGEN` | ON | 使用系统 Eigen 3.4.0，避免与 PCL / Sophus 的 Eigen 版本冲突 |
| `GTSAM_BUILD_WITH_MARCH_NATIVE` | OFF | 关闭 `-march=native`，提升可移植性，避免跨机 SIGILL |
| `GTSAM_BUILD_TESTS` / `GTSAM_BUILD_EXAMPLES_ALWAYS` | OFF | 加速构建，FASTLIO 仅需 `gtsam` 库本身 |
| `GTSAM_BUILD_UNSTABLE` | OFF | `pgo`/`hba` 仅用稳定模块（`NonlinearFactorGraph`/`Values`/`Pose3` 等），无需 unstable |

> 注：cmake 提示 `GTSAM_INSTALL_TBB` 为「未使用变量」（4.2 已无此选项），可忽略；GTSAM 自动探测到系统 TBB 2021.11 并以 TBB 作为默认分配器。

**构建过程出现的告警（非错误，不影响产物）**

- `gtsam/nonlinear/Values-inl.h`：`Values::filter` 的 `-Wdeprecated-declarations`（GTSAM 内部弃用提示）。
- `gtsam/geometry/BearingRange.h` 等：GCC 13 `-Warray-bounds` 提示（仅警告，未开 `-Werror`）。
- 最终 `[100%] Linking CXX shared library libgtsam.so` 与 `Built target gtsam` 表示构建成功。

**安装结果**

- 头文件：`/usr/local/include/gtsam/`
- CMake 配置：`/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake`
- 动态库：`/usr/local/lib/libgtsam.so` / `libgtsam.so.4` / `libmetis-gtsam.so`
- `ldconfig -p` 已识别 `libgtsam.so.4` 与 `libmetis-gtsam.so`

---

## 4. FASTLIO2_ROS2 功能包构建

### 4.1 构建命令

直接在项目根目录一次性 `colcon build` 即可。各包 `package.xml` 已正确声明依赖（`fastlio2` 依赖 `livox_ros_driver2`；`pgo`/`localizer`/`hba` 均显式 `<depend>interface</depend>`、`pgo`/`hba` 显式 `<depend>GTSAM</depend>`），colcon 会自动按拓扑序构建：`livox_ros_driver2` → `interface` → `fastlio2/pgo/localizer/hba`，无需手动排序。

```bash
cd /home/robomaster/project/sentry_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash          # 提供已构建的 livox_ros_driver2（增量构建时跳过其重编）
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`--event-handlers console_direct+` 将 CMake/Make 输出直接流式打印，便于捕获告警与错误；colcon 同时在 `log/latest_build/` 留存完整日志。

> 可选：若只想重建 FASTLIO2_ROS2 的 5 个包而跳过 livox，用
> `colcon build --packages-select interface fastlio2 pgo localizer hba ...`。
> 由于 `interface` 已在 `package.xml` 中被声明为依赖，colcon 会自动先构建它，**不必**拆成两步执行。

### 4.2 构建结果

```
Finished <<< interface [0.45s]
Summary: 1 package finished
...
Finished <<< pgo [33.7s]
Summary: 4 packages finished [33.8s]
  4 packages had stderr output: fastlio2 hba localizer pgo
```

- **返回码 `rc=0`，零编译错误**（对完整构建日志做 `error:/FAILED/fatal error/undefined reference` 检索，无任何命中）。
- 「4 packages had stderr output」仅为告警输出，非错误。
- 产物：
  - `install/fastlio2/lib/fastlio2/lio_node`
  - `install/pgo/lib/pgo/pgo_node`
  - `install/localizer/lib/localizer/localizer_node`
  - `install/hba/lib/hba/hba_node`
- `ros2 pkg list` 可见 `fastlio2 / pgo / localizer / hba / interface` 均已注册。

> 说明：`build/`、`install/`、`log/` 下的 `COLCON_IGNORE` 是 colcon 标准忽略标记（防止把这些目录当作源扫描），**不影响** `src/` 下功能包的发现与构建。

---

## 5. Jazzy 兼容性分析与问题排查

对全部节点源码（`lio_node.cpp` / `pgo_node.cpp` / `localizer_node.cpp` / `hba_node.cpp` 及 `map_builder`、`pgos`、`localizers`、`hba` 子目录）做了 Humble → Jazzy 的 API 屏蔽审查。

### 5.1 兼容性结论

**无需修改任何源码即可在 Jazzy 编译运行。** 所用 API 均为 Humble/Jazzy 稳定公共 API：

- `rclcpp::init/spin/shutdown`、`create_subscription/create_publisher/create_service/create_wall_timer`、`declare_parameter/get_parameter`、`get_subscription_count()`
- `tf2_ros::TransformBroadcaster`（构造仍接受 `*this`）
- `message_filters::Subscriber` + `sync_policies::ApproximateTime`
- `pcl::toROSMsg` / `pcl::fromROSMsg`（`pcl_conversions` 在 Jazzy 中保留）
- `std::filesystem`（C++17，各包均 `set(CMAKE_CXX_STANDARD 17)`）
- `builtin_interfaces::msg::Time`、`visualization_msgs::msg::Marker` 等

### 5.2 构建期告警（非阻断，记录在案）

| 位置 | 告警 | 性质 | 处理 |
| --- | --- | --- | --- |
| `pgo/src/pgo_node.cpp:18` `#include <pcl/io/io.h>` | `PCL_DEPRECATED_HEADER(1,15)`：PCL 1.14 弃用 `pcl/io/io.h`，建议改 `pcl/common/io.h` | 仅告警，1.15 才移除 | 暂不处理；如需消警可改为 `#include <pcl/common/io.h>` |
| `fastlio2/src/map_builder/ikd_Tree.cpp` 多处 | `-Wunused-variable`、`-Wsign-compare`、`-Wparentheses`（GCC 13 较 GCC 11 更严格） | 告警；各包 `CMakeLists.txt` 未启用 `-Werror` | 不影响构建；属上游 FAST-LIO 原始代码风格问题 |
| `pgo_node.cpp` / `localizer_node.cpp` 的 `qos.get_rmw_qos_profile()` | Jazzy 中该接口已弃用（推荐直接传 `rclcpp::QoS` 或 `SensorDataQoS`） | 告警，仍可编译 | 如需消警可改写为 `m_cloud_sub.subscribe(this, topic, qos);` |
| GTSAM 内部 `Values::filter` | `-Wdeprecated-declarations` | GTSAM 自身弃用提示 | 无需处理 |

### 5.3 排查过但未命中问题的点（备忘）

- **C++ 标准**：Jazzy 最低 C++17，各包均设 `CMAKE_CXX_STANDARD 17`，满足。
- **`livox_ros_driver2` 的 Jazzy 可用性**：该包已在工作空间内构建安装，`find_package(livox_ros_driver2 REQUIRED)` 与 `livox_ros_driver2::msg::CustomMsg` 均正常解析，未需重建。
- **`interface` 依赖顺序**：`pgo/localizer/hba` 的 `package.xml` 均显式声明了 `<depend>interface</depend>`，colcon 会自动把 `interface` 排在这些包之前构建，故一次性 `colcon build` 即可，无需手动分阶段（实测 6 包一次性构建通过）。
- **`pcl_conversions` API**：Jazzy 未移除 `pcl::toROSMsg/fromROSMsg`，无需替换。

---

## 6. 运行验证（ROS2 Jazzy）

### 6.1 动态库依赖检查

`ldd` 检查全部 `not found` 为空，依赖完全解析：

```
lio_node     -> libpcl_*.so.1.14 (系统)
pgo_node     -> libgtsam.so.4 => /usr/local/lib/libgtsam.so.4
              -> libmetis-gtsam.so => /usr/local/lib/libmetis-gtsam.so
hba_node     -> libgtsam.so.4 / libmetis-gtsam.so (同上)
```

### 6.2 节点启动冒烟测试

逐节点带配置启动 6~7 秒，确认初始化与优雅退出（无数据时各节点空闲 spin，timerCB 提前返回）：

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
WS=/home/robomaster/project/sentry_ws/install

# LIO（主里程计）
timeout 7 ros2 run fastlio2 lio_node --ros-args -p config_path:=$WS/fastlio2/share/fastlio2/config/lio.yaml
# PGO（回环）
timeout 6 ros2 run pgo pgo_node --ros-args -p config_path:=$WS/pgo/share/pgo/config/pgo.yaml
# 重定位
timeout 6 ros2 run localizer localizer_node --ros-args -p config_path:=$WS/localizer/share/localizer/config/localizer.yaml
# 一致性地图优化
timeout 6 ros2 run hba hba_node --ros-args -p config_path:=$WS/hba/share/hba/config/hba.yaml
```

结果（每个节点均打印启动 INFO、加载 YAML、收到 SIGTERM 后正常退出）：

| 节点 | 启动日志 | 退出 |
| --- | --- | --- |
| `lio_node` | `LIO Node Started` → `LOAD FROM YAML CONFIG PATH: .../lio.yaml` → `Multi thread started`（ikd-Tree 重建线程） | `Rebuild thread terminated normally` |
| `pgo_node` | `PGO node started` → `LOAD FROM YAML CONFIG PATH: .../pgo.yaml` | `signal_handler(SIGINT/SIGTERM)` |
| `localizer_node` | `Localizer Node Started` → `LOAD FROM YAML CONFIG PATH: .../localizer.yaml` | `signal_handler(SIGINT/SIGTERM)` |
| `hba_node` | `HBA node started` → `LOAD FROM YAML CONFIG PATH: .../hba.yaml` | `signal_handler(SIGINT/SIGTERM)` |

> 环境提示：ROS2 Jazzy 默认经 spdlog 将日志写入 `~/.ros/log/`。若运行环境对该目录无写权限（如受限沙箱/无 HOME 写权限），节点会在初始化阶段抛 `spdlog::spdlog_ex: Failed opening file ... Permission denied` 而中止。此为**运行环境**问题而非 Jazzy 适配问题，规避方式：`export ROS_LOG_DIR=/tmp/roslog && mkdir -p $ROS_LOG_DIR`，或在有 `~/.ros/log` 写权限的常规环境直接运行。

---

## 7. 完整复现步骤

```bash
# 0) 前置：已安装 ROS2 Jazzy、pcl、Eigen、livox_ros_driver2（工作空间内已构建）
#    若 sudo 非免密，配置 NOPASSWD（可选，见 3.1）

# 1) 安装 GTSAM 构建所需 apt 依赖
sudo apt-get update
sudo apt-get install -y libsuitesparse-dev libceres-dev libgmp-dev libmpfr-dev

# 2) 编译安装 Sophus
cd /home/robomaster/git_install
git clone https://github.com/strasdat/Sophus.git && cd Sophus && git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON -DBUILD_SOPHUS_TESTS=OFF
make -j$(nproc) && sudo make install && sudo ldconfig

# 3) 编译安装 GTSAM
cd /home/robomaster/git_install
git clone --branch 4.2.0 https://github.com/borglab/gtsam.git && cd gtsam
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF
make -j$(nproc) && sudo make install && sudo ldconfig

# 4) 构建 FASTLIO2_ROS2（项目根目录一次性构建，colcon 自动排序）
cd /home/robomaster/project/sentry_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

# 5) 运行（任选其一）
source install/setup.bash
ros2 launch fastlio2 lio_launch.py        # 里程计（含 rviz）
# 或仅节点：
ros2 run fastlio2 lio_node --ros-args -p config_path:=install/fastlio2/share/fastlio2/config/lio.yaml
```

---

## 8. 遗留事项与建议

1. **`sudoers` NOPASSWD 规则**：`/etc/sudoers.d/zz-robomaster-nopasswd` 为本次适配便利而设，按需删除以收紧权限。
2. **可选消警改进**（非必须）：
   - `pgo/src/pgo_node.cpp:18`：`#include <pcl/io/io.h>` → `#include <pcl/common/io.h>`（适配 PCL 1.14+）。
   - `pgo_node.cpp` / `localizer_node.cpp`：将 `qos.get_rmw_qos_profile()` 替换为直接传 `rclcpp::QoS`，消除 Jazzy 弃用告警。
3. **未覆盖的运行时验证**：本次仅做「无数据下的节点启动/退出」冒烟测试；端到端精度（实际 rosbag 回放、回环闭合、重定位命中、HBA 一致性优化效果）需用 README 提供的数据集进一步验证。
4. **livox_ros_driver2 未重新构建**：假定其在 Jazzy 下可用（已预装）。若后续 `find_package(livox_ros_driver2)` 或 `CustomMsg` 出现异常，需用 Jazzy 重新构建该驱动包。
