# RBF-LIO

> Ubuntu 20.04 换机接续开发请先阅读 [开发交接文档](docs/HANDOFF_UBUNTU.md)，其中记录了当前 M20 迁移状态、测试包校验值、首次构建步骤和未完成项。

## Overview

RBF-LIO 是面向轮腿机器人非平整地形运动的 ROS 1 激光雷达-惯性里程计与建图工程。当前主实现位于 `LIO-SAM-MID360`，已增加 M20 四轮运动学、前置 RoboSense Airy 点云/IMU 输入和轮地 RBF 约束；`rbf_cuda/src/lio_sam` 是旧版或实验性分支。

代码行为是首要事实来源。论文 [support/RBF_LIO.pdf](support/RBF_LIO.pdf) 用于解释算法背景，但当前仓库并非论文处理链的完整复现。

## Key Features

- 接收 Livox `CustomMsg` 或 Airy `sensor_msgs/PointCloud2`，使用 IMU 旋转去畸变。
- 提取 LOAM 边缘和平面特征，执行扫描到地图 LM 优化。
- 使用 GTSAM IMU 预积分和 iSAM2 关键帧因子图优化。
- 支持历史 Tron1A 两轮运动学和 M20 四轮运动学。
- 将轮地/RBF 接触残差加入主位姿优化。
- 发布里程计、路径、特征点云和局部/全局地图，支持 TUM 轨迹与 PCD 地图保存。
- 提供独立的 CUDA RBF 地形拟合、可视化及耗时/MAE 统计节点。

## Architecture

```mermaid
flowchart LR
    L[Front Airy PointCloud2] --> P[imageProjection]
    I[Front Airy IMU] --> P
    I --> U[imuPreintegration]
    P --> F[featureExtraction]
    F --> M[mapOptmization]
    J[/JOINTS_DATA] --> A[M20 JointState adapter]
    A --> K[M20 four-wheel kinematics]
    K --> M
    U --> P
    M --> U
    M --> O[Odometry / Path / Map]
```

主优化器的轮地残差当前由 CPU 代码计算；独立 `elevation_rbf` 节点使用 CUDA，但其模型不反馈给主估计器。详细关系见 [软件架构](docs/ARCHITECTURE.md) 和 [算法映射](docs/ALGORITHM.md)。

## Repository Structure

| Path | Purpose |
|---|---|
| `LIO-SAM-MID360/` | 当前主 ROS 1 包，包名 `lio_sam` |
| `rbf_cuda/` | CUDA RBF 库、独立高程节点及旧版 LIO-SAM 分支 |
| `robot-description/M20/` | M20 URDF 与网格，运动学几何事实来源 |
| `data/rosbag/` | 本地测试数据，已由 `.gitignore` 排除 |
| `manual/` | M20 手册等只读资料 |
| `support/` | 论文、临时 UDP bridge、Airy ROS 2 驱动参考与 submodule |
| `docs/` | 架构、算法、实验和部署文档 |

## Requirements

可从代码确认的主环境为 Linux、ROS 1 Noetic 和 Catkin，依赖 PCL、OpenCV、Eigen3、GTSAM、Boost、OpenMP 及常用 ROS 消息包。Livox 支持是可选项；仅运行 M20/Airy 主链时不要求 Livox 消息包。构建 `cuda_rbf` 还需要 CUDA Toolkit、Thrust、cuBLAS、cuSOLVER 和 `nlohmann_json`。

具体 Ubuntu、GCC 与 CUDA 版本未由仓库锁定，标记为 **UNKNOWN**。

## Build

根目录 `CMakeLists.txt` 不是可直接使用的 Catkin 顶层符号链接。建议在外部工作区构建：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/rbf_lio_ws/src
cd ~/rbf_lio_ws/src
ln -s /absolute/path/to/rbf_lio_20260402/LIO-SAM-MID360 lio_sam
ln -s /absolute/path/to/rbf_lio_20260402/robot-description robot_description
catkin_init_workspace
cd ~/rbf_lio_ws
catkin_make
source devel/setup.bash
```

需要独立 CUDA 高程节点时，再将 `rbf_cuda` 链接到 `src/cuda_rbf` 后重新编译。当前 Windows 环境未安装 ROS Noetic，因此本次 M20 修改尚未完成 Linux/ROS 实编译。

## Quick Start

测试 bag 已包含 ROS 1 的前 Airy点云、前 Airy IMU 和 `/JOINTS_DATA`：

```bash
# Terminal 1
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_m20.launch \
  use_sim_time:=true \
  with_rviz:=true \
  with_robot_tf:=true \
  enable_rbf_constraint:=false \
  tum_trajectory_path:=/absolute/output/m20_upstairs_tum.txt

# Terminal 2
rosbag play --clock /absolute/path/to/rbf_lio_20260402/data/rosbag/m20_upstairs.bag
```

测试 bag 的关节消息由约 10 Hz 有效样本重复成约 500 Hz，且部分值超出 URDF 限位，只适合接口联调，因此 M20 launch 默认关闭主 RBF 约束。最终部署必须换为 GOS 直接发布的真实 500 Hz 数据，验证关节约定后再传入 `enable_rbf_constraint:=true`。

真实 Airy 驱动的已确认入口是 `ros2 launch rslidar_sdk start.py`。主估计器是 ROS 1，仓库未包含已验证的 ROS 1/ROS 2 bridge 启动方案，因此实机跨版本连通仍为 **TODO**。

## ROS Interfaces

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Input | `/rslidar_points_front` | `sensor_msgs/PointCloud2` | 前 Airy 原始点云和绝对逐点时间戳 |
| Input | `/rslidar_imu_data_front` | `sensor_msgs/Imu` | 前 Airy 六轴 IMU |
| Input | `/JOINTS_DATA` | 与 `M20JointsData.msg` 同 MD5 的消息 | M20 腿关节角和轮速 |
| Internal | `/joint_states_m20` | `sensor_msgs/JointState` | M20 URDF/四轮运动学输入 |
| Output | `lio_sam/mapping/odometry` | `nav_msgs/Odometry` | 建图校正里程计 |
| Output | `odometry/imu` | `nav_msgs/Odometry` | 高频 IMU 融合里程计 |
| Output | `lio_sam/mapping/path` | `nav_msgs/Path` | 关键帧轨迹 |
| Output | `lio_sam/mapping/map_local` | `sensor_msgs/PointCloud2` | 局部地图 |

第一阶段明确不使用后置 Airy 点云、后置 Airy IMU 或桥接包的 `/IMU`。

## Configuration

- `LIO-SAM-MID360/config/paramsM20.yaml`：M20 + 前 Airy 参数、组合外参和 RBF 参数。
- `LIO-SAM-MID360/launch/run_m20.launch`：M20 主启动入口。
- `LIO-SAM-MID360/config/paramsLivoxIMU.yaml`：历史 Tron1A/MID360 六轴配置。
- `support/rslidar_ros2_ws/config/m20_airy.yaml`：只读 Airy ROS 2 驱动参考配置，不由 ROS 1 主包加载。

M20 点云坐标变换由主包完成，驱动构建必须保持 `ENABLE_TRANSFORM=OFF`，否则会发生重复变换。DIFOP 外参和 URDF 安装位置的推导见 [部署文档](docs/DEPLOYMENT.md)。

## Experiments

仓库可输出 TUM 轨迹、PCD 地图以及 RBF 耗时/MAE 日志，但不包含 ground truth 或 ATE、z-ATE、RTE 评估脚本。现有数据、可执行命令和复现缺口见 [实验手册](docs/EXPERIMENT.md)。

## Deployment

传感器启动顺序、ROS 1/ROS 2 边界、坐标外参、时间同步和排障入口见 [部署手册](docs/DEPLOYMENT.md)。

## Results

- [support/RBF_LIO.pdf](support/RBF_LIO.pdf) 包含论文中的结果、表格和图示；仓库没有完整的结果生成数据与评估脚本。
- `LIO-SAM-MID360/doc/` 包含已有演示动画。

## Citation

**TODO:** 仓库中没有可确认的 BibTeX 条目，因此不自行生成论文元数据。

## License

仓库根目录没有统一 `LICENSE`。`LIO-SAM-MID360/LICENSE` 为 BSD 3-Clause；`robot-description/LICENSE` 为 Apache License 2.0；`rbf_cuda` 未提供独立许可证文件，其 `package.xml` license 仍为 `TODO`。
