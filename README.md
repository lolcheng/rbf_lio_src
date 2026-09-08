# RBF-LIO

## Overview

RBF-LIO 是一个面向轮腿机器人非平整地形运动的 ROS 1 激光雷达-惯性里程计与建图工程。当前主实现仍适配 Livox MID360 和 Tron1A 两轮运动学；仓库已加入 M20 模型、测试 rosbag、测试桥接包和 RoboSense Airy ROS 2 驱动参考资料，但 M20/Airy 尚未接入主运行链。

代码是当前行为的首要事实来源。仓库包含论文算法的多个组成部分，但当前实现与论文描述并非完全一致，不能视为论文处理链的完整复现。

## Key Features

- 接收 Livox MID360 `CustomMsg`，执行点云坐标变换和 IMU 旋转去畸变。
- 提取 LOAM 风格的边缘特征和平面特征。
- 执行扫描到地图 LM 优化、关键帧管理和 GTSAM iSAM2 因子图优化。
- 使用 GTSAM IMU 预积分进行高频状态传播、速度和偏置估计。
- 根据 Tron1A 的 8 个关节位置计算左右轮几何，并将轮地/RBF 残差加入主位姿优化。
- 支持内置回环检测、可选 GPS 因子和在线 LiDAR z 偏移反馈。
- 提供独立的 CUDA RBF 高程拟合、可视化、耗时和 MAE 统计节点。
- 发布里程计、路径、特征点云、局部/全局地图，并支持 TUM 轨迹和 PCD 地图保存。

## Architecture

```mermaid
flowchart LR
    L[Livox MID360] --> P[imageProjection]
    I[IMU] --> P
    I --> U[imuPreintegration]
    P --> F[featureExtraction]
    F --> M[mapOptmization]
    J[/joint_states] --> K[Tron1A wheel kinematics]
    K --> M
    U --> P
    M --> U
    M --> O[Odometry / Path / Map]
    M --> R[RBF residual stats]
    M -. local map .-> E[optional elevation_rbf CUDA node]
    E --> V[/rbf_elevation_map]
```

主估计器中的轮地残差当前由 CPU 代码计算；独立 `elevation_rbf` 使用 CUDA，但其拟合模型不会反馈给主估计器。

完整节点、数据流和 CPU/CUDA 关系见 [软件架构文档](docs/ARCHITECTURE.md)。算法到代码的映射见 [算法文档](docs/ALGORITHM.md)。

## Repository Structure

| Path | Purpose |
|---|---|
| `LIO-SAM-MID360/` | 当前主 ROS 包，包名为 `lio_sam` |
| `rbf_cuda/` | CUDA RBF 库、独立高程节点和旧版/实验性 LIO-SAM 分支 |
| `robot-description/` | 当前保留的 M20 URDF 和网格资源；主 launch/CMake 尚未完成适配 |
| `data/rosbag/` | M20 接口测试 bag；不作为已验证算法结果数据集 |
| `manual/` | M20 硬件手册等只读参考资料 |
| `support/` | 论文、测试 UDP bridge、Airy ROS 2 官方 submodule、M20 驱动配置及参考启动脚本 |
| `docs/` | 架构、算法、实验和部署文档 |
| `AGENTS.md` | 仓库协作规则和已知代码/论文差异 |

`rbf_cuda/src/lio_sam` 不是当前主实现，不应与 `LIO-SAM-MID360` 混用。

## Requirements

当前代码目标环境为 Linux 上的 ROS 1 Noetic 和 Catkin。可从 CMake 确认的主要依赖包括：

- PCL、OpenCV、Eigen3、GTSAM、Boost、OpenMP。
- ROS `roscpp`、`rospy`、`tf`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`visualization_msgs`、`cv_bridge` 和 `pcl_conversions`。
- ROS 1 Livox 驱动；CMake 声明 `livox_ros_driver`，源码可在头文件存在时选择 driver 或 driver2 的 `CustomMsg`。
- 构建 `cuda_rbf` 所需的 CUDA Toolkit、Thrust、cuBLAS、cuSOLVER 和 `nlohmann_json`。
- 可选 FK 脚本所需的 NumPy、`urdfpy` 和 `tf_conversions`。

具体 Ubuntu、CUDA、GCC 和依赖版本未被仓库锁定。

## Build

根目录 `CMakeLists.txt` 当前不是可直接使用的 Catkin 顶层文件。最短的无源码修改构建方式是在外部工作区链接三个包：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/rbf_lio_ws/src
cd ~/rbf_lio_ws/src
ln -s /absolute/path/to/rbf_lio_20260402/LIO-SAM-MID360 lio_sam
ln -s /absolute/path/to/rbf_lio_20260402/rbf_cuda cuda_rbf
ln -s /absolute/path/to/rbf_lio_20260402/robot-description robot_description
catkin_init_workspace
cd ~/rbf_lio_ws
catkin_make
source devel/setup.bash
```

完整工作区包含 `cuda_rbf`，因此构建时必须存在 CUDA。仓库没有提供 Dockerfile 或固定依赖安装脚本。

当前 `robot-description/CMakeLists.txt` 仍引用已删除的 `pointfoot/wheellegged`，尚未安装 `M20`。因此上面是工作区组织命令，不是当前检出已成功构建的证明；M20 描述包需要在迁移中修正。

## Quick Start

先启动能够发布 Livox `CustomMsg`、IMU 和 `/joint_states` 的实际驱动。仓库只确认 Livox 驱动应使用 `livox_lidar_msg.launch` 的自定义消息模式，具体驱动包、网络配置和机器人驱动命令未包含在仓库中。

历史主定位链的启动形式如下：

```bash
mkdir -p /absolute/output
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_tron1a_all.launch \
  with_rbf:=false \
  with_robot_tf:=false \
  with_rviz:=true \
  tum_trajectory_path:=/absolute/output/trajectory_tum.txt
```

当前 `run_tron1a_all.launch` 仍引用已删除的 Tron1A 模型，且主点云入口只接收 Livox `CustomMsg`；因此此命令不能直接运行 M20/Airy。M20 可用的最小启动命令为 **TODO**。

Airy 驱动现场已确认的启动命令为：

```bash
ros2 launch rslidar_sdk start.py
```

该命令属于独立 ROS 2 驱动工作区，不等于已经与 ROS 1 RBF-LIO 连通。`support/rslidar_start.sh` 混用了 ROS 2 环境和 ROS 1 `roslaunch`，当前不能作为已验证入口。

`with_rbf:=false` 只关闭独立 CUDA 高程可视化节点；主优化器中的 RBF/轮地约束仍由 YAML 的 `enableRbfConstraint: true` 启用。

## ROS Interfaces

### Key Inputs

| Topic | Type | Purpose |
|---|---|---|
| `lio_sam/pointCloudTopic` 配置值 | Livox `CustomMsg` | MID360 原始点云和每点相对时间 |
| `lio_sam/imuTopic` 配置值 | `sensor_msgs/Imu` | 去畸变和 IMU 预积分 |
| `/joint_states` | `sensor_msgs/JointState` | Tron1A 左右轮几何和轮地约束 |
| `lio_sam/gpsTopic` 配置值 | `nav_msgs/Odometry` | 可选 GPS 因子；当前综合 launch 未提供完整生产链 |

M20 迁移目标输入为 `/rslidar_points_front`（`sensor_msgs/PointCloud2`）、`/rslidar_imu_data_front`（`sensor_msgs/Imu`）和 `/JOINTS_DATA`（测试桥定义的 M20 自定义消息）。这些接口已在测试 bag/驱动配置中确认，但当前主代码尚未订阅；第一阶段不使用后置 Airy 点云或其 IMU。

### Key Outputs

| Topic | Type | Purpose |
|---|---|---|
| `lio_sam/mapping/odometry` | `nav_msgs/Odometry` | 建图校正位姿 |
| `odometry/imu` | `nav_msgs/Odometry` | LiDAR 校正后的高频 IMU 融合里程计 |
| `lio_sam/mapping/path` | `nav_msgs/Path` | 关键帧轨迹 |
| `lio_sam/mapping/map_local` | `sensor_msgs/PointCloud2` | 局部地图 |
| `lio_sam/mapping/map_global` | `sensor_msgs/PointCloud2` | 全局可视化地图 |
| `lio_sam/rbf/residual_stats` | `std_msgs/Float64MultiArray` | 主轮地约束诊断 |
| `/rbf_elevation_map` | `sensor_msgs/PointCloud2` | 可选独立 CUDA RBF 高程图 |

地图保存服务为 `/lio_sam/save_map`。

## Configuration

- `LIO-SAM-MID360/config/paramsLivoxIMU.yaml`：Tron1A + MID360 内置六轴 IMU 的综合入口配置；当前启用主 RBF 约束。
- `LIO-SAM-MID360/config/params9axisIMU.yaml`：九轴 IMU 配置；当前默认关闭主 RBF 约束。
- `LIO-SAM-MID360/config/params.yaml`：通用配置；当前默认关闭主 RBF 约束。
- `LIO-SAM-MID360/launch/run_tron1a_all.launch`：历史综合入口；当前仍引用已删除的 Tron1A 模型，待 M20 launch 替换。
- `rbf_cuda/launch/run_rbf.launch`：独立 CUDA RBF 高程与统计节点，不控制主估计器约束。
- `support/rslidar_ros2_ws/config/m20_airy.yaml`：Airy 双雷达 ROS 2 驱动参考配置，不会被当前 ROS 1 主包加载。

更换设备或机器人前必须核对 topic、Livox 消息包、IMU 类型与单位、外参、frame、关节名称/顺序和轮半径。详细清单见 [部署文档](docs/DEPLOYMENT.md)。

## Experiments

仓库能够输出 TUM 轨迹、PCD 地图以及独立 RBF 节点的 kernel/weight 耗时和 MAE。`data/rosbag/m20_upstairs.bag` 是现有 M20 接口测试数据，但当前主代码不能直接消费其 PointCloud2/M20 关节消息；该 bag 中关节源样本约 10 Hz、经重复发布为约 500 Hz，且部分值不符合 URDF 限位，因此不能作为最终运动学或精度验证数据。仓库仍不包含 ground truth、ATE/z-ATE/RTE 评估程序、论文消融批处理或图表生成脚本。

已确认的运行命令、参数变体、结果目录和缺失复现信息见 [实验手册](docs/EXPERIMENT.md)。

## Deployment

真实传感器接入、ROS topic、坐标系、时间同步、外参检查和故障排查见 [真实机器人部署手册](docs/DEPLOYMENT.md)。

## Results

- [support/RBF_LIO.pdf](support/RBF_LIO.pdf) 包含论文报告的算法、实验表格和图示，但仓库没有提供生成这些结果所需的完整数据与评估脚本。
- `LIO-SAM-MID360/doc/` 包含现有的 [室内旋转](LIO-SAM-MID360/doc/indoorTest.gif)、[室外倾斜手持](LIO-SAM-MID360/doc/Outdoor.gif) 和 [室内环境](LIO-SAM-MID360/doc/indoor.gif) 演示动画。

## Citation

**TODO:** 仓库中没有可确认的 BibTeX 条目。补充经过论文正式元数据核对的引用信息前，不在此处自行生成作者、标题或出版信息。

## License

仓库根目录没有统一的 `LICENSE`，各部分的现有许可证状态不同：

- `LIO-SAM-MID360/LICENSE`：BSD 3-Clause，Copyright (c) 2020, Tixiao Shan。
- `robot-description/LICENSE`：Apache License 2.0；其 `package.xml` 的 license 字段仍为 `TODO`。
- `rbf_cuda/package.xml` 的 license 字段为 `TODO`，且该目录没有独立 `LICENSE`。

因此不能将某一个子目录的许可证自动视为整个仓库的统一许可证。
