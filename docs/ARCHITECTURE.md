# RBF-LIO 软件架构

本文档记录当前仓库中能够由代码、配置和脚本直接确认的软件架构。事实优先级为：

1. `LIO-SAM-MID360` 当前代码和配置。
2. `rbf_cuda` 中实际可达的代码路径。
3. `robot-description` 中的机器人模型。
4. `support/RBF_LIO.pdf` 仅作为理论背景，不用于证明某项功能已经在代码中接通。

无法仅从仓库确认的内容统一标记为 **UNKNOWN** 或 **TODO**。`rbf_cuda/src/lio_sam` 是旧版或实验性处理链，不视为当前主实现。

# 1. Project Overview（项目概述）

## 1.1 项目解决的问题

本项目是面向轮腿机器人非平整地形运动的 ROS 1 激光雷达-惯性里程计与建图工程。当前主实现基于 LIO-SAM 风格的处理链，接收 Livox MID360 点云和 IMU，完成：

- 点云坐标变换、扫描组织和 IMU 旋转去畸变。
- LOAM 风格的边缘和平面特征提取。
- 扫描到局部地图的六自由度迭代优化。
- 基于 GTSAM iSAM2 的关键帧因子图优化。
- 基于 GTSAM IMU 预积分的速度、位姿和 IMU 偏置估计。
- 基于 Tron1A 关节状态和轮地接触几何的附加约束。
- 地图、里程计、轨迹、回环和诊断结果发布。

主程序位于 `LIO-SAM-MID360`，ROS 包名为 `lio_sam`。历史综合启动入口是 `LIO-SAM-MID360/launch/run_tron1a_all.launch`；该文件仍引用已经从 `robot-description` 删除的 Tron1A 模型，因此当前检出中不能把它视为已完成的 M20 启动入口。

## 1.2 RBF-LIO 在系统中的定位

当前仓库存在两条需要严格区分的 RBF 路径：

1. **主估计器中的轮地约束路径**：`mapOptimization::updateRbfLmConstraints()` 构造请求，经 `RbfCudaGradientInterface` 生成轮地残差和位姿 Jacobian，再由 `mapOptimization::LMOptimization()` 将这些行追加到 LOAM 线性系统。该路径直接影响最终位姿，但当前实际实现位于 CPU C++ 文件 `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp`。
2. **独立 CUDA 地形拟合路径**：`cuda_rbf/elevation_rbf` 从局部地图拟合 `VoxelGridRBF`，发布 `/rbf_elevation_map` 并记录耗时和 MAE。它使用 `rbf_cuda/src/cuda/rbf_cuda.cu`，但其拟合结果当前没有作为输入送回主地图优化器。

因此，当前代码不能简单描述为“主估计器直接使用 CUDA 拟合的 RBF 地形模型”。两条路径在 ROS 话题和模型数据上没有端到端连接。

## 1.3 M20/Airy 迁移现状

M20 迁移的硬件与接口参考已经进入仓库，但尚未进入上述主运行链：

- `robot-description/M20/urdf/M20.urdf` 提供四条腿、16 个关节、四个车轮和前后 LiDAR 固定关节；迁移时以此处的关节轴、零位、限位和轮半径 `0.09 m` 为准。
- `data/rosbag/m20_upstairs.bag` 提供前后 Airy `PointCloud2`、前后 Airy IMU 以及 M20 自定义关节/机体消息，用于接口分析。
- 第一阶段目标输入已限定为前置 Airy 点云 `/rslidar_points_front`、前置 Airy IMU `/rslidar_imu_data_front` 和 `/JOINTS_DATA`。后置点云和后置 IMU不进入第一阶段估计。
- `support/rslidar_ros2_ws` 通过 Git submodule 固定 RoboSense SDK v1.5.19 和 `rslidar_msg`；M20 配置及构建差异位于 submodule 外。当前主估计器是 ROS 1，仓库中尚无 ROS 1/ROS 2 桥接或原生 ROS 2 主估计器实现。
- `support/m20_udp_bridge.zip` 是临时测试桥。当前 bag 中 `/JOINTS_DATA` 虽约为 500 Hz，但仅约 10 Hz 的源样本有效更新，其余为重复发布；最终接口要求由 GOS 直接提供真实 500 Hz 数据。

因此，“M20/Airy 输入资料齐备”和“主算法已适配 M20”是两个不同状态；后者当前仍为 **TODO**。

# 2. Repository Structure（仓库结构）

| 路径 | 用途 | 当前地位 |
| --- | --- | --- |
| `AGENTS.md` | 仓库协作约束、构建和代码/论文差异说明 | 整仓适用，不由本文档替代 |
| `CMakeLists.txt` | 内容仅为 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake` | 不是可直接解析的常规顶层 CMake 文件 |
| `LIO-SAM-MID360/` | MID360 主 LIO、轮地约束、IMU 预积分、地图优化 | 当前主实现 |
| `LIO-SAM-MID360/src/` | 四个核心节点及轮地约束实现 | 主运行代码 |
| `LIO-SAM-MID360/include/` | 参数服务器、消息辅助、RBF 接口、运动学声明 | 主公共头文件 |
| `LIO-SAM-MID360/config/` | 通用、六轴/MID360、九轴 IMU 参数 | 主配置来源 |
| `LIO-SAM-MID360/launch/` | 综合启动和模块化 launch | 主运行入口 |
| `LIO-SAM-MID360/msg/` | `cloud_info.msg` | 节点间点云和初值载体 |
| `LIO-SAM-MID360/srv/` | `save_map.srv` | 地图保存服务契约 |
| `LIO-SAM-MID360/scripts/` | 关节名称适配器 | 可选机器人 TF 辅助 |
| `rbf_cuda/` | CUDA RBF 库、独立高程节点、诊断程序 | 辅助/实验包 |
| `rbf_cuda/include/rbf_fitting/` | `VoxelGridRBF` 模板实现 | 独立 RBF 拟合核心 |
| `rbf_cuda/src/cuda/` | CUDA/Thrust/cuBLAS/cuSOLVER 实现 | 独立 RBF GPU 后端 |
| `rbf_cuda/src/rbf/` | `elevation_rbf`、`debug_rbf` | 可视化、统计和离线诊断 |
| `rbf_cuda/src/lio_sam/` | 旧版 LIO-SAM 分支和实验性 `lidar_odometry` | 非当前主实现 |
| `rbf_cuda/script/` | FK、GPS/TUM 转换、里程计记录 | 辅助脚本 |
| `robot-description/M20/` | M20 URDF、STL 网格和相关模型文件 | M20 迁移的几何事实来源 |
| `data/rosbag/m20_upstairs.bag` | M20/Airy 接口测试数据 | 可用于输入适配，不足以验证最终关节运动学或定位精度 |
| `manual/` | M20 硬件手册 | 只读硬件参考，代码事实仍优先 |
| `support/RBF_LIO.pdf` | 论文原文 | 只读理论参考 |
| `support/m20_udp_bridge.zip` | 测试用 UDP/ROS 1 桥接代码 | 临时接口参考，不是最终 500 Hz GOS 实现 |
| `support/rslidar_ros2_ws/` | Airy ROS 2 官方源码 submodule、M20 配置和构建补丁 | 参考工作区；需递归初始化 submodule |
| `support/rslidar_start.sh` | Airy 启动参考脚本 | 与现场确认命令不一致，未验证 |

# 3. Build Architecture（构建架构）

## 3.1 Workspace 与 ROS 包

主 ROS 1/Catkin 工作区包含三个包：

| 目录 | ROS 包名 | 构建系统 | C++ 标准 |
| --- | --- | --- | --- |
| `LIO-SAM-MID360` | `lio_sam` | Catkin | C++14 |
| `rbf_cuda` | `cuda_rbf` | Catkin + FindCUDA | C++17 |
| `robot-description` | `robot_description` | 根据 `ROS_VERSION` 选择 Catkin 或 ament | 不适用 |

`support/rslidar_ros2_ws/src` 以 Git submodule 引入 ROS 2 `rslidar_msg` 和
`rslidar_sdk`；`rslidar_sdk` 还包含嵌套的 `rs_driver` submodule。它不属于上述
ROS 1 主工作区。克隆后必须执行 `git submodule update --init --recursive`，并按
`support/rslidar_ros2_ws/README.md` 应用 M20 配置/构建差异后再构建。

根目录 `CMakeLists.txt` 当前不是有效的 Catkin 顶层包装文件。推荐在外部 Catkin 工作区中链接三个包：

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

代码和 launch 目标环境是 Linux ROS 1 Noetic；仓库没有提供原生 Windows 构建方案。原生 Windows 构建状态为 **UNKNOWN**。

## 3.2 `lio_sam` 构建目标

`LIO-SAM-MID360/CMakeLists.txt` 定义：

| 目标 | 源文件 |
| --- | --- |
| `lio_sam_imageProjection` | `src/imageProjection.cpp` |
| `lio_sam_featureExtraction` | `src/featureExtraction.cpp` |
| `lio_sam_mapOptmization` | `src/mapOptmization.cpp`、`src/rbf_wheel_kinematics.cpp`、`src/rbf_cuda_wheel_gradient.cpp` |
| `lio_sam_imuPreintegration` | `src/imuPreintegration.cpp` |
| `lio_sam_rbf_cuda` | `src/cuda/rbf_cuda_wheel_gradient.cu`，仅在 `CUDA_FOUND` 时生成 |

`lio_sam_mapOptmization` 无论是否找到 CUDA都会编译 CPU 轮地实现。找到 CUDA 后只会额外编译并链接 `lio_sam_rbf_cuda`；当前没有找到调用 `ComputeWheelResidualJacobianCuda()` 的运行时代码。

## 3.3 `cuda_rbf` 构建目标

`rbf_cuda/CMakeLists.txt` 将 CUDA 设为必需依赖并定义：

- CUDA 库 `rbf_cuda`，源文件为 `src/cuda/rbf_cuda.cu`。
- 一套旧版 `cuda_rbf_*` LIO-SAM 节点。
- `cuda_rbf_lidarOdometry` 实验节点。
- `debug_rbf` 离线诊断程序。
- `elevation_rbf` 独立高程拟合节点。

## 3.4 ROS 与外部依赖

| 类别 | 依赖 | 来源/用途 |
| --- | --- | --- |
| ROS | `roscpp`、`rospy`、`tf` | 节点、回调、TF |
| ROS 消息 | `sensor_msgs`、`nav_msgs`、`geometry_msgs`、`std_msgs`、`visualization_msgs` | 传感器、里程计、地图和诊断 |
| 点云 | PCL、`pcl_conversions` | 点云转换、KD 树、体素滤波、ICP |
| 优化 | GTSAM | iSAM2、IMU 预积分、因子图 |
| 数值/视觉 | Eigen3、OpenCV | 变换、矩阵和 LM 线性系统 |
| 并行 | OpenMP、Boost timer | 地图优化并行和计时 |
| LiDAR | `livox_ros_driver` | `CustomMsg` 编译依赖 |
| GPU | CUDA Toolkit、Thrust、cuBLAS、cuSOLVER | 独立 RBF 核矩阵和权重求解 |
| JSON | `nlohmann_json` | `elevation_rbf` 序列化代码；实际 JSON 写入被 `if (false)` 禁用 |
| 导航 | `robot_localization` | `module_navsat.launch` 中的 EKF/NavSat 节点 |
| Python | `urdfpy`、NumPy、`tf_conversions` | `rbf_cuda/script/fk.py` |
| Python | `rosbag`、`pyproj`、`tqdm` | `rbf_cuda/script/gps2tum.py` |

依赖版本没有锁定文件；除 ROS Noetic 目标环境外，精确兼容版本为 **UNKNOWN**。

# 4. Runtime Architecture（运行时架构）

## 4.1 历史综合启动入口

`LIO-SAM-MID360/launch/run_tron1a_all.launch` 执行以下操作：

1. 加载 `config/paramsLivoxIMU.yaml`。
2. 设置 `lio_sam/saveTUMTrajectoryPath`。
3. 通过 `launch/include/module_loam.launch` 启动四个主节点。
4. 尝试加载已经删除的 Tron1A `robot_description`，因此当前检出会在此处遇到缺失文件。
5. 可选启动 `joint_state_name_adapter.py` 和 `robot_state_publisher`。
6. 可选包含 `cuda_rbf/launch/run_rbf.launch`。
7. 可选启动 RViz。

## 4.2 主节点

| ROS 可执行文件 | 主要类 | 输入 | 主要输出 |
| --- | --- | --- | --- |
| `lio_sam_imageProjection` | `ImageProjection` | Livox、IMU、高频 IMU 里程计、LiDAR z 偏移 | 去畸变点云、deskew `cloud_info` |
| `lio_sam_featureExtraction` | `FeatureExtraction` | deskew `cloud_info` | 角点、平面点、feature `cloud_info` |
| `lio_sam_mapOptmization` | `mapOptimization` | feature `cloud_info`、GPS、外部回环、关节状态 | 地图里程计、地图、路径、RBF 统计、z 偏移 |
| `lio_sam_imuPreintegration` | `IMUPreintegration`、`TransformFusion` | IMU、地图校正、高频 IMU 里程计 | IMU 增量里程计、融合里程计、IMU 路径、TF |

`mapOptimization::main()` 还启动回环线程 `loopClosureThread()` 和全局地图可视化线程 `visualizeGlobalMapThread()`。

## 4.3 可选/辅助节点

| 节点 | 启动条件 | 作用 |
| --- | --- | --- |
| `elevation_rbf` | `with_rbf:=true` | 独立 CUDA RBF 拟合和 `/rbf_elevation_map` 发布 |
| `legged_fk` (`fk.py`) | 随 `run_rbf.launch` 启动 | 从 URDF 和 `/joint_states` 发布轮子 TF |
| `joint_state_name_adapter` | `with_robot_tf:=true` | 将 `joint_0...joint_7` 映射为 Tron1A 具名关节 |
| `robot_state_publisher` | `with_robot_tf:=true` | 发布机器人 link TF |
| `ekf_gps`、`navsat` | 仅由部分通用 launch 包含 | `robot_localization` GPS 辅助链 |

`with_rbf` 只控制独立 `elevation_rbf` 路径。主地图优化中的轮地约束由参数 `lio_sam/enableRbfConstraint` 控制。

# 5. Sensor Data Flow（传感器数据流）

## 5.1 LiDAR

```text
pointCloudTopic (Livox CustomMsg)
  -> ImageProjection::cloudHandler()
  -> cachePointCloud()
  -> moveFromCustomMsg()
  -> deskewInfo()/projectPointCloud()/cloudExtraction()
  -> lio_sam/deskew/cloud_info
  -> FeatureExtraction::laserCloudInfoHandler()
  -> lio_sam/feature/cloud_info
  -> mapOptimization::laserCloudInfoHandler()
  -> scan2MapOptimization()
  -> mapping odometry / keyframes / map
```

`ImageProjection` 缓存两帧后处理队首扫描。`moveFromCustomMsg()` 可应用 `pointCloudRot`、`pointCloudTrans` 和动态 z 偏移。代码只接受 Livox `CustomMsg` 路径；非 `SensorType::LIVOX` 会报错并关闭节点。

扫描去畸变使用 IMU 积分旋转。`ImageProjection::findPosition()` 当前将位置增量设为零，平移去畸变未接通。

## 5.2 IMU

IMU 同时进入两条路径：

```text
imuTopic
  -> ImageProjection::imuHandler()
  -> imuDeskewInfo()
  -> 扫描旋转去畸变和初始 RPY
```

```text
imuTopic
  -> IMUPreintegration::imuHandler()
  -> imuConverter()
  -> GTSAM PreintegratedImuMeasurements
  -> odomTopic + "_incremental"
  -> ImageProjection 和 TransformFusion
```

`IMUPreintegration::odometryHandler()` 接收 `lio_sam/mapping/odometry_incremental` 作为低频校正，估计 `X`（Pose3）、`V`（速度）和 `B`（IMU 偏置），然后重新传播尚未消费的高频 IMU。

## 5.3 其他实际输入

| 输入 | Topic | Callback/消费者 | 作用 |
| --- | --- | --- | --- |
| 关节编码器 | `jointStateTopic`，主配置为 `/joint_states` | `JointStateWheelKinematics::jointStateHandler()` | 计算左右轮轮心和轮轴 |
| GPS 里程计 | `gpsTopic` | `mapOptimization::gpsHandler()` | 满足协方差和运动条件时添加 GPS 因子 |
| 外部回环提示 | `lio_loop/loop_closure_detection` | `mapOptimization::loopInfoHandler()` | 提供外部回环候选 |
| 内部回环 | 当前关键帧地图 | `performLoopClosure()` | ICP 验证并添加回环因子 |
| LiDAR z 校准反馈 | `lio_sam/lidar_calib/z_offset` | `ImageProjection::lidarCalibOffsetHandler()` | 修改后续点云 z 坐标 |

GPS 原始接收机、关节硬件驱动和外部回环提示生产者不在仓库中，其实际来源为 **UNKNOWN**。

## 5.4 M20 测试输入（尚未接入）

| 输入 | 已确认接口 | 测试数据特征 | 当前主代码状态 |
| --- | --- | --- | --- |
| 前 Airy 点云 | `/rslidar_points_front`, `sensor_msgs/PointCloud2` | 约 10 Hz；96 rings；字段 `x/y/z/intensity/ring/timestamp`；每点时间为绝对时间 | `ImageProjection` 只接收 Livox `CustomMsg`，待适配 |
| 前 Airy IMU | `/rslidar_imu_data_front`, `sensor_msgs/Imu` | 约 200 Hz；orientation 未提供；加速度量级约 `1 g` | 消息类型可复用，但外参仍待确认 |
| M20 关节 | `/JOINTS_DATA`, M20 自定义消息 | 表面约 500 Hz；测试桥实际约 10 Hz 新样本并重复发布；若干关节值超出 URDF 限位 | 主代码只订阅 `sensor_msgs/JointState` 且硬编码两轮运动学，待适配 |
| 后 Airy 点云/IMU | 对应 `_rear` topic | bag 中存在 | 第一阶段明确不使用 |

`/IMU` 机体自定义消息在测试 bag 中存在，但其有效更新频率同样约 10 Hz，且字段是
桥接后的 RPY/速度等派生量，不是当前 GTSAM 入口要求的原始 `sensor_msgs/Imu`。
第一阶段估计按当前约定使用前 Airy IMU，而不是该 `/IMU` 消息。

# 6. Core Estimation Pipeline（核心估计流程）

## 6.1 点云预处理

入口：`ImageProjection::cloudHandler()`，文件 `LIO-SAM-MID360/src/imageProjection.cpp`。

1. `cachePointCloud()` 缓存和取出扫描，确定 `timeScanCur/timeScanEnd`。
2. `moveFromCustomMsg()` 将 Livox 点转换为内部 `PointXYZIRT`。
3. `deskewInfo()` 检查扫描时间范围内是否存在完整 IMU 数据。
4. `imuDeskewInfo()` 积分角速度并生成按时间查询的旋转数组。
5. `odomDeskewInfo()` 提取增量里程计初值，写入 `cloud_info.initialGuess*`。
6. `projectPointCloud()` 按 Livox line/ring 组织距离图并执行旋转去畸变。
7. `cloudExtraction()` 输出紧凑点云、ring 起止索引、列号和距离。

## 6.2 特征提取

入口：`FeatureExtraction::laserCloudInfoHandler()`，文件 `LIO-SAM-MID360/src/featureExtraction.cpp`。

- `calculateSmoothness()` 根据相邻距离计算曲率。
- `markOccludedPoints()` 标记遮挡和近似平行光束点。
- `extractFeatures()` 将每条 scan 分为六段，选择高曲率角点和低曲率平面点。
- 平面点按 `odometrySurfLeafSize` 体素下采样。
- 结果写回 `lio_sam/feature/cloud_info`。

## 6.3 扫描到地图匹配

入口：`mapOptimization::laserCloudInfoHandler()`，文件 `LIO-SAM-MID360/src/mapOptmization.cpp`。

单帧处理顺序为：

```text
updateInitialGuess()
-> extractSurroundingKeyFrames()
-> downsampleCurrentScan()
-> scan2MapOptimization()
-> saveKeyFramesAndFactor()
-> correctPoses()
-> publishOdometry()
-> publishFrames()
```

`scan2MapOptimization()` 最多迭代 25 次：

- `cornerOptimization()` 建立当前角点到地图线特征的残差。
- `surfOptimization()` 建立当前平面点到地图平面的残差。
- `combineOptimizationCoeffs()` 合并 OpenMP 工作结果。
- `LMOptimization()` 组装六自由度线性系统，追加 RBF 轮地约束，使用 OpenCV QR 求解。
- 首次迭代检查特征值并对退化方向投影。
- `transformUpdate()` 融合 IMU roll/pitch 并限制位姿范围。

## 6.4 关键帧与地图

`saveFrame()` 使用平移和旋转阈值决定是否新增关键帧。新增关键帧时：

- `addOdomFactor()` 添加首帧先验或相邻关键帧 BetweenFactor。
- `addGPSFactor()` 按队列时间、GPS 协方差、当前位姿协方差和运动距离筛选 GPS。
- `addLoopFactor()` 消费已经通过 ICP 或外部提示生成的回环约束。
- `saveKeyFramesAndFactor()` 调用 iSAM2 更新，并保存角点/平面点关键帧。
- `correctPoses()` 在回环发生后重写所有关键帧位姿和路径。

## 6.5 IMU 预积分与融合

`IMUPreintegration` 维护两个预积分器：

- `imuIntegratorOpt_`：在相邻地图校正之间建立 `ImuFactor`、偏置 BetweenFactor 和校正 Pose PriorFactor。
- `imuIntegratorImu_`：使用最新优化状态和偏置进行高频传播并发布增量里程计。

图键达到 100 时，代码读取最新 Pose、Velocity、Bias 的边缘协方差，重建较小的 iSAM2 图。速度或偏置超过配置阈值时调用 `resetParams()`。

`TransformFusion` 将低频 `lio_sam/mapping/odometry` 与高频 `odomTopic+"_incremental"` 的相对增量组合，发布 `odomTopic`。

# 7. RBF Pipeline（RBF 处理链）

## 7.1 主估计器轮地约束

核心接口位于 `LIO-SAM-MID360/include/rbf_lm_interface.h`：

- `RbfCudaRequest`：当前位姿、当前 LiDAR 特征采样、RBF 节点、核宽度、轮半径、权重和残差截断阈值。
- `RobotKinematicsModel`：由位姿和关节状态计算车轮几何。
- `RbfCudaGradientInterface`：输出 `RbfLmConstraint` 列表。
- `RbfLmConstraint`：一个标量残差、六维位姿 Jacobian 和权重。

调用顺序：

```text
mapOptimization::LMOptimization()
  -> updateRbfLmConstraints()
  -> JointStateWheelKinematics::computeWheelGeometry()
  -> CudaWheelRbfGradient::computeGradient()  [当前为 CPU]
  -> evaluateResidualVector()
  -> solveContactOnWheel()
  -> evaluateTerrain()
  -> RbfLmConstraint rows
  -> 追加到 LOAM matA/matB
```

### 输入

- `request.pose`：`transformTobeMapped[roll,pitch,yaw,x,y,z]`。
- `request.points_lidar`：当前降采样平面点，随后补充角点，受 `rbfConstraintMaxPoints` 限制。
- `request.rbf_nodes`：局部地图降采样平面点，受 `rbfNodeMaxNum` 限制。
- 两轮几何：由 `/joint_states` 和 Tron1A 硬编码运动学计算。

### 当前实际地形求值

`evaluateTerrain()` 遍历 `request.rbf_nodes`，使用节点 `(x,y)` 作为高斯中心，并直接将节点 `z` 作为核系数：

```text
h(x,y) = sum(node.z * gaussian(x-node.x, y-node.y))
```

`request.points_lidar` 在 `CudaWheelRbfGradient` 中没有被读取。它目前只在 `updateRbfLmConstraints()` 中用于非空检查和请求填充。

### 输出与最终估计

每个车轮输出六个残差行：

1. `r1 = contact_z - terrain_height`。
2. `r2 = wheel_axis dot (contact - wheel_center)`。
3. `r3 = squared_distance(contact, center) - wheel_radius^2`。
4. `r4 = wheel_radial cross terrain_normal` 的三个分量。

接触点由 `solveContactOnWheel()` 内最多八次局部 Gauss-Newton 求解。`computeGradient()` 对六个位姿变量执行中心有限差分。`LMOptimization()` 将加权残差和 Jacobian 直接追加到扫描匹配矩阵，因此这条路径会改变最终 `transformTobeMapped`。

## 7.2 自适应 LiDAR z 偏移

`updateRbfLmConstraints()` 统计轮地残差均值、绝对均值和各残差块均值，发布 `lio_sam/rbf/residual_stats`。若 `enableLidarZCalibration=true`，残差均值经过 EMA 和 deadband 后更新 `lidarZOffsetAdaptive`，发布到 `lio_sam/lidar_calib/z_offset`。`ImageProjection` 将该偏移加到后续每个点的 z 坐标。

## 7.3 独立 CUDA RBF 拟合

入口为 `rbf_cuda/src/rbf/elevation_rbf.cpp`：

```text
lio_sam/mapping/map_local
  -> localMapCallback()
lio_sam/mapping/odometry_incremental
  -> odomCallback()
  -> VoxelGridRBF::setCenter()/setInputCloud()/generateRBFGrid()
  -> VoxelGridRBF::initialWeightCuda()
  -> RBFCudaCore::calKernelMatrix()
  -> RBFCudaCore::initialWeightKernel()
  -> compute_error()/visualize()
  -> /rbf_elevation_map + timing/MAE files
```

该节点在平面位移累计超过 `move_threshold=0.5 m` 后触发拟合。`VoxelGridRBF::updateWeightCuda()` 和 `RBFCudaCore::updateWeightKernel()` 已实现递归式更新接口，但当前活动调用点使用 `initialWeightCuda()`，没有找到活动的 `updateWeightCuda()` 调用。

## 7.4 CPU/CUDA 边界

| 功能 | 当前执行位置 | 是否进入主估计 |
| --- | --- | --- |
| 轮地接触点求解 | CPU，`rbf_cuda_wheel_gradient.cpp` | 是 |
| 轮地位姿 Jacobian | CPU 中心有限差分 | 是 |
| `ComputeWheelResidualJacobianCuda()` | CUDA 文件中已实现 | 否，未找到调用点 |
| RBF 核矩阵 | `RBFCudaCore`，Thrust/CUDA | 仅独立高程节点 |
| 初始岭回归权重 | cuBLAS/cuSOLVER | 仅独立高程节点 |
| 递归权重更新 | CUDA 接口已实现 | 当前没有活动调用者 |
| RBF 高程可视化 | CUDA/Thrust | 仅发布 `/rbf_elevation_map` |

# 8. Coordinate Frames（坐标系）

## 8.1 配置帧

`ParamServer` 从 YAML 加载：

| 参数 | 主配置默认值 | 用途 |
| --- | --- | --- |
| `lidarFrame` | `base_link` | 去畸变/特征点云 frame，地图 TF 子帧 |
| `baselinkFrame` | `base_link` | 机器人基座帧 |
| `odometryFrame` | `odom` | 地图优化结果和地图点云的 frame |
| `mapFrame` | `map` | `TransformFusion` 发布的全局父帧 |

当前主配置令 `lidarFrame == baselinkFrame`。若两者不同，`TransformFusion` 会查询静态 `lidarFrame -> baselinkFrame` 变换并发布 `odometryFrame -> baselinkFrame`。

## 8.2 TF 发布

- `TransformFusion::imuOdometryHandler()` 发布恒等 `mapFrame -> odometryFrame`。
- `mapOptimization::publishOdometry()` 发布 `odometryFrame -> lidarFrame`。
- 当 `lidarFrame != baselinkFrame` 时，`TransformFusion` 另行发布 `odometryFrame -> baselinkFrame`。
- `robot_state_publisher` 根据 URDF 和适配后的关节状态发布机器人 link TF。
- `rbf_cuda/script/fk.py` 直接发布 URDF base link 到 `wheel_R_Link`、`wheel_L_Link` 的 TF。

`fk.py` 发布的轮 TF不被 `JointStateWheelKinematics` 消费；主轮地约束直接订阅关节状态并自行计算几何。

## 8.3 IMU/LiDAR 外参

`extrinsicRot` 用于旋转加速度和角速度，`extrinsicRPY` 用于 IMU 姿态变换，`extrinsicTrans` 用于 `lidar2Imu/imu2Lidar` 的平移 Pose3。相关代码在 `ParamServer::imuConverter()` 和 `IMUPreintegration` 构造函数中。

`pointCloudTransformEnable`、`pointCloudRot`、`pointCloudTrans` 是另一组直接作用于 Livox 点坐标的变换。主配置中它被启用，并设置 `pointCloudTrans.z=0.15`。

当 `lidarFrame` 不等于 `base_link` 时，硬编码轮腿运动学的位姿究竟应解释为 LiDAR 位姿还是 base 位姿，代码没有显式转换，标记为 **UNKNOWN/TODO**。

## 8.4 M20 与 Airy 坐标资料

M20 URDF 以 `base_link` 的 `+X` 前、`+Y` 左、`+Z` 上为模型约定。URDF 中前后
LiDAR 固定关节分别为：

| frame | parent | URDF xyz | URDF rpy | 来源 |
| --- | --- | --- | --- | --- |
| `front_lidar_link` | `base_link` | `[0.32028, 0, -0.013]` | `[0, 0, 0]` | `robot-description/M20/urdf/M20.urdf` |
| `rear_lidar_link` | `base_link` | `[-0.32028, 0, -0.013]` | `[0, 0, 0]` | 同上 |

Airy 驱动配置却对前雷达设置 `[0.0501, 0, 0.739]`、pitch `+1.570795`，对后雷达
设置 `[-0.32028, 0, -0.013]`、pitch `-1.57079`；两路消息 frame 分别写为
`rslidar_front` 和 `rslidar_rear`。这些数值与 URDF 不能直接视为同一变换。驱动是否
以 `ENABLE_TRANSFORM=ON` 构建、点云是否已在驱动内变换，以及变换目标 frame 的
精确定义均为 **UNKNOWN**。

此外，当前资料未包含前 Airy 单机 DIFOP 中 LiDAR 与内置 IMU 的工厂旋转/平移
标定。启动日志只打印端口和通用变换，不打印该内部标定。迁移时必须保证点云和 IMU
的坐标转换各执行一次，且最终坐标约定一致。

# 9. Configuration System（配置系统）

## 9.1 参数加载

所有主节点继承 `LIO-SAM-MID360/include/utility.h` 中的 `ParamServer`。构造节点时从全局 ROS 参数服务器加载 `lio_sam/*` 参数。

| 配置文件 | 用途 | RBF 主约束默认值 |
| --- | --- | --- |
| `LIO-SAM-MID360/config/paramsLivoxIMU.yaml` | Tron1A/MID360 六轴 IMU 主配置 | `true` |
| `LIO-SAM-MID360/config/params9axisIMU.yaml` | 九轴 IMU 配置 | `false` |
| `LIO-SAM-MID360/config/params.yaml` | 通用 MID360 配置 | `false` |
| `rbf_cuda/config/params.yaml` | 旧版 Velodyne/独立 RBF 包配置 | 不包含主约束参数 |
| `support/rslidar_ros2_ws/config/m20_airy.yaml` | 双 Airy ROS 2 驱动参考配置 | 不由当前 ROS 1 主节点加载 |

## 9.2 主要参数组

| 参数组 | 代表参数 | 主要消费者 |
| --- | --- | --- |
| Topic | `pointCloudTopic`、`imuTopic`、`odomTopic`、`gpsTopic` | 所有主节点 |
| Frame | `lidarFrame`、`baselinkFrame`、`odometryFrame`、`mapFrame` | 发布、TF、消息头 |
| LiDAR | `sensor`、`N_SCAN`、`Horizon_SCAN`、距离范围 | `ImageProjection` |
| IMU | `imuType`、噪声、偏置随机游走、重力、外参 | `imuConverter`、`IMUPreintegration` |
| 点云变换 | `pointCloudTransformEnable/Rot/Trans` | `moveFromCustomMsg()` |
| 特征 | `edgeThreshold`、`surfThreshold`、最少有效特征数 | `FeatureExtraction`、地图优化 |
| 地图 | 体素尺寸、处理间隔、局部关键帧半径 | `mapOptimization` |
| RBF 约束 | `enableRbfConstraint`、核宽、节点/点数限制 | `updateRbfLmConstraints()` |
| 轮地残差 | `rbfWeightR1...R4`、截断阈值 | CPU 轮地梯度实现 |
| 运动学 | `jointStateTopic`、`wheelRadius` | `JointStateWheelKinematics` |
| z 校准 | 初值、上下限、学习率、EMA、deadband | `mapOptimization`、`ImageProjection` |
| 关键帧/回环 | 新增阈值、搜索半径、ICP fitness | `mapOptimization` |
| 输出 | `savePCD*`、`saveTUMTrajectoryPath` | 地图服务、轨迹记录 |

## 9.3 Launch 文件

| Launch | 加载配置 | 说明 |
| --- | --- | --- |
| `run_tron1a_all.launch` | `paramsLivoxIMU.yaml` | 历史综合入口；Tron1A 模型引用已失效 |
| `run6axis.launch` | `paramsLivoxIMU.yaml` | 六轴 IMU，包含通用 robot state publisher |
| `run9axis.launch` | `params9axisIMU.yaml` | 九轴 IMU |
| `run.launch` | `params.yaml` | 通用入口，并启用 NavSat 模块 |
| `rbf_cuda/launch/run_rbf.launch` | 使用代码默认 `rbf/*` 参数 | 独立高程拟合和 FK |
| `rbf_cuda/launch/run.launch` | `rbf_cuda/config/params.yaml` | 旧版五节点处理链 |

多个 launch 和脚本包含 `/root/...` 绝对路径。这些默认路径是否对应当前实验环境为 **UNKNOWN**，运行时应显式覆盖。

# 10. Outputs（输出）

## 10.1 ROS Topics

| Topic | 类型 | 发布者 | 含义 |
| --- | --- | --- | --- |
| `lio_sam/deskew/cloud_deskewed` | `sensor_msgs/PointCloud2` | ImageProjection | 去畸变点云 |
| `lio_sam/deskew/cloud_info` | `lio_sam/cloud_info` | ImageProjection | 去畸变点云及 IMU/里程计初值 |
| `lio_sam/feature/cloud_corner` | `PointCloud2` | FeatureExtraction | 边缘特征 |
| `lio_sam/feature/cloud_surface` | `PointCloud2` | FeatureExtraction | 平面特征 |
| `lio_sam/feature/cloud_info` | `cloud_info` | FeatureExtraction | 地图优化输入 |
| `lio_sam/mapping/odometry` | `nav_msgs/Odometry` | mapOptimization | 低频地图优化里程计 |
| `lio_sam/mapping/odometry_incremental` | `Odometry` | mapOptimization | 供 IMU 预积分校正的增量结果 |
| `odomTopic+"_incremental"` | `Odometry` | IMUPreintegration | 高频 IMU 传播里程计 |
| `odomTopic` | `Odometry` | TransformFusion | 地图校正与高频 IMU 融合结果 |
| `lio_sam/mapping/path` | `nav_msgs/Path` | mapOptimization | 关键帧路径 |
| `lio_sam/imu/path` | `Path` | TransformFusion | 最近一段高频 IMU 路径 |
| `lio_sam/mapping/map_local` | `PointCloud2` | mapOptimization | 降采样局部平面地图 |
| `lio_sam/mapping/map_global` | `PointCloud2` | mapOptimization | 全局可视化地图 |
| `lio_sam/mapping/trajectory` | `PointCloud2` | mapOptimization | 关键帧位置点云 |
| `lio_sam/mapping/cloud_registered` | `PointCloud2` | mapOptimization | 配准后的特征点云 |
| `lio_sam/mapping/cloud_registered_raw` | `PointCloud2` | mapOptimization | 配准后的原始去畸变点云 |
| `lio_sam/mapping/slam_info` | `cloud_info` | mapOptimization | 关键帧/局部图信息 |
| `lio_sam/rbf/residual_stats` | `Float64MultiArray` | mapOptimization | RBF 残差和 z 校准统计 |
| `lio_sam/lidar_calib/z_offset` | `Float64` | mapOptimization | 动态点云 z 偏移 |
| `/rbf_elevation_map` | `PointCloud2` | elevation_rbf | 独立 CUDA RBF 高程可视化 |

回环历史点云、校正点云和 MarkerArray 也由 `mapOptimization` 发布，名称见其构造函数。

## 10.2 TF

- `map -> odom`：恒等变换，由 `TransformFusion` 发布。
- `odom -> lidarFrame`：地图优化位姿。
- `odom -> baselinkFrame`：仅在 LiDAR/base 帧不同时由融合节点发布。
- 机器人 link TF：由 `robot_state_publisher` 或 `fk.py` 发布。

## 10.3 文件输出

| 输出 | 产生位置 | 触发方式 |
| --- | --- | --- |
| TUM 轨迹 | `saveTUMTrajectoryPath` | 路径参数非空时，`publishOdometry()` 每帧写入 |
| `trajectory.pcd`、`transformations.pcd` | 地图保存目录 | 调用 `lio_sam/save_map` |
| `CornerMap.pcd`、`SurfMap.pcd`、`GlobalMap.pcd` | 地图保存目录 | 调用 `lio_sam/save_map` |
| `kernel_matrix_time.txt` | `elevation_rbf` 的 `file_path` | 节点启动后打开，触发拟合时写入 |
| `weight_time.txt` | 同上 | 记录权重求解耗时 |
| `mae.txt` | 同上 | 记录 RBF 拟合 MAE |
| GPS TUM | `gps2tum.py` 硬编码路径 | 离线运行脚本 |
| Odometry TUM | `odom_recorder.py` 硬编码路径 | 运行记录节点 |

`elevation_rbf.cpp` 会创建硬编码 JSON 目录并构造 JSON 内容，但实际写文件代码被 `if (false)` 包裹，因此当前不会写出 JSON。

# 11. Module Dependency Map（模块依赖图）

```mermaid
flowchart LR
    Livox[Livox CustomMsg] --> IP[ImageProjection]
    IMU[IMU] --> IP
    IMU --> PI[IMUPreintegration]
    PI -->|odomTopic_incremental| IP
    IP -->|deskew cloud_info| FE[FeatureExtraction]
    FE -->|feature cloud_info| MO[mapOptimization]

    GPS[GPS Odometry] --> MO
    Loop[External Loop Hint] --> MO
    Joint[JointState] --> Kin[JointStateWheelKinematics]
    Kin --> CpuRbf[CPU Wheel/RBF Gradient]
    CpuRbf --> MO

    MO -->|mapping correction| PI
    MO --> Graph[iSAM2 Keyframe Graph]
    Graph --> MO
    MO --> Odom[Odometry / Path / Map]
    MO -->|z offset| IP

    MO -->|map_local| Elev[elevation_rbf]
    MO -->|odometry_incremental| Elev
    Elev --> Voxel[VoxelGridRBF]
    Voxel --> Cuda[RBFCudaCore: CUDA/Thrust/cuBLAS/cuSOLVER]
    Cuda --> ElevMap[/rbf_elevation_map + stats]

    Robot[robot-description] --> RSP[robot_state_publisher]
    Robot --> FK[fk.py]

    WheelCuda[Wheel CUDA Kernel] -. compiled but no caller .-> MO
```

# 12. End-to-End Data Flow（端到端数据流）

```mermaid
flowchart TD
    L[Livox MID360 CustomMsg] --> T[点云固定变换 + 动态 z 偏移]
    T --> Q[两帧扫描缓存]
    I[IMU] --> IC[imuConverter]
    IC --> D[扫描旋转去畸变]
    Q --> D
    D --> RI[距离图组织]
    RI --> F[边缘/平面特征提取]

    I --> P[IMU 预积分与高频传播]
    P --> G[扫描初始位姿]
    G --> S[扫描到局部地图匹配]
    F --> S

    J[/joint_states] --> K[Tron1A 两轮运动学]
    K --> C[轮地接触点 + RBF 残差/Jacobian]
    M[局部平面地图点] --> C
    C --> LM[LOAM + 轮地约束六自由度 LM]
    S --> LM

    LM --> KF[关键帧选择]
    KF --> FG[iSAM2: Odom/GPS/Loop Factors]
    GPS[GPS Odometry] --> FG
    LOOP[Loop Closure] --> FG
    FG --> POSE[优化位姿]
    POSE --> OUT[Odometry / Path / Map / TUM]
    POSE --> CORR[地图校正]
    CORR --> P

    C --> STAT[RBF residual stats]
    STAT --> Z[自适应 LiDAR z offset]
    Z --> T

    M --> ERBF[独立 elevation_rbf]
    POSE --> ERBF
    ERBF --> GPU[CUDA RBF 拟合]
    GPU --> VIS[/rbf_elevation_map + timing/MAE]
```

# 13. Key Source Files（关键源文件）

| 文件 | 主要类/函数 | 作用 | 被谁调用/启动 |
| --- | --- | --- | --- |
| `LIO-SAM-MID360/include/utility.h` | `ParamServer`、`imuConverter()` | 参数、外参、公共点云/IMU工具 | 四个主节点继承/调用 |
| `LIO-SAM-MID360/src/imageProjection.cpp` | `ImageProjection`、`cloudHandler()` | Livox 转换、去畸变、距离图 | `lio_sam_imageProjection` |
| `LIO-SAM-MID360/src/featureExtraction.cpp` | `FeatureExtraction`、`extractFeatures()` | LOAM 特征提取 | `lio_sam_featureExtraction` |
| `LIO-SAM-MID360/src/mapOptmization.cpp` | `mapOptimization`、`scan2MapOptimization()` | 扫描匹配、因子图、地图、回环 | `lio_sam_mapOptmization` |
| `LIO-SAM-MID360/src/mapOptmization.cpp` | `updateRbfLmConstraints()`、`LMOptimization()` | 将轮地约束注入最终位姿优化 | `scan2MapOptimization()` |
| `LIO-SAM-MID360/src/imuPreintegration.cpp` | `IMUPreintegration` | GTSAM IMU 预积分 | `lio_sam_imuPreintegration` |
| `LIO-SAM-MID360/src/imuPreintegration.cpp` | `TransformFusion` | 低频地图里程计与高频 IMU 融合 | 同一进程中的第二个对象 |
| `LIO-SAM-MID360/include/rbf_lm_interface.h` | `RbfCudaRequest`、`RbfCudaGradientInterface` | 主 RBF/LM 抽象契约 | mapOptimization、CPU 实现 |
| `LIO-SAM-MID360/src/rbf_wheel_kinematics.cpp` | `JointStateWheelKinematics` | 关节状态到两轮几何 | mapOptimization 构造 |
| `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp` | `CudaWheelRbfGradient` | CPU 接触求解、残差和数值 Jacobian | `updateRbfLmConstraints()` |
| `LIO-SAM-MID360/src/cuda/rbf_cuda_wheel_gradient.cu` | `ComputeWheelResidualJacobianCuda()` | CUDA 高度残差/Jacobian 核 | 当前调用者 **UNKNOWN/不存在** |
| `rbf_cuda/include/rbf_fitting/rbf_fit.h` | `VoxelGridRBF` | ROI、中心管理、拟合和可视化封装 | elevation_rbf、debug_rbf、旧 lidar odometry |
| `rbf_cuda/src/cuda/rbf_cuda.cu` | `RBFCudaCore` | GPU 核矩阵、权重、协方差和可视化 | `VoxelGridRBF` |
| `rbf_cuda/src/rbf/elevation_rbf.cpp` | `odomCallback()` | 独立 RBF 拟合、发布和统计 | `cuda_rbf/elevation_rbf` |
| `rbf_cuda/src/lio_sam/lidar_odometry.cpp` | `NDTOdometry` | 实验性局部地图/RBF 节点；NDT/ICP 主体已注释 | 仅旧 `rbf_cuda/run.launch` |
| `LIO-SAM-MID360/scripts/joint_state_name_adapter.py` | `JointStateNameAdapter` | 通用关节名到 Tron1A 名称 | `with_robot_tf:=true` |
| `rbf_cuda/script/fk.py` | `ForwardKinematicsTFBroadcaster` | URDF 正运动学和轮 TF | `run_rbf.launch` |
| `LIO-SAM-MID360/msg/cloud_info.msg` | 消息定义 | deskew、特征、初值和关键帧载体 | 三个点云主节点 |
| `LIO-SAM-MID360/launch/run_tron1a_all.launch` | 综合启动配置 | 主程序入口 | `roslaunch` |
| `robot-description/M20/urdf/M20.urdf` | M20 links/joints/limits | 四轮运动学和传感器安装参考 | 当前尚无主代码调用 |
| `support/rslidar_ros2_ws/config/m20_airy.yaml` | 双 Airy 网络、topic、时间戳和驱动变换 | ROS 2 驱动参考配置 | 通过 `start.py` 的 `config_path` 加载 |
| `support/rslidar_ros2_ws/src/rslidar_sdk/launch/start.py` | ROS 2 launch | 启动 SDK 节点和 RViz2 | `ros2 launch rslidar_sdk start.py` |

# 14. Known Ambiguities（已知歧义与待确认项）

1. **UNKNOWN：实际构建状态。** 当前分析没有在 ROS Noetic、GTSAM、Livox 和 CUDA 环境中执行完整编译。
2. **UNKNOWN：根 CMake 历史。** 根 `CMakeLists.txt` 很像被复制为普通文本的 Catkin 符号链接目标，但仓库无法证明其原始文件类型。
3. **TODO：M20 输入尚未接入。** 测试 bag 和 Airy topics 已确认，但主点云入口仍只接受 Livox `CustomMsg`，关节入口仍是 Tron1A 两轮 `JointState`。
4. **UNKNOWN：外部回环生产者。** 仓库订阅 `lio_loop/loop_closure_detection`，但不包含发布该话题的实现。
5. **UNKNOWN：GPS 来源。** 仓库包含 NavSat launch 和 GPS 因子，但没有硬件驱动、datum 或数据集级配置。
6. **TODO：主 RBF 地形语义。** 主求值器将局部地图点 `z` 当作高斯核系数，但这些点不是 `VoxelGridRBF` 输出的中心/权重。该行为是否为最终设计无法确认。
7. **TODO：`points_lidar` 未使用。** 主请求会采样当前帧特征，但 CPU 梯度实现不读取这些点。
8. **TODO：轮地 CUDA 接口未接通。** `ComputeWheelResidualJacobianCuda()` 已编译但没有运行时调用点；主约束为 CPU 数值差分。
9. **TODO：递归权重更新未进入活动链。** `updateWeightCuda()` 已实现，但 `elevation_rbf` 每次调用 `initialWeightCuda()`。
10. **TODO：论文联合优化未在代码中等价实现。** 当前代码先局部求接触点，再有限差分位姿；没有在同一优化变量中联合求解位姿和显式接触点。
11. **UNKNOWN：非默认帧配置。** 当 LiDAR 帧与 base 帧不同时，硬编码轮运动学和地图位姿之间缺少显式转换说明。
12. **TODO：M20 关节值约定。** 测试桥直接转发 APDU 数值，当前 bag 的若干值超出 URDF 限位。APDU 到 URDF 关节的符号、零偏和顺序转换尚未由可执行代码确认；最终实现必须以 URDF 为准。
13. **UNKNOWN：实验精度复现。** 仓库已有接口测试 bag，但没有 ground truth、ATE/RTE 计算、轨迹对齐或论文绘图脚本。
14. **TODO：硬编码路径。** 多个 launch 和脚本使用 `/root/...` 路径，未参数化的脚本不能直接移植。
15. **TODO：辅助脚本安装。** `fk.py` 被 launch 直接引用，但 `rbf_cuda/CMakeLists.txt` 没有对应 `catkin_install_python()`；安装空间中的可发现性未经验证。
16. **TODO：机器人描述构建与 launch。** `robot-description/CMakeLists.txt` 仍安装已经不存在的 `pointfoot/wheellegged`，且 `run_tron1a_all.launch` 仍引用已删除的 Tron1A URDF；M20 目录尚未接入安装和主 launch。
17. **UNKNOWN：测试覆盖。** 当前仓库没有发现自动化单元测试、集成测试或 rosbag 回归测试。
18. **UNKNOWN：Airy 内部 IMU 标定。** 当前配置和启动日志均未给出前 Airy DIFOP 中的 LiDAR-IMU 工厂旋转/平移参数。
19. **UNKNOWN：驱动侧坐标变换是否生效。** 配置和日志打印了变换参数，但仓库缺少实际构建缓存/完整构建文件，不能确认 `ENABLE_TRANSFORM` 状态；其数值也与 M20 URDF 不一致。
20. **TODO：真实 500 Hz 关节链。** `support/m20_udp_bridge.zip` 和测试 bag 只提供重复发布方案；GOS 直接 500 Hz 的最终消息生产者尚未进入仓库。
