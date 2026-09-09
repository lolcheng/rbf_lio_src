# RBF-LIO 实验与复现手册

本文只记录当前仓库中的代码、配置、launch、脚本、README 和 `support/RBF_LIO.pdf` 能够支持的实验流程。命令中的 `<...>` 是必须由复现者提供的占位符；仓库无法确认的内容统一标记为 **UNKNOWN**。

> 事实边界：仓库已有 `data/rosbag/m20_upstairs.bag`，当前 ROS 1 主代码已增加 Airy `PointCloud2` 和 M20 自定义关节消息适配，但尚未在 Linux/ROS 环境中完成编译和整包回放。仓库能够生成 RBF-LIO 里程计、TUM 轨迹、PCD 地图和独立 RBF 节点的耗时/MAE 日志，但没有 ground truth、ATE、z-ATE、RTE 计算脚本，也没有论文表格或绘图脚本。因此，从现有 M20 bag 到论文指标的链路仍不完整。

# 1. Experimental Pipeline

## 1.1 当前仓库可确认的处理链

```text
历史 MID360 dataset / rosbag（文件位置 UNKNOWN）
  |  Livox CustomMsg + sensor_msgs/Imu + sensor_msgs/JointState
  v
run_tron1a_all.launch
  |-- paramsLivoxIMU.yaml
  |-- imageProjection -> featureExtraction -> mapOptmization
  |-- imuPreintegration
  `-- 可选 elevation_rbf（独立 CUDA 地形拟合/统计节点）
  |
  |-- lio_sam/mapping/odometry
  |-- trajectory_tum.txt
  |-- PCD map（通过 lio_sam/save_map 或退出保存）
  `-- kernel_matrix_time.txt / weight_time.txt / mae.txt
  v
trajectory evaluation（仓库缺失，UNKNOWN）
  v
ATE / z-ATE / RTE / RMSE（论文报告，仓库无实现）
  v
table / figure generation（仓库缺失，UNKNOWN）
```

当前新增的 M20 测试链路为：

```text
M20 + 双 RoboSense Airy
  -> ROS 2 rslidar_sdk v1.5.19
     -> /rslidar_points_front + /rslidar_imu_data_front
  -> 测试 ROS 1 m20_udp_bridge
     -> /JOINTS_DATA
  -> data/rosbag/m20_upstairs.bag
  -> run_m20.launch + paramsM20.yaml
  -> Airy PointCloud2/IMU + M20 四轮运动学适配
  -> RBF-LIO -> trajectory/map（TODO：ROS 回放验证）
  -> ground truth/evaluation/figure（UNKNOWN/缺失）
```

M20 对应入口为 `LIO-SAM-MID360/launch/run_m20.launch`，历史 MID360/Tron1A 入口为 `run_tron1a_all.launch`。主定位器在 `LIO-SAM-MID360/src/mapOptmization.cpp` 中直接写出 TUM 轨迹；独立 RBF 节点在 `rbf_cuda/src/rbf/elevation_rbf.cpp` 中写出耗时和 MAE。

## 1.2 可复现边界

| 阶段 | 仓库状态 | 证据 |
|---|---|---|
| 数据读取 | 已实现，待运行验证 | M20 测试 bag、`airyCloudHandler()`、`m20_joint_state_adapter.py` |
| ROS 启动 | 存在 | `LIO-SAM-MID360/launch/*.launch` |
| RBF-LIO 估计 | 存在 | `LIO-SAM-MID360/src/` |
| TUM 轨迹 | 存在 | `mapOptmization::saveTUMTrajectoryToFile()` |
| GPS 转 TUM | 存在但硬编码 | `rbf_cuda/script/gps2tum.py` |
| ATE/z-ATE/RTE | **UNKNOWN / 缺失** | 未找到对应脚本或程序 |
| 论文图表 | **UNKNOWN / 缺失** | 未找到 plotting/table 脚本 |

# 2. Environment

## 2.1 仓库要求或可直接确认的环境

| 项目 | 可确认内容 | 来源 |
|---|---|---|
| 操作系统 | 项目协作说明将 Ubuntu 20.04 作为 ROS Noetic 的典型目标环境；实际生成论文结果的 OS 镜像为 **UNKNOWN** | `AGENTS.md`、根 `CMakeLists.txt` |
| ROS | ROS 1 Noetic；根 `CMakeLists.txt` 指向 `/opt/ros/noetic/share/catkin/cmake/toplevel.cmake` | 根 `CMakeLists.txt` |
| 构建系统 | Catkin / CMake | 两个 ROS 包的 `CMakeLists.txt` |
| C++ 标准 | `lio_sam` 使用 C++14；`cuda_rbf` 使用 C++17 | `LIO-SAM-MID360/CMakeLists.txt`、`rbf_cuda/CMakeLists.txt` |
| 编译器 | 支持对应 C++ 标准且兼容 ROS/CUDA 的编译器；具体 GCC 版本 **UNKNOWN** | CMake 未锁定版本 |
| CUDA | 构建 `cuda_rbf` 时必需；`lio_sam` 自身以可选方式查找 CUDA | 两个包的 `CMakeLists.txt` |
| CUDA 版本 | **UNKNOWN** | 仓库未锁定版本 |
| ROS C++ 依赖 | `roscpp`、`rospy`、`tf`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`visualization_msgs`、`pcl_conversions`、`cv_bridge`、消息生成；Livox 包为可选 | `package.xml`、CMake |
| 外部 C++ 依赖 | PCL、OpenCV、Eigen3、GTSAM、Boost、OpenMP | CMake |
| CUDA 依赖 | CUDA Toolkit、Thrust、cuBLAS、cuSOLVER | `rbf_cuda/CMakeLists.txt`、`rbf_cuda/src/cuda/` |
| 独立 RBF 节点 | `nlohmann_json` | `rbf_cuda/CMakeLists.txt` |
| GPS 转换脚本 | Python 3、ROS `rosbag`、`pyproj`、`tqdm` | `rbf_cuda/script/gps2tum.py` |
| 机器人 FK 脚本 | Python 3、`rospy`、NumPy、`urdfpy`、`tf_conversions` | `rbf_cuda/script/fk.py` |
| Airy 参考驱动 | ROS 2；现场日志确认 RSLidar SDK v1.5.19 | `support/rslidar_ros2_ws`、现场启动日志 |

`support/RBF_LIO.pdf` 报告的实验硬件为 Intel i5-12490F、32 GB RAM 和 NVIDIA RTX 3070。这是论文中的实验描述，不代表仓库在该环境上已经由本手册重新验证。GPU 驱动、CUDA Toolkit、cuDNN 和精确编译器版本均为 **UNKNOWN**。

# 3. Build Instructions

## 3.1 从空 Catkin 工作区构建

仓库根 `CMakeLists.txt` 当前是包含绝对 Catkin 路径的一行普通文本，不能可靠地作为正常 Catkin 顶层文件使用。保持仓库不变时，应在外部工作区链接三个源包：

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

命令来源：`AGENTS.md` 的构建约束，以及三个包的 `package.xml` / `CMakeLists.txt`。

## 3.2 构建前置条件

- 运行 Livox 输入时，其消息 MD5 必须与编译期选择的 `livox_ros_driver/CustomMsg.h` 或 `livox_ros_driver2/CustomMsg.h` 一致。M20/Airy 路径不要求安装 Livox 包。
- 构建完整工作区时 `cuda_rbf` 使用 `find_package(CUDA REQUIRED)`，没有 CUDA 的环境不能按上述命令完整构建。
- 仓库没有 Dockerfile、rosdep 锁文件或固定依赖版本，精确依赖安装命令为 **UNKNOWN**。
- 本仓库没有可确认的成功构建日志，因此上述命令是代码结构支持的构建方式，不代表当前机器已经编译通过。
- `robot-description/CMakeLists.txt` 已安装 `M20` 目录，不再引用已删除的 Tron1A 模型目录。
- `support/rslidar_ros2_ws` 通过 Git submodule 固定官方 SDK v1.5.19 和 `rslidar_msg`。克隆后需递归初始化，并应用该目录 README 所述的 M20 配置/构建差异；实际 ROS 2 依赖版本仍为 **UNKNOWN**。

# 4. Dataset Organization

## 4.1 仓库中实际存在的数据

仓库包含一个测试 bag：`data/rosbag/m20_upstairs.bag`，文件大小为 420,763,439 bytes，时长约 16.517 s，共约 18,500 条消息。它包含：

- `/rslidar_points_front`、`/rslidar_points_rear`：`sensor_msgs/PointCloud2`，各约 10 Hz，frame 分别为 `rslidar_front/rear`。点字段为 `x/y/z/intensity/ring/timestamp`，ring 范围 `0..95`，点时间为绝对时间。
- `/rslidar_imu_data_front`、`/rslidar_imu_data_rear`：`sensor_msgs/Imu`，各约 200 Hz；orientation 未填充，加速度量级约为 `1 g`。
- `/JOINTS_DATA`：`m20_udp_bridge/M20JointsData`，表面发布率约 500 Hz，但只有约 10 Hz 的源样本更新，其余消息带重复语义。
- `/IMU`：`m20_udp_bridge/M20Imu`，表面发布率约 200 Hz，但同样只有约 10 Hz 的源样本更新；内容为桥接后的运动状态派生字段，不是原始 IMU 三轴测量。

该 bag 不含 `/tf`、`/tf_static`、`/joint_states`、GNSS 或 ground truth。若干关节角还超出 M20 URDF 限位，因此它是接口测试数据，不是已标定的四轮运动学或定位精度数据。最终 `/JOINTS_DATA` 要求由 GOS 直接提供真实 500 Hz 样本。

旧 `mountain.bag` 仍只出现在以下两处：

- `LIO-SAM-MID360/config/paramsLivoxIMU.yaml` 的注释；
- `rbf_cuda/script/gps2tum.py` 的硬编码路径 `/root/dataset/mountain.bag`。

因此论文数据集下载地址、完整序列清单、每条序列的起止时间和 ground truth 文件来源仍为 **UNKNOWN**。

## 4.2 主配置期待的输入 topic

`run_tron1a_all.launch` 和 `run6axis.launch` 加载 `LIO-SAM-MID360/config/paramsLivoxIMU.yaml`：

| 数据 | 当前生效 topic | 消息类型/消费位置 |
|---|---|---|
| LiDAR | `/livox/lidar_10_192_1_141` | Livox `CustomMsg`，`ImageProjection::cloudHandler()` |
| IMU | `/livox/imu_10_192_1_141` | `sensor_msgs/Imu`，点云去畸变和 IMU 预积分 |
| 关节状态 | `/joint_states` | `sensor_msgs/JointState`，`JointStateWheelKinematics` |
| GPS 里程计 | `odometry/gpsz` | `nav_msgs/Odometry`，`mapOptimization::gpsHandler()`；可选 |
| 外部回环 | `lio_loop/loop_closure_detection` | `std_msgs/Float64MultiArray`；可选，仓库无生产者 |

配置注释称 `mountain.bag` 中 LiDAR/IMU topic 的地址后缀是 `_123`，而当前生效参数使用 `_141`。必须使用 `rosbag info` 核对实际 bag 后再选择配置或 topic remap；仓库无法确认哪一组与可用数据一致。

## 4.3 数据格式约束

- Livox 路径要求 `CustomMsg`；M20/Airy 路径要求包含 float32 `x/y/z/intensity`、uint16 `ring`、float64 绝对 `timestamp` 的 `PointCloud2`。
- README 明确要求 LiDAR 与 IMU 时间戳对齐。允许的时间偏差、硬件同步方案和 rosbag 时间修正方法为 **UNKNOWN**。
- `imuType: 0` 时，`imuConverter()` 将输入线加速度乘以重力常数，表示该配置期待以 `g` 为单位的 MID360 内置 IMU 加速度；外部 IMU 的单位和外参必须另行核对。
- Tron1A 轮地约束需要 8 个关节位置；M20 路径需要 12 个腿关节位置，并由适配器发布 16 个 URDF 关节。M20 默认顺序和名称在 `m20_joint_state_adapter.py` 中明确。
- `gps2tum.py` 期待 `/ublox_driver/receiver_lla` 消息具有 `latitude`、`longitude`、`altitude` 字段，但脚本没有声明其具体 ROS 消息类型，故类型为 **UNKNOWN**。

# 5. Running the Baseline

本节中的“标准 RBF-LIO”指当前主实现 `LIO-SAM-MID360`，不是 `rbf_cuda/src/lio_sam` 的旧版/实验性分支。

`run_tron1a_all.launch` 是历史 Tron1A 入口。M20/Airy 基线由 `run_m20.launch` 与 `paramsM20.yaml` 定义，但尚无本仓库可引用的成功 ROS 回放日志。

## 5.1 M20 接口回放基线

```bash
mkdir -p /absolute/output
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_m20.launch \
  use_sim_time:=true \
  with_rviz:=true \
  with_robot_tf:=true \
  with_rbf_visualization:=false \
  enable_rbf_constraint:=false \
  tum_trajectory_path:=/absolute/output/m20_upstairs_tum.txt
```

另一个终端播放：

```bash
rosbag play --clock /absolute/path/to/rbf_lio_20260402/data/rosbag/m20_upstairs.bag
```

命令来源：`LIO-SAM-MID360/launch/run_m20.launch` 和 `config/paramsM20.yaml`。`--clock` 与 `use_sim_time` 用于按 bag 时间运行；测试 bag 不含 ground truth，不能由该命令计算 ATE/RTE。

## 5.2 历史主入口

先创建输出目录，因为 `mapOptmization` 不会创建 TUM 文件的父目录：

```bash
mkdir -p /absolute/output/rbf_stats
```

在终端 1 启动：

```bash
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_tron1a_all.launch \
  with_rbf:=true \
  with_robot_tf:=false \
  with_rviz:=true \
  tum_trajectory_path:=/absolute/output/trajectory_tum.txt \
  rbf_file_path:=/absolute/output/rbf_stats
```

命令来源：`LIO-SAM-MID360/launch/run_tron1a_all.launch`。必须覆盖其中 `/root/code/rbf_lio/result_0331/staircase2/...` 的机器相关默认路径。

配置 `LIO-SAM-MID360/config/paramsLivoxIMU.yaml` 中 `enableRbfConstraint: true`，因此该入口启用主地图优化中的轮地/RBF 约束。`with_rbf:=true` 只额外启动 `cuda_rbf/launch/run_rbf.launch` 中的独立 `elevation_rbf` 和 `fk.py`，并不控制主优化器的 RBF 约束。

## 5.3 单独检查 rosbag

现有 M20 测试 bag 可用以下 ROS 1 命令检查和播放：

```bash
rosbag info /absolute/path/to/rbf_lio_20260402/data/rosbag/m20_upstairs.bag
rosbag play --clock /absolute/path/to/rbf_lio_20260402/data/rosbag/m20_upstairs.bag
```

`lio_sam/M20JointsData` 与 bag 中的 `m20_udp_bridge/M20JointsData` 定义具有相同 ROS 1 MD5 `fb86770770e356b2075846629adc617b`，ROS 1 可按相同序列化布局连接。该兼容性仍需在目标 ROS 环境中实测。

`support/m20_udp_bridge.zip` 内的 `record_bag.sh` 是测试桥录包入口，默认录制 `/IMU`、`/JOINTS_DATA` 和前后点云。该压缩包的重复发布策略不是最终 GOS 500 Hz 方案。

## 5.4 其他现有入口

```bash
roslaunch lio_sam run6axis.launch \
  tum_trajectory_path:=/absolute/output/rbf_odometry_trajectory_tum.txt
```

来源：`LIO-SAM-MID360/launch/run6axis.launch`。它同样加载 `paramsLivoxIMU.yaml`，启动 LIO 核心、机器人状态发布和 RViz，但不会启动独立 `elevation_rbf`。

```bash
roslaunch lio_sam run9axis.launch
```

来源：`LIO-SAM-MID360/launch/run9axis.launch`。它加载 `params9axisIMU.yaml`，其中 `enableRbfConstraint: false`，而且 launch 没有 TUM 路径参数，不能直接等同于完整 RBF-LIO 实验。

`LIO-SAM-MID360/launch/run.launch` 加载通用 `params.yaml`，会启动 navsat，但其中 RBF 约束默认关闭。`rbf_cuda/launch/run.launch` 启动的是旧版 `cuda_rbf` LIO-SAM 处理链，不作为当前基线入口。

## 5.5 保存地图

运行时服务接口来自 `LIO-SAM-MID360/srv/save_map.srv` 和 `mapOptimization::saveMapService()`：

```bash
rosservice call /lio_sam/save_map "resolution: 0.0
destination: '/rbf_lio_result/map'"
```

服务把 `destination` 直接拼接到 `$HOME`，上述命令会写入 `$HOME/rbf_lio_result/map`。**注意：实现会先递归删除该目标目录，再重新创建。** `resolution: 0.0` 保存未额外降采样的角点/面点地图；非零值作为保存时的体素分辨率。

# 6. Experiment Variants

## 6.1 当前代码能够表达的变体

| 变体 | 参数/入口 | 实际作用 | 自动化与结果位置 |
|---|---|---|---|
| Full model | M20：`run_m20.launch enable_rbf_constraint:=true`；历史 Tron1A：`paramsLivoxIMU.yaml` + `run_tron1a_all.launch` | 主 LM 中加入轮地/RBF 残差 | M20 必须先换用已验证的关节数据；无批处理脚本 |
| w/o manifold | 将所加载 YAML 的 `enableRbfConstraint` 设为 `false` | `updateRbfLmConstraints()` 直接跳过约束 | 无专用 YAML、launch 或命令；必须另行维护实验配置，当前仓库不能一条命令复现 |
| LiDAR z 自适应关闭 | `enableLidarZCalibration: false` | 保留 RBF 残差，但停止残差驱动的 LiDAR z 偏移更新 | 无专用实验入口 |
| 回环开/关 | `loopClosureEnableFlag` | 控制内置回环线程/因子 | 无参数扫描脚本 |
| 独立地形拟合开/关 | `with_rbf:=true/false` | 只控制 `elevation_rbf` 可视化和耗时/MAE 日志 | `<rbf_file_path>/*.txt` |
| 主约束节点数上限 | `rbfNodeMaxNum` | 限制从局部平面地图送入 CPU RBF 求值器的节点数 | 无扫描脚本 |
| 主约束核宽与权重 | `rbfSigmaX/Y`、`rbfConstraintWeight`、`rbfWeightR1..R4`、残差截断参数 | 改变主 LM 轮地约束 | 无扫描或汇总脚本 |

`with_rbf:=false` **不是**论文中的 `w/o manifold`：前者只关闭独立 CUDA 地形图节点，主地图优化仍由 `enableRbfConstraint` 决定。

## 6.2 RBF 参数实验的真实接口

主优化器参数位于 `LIO-SAM-MID360/config/paramsM20.yaml`，历史基线的同名参数位于 `paramsLivoxIMU.yaml`：

- `rbfConstraintWeight: 0.35`
- `rbfConstraintMaxPoints: 2000`
- `rbfSigmaX: 0.07`、`rbfSigmaY: 0.07`
- `rbfNodeMaxNum: 4000`
- `rbfWeightR1: 1.0`、`rbfWeightR2: 0.05`、`rbfWeightR3: 0.02`、`rbfWeightR4: 0.05`

独立 `elevation_rbf` 从 ROS 参数 `rbf/*` 读取参数，但仓库 launch/YAML 没有设置这些键，因此当前运行使用 `elevation_rbf.cpp::main()` 中的默认值：`sigma_x/y=0.14`、中心网格叶尺寸 `0.2`、范围前后左右各 `3.0`、点云噪声 `0.1`、权重噪声 `0.5`、点云降采样 `0.05`、`margin=1e-3`。

论文参数研究包含岭惩罚、中心分辨率和带宽，但仓库没有 sweep、CPU/GPU 对照或绘图程序。岭回归参数在 CUDA/RBF 实现中存在固定值，未暴露为 launch/YAML 实验参数；因此论文 Fig. 6 不能只靠当前配置文件复现。

## 6.3 仓库中缺失的论文消融

| 论文实验 | 论文描述 | 当前仓库状态 |
|---|---|---|
| Manifold ablation | Full 与 w/o manifold 仅切换 manifold residual | 可通过 `enableRbfConstraint` 表达开关，但无固定配置、运行脚本和评估脚本 |
| Adaptive-center ablation | 自适应中心与全部候选中心 | **UNKNOWN / 不可复现**：无运行时开关或专用程序 |
| GPU acceleration ablation | 相同 RBF 计算的 GPU/CPU 对照 | **UNKNOWN / 不可复现**：无运行时后端选择和论文计时汇总脚本 |
| Sparse 75% / 50% | 每帧均匀随机保留点 | **UNKNOWN / 缺失**：无确定性采样器、随机种子或参数 |
| 30 度扇区遮挡 | 每帧随机移除连续方位扇区 | **UNKNOWN / 缺失**：无遮挡实现或参数 |

不能把 `downsampleRate`、体素叶尺寸或 `rbfConstraintMaxPoints` 自动解释为论文的 observation-reduction 实验。特别是主 CPU 轮地梯度实现当前不消费 `RbfCudaRequest::points_lidar`；`rbfConstraintMaxPoints=0` 会整体跳过约束，而非构成论文的 50%/75% 点云实验。

# 7. Evaluation

## 7.1 轨迹输出

`mapOptimization::saveTUMTrajectoryToFile()` 每次发布全局里程计时写一行：

```text
timestamp x y z qx qy qz qw
```

时间来自当前 LiDAR 帧，位置与四元数来自 `transformTobeMapped`。文件以输出模式打开并逐行 flush；启动前必须确保父目录存在。`run_tron1a_all.launch` 的 `tum_trajectory_path` 和 `run6axis.launch` 的同名参数负责设置路径。

`rbf_cuda/script/odom_recorder.py` 也写 TUM 格式，但当前硬编码输出 `/root/cuda_rbf_ws/src/result/fast_lio_trajectory.tum`，订阅 `/Odometry`；主 RBF-LIO topic `/lio_sam/mapping/odometry` 仅存在于被注释的代码中。该脚本没有 launch、参数化或目录创建，不能直接作为主实验记录器。

## 7.2 GPS 参考轨迹准备

仓库唯一的 GPS 转换命令是：

```bash
python3 rbf_cuda/script/gps2tum.py
```

来源：`rbf_cuda/script/gps2tum.py`。它不可接受命令行参数，并硬编码：

- 输入：`/root/dataset/mountain.bag`
- topic：`/ublox_driver/receiver_lla`
- 输出：`/root/code/ws_lio/result/mountain/gps_trajectory.tum`
- 坐标转换：EPSG:4979 LLA -> EPSG:4978 ECEF -> 以第一帧为原点的 ENU
- 时间戳：rosbag record time `t.to_sec()`
- 姿态：固定 `0 0 0 1`

脚本是否对应论文使用的 RTK reference、是否需要天线杆臂补偿、与 LiDAR 的时间偏移以及输出坐标轴是否满足评估工具要求，均为 **UNKNOWN**。

## 7.3 指标可用性

| 指标 | 论文中的使用 | 仓库实现 |
|---|---|---|
| ATE | 报告 RMSE 和最大值 | **缺失** |
| z-ATE | 报告 z 轴绝对轨迹误差 | **缺失** |
| RTE | 报告 `Delta t = 0.1 s` 的相对轨迹误差 | **缺失** |
| RMSE | 用于 ATE/z-ATE/RTE 汇总 | **缺失** |
| RBF terrain MAE | 独立 RBF 节点调用 `compute_error()` | 存在，写入 `mae.txt` |
| RBF kernel/weight time | 独立 RBF 节点内部计时 | 存在，分别写入两个文本文件 |
| 主优化 RBF 残差统计 | 均值、绝对均值、EMA、z offset、约束数和分量均值 | 只发布 `lio_sam/rbf/residual_stats`，仓库无落盘脚本 |

论文说明各方法使用相同轨迹区间、RTK reference、时间关联和对齐协议，但仓库没有给出时间关联阈值、SE(3)/Sim(3) 对齐实现、首帧/全轨迹对齐方式、插值方法或外部评估工具版本。因此不能从当前仓库生成论文 ATE/RTE 数值。

# 8. Plot / Table Generation

遍历结果如下：

- 没有 `evaluation/` 或 `tools/` 目录；
- 没有用于 RBF-LIO 评估、消融或绘图的 `.sh` / `.bash` 脚本；`support/rslidar_start.sh` 是驱动参考脚本，但其 ROS 2 环境与 ROS 1 `roslaunch` 命令不一致，不能作为实验流水线入口；
- 没有 Matplotlib、gnuplot、Pandas、LaTeX 表格或其他 plotting/table generation 脚本；
- `LIO-SAM-MID360/README.md` 包含演示 GIF/图片引用，但没有它们的生成流程；
- `elevation_rbf.cpp` 只生成逐时间戳数值日志和 `/rbf_elevation_map`，不生成直方图、箱线图或表格；
- JSON 序列化代码被 `if (false)` 禁用，运行时不会写出规划中的 RBF JSON 文件。

因此论文 Fig. 6 至 Fig. 11、Table I 至 Table VIII 的原始数据文件、绘图命令、颜色/坐标配置和表格汇总规则均为 **UNKNOWN**。仓库当前不能端到端复现这些图表。

# 9. Result Directory Structure

仓库没有统一、可移植的结果目录规范。代码实际产生的结构如下：

```text
<tum_trajectory_path>                 # 单个 TUM 轨迹文件

<rbf_file_path>/                     # elevation_rbf 会创建此目录
  kernel_matrix_time.txt              # timestamp kernel_time_ms
  weight_time.txt                     # timestamp weight_time_ms
  mae.txt                             # timestamp mae

$HOME<save_map.destination>/          # save_map 服务会删除后重建
  trajectory.pcd
  transformations.pcd
  CornerMap.pcd
  SurfMap.pcd
  GlobalMap.pcd
```

相关默认路径均带有旧机器的绝对目录：

- `run_tron1a_all.launch`：`/root/code/rbf_lio/result_0331/staircase2/...`
- `run6axis.launch`：`/root/code/ws_lio/result/staircase2/...`
- `run_rbf.launch`：`/root/code/rbf_lio/result/rbf/default`
- `gps2tum.py`：`/root/code/ws_lio/result/mountain/gps_trajectory.tum`
- `odom_recorder.py`：`/root/cuda_rbf_ws/src/result/fast_lio_trajectory.tum`
- `paramsLivoxIMU.yaml` 的默认地图目录：`$HOME/Downloads/LOAM/`

序列命名规则、方法名、随机种子、重复实验编号和论文表格的结果组织规则均为 **UNKNOWN**。复现时必须显式覆盖 launch 输出路径，避免不同实验互相覆盖。

# 10. Reproducibility Checklist

- [ ] 已确认 Linux/ROS Noetic/CUDA 环境，记录实际 OS、GCC、CUDA、GPU 驱动和 GTSAM 版本。
- [ ] 已在外部 Catkin 工作区构建 `lio_sam`、`cuda_rbf` 和 `robot_description`。
- [ ] 已记录 `m20_upstairs.bag` 的路径/校验和，并明确它只用于接口测试；论文数据集仍为 **UNKNOWN**。
- [ ] 已用 `rosbag info` 核对 Livox 消息包、LiDAR/IMU topic 和 `_123`/`_141` 地址差异。
- [ ] 已核对 LiDAR/IMU 时间同步、IMU 单位和 `extrinsicRot` / `extrinsicRPY` / `extrinsicTrans`。
- [ ] 已确认 `/joint_states` 包含 8 个关节、名称/顺序正确且时间覆盖 LiDAR 帧。
- [ ] 对 M20 已确认 `/JOINTS_DATA` 是 GOS 直接真实 500 Hz，而不是测试桥对约 10 Hz 源样本的重复发布。
- [ ] 对 M20 已将 APDU 关节值转换到 URDF 符号、零位、顺序和限位；当前规则为 **UNKNOWN**。
- [ ] 对 Airy 已确认驱动侧点云 transform 是否启用，并消除与 RBF-LIO/URDF 的重复变换。
- [ ] 已取得前 Airy DIFOP LiDAR-IMU 工厂旋转/平移标定；当前缺失。
- [ ] 已为每次实验创建独立 TUM 与 RBF 日志目录，并覆盖 launch 中的 `/root/...` 默认值。
- [ ] 已记录所用 launch 和完整 YAML；Full 模型确认 `enableRbfConstraint=true`。
- [ ] 已确认日志中 RBF 约束未因缺关节、空地图、零节点或非法数值被跳过。
- [ ] 已检查 `trajectory_tum.txt` 非空、时间单调、位置/四元数为有限值。
- [ ] 如启用独立 RBF 节点，已检查三个统计文件非空且 `/rbf_elevation_map` 有输出。
- [ ] 如需地图，已理解 `save_map` 会删除目标目录，并已使用隔离的保存路径。
- [ ] 已获得 ground truth，并记录其传感器、坐标系、时间基准、杆臂和转换过程；当前均未由仓库完整给出。
- [ ] 已固定轨迹关联、插值、对齐和 ATE/z-ATE/RTE 定义；当前仓库无实现。
- [ ] 已获得或实现论文消融的采样、扇区遮挡、中心选择和 CPU/GPU 对照工具；当前仓库无实现。
- [ ] 已获得论文图表生成脚本与原始汇总数据；当前仓库无实现。

# 11. Known Missing Information

1. **UNKNOWN：论文数据集获取方式。** 仓库只有 M20 接口测试 bag，没有论文数据集下载 URL、许可证、校验和或序列清单。
2. **TODO：M20 bag 回放验证。** PointCloud2、每点绝对时间和 M20 四轮关节消息已接入主代码，但尚未在 ROS Noetic 环境完成整包运行。
3. **UNKNOWN：论文序列与本地文件的映射。** Staircase 1/2、Artificial Hill、Rose Garden、Botanical Garden、Indoor Staircase Only、Wet Grass 没有文件名映射。
4. **UNKNOWN：ground truth 来源与标定。** PDF 提到 RTK reference；仓库未给出 RTK 解算文件、基站信息、天线外参或质量筛选规则。
5. **UNKNOWN：时间同步与轨迹关联。** 没有传感器时偏、关联阈值和插值方法。
6. **UNKNOWN：ATE/z-ATE/RTE 的代码定义。** 没有评估工具、命令、对齐方式或版本；仅论文给出 RTE 的 `Delta t=0.1 s`。
7. **UNKNOWN：比较基线。** Fast-LIO2、ROLO-SLAM、Leg-KILO、GLIM 和 Wheel-Legged SLAM 的代码、commit、配置和适配修改均不在仓库中。
8. **UNKNOWN：消融自动化。** 没有 manifold、adaptive center、GPU/CPU、稀疏点云或扇区遮挡的批处理入口。
9. **UNKNOWN：随机性控制。** 点云随机下采样/扇区遮挡的种子、重复次数和统计规则未提供。
10. **UNKNOWN：RBF 参数研究数据。** 没有 Fig. 6 使用的参数列表、逐帧残差原始文件或 top-1% 剔除脚本。
11. **UNKNOWN：论文图表流水线。** 没有绘图、表格、地图截图和轨迹可视化脚本。
12. **UNKNOWN：精确运行环境。** OS 镜像、GCC、CUDA、驱动、GTSAM、PCL 和 Livox driver 版本未锁定。
13. **TODO：将硬编码路径参数化。** `gps2tum.py`、`odom_recorder.py` 和若干 launch 默认值依赖旧机器路径；本手册未修改这些文件。
14. **TODO：补充无源码修改的实验配置集合。** 当前 w/o-manifold 等变体需要切换 YAML，但仓库没有只读预设配置或 launch 参数。
15. **TODO：补充从 rosbag 到论文表格的验证脚本。** 当前可复现链路在 TUM 轨迹和 RBF 原始日志处终止。
16. **UNKNOWN：Airy 内部标定。** 前 Airy DIFOP 中 LiDAR-IMU 工厂旋转/平移参数未包含在配置或启动日志中。
17. **UNKNOWN：驱动点云变换归属。** 驱动配置打印的前雷达 `[0.0501,0,0.739]`、pitch `+pi/2` 与 M20 URDF 前雷达位姿不一致；`ENABLE_TRANSFORM` 的实际构建状态也未提供。
18. **TODO：最终关节数据源。** 测试 bridge 的重复发布只可用于联调；GOS 原生 500 Hz 生产者、消息时间戳语义和实测 bag 尚未提供。
