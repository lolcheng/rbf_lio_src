# RBF-LIO 工程协作指南

## 适用范围与事实来源

本文件适用于整个仓库。

以仓库中当前提交的代码作为首要事实来源。`support/RBF_LIO.pdf` 用于理解预期的算法、术语、残差定义和实验设置，但不得假定论文中的所有功能均已实现。
当论文与代码不一致时，应记录差异，并保持当前代码行为，除非任务明确要求弥合该差异。

当前主要实现位于 `LIO-SAM-MID360`。将 `rbf_cuda/src/lio_sam` 视为旧版或实验性 LIO-SAM 分支，不要自动在两个目录中重复同一项修改。

## 项目目的

RBF-LIO 是一个面向轮腿机器人非平整地形运动的 ROS 1 激光雷达-惯性里程计与建图工程。它在 LIO-SAM 风格的处理链上增加了以下能力：

- 接收 Livox MID360 点云，并使用 IMU 对扫描进行去畸变。
- 提取 LOAM 边缘特征和平面特征。
- 执行扫描到地图位姿优化以及 iSAM2 关键帧因子图优化。
- 根据关节状态计算车轮运动学。
- 添加轮地软接触残差，用于减小楼梯、斜坡等剧烈起伏地形上的竖直方向和姿态漂移。
- 提供独立的 CUDA RBF 地形拟合与可视化处理链。

论文描述的是递归更新的高斯 RBF 地形曲面，并使用曲面高度和法向量构造轮地接触约束。仓库中包含该设计的多个组成部分，但论文所述的完整处理链目前尚未端到端接通，具体见“论文与代码的差异”。

## 软件架构

### 仓库软件包

- `LIO-SAM-MID360`（ROS 包名 `lio_sam`）：主要运行包和当前事实来源。包含 MID360 预处理、特征提取、IMU 预积分、地图优化、回环检测、车轮运动学以及RBF/接触约束注入。
- `rbf_cuda`（ROS 包名 `cuda_rbf`）：包含 CUDA RBF 拟合库、独立高程节点、诊断程序，以及一套旧版 LIO-SAM 处理链。
- `robot-description`（ROS 包名 `robot_description`）：当前保留 M20 URDF 和网格
  资源，用于 M20 迁移时的机器人模型与运动学事实来源。原 Tron1A 模型文件已被
  删除，但现有 launch/CMake 尚未全部适配这一变化。
- `support/RBF_LIO.pdf`：只读论文参考资料。
- `support/m20_udp_bridge.zip`：测试用 M20 UDP/ROS 1 桥接代码快照；当前
  `JOINTS_DATA` 的约 500 Hz 发布由约 10 Hz 源数据重复产生，不能视为最终接口。
- `support/rslidar_ros2_ws`：RoboSense Airy ROS 2 参考工作区。`rslidar_msg` 和
  `rslidar_sdk` 以官方 Git submodule 固定版本，M20 配置及本地构建差异保存在
  submodule 外；克隆后需执行 `git submodule update --init --recursive`。
- `support/rslidar_start.sh`：现有参考脚本。它 source ROS 2 工作区却调用 ROS 1
  `roslaunch`，与已确认的 `ros2 launch rslidar_sdk start.py` 不一致，不得作为
  已验证启动命令。

### 主要运行节点

`LIO-SAM-MID360/launch/run_tron1a_all.launch` 是历史综合入口；其 LIO 核心
由以下节点组成：

1. `lio_sam_imageProjection`
   - 输入：Livox `CustomMsg`、IMU、增量 IMU 里程计、自适应 LiDAR z 偏移。
   - 处理：点云坐标变换、扫描缓存、旋转去畸变、距离图组织。
   - 输出：`lio_sam/deskew/cloud_info` 和去畸变点云。
2. `lio_sam_featureExtraction`
   - 输入：去畸变后的 `cloud_info`。
   - 处理：平滑度计算、遮挡剔除、边缘和平面特征选择。
   - 输出：包含角点和平面点云的 `lio_sam/feature/cloud_info`。
3. `lio_sam_mapOptmization`
   - 输入：特征点云、GPS、外部回环提示；车轮运动学对象还会直接订阅
     `/joint_states`。
   - 处理：扫描到地图 LM 优化、轮地残差注入、关键帧选择、GPS/里程计/回环
     因子以及 iSAM2 优化。
   - 输出：建图里程计、轨迹、局部/全局地图、RBF 残差统计、自适应 LiDAR z
     偏移。
4. `lio_sam_imuPreintegration`
   - 输入：IMU 和建图里程计校正结果。
   - 处理：GTSAM IMU 预积分、速度和偏置估计、高频状态传播、变换融合。
   - 输出：`odometry/imu_incremental`，反馈给点云投影和地图初值估计。

### RBF 与轮地接触组件

- `LIO-SAM-MID360/src/rbf_wheel_kinematics.cpp`：接收 Tron1A 的 8 个关节位置，
  计算左右轮在世界坐标系下的轮心和轮轴。
- `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp`：求解等效轮地接触点，计算
  地形/接触残差块，并通过数值差分计算位姿 Jacobian。
- `LIO-SAM-MID360/src/mapOptmization.cpp::updateRbfLmConstraints`：构造 RBF
  请求并缩放各项约束。
- `LIO-SAM-MID360/src/mapOptmization.cpp::LMOptimization`：将 RBF 残差行追加到
  LOAM 线性系统，并求解六自由度位姿增量。
- `rbf_cuda/include/rbf_fitting/rbf_fit.h`：负责自适应网格/中心管理、RBF 权重
  状态、地形查询和可视化。
- `rbf_cuda/src/cuda/rbf_cuda.cu`：使用 CUDA/Thrust 计算核函数，并使用
  cuBLAS/cuSOLVER 执行岭回归相关运算。
- `rbf_cuda/src/rbf/elevation_rbf.cpp`：独立执行局部地图 RBF 拟合并发布
  `/rbf_elevation_map`。它是辅助可视化和统计处理链，不是主地图优化器的输入。

### 运行时数据流

```text
Livox 点云 + IMU
    -> imageProjection
    -> featureExtraction
    -> mapOptimization（LOAM 残差 + 轮地残差）
    -> 关键帧/iSAM2 -> 里程计、地图、轨迹

mapOptimization 校正 -> IMUPreintegration
IMUPreintegration 预测 -> imageProjection + mapOptimization 初值

/joint_states -> JointStateWheelKinematics -> 车轮几何 -> RBF 约束
mapOptimization 残差 -> 自适应 LiDAR z 偏移 -> imageProjection

局部地图 + 建图里程计 -> elevation_rbf -> 仅用于 RBF 可视化
```

### M20/Airy 迁移边界

当前主代码尚未完成 M20/Airy 适配。第一阶段迁移只使用前置 Airy 点云、前置
Airy IMU 和机体关节数据；后置 Airy 点云及两台雷达之外的其他 IMU 不进入主估计：

- `/rslidar_points_front`：`sensor_msgs/PointCloud2`，96 个 ring，点字段包含
  `x/y/z/intensity/ring/timestamp`；点 `timestamp` 为绝对时间，扫描头时间对应首点。
- `/rslidar_imu_data_front`：`sensor_msgs/Imu`，约 200 Hz；实测 orientation 未填充，
  加速度量级约为 `1 g`。
- `/JOINTS_DATA`：测试包中约 500 Hz 发布，但有效源样本约 10 Hz 且大量重复；最终
  版本要求由 GOS 直接提供真实 500 Hz 数据。

M20 运动学、关节方向、零位、限位和轮尺寸以
`robot-description/M20/urdf/M20.urdf` 为准。当前测试 bag 的若干关节值超出 URDF
限位，因此只能验证消息接口，不能验证轮地接触几何。

Airy 驱动配置和启动日志已经确认双雷达端口、话题、时间戳选项及驱动侧变换参数，
但没有提供前置 Airy 单机 DIFOP 中的 LiDAR-IMU 内部旋转/平移标定。驱动侧前雷达
变换 `[0.0501, 0, 0.739]`、pitch `+pi/2` 也与 M20 URDF 的前雷达安装位姿
`[0.32028, 0, -0.013]`、零 RPY 不一致。修改主链前必须明确点云在哪一层完成坐标
变换，避免驱动和 RBF-LIO 重复变换；IMU 与点云必须最终落在一致且有据可查的坐标
约定中。

## 编译方法

### 支持环境

代码目标环境为 Linux 上的 ROS 1 Noetic。典型依赖包括：

- Ubuntu 20.04、ROS Noetic 和 Catkin。
- PCL、OpenCV、Eigen3、Boost（包括 `timer`）、OpenMP 和 GTSAM。
- ROS 1 Livox 驱动。CMake 当前声明依赖 `livox_ros_driver`；源代码在头文件
  可用时能够包含 `livox_ros_driver/CustomMsg.h` 或
  `livox_ros_driver2/CustomMsg.h`。
- `cuda_rbf` 包所需的 CUDA Toolkit、Thrust、cuBLAS 和 cuSOLVER。
- 独立高程节点所需的 `nlohmann_json`。
- 使用 `fk.py` 时所需的 Python ROS 包、`urdfpy`、NumPy 和
  `tf_conversions`。

原生 Windows 不是 ROS Noetic 的受支持构建环境。应使用 Linux，或使用具备
可用 ROS/CUDA 环境的 WSL2。

### 顶层 CMake 的重要限制

仓库根目录的 `CMakeLists.txt` 只包含文本
`/opt/ros/noetic/share/catkin/cmake/toplevel.cmake`。它很可能原本是 Catkin
符号链接，但在复制仓库时被展开成了普通文本文件。因此当前仓库根目录不能直接
作为 Catkin 工作区构建。

不要在无关任务中重写该文件。应在独立 Catkin 工作区中构建，以保持仓库内容
不变：

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

`cuda_rbf` 在配置和编译时必须存在 CUDA。`lio_sam` 在 CMake 中将 CUDA 视为
可选依赖，但当前车轮接触残差仍在 CPU 上运行，即使辅助 CUDA 库已成功构建。

当前 `robot-description/CMakeLists.txt` 仍安装已删除的 `pointfoot/wheellegged`
目录，尚未安装 `M20`。因此上述三包命令只记录预期工作区结构，不得声称当前检出
已经能够完整构建；M20 描述包适配属于后续迁移修改。

除非已经在受支持的 Linux/ROS 环境中实际完成编译，否则不要声称构建成功。
构建产物应放在外部工作区，或标准的 `build`、`devel`、`install`、`log`
目录中。

## 运行方法

历史 Tron1A 综合启动命令如下：

```bash
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_tron1a_all.launch \
  with_rbf:=true \
  with_robot_tf:=false \
  with_rviz:=true \
  tum_trajectory_path:=/absolute/output/trajectory_tum.txt \
  rbf_file_path:=/absolute/output/rbf_stats
```

当前 `robot-description` 中的 Tron1A 模型已被删除，而该 launch 仍引用相应模型，
因此不得将上述命令描述为当前检出的可用 M20 启动方式。M20 launch 和参数文件尚未
接入主代码，状态为 **TODO**。

运行前应检查 `LIO-SAM-MID360/config/paramsLivoxIMU.yaml` 中的话题配置。仓库
中的默认值对应一个特定 MID360 地址，通常不会自动匹配其他设备或 rosbag。
完整约束处理链需要以下输入：

- `lio_sam/pointCloudTopic` 指定的 Livox 点云。
- `lio_sam/imuTopic` 指定的 IMU 数据。
- `/joint_states` 上至少 8 个关节位置。
- 使用相关功能时所需的可选 GPS 和回环输入。

`with_rbf` 控制独立的 `elevation_rbf` 可视化/统计节点，不负责启用或禁用
`mapOptmization` 内部的 RBF 残差。后者由 YAML 中的
`lio_sam/enableRbfConstraint` 控制。

`with_robot_tf` 控制关节名称适配和机器人状态 TF 发布。主轮地约束会独立订阅
`jointStateTopic`，并同时支持 Tron1A 关节名称和 `joint_0` 至 `joint_7` 的
后备命名。

常用输出包括：

- `lio_sam/mapping/odometry` 和 `lio_sam/mapping/odometry_incremental`。
- `lio_sam/mapping/path`、`map_local` 和 `map_global`。
- `lio_sam/rbf/residual_stats`。
- `lio_sam/lidar_calib/z_offset`。
- 启用独立高程节点时的 `/rbf_elevation_map`。

其他启动文件（`run.launch`、`run6axis.launch` 和 `run9axis.launch`）使用不同
配置。仓库中的通用 YAML 和 9 轴 IMU YAML 默认将 `enableRbfConstraint`
设为 `false`。

## 论文与代码的差异

分析、文档、测试和后续修改应首先以当前代码行为为准，并保留以下区别：

- 论文使用递归更新的 RBF 权重。`VoxelGridRBF` 提供了
  `updateWeightCuda()`，但当前运行时调用者使用 `initialWeightCuda()`，即
  重新拟合活动曲面，而没有调用递归更新。
- 论文要求候选中心附近至少存在 3 个地形观测点。当前中心过滤逻辑只要自定义
  二维 KD 树在半径内找到任意一个点就会接受该中心。
- 主地图优化器不使用 `elevation_rbf` 生成的 RBF 模型。它直接从降采样局部
  平面地图构造 `rbf_nodes`，当前求值器将每个节点的 `z` 坐标解释为高斯核
  系数。
- `mapOptmization` 会填充 `RbfCudaRequest::points_lidar`，但当前梯度实现没有
  使用该字段。
- 论文联合优化位姿和显式接触点变量。代码先通过小规模局部 Gauss-Newton 循环
  求每个接触点，再使用中心有限差分计算位姿导数。
- CUDA 车轮残差/Jacobian 内核已经存在，并在找到 CUDA 时参与编译，但没有
  运行时调用点。当前有效的轮地残差和 Jacobian 实现为 CPU 代码。
- 论文公式支持与平台相关的车轮集合，并包含四轮实验。仓库中的运动学实现仅针对
  Tron1A 两轮结构。
- 代码增加了由接触残差统计驱动的自适应 LiDAR z 偏移反馈环。它是仓库特有的
  扩展，不是从论文直接推导出的必要实现。

在缺失连接尚未实现和验证前，不得将本仓库描述为论文的完整复现。

## 不允许修改的位置

除非用户明确要求修改相关位置，否则：

- 不要修改 `support/RBF_LIO.pdf`、`support/m20_udp_bridge.zip` 或
  `support/rslidar_ros2_ws` 中的供应商/参考代码，除非任务明确要求维护这些材料。
- 不要修改 `robot-description/**/meshes` 下的二进制资源。
- 不要手工修改 `build`、`devel` 或 `install` 中生成的 Catkin 消息和服务
  文件；应修改源 `.msg` 或 `.srv`。
- 不要仅为了让本地检出可编译而修改根目录 `CMakeLists.txt`；优先使用前述外部
  Catkin 工作区。
- 不要把 `rbf_cuda/src/lio_sam` 当作当前主要实现，也不要在没有明确迁移任务时
  将它与 `LIO-SAM-MID360` 同步。
- 修改 URDF 尺寸、轮半径、关节顺序、传感器外参或默认话题前，必须同时核对
  机器人模型、launch 文件、YAML 配置和运动学代码。
- 不要提交生成的地图、轨迹、计时日志、RBF JSON、PCD、rosbag、CUDA 产物或
  构建产物，除非任务明确要求。

## 编码规范

### 通用要求

- 修改范围应尽量小。除非任务明确要求改变接口，否则保持现有 ROS 话题和消息
  契约。
- 以代码行为优先于论文假设。当实现有意偏离论文时，应增加简短注释或文档说明。
- 不要新增绝对 `/root/...` 路径。使用 ROS 参数、launch 参数、包相对路径或
  明确的命令行输入。
- 在脏工作树中保留用户已有修改，避免无关格式化和重构。
- 数学实现发生变化时，应尽可能添加测试或确定性的离线检查。估计器修改应记录
  测试配置、输入话题以及相关轨迹/残差指标。

### C++ 与 CUDA

- `LIO-SAM-MID360` 使用 C++14 编译，`rbf_cuda` 当前要求 C++17。未经有意更新
  和工具链验证，不要在 C++14 包中使用更高版本语言特性。
- 遵循被编辑文件现有的四空格缩进和大括号风格。新增源代码注释使用 ASCII 英文，
  避免加重旧注释中已有的编码损坏。
- 新增所有权应使用 RAII 和智能指针，避免新增全局可变状态。
- ROS 回调或工作线程共享数据时必须使用互斥锁，并保持锁作用域尽可能小。
- STL 容器存储固定尺寸、可向量化 Eigen 类型时，应使用 Eigen 对齐分配器。
- 进入优化或 CUDA 调用前，检查向量/矩阵尺寸、空点云、时间戳、有限数值和
  求解器状态。
- 明确记录位姿约定：坐标变换方向、旋转顺序，以及向量属于 base、LiDAR、IMU、
  odometry 还是 map 坐标系。
- 修改 CUDA 时，应检查所有 CUDA/cuBLAS/cuSOLVER 返回值，确保所有路径释放
  设备资源，并提供明确的 CPU 后备方案或失败行为。
- 不能仅因链接了 CUDA 库就把 CPU 实现称为“CUDA 实现”，必须验证实际调用链。

### ROS 与配置

- 可调参数应通过 `ParamServer` 和 YAML 加载，不要在回调函数中嵌入新的常量。
- 队列大小和回调线程数必须经过考虑；高频 IMU 和关节输入不能被耗时地图或 RBF
  运算阻塞。
- 重新发布消息时应保持时间戳同步和正确的 frame ID。
- 新增或重命名参数时，应在同一修改中更新参数声明、加载代码、相关 YAML、launch
  文件和文档。
- 修改关节运动学时，应以 `robot-description/M20/urdf/M20.urdf` 核对关节顺序、
  轴向、零位、限位和轮半径；不得直接假定测试桥接包中的 APDU 数值已经符合 URDF。
- M20 最终关节输入按 GOS 直接提供的真实 500 Hz `JOINTS_DATA` 设计；当前
  `m20_udp_bridge.zip` 的约 10 Hz 数据重复方案只能作为临时测试后备。

### 验证要求

修改主要运行链后，至少验证：

1. 软件包能够在 ROS Noetic 中编译。
2. 节点能够启动，且没有持续 respawn 或缺少参数。
3. LiDAR、IMU 和关节话题能够接收，时间戳覆盖完整。
4. 去畸变点云、特征点云和局部地图非空。
5. RBF 约束确实被使用，而不是被跳过。
6. LM 增量、IMU 偏置估计、RBF 残差统计和 LiDAR z 偏移均为有限值。
7. 分别在启用和禁用 RBF 约束时检查轨迹与地图表现。
