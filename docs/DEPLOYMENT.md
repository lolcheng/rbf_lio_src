# RBF-LIO 真实机器人部署手册

本文只依据当前仓库的代码、配置、launch、模型、测试 bag 和 `support/` 参考资料，说明如何将主实现 `LIO-SAM-MID360` 部署到真实机器人。`rbf_cuda/src/lio_sam` 是旧版或实验性处理链，不作为本文的主部署入口。无法由仓库确认的硬件、驱动命令、标定值或系统行为统一标记为 **UNKNOWN**。

> 重要边界：本文记录的是当前软件接口，不证明某一具体实机组合已经完成标定或现场验证。任何外参、topic、关节顺序和时间同步值都必须在目标机器人上核对。

> M20 状态：Airy 驱动配置/日志和测试 bag 已经满足“确认实际消息接口、点字段、频率和时间戳模式”这一输入要求；尚未满足“取得前 Airy 单机 DIFOP LiDAR-IMU 工厂外参”这一要求。当前主代码也仍是 Livox/Tron1A 实现，因此可以开始结构性迁移，但不能开始标定级轨迹验收。

# 1. Supported Hardware

## 1.1 LiDAR

### 从代码确认

- 当前主点云回调 `ImageProjection::cloudHandler()` 订阅 Livox `CustomMsg`，不是 `sensor_msgs/PointCloud2`，见 `LIO-SAM-MID360/src/imageProjection.cpp`。
- `LIO-SAM-MID360/README.md` 明确将本分支描述为 Livox MID360 适配版，并要求驱动发布 `CustomMsg`。
- `LIO-SAM-MID360/include/utility.h` 会在编译期优先选择 `livox_ros_driver2/CustomMsg.h`，否则选择 `livox_ros_driver/CustomMsg.h`。
- 当前 Tron1A 配置 `paramsLivoxIMU.yaml` 设置 `sensor: livox`、`N_SCAN: 4`、`Horizon_SCAN: 6000`。

### 限制

`ParamServer` 中保留了 `velodyne`、`ouster` 和 `livox` 枚举，投影函数也保留 Velodyne/Ouster 的列索引分支，但主订阅和缓存函数只接收 Livox `CustomMsg`，非 Livox 分支会报错并关闭节点。因此，不能将当前主运行链描述为已支持 Velodyne 或 Ouster 实机接入。

## 1.2 IMU

| 模式 | 当前配置 | 输入 | 代码行为 |
|---|---|---|---|
| 六轴 IMU | `imuType: 0`，`paramsLivoxIMU.yaml` | `sensor_msgs/Imu` | 加速度先乘 `imuGravity`，再用 `extrinsicRot` 旋转；姿态被设为固定的 `extQRPY` |
| 九轴 IMU | `imuType: 1`，`params9axisIMU.yaml` | `sensor_msgs/Imu` | 加速度和角速度用 `extrinsicRot` 旋转；输入姿态与 `extQRPY` 相乘 |

README 和六轴配置把 `imuType: 0` 对应到 MID360 内置 IMU，并声明其线加速度单位为 `g`。其他六轴 IMU 是否也输出 `g` 为 **UNKNOWN**，不能直接沿用该转换。

九轴配置存在，但具体 IMU 型号、驱动、安装方向和实机验证状态均为 **UNKNOWN**。

## 1.3 GNSS

### 从代码确认

- 地图优化器可选订阅 `gpsTopic` 上的 `nav_msgs/Odometry`，在 `mapOptimization::addGPSFactor()` 中加入 GTSAM GPS 因子。
- 配置支持 `useImuHeadingInitialization`、`useGpsElevation`、`gpsCovThreshold` 和 `poseCovThreshold`。
- `module_navsat.launch` 包含 `robot_localization/ekf_localization_node` 和 `navsat_transform_node`，并期待 `gps/fix`、`imu_correct`、`odometry/navsat`、`odometry/gps` 等接口。

### 当前未闭合的接口

主配置的 `gpsTopic` 是 `odometry/gpsz`，但仓库内没有节点发布该 topic。`module_navsat.launch` 和 YAML 中出现的是 `odometry/gps` 与 `odometry/navsat`。此外，`run_tron1a_all.launch`、`run6axis.launch` 和 `run9axis.launch` 都没有启用 navsat 模块。

因此，具体 GNSS 接收机、GNSS 驱动命令，以及从 `gps/fix` 到 `odometry/gpsz` 的完整部署链均为 **UNKNOWN**。当前不能宣称 GNSS 已在综合入口中即插即用。

## 1.4 Wheel / Leg Kinematics

### 从代码确认

- 主 RBF 约束订阅 `sensor_msgs/JointState`，默认 topic 为 `/joint_states`。
- `JointStateWheelKinematics` 需要至少 8 个 `position`，支持以下 Tron1A 名称：
  - `abad_L_Joint`、`hip_L_Joint`、`knee_L_Joint`、`wheel_L_Joint`
  - `abad_R_Joint`、`hip_R_Joint`、`knee_R_Joint`、`wheel_R_Joint`
- 也支持 `joint_0` 至 `joint_7`；名称找不到时按索引 0 至 7 回退。
- 运动学尺寸被写在 `LIO-SAM-MID360/src/rbf_wheel_kinematics.cpp` 中。它是历史 Tron1A 几何；对应的模型文件已从当前 `robot-description` 删除，现只能从代码确认这些常量。
- 默认轮半径由 `paramsLivoxIMU.yaml` 的 `wheelRadius: 0.127` 给出。

该模块不是一个独立的 wheel odometry 或 leg odometry 节点。它没有订阅轮速里程计，也不发布运动学里程计；它使用最新关节位置计算左右轮心和轮轴，供主 LM 的轮地接触残差使用。轮关节角 `q3/q7` 被读入，但当前轮心/轮轴计算不使用轮转角。

## 1.5 其他接口

- 可选外部回环提示：`lio_loop/loop_closure_detection`，类型为 `std_msgs/Float64MultiArray`；仓库没有对应生产者。
- 相机、视觉里程计、接触开关、力传感器和轮速编码器输入：**UNKNOWN / 未在主链中发现**。
- 当前 `robot-description` 已不含 Tron1A 模型；历史 launch 中对它的引用已经失效。

## 1.6 M20 与 RoboSense Airy 迁移目标

### 从仓库资料确认

- `robot-description/M20/urdf/M20.urdf` 描述 M20 四条腿、16 个关节和四个车轮。
- URDF 车轮 collision 半径是 `0.09 m`；迁移以 URDF 为准。
- `support/rslidar_ros2_ws/config/m20_airy.yaml` 配置两台 `RSAIRY`；现场日志确认驱动版本为 RSLidar SDK v1.5.19。
- 第一阶段只使用前 Airy 点云和前 Airy IMU。后 Airy 数据存在，但不进入第一阶段 RBF-LIO。
- `data/rosbag/m20_upstairs.bag` 已确认包含上述 Airy 数据和测试桥的 M20 状态。

### 当前限制

- 主 `ImageProjection` 尚不接收 Airy `sensor_msgs/PointCloud2`。
- 主运动学尚不接收 `m20_udp_bridge/M20JointsData`，也尚未计算 M20 四轮几何。
- Airy 驱动是 ROS 2，而主估计器是 ROS 1；仓库中没有已确认的 ROS 1/ROS 2 桥。
- 前 Airy DIFOP 中 LiDAR 与内置 IMU 的单机旋转/平移标定仍为 **UNKNOWN**。

# 2. ROS Interfaces

以下表格以 `paramsLivoxIMU.yaml` 和 `run_tron1a_all.launch` 为主。没有命名空间前导 `/` 的 topic 会按 ROS 名称解析规则解析；表中保留代码/配置原名。

## 2.1 外部输入与适配接口

| Topic | Message type | Publisher | Subscriber | Purpose |
|---|---|---|---|---|
| `/livox/lidar_10_192_1_141` | `livox_ros_driver/CustomMsg` 或 `livox_ros_driver2/CustomMsg` | Livox 驱动，仓库外 | `lio_sam_imageProjection` | MID360 原始点和每点相对时间 |
| `/livox/imu_10_192_1_141` | `sensor_msgs/Imu` | Livox/IMU 驱动，仓库外 | `lio_sam_imageProjection`、`lio_sam_imuPreintegration` | 去畸变、IMU 预积分和姿态初值 |
| `/joint_states` | `sensor_msgs/JointState` | 机器人关节驱动，仓库外，具体驱动 **UNKNOWN** | `JointStateWheelKinematics`；启用独立 RBF 时还被 `fk.py` 订阅；可被名称适配器订阅 | 左右腿/轮几何 |
| `/joint_states_tron1a` | `sensor_msgs/JointState` | `joint_state_name_adapter.py` | `robot_state_publisher`，仅 `with_robot_tf:=true` | 将 `joint_0..7` 名称转换为 Tron1A 关节名，用于 URDF TF |
| `odometry/gpsz` | `nav_msgs/Odometry` | **UNKNOWN；仓库无发布者** | `lio_sam_mapOptmization` | 可选 GPS 因子 |
| `lio_loop/loop_closure_detection` | `std_msgs/Float64MultiArray` | **UNKNOWN；仓库无发布者** | `lio_sam_mapOptmization` | 外部回环配对提示 |
| `gps/fix` | `sensor_msgs/NavSatFix` | GNSS 驱动，仓库外，型号/命令 **UNKNOWN** | `navsat_transform_node`，仅启用 `module_navsat.launch` 时 | 原始 GNSS 定位 |
| `imu_correct` | `sensor_msgs/Imu` | **UNKNOWN** | `navsat_transform_node`、`ekf_gps` | 可选 navsat/EKF 输入；不是当前 MID360 主 IMU topic |

配置注释称示例 `mountain.bag` 的 Livox topic 后缀为 `_123`，但当前生效值为 `_141`。真实机器人必须按驱动实际发布名检查。

## 2.4 M20/Airy 外部接口（目标接口，当前未接入主代码）

| Topic | Message type | Publisher | Subscriber | Purpose |
|---|---|---|---|---|
| `/rslidar_points_front` | `sensor_msgs/PointCloud2` | ROS 2 `rslidar_sdk_node` | **TODO：RBF-LIO PointCloud2 入口** | 第一阶段唯一 LiDAR 点云 |
| `/rslidar_imu_data_front` | `sensor_msgs/Imu` | ROS 2 `rslidar_sdk_node` | **TODO：RBF-LIO IMU 入口/ROS bridge** | 第一阶段去畸变和预积分 IMU |
| `/rslidar_points_rear` | `sensor_msgs/PointCloud2` | ROS 2 `rslidar_sdk_node` | 第一阶段无订阅者 | 后 Airy 点云，第一阶段不使用 |
| `/rslidar_imu_data_rear` | `sensor_msgs/Imu` | ROS 2 `rslidar_sdk_node` | 第一阶段无订阅者 | 后 Airy IMU，第一阶段不使用 |
| `/JOINTS_DATA` | `m20_udp_bridge/M20JointsData` | 测试 ROS 1 bridge；最终由 GOS 直接提供 | **TODO：M20 运动学适配器** | 12 个腿关节角和 4 个轮速 |
| `/IMU` | `m20_udp_bridge/M20Imu` | 测试 ROS 1 bridge | 第一阶段无订阅者 | 派生机体运动状态；不替代前 Airy 原始 IMU |

`support/rslidar_ros2_ws` 的 wrapper 对点云和 IMU填写相同配置 frame id，但当前可见
源码没有证明三轴 IMU 数值随点云通用 transform 同步旋转。不能仅凭相同 frame id
认定两类数据已经处于同一坐标系。

## 2.2 LIO 内部接口

| Topic | Message type | Publisher | Subscriber | Purpose |
|---|---|---|---|---|
| `lio_sam/deskew/cloud_deskewed` | `sensor_msgs/PointCloud2` | `lio_sam_imageProjection` | 外部可视化；核心通过 `cloud_info` 传递同一云 | 旋转去畸变后的点云 |
| `lio_sam/deskew/cloud_info` | `lio_sam/cloud_info` | `lio_sam_imageProjection` | `lio_sam_featureExtraction` | 去畸变点云、ring 索引、范围和初值 |
| `lio_sam/feature/cloud_corner` | `sensor_msgs/PointCloud2` | `lio_sam_featureExtraction` | 外部可视化 | 边缘特征 |
| `lio_sam/feature/cloud_surface` | `sensor_msgs/PointCloud2` | `lio_sam_featureExtraction` | 外部可视化 | 平面特征 |
| `lio_sam/feature/cloud_info` | `lio_sam/cloud_info` | `lio_sam_featureExtraction` | `lio_sam_mapOptmization` | 带角点/面点的扫描信息 |
| `lio_sam/mapping/odometry` | `nav_msgs/Odometry` | `lio_sam_mapOptmization` | `TransformFusion`；外部消费者 | LiDAR 建图校正位姿 |
| `lio_sam/mapping/odometry_incremental` | `nav_msgs/Odometry` | `lio_sam_mapOptmization` | `IMUPreintegration`；可选 `elevation_rbf` | 地图优化增量校正 |
| `odometry/imu_incremental` | `nav_msgs/Odometry` | `IMUPreintegration` | `ImageProjection`、`TransformFusion` | IMU 高频传播结果和点云初值 |
| `odometry/imu` | `nav_msgs/Odometry` | `TransformFusion` | 仓库内无主消费者 | LiDAR 校正与 IMU 增量融合后的高频里程计 |
| `lio_sam/imu/path` | `nav_msgs/Path` | `TransformFusion` | RViz/外部消费者 | 最近约 1 秒的 IMU 融合路径 |
| `lio_sam/lidar_calib/z_offset` | `std_msgs/Float64` | `lio_sam_mapOptmization` | `lio_sam_imageProjection` | 在线 LiDAR z 偏移反馈 |
| `lio_sam/rbf/residual_stats` | `std_msgs/Float64MultiArray` | `lio_sam_mapOptmization` | 仓库内无消费者 | 主轮地约束残差诊断 |

## 2.3 地图、回环与可视化输出

| Topic | Message type | Publisher | Subscriber | Purpose |
|---|---|---|---|---|
| `lio_sam/mapping/trajectory` | `sensor_msgs/PointCloud2` | `lio_sam_mapOptmization` | RViz/外部消费者 | 关键帧位置点云 |
| `lio_sam/mapping/path` | `nav_msgs/Path` | `lio_sam_mapOptmization` | RViz/外部消费者 | 全局关键帧路径 |
| `lio_sam/mapping/map_local` | `sensor_msgs/PointCloud2` | `lio_sam_mapOptmization` | `elevation_rbf`，通过 launch remap | 局部平面地图 |
| `lio_sam/mapping/map_global` | `sensor_msgs/PointCloud2` | `lio_sam_mapOptmization` | RViz/外部消费者 | 周期发布的全局可视化地图 |
| `lio_sam/mapping/cloud_registered` | `sensor_msgs/PointCloud2` | `lio_sam_mapOptmization` | RViz/外部消费者 | 当前降采样特征在 odom 中的配准结果 |
| `lio_sam/mapping/cloud_registered_raw` | `sensor_msgs/PointCloud2` | `lio_sam_mapOptmization` | RViz/外部消费者 | 当前高分辨率扫描在 odom 中的配准结果 |
| `lio_sam/mapping/slam_info` | `lio_sam/cloud_info` | `lio_sam_mapOptmization` | 外部消费者 | 新关键帧、关键帧位姿和局部地图 |
| `lio_sam/mapping/icp_loop_closure_history_cloud` | `sensor_msgs/PointCloud2` | `lio_sam_mapOptmization` | RViz/诊断 | 回环历史子图 |
| `lio_sam/mapping/icp_loop_closure_corrected_cloud` | `sensor_msgs/PointCloud2` | `lio_sam_mapOptmization` | RViz/诊断 | ICP 回环校正云 |
| `/lio_sam/mapping/loop_closure_constraints` | `visualization_msgs/MarkerArray` | `lio_sam_mapOptmization` | RViz | 回环节点和边 |
| `/rbf_elevation_map` | `sensor_msgs/PointCloud2` | `elevation_rbf` | RViz/外部消费者 | 独立 CUDA RBF 地形可视化，不反馈主优化器 |

服务 `/lio_sam/save_map` 的类型为 `lio_sam/save_map`，由 `lio_sam_mapOptmization` 提供，用于保存 PCD 地图。

# 3. Coordinate Frames

## 3.1 主 LIO frame 链

| frame | parent | meaning | configuration source |
|---|---|---|---|
| `map` | 根 frame | 全局地图 frame | `lio_sam/mapFrame`，`paramsLivoxIMU.yaml` |
| `odom` | `map` | 连续里程计/建图输出 frame | `lio_sam/odometryFrame`；`TransformFusion` 发布恒等 `map -> odom` |
| `base_link` | `odom` | 当前配置同时作为 LiDAR frame 和机器人 base frame | `lidarFrame`、`baselinkFrame`；`mapOptmization::publishOdometry()` 发布 `odom -> lidarFrame` |
| `odom_mapping` | 仅消息字段 | 地图里程计消息的 `child_frame_id` | `mapOptmization.cpp`；未发现同名 TF 广播 |
| `odom_imu` | 仅消息字段 | IMU 预积分消息的 `child_frame_id` | `imuPreintegration.cpp`；未发现同名 TF 广播 |
| `wheel_L_Link` / `wheel_R_Link` | 当前 TF 链 **UNKNOWN** | 历史左右轮 link 名称 | `fk.py`；其引用的 Tron1A URDF 已删除 |

当 `lidarFrame != baselinkFrame` 时，`TransformFusion` 会查找 `lidarFrame -> baselinkFrame`，再发布 `odom -> baselinkFrame`。当前二者都设为 `base_link`，所以这一分支不执行。

`elevation_rbf` 将 `/rbf_elevation_map` 的 frame 固定为 `map`；它接收的 `map_local` 实际由主优化器以 `odometryFrame` 发布。当前 `map -> odom` 是恒等变换，所以数值上重合，但 frame 标签的语义并不完全一致。

## 3.2 M20 URDF 与 Airy frame

M20 URDF 采用 `base_link`，模型坐标方向为 `+X` 前、`+Y` 左、`+Z` 上。与 Airy
有关的已确认记录如下：

| frame | parent | meaning | configuration source |
|---|---|---|---|
| `front_lidar_link` | `base_link` | 前 Airy 安装 link；xyz `[0.32028,0,-0.013]`，rpy `[0,0,0]` | `robot-description/M20/urdf/M20.urdf` |
| `rear_lidar_link` | `base_link` | 后 Airy 安装 link；xyz `[-0.32028,0,-0.013]`，rpy `[0,0,0]` | 同上 |
| `rslidar_front` | **UNKNOWN** | 驱动给前点云/IMU写入的 frame id | `support/rslidar_ros2_ws/config/m20_airy.yaml` |
| `rslidar_rear` | **UNKNOWN** | 驱动给后点云/IMU写入的 frame id | 同上 |

驱动配置同时给前雷达设置 transform xyz `[0.0501,0,0.739]`、pitch `+1.570795`，
给后雷达设置 xyz `[-0.32028,0,-0.013]`、pitch `-1.57079`。前者的位置和两者的
旋转均与 URDF 不一致。该 transform 的目标 frame、是否按 `ENABLE_TRANSFORM=ON`
实际编译生效，以及为何仍使用 `rslidar_front/rear` frame id，均为 **UNKNOWN**。

部署时必须选定唯一变换责任边界：要么驱动输出原始 Airy frame、由 RBF-LIO/TF
统一变换到 `base_link`；要么驱动完成点云变换、RBF-LIO 不再重复变换，并将消息
frame 语义同步改正确。当前资料不足以确认哪种状态正在实机上运行。

## 3.3 外参配置

外参位于所加载 YAML 的 `lio_sam` 命名空间：

- `extrinsicRot`：在 `imuConverter()` 中旋转 IMU 加速度和角速度。
- `extrinsicRPY`：转为 `extQRPY = Quaternion(extrinsicRPY).inverse()`，用于 IMU 姿态。
- `extrinsicTrans`：IMU 预积分模块中构造 LiDAR/IMU 平移关系；当前代码在这里使用单位旋转。
- `pointCloudTransformEnable`、`pointCloudRot`、`pointCloudTrans`：在所有点云处理前直接变换每个 MID360 点。
- `enableLidarZCalibration` 及 `lidarZ*`：在 `pointCloudTrans.z` 基础上叠加在线动态 z 偏移。

当前 `paramsLivoxIMU.yaml` 对 `_141` 设备设置：

```yaml
extrinsicTrans: [0.0, 0.0, -0.15]
extrinsicRot: [1, 0, 0,
               0, -1, 0,
               0, 0, -1]
extrinsicRPY: [1, 0, 0,
               0, 1, 0,
               0, 0, 1]
pointCloudTransformEnable: true
pointCloudTrans: [0.0, 0.0, 0.15]
pointCloudRot: [1, 0, 0,
                0, -1, 0,
                0, 0, -1]
```

这些值只代表仓库当前机器人配置。其标定方法、测量不确定度和是否适用于其他设备均为 **UNKNOWN**。

M20/Airy 不能沿用上述 MID360 外参。M20 URDF只给出机体到 LiDAR 安装 link；前
Airy DIFOP 中 LiDAR 与内置 IMU 的工厂 `q/t` 尚未提供。通用驱动 transform 不能
代替这一内部标定。

# 4. Sensor Configuration

## 4.1 当前 MID360 配置

| Category | Parameter | Current value | Code use |
|---|---|---|---|
| Topic | `pointCloudTopic` | `/livox/lidar_10_192_1_141` | Livox `CustomMsg` 订阅 |
| Topic | `imuTopic` | `/livox/imu_10_192_1_141` | 两个 IMU 消费节点 |
| LiDAR organization | `N_SCAN` | `4` | range image 行数、特征数组大小 |
| LiDAR organization | `Horizon_SCAN` | `6000` | 每行容量、Livox 顺序列索引上限 |
| Ring reduction | `downsampleRate` | `1` | 跳过 `ring % downsampleRate != 0` 的点 |
| Range filter | `lidarMinRange` | `1.0 m` | 投影前剔除近点 |
| Range filter | `lidarMaxRange` | `40.0 m` | 投影前剔除远点 |
| Feature | `edgeThreshold` | `1` | 边缘特征阈值 |
| Feature | `surfThreshold` | `0.1` | 平面特征阈值 |
| Feature minimum | `edgeFeatureMinValidNum` | `10` | 扫描到地图优化有效角点下限 |
| Feature minimum | `surfFeatureMinValidNum` | `100` | 扫描到地图优化有效面点下限 |
| Voxel filter | `odometrySurfLeafSize` | `0.2 m` | 当前面点下采样 |
| Voxel filter | `mappingCornerLeafSize` | `0.1 m` | 地图角点下采样 |
| Voxel filter | `mappingSurfLeafSize` | `0.2 m` | 地图面点下采样 |
| Mapping cadence | `mappingProcessInterval` | `0.15 s` | 限制地图优化处理频率 |

代码/配置没有 LiDAR 扫描频率参数。真实 MID360 的输出频率以及该配置对应的实际频率为 **UNKNOWN**，不能把 `mappingProcessInterval` 当作传感器频率。

## 4.2 点云预处理

`ImageProjection::moveFromCustomMsg()` 执行：

1. 读取 `x/y/z`、`reflectivity`、`line` 和 `offset_time`。
2. 可选应用 `pointCloudRot * p + pointCloudTrans`。
3. 在 z 上叠加在线 `lidarZOffsetDynamic`。
4. 将 `offset_time` 从纳秒乘 `1e-9` 转成秒。
5. 按距离、ring 范围、`downsampleRate` 和 range-image 重复单元过滤。

代码要求转换后的 PCL 云为 dense；检测到非 dense 时会报错并关闭节点。驱动侧 NaN 过滤方式为 **UNKNOWN**。

## 4.3 IMU 配置

`paramsLivoxIMU.yaml` 当前设置：

- `imuType: 0`
- `imuAccNoise: 1.0e-04`
- `imuGyrNoise: 1.0e-04`
- `imuAccBiasN: 1.0e-04`
- `imuGyrBiasN: 1.0e-05`
- `imuGravity: 9.8035`
- `imuRPYWeight: 0.00`
- `failureDetectionVelTh: 30.0`
- `failureDetectionAccBiasTh: 3.0`
- `failureDetectionGyrBiasTh: 1.0`

噪声参数的测量或标定来源为 **UNKNOWN**。README 只说明出现 IMU 里程计抖动时需要调节这些参数。

## 4.4 已确认的 Airy 驱动配置

来源：`support/rslidar_ros2_ws/config/m20_airy.yaml` 和现场启动日志。

| Item | Front | Rear |
|---|---|---|
| LiDAR type | `RSAIRY` | `RSAIRY` |
| MSOP / DIFOP / IMU port | `6691 / 7781 / 6681` | `6692 / 7782 / 6682` |
| multicast | `224.10.10.201` | `224.10.10.202` |
| point topic | `/rslidar_points_front` | `/rslidar_points_rear` |
| IMU topic | `/rslidar_imu_data_front` | `/rslidar_imu_data_rear` |
| frame id | `rslidar_front` | `rslidar_rear` |
| range | `0.2..200 m` | `0.2..60 m` |
| timestamp | LiDAR clock；header uses first point | 同左 |
| dense points | `true` | `true` |
| azimuth | `0..360 deg` | `0..360 deg` |

测试 bag 显示 Airy 点云为 96 rings、约 10 Hz，点字段包含 float64 绝对
`timestamp`；前 Airy IMU 约 200 Hz，加速度量级约 `1 g`。这些事实足以设计
PointCloud2 适配和相对时间转换，但不提供 IMU 噪声或单机外参。

# 5. Launch Procedure

## 5.1 启动顺序

```text
Livox driver + IMU publisher + joint-state publisher
  -> 检查 topic 类型、名称和时间戳
  -> RBF-LIO 主节点
  -> 可选独立 CUDA elevation_rbf
  -> RViz / odometry / map / trajectory files
```

## 5.2 传感器驱动

README 只确认 MID360 驱动应以 `CustomMsg` 模式启动，并点名 `livox_lidar_msg.launch`。仓库不包含驱动包、驱动配置或 MID360 网络配置，因此可确认的命令只能写成：

```bash
source <UNKNOWN_LIVOX_WORKSPACE>/devel/setup.bash
roslaunch <UNKNOWN_LIVOX_DRIVER_PACKAGE> livox_lidar_msg.launch
```

安装的是 `livox_ros_driver` 还是 `livox_ros_driver2`、launch 参数、设备 IP 和广播码均为 **UNKNOWN**。驱动启动后，其实际 topic 必须与 `paramsLivoxIMU.yaml` 一致。

关节驱动也必须先发布 `/joint_states`，但仓库没有对应硬件驱动命令：

```text
UNKNOWN: start Tron1A joint-state publisher/robot driver
```

GNSS 驱动和 `odometry/gpsz` 生产者命令同样为 **UNKNOWN**。

### M20/Airy 已确认启动入口

完整外部 ROS 2 驱动工作区中的现场命令为：

```bash
ros2 launch rslidar_sdk start.py
```

现场日志确认该命令启动 `rslidar_sdk_node` 和 RViz2，并加载
`/home/user/rslidar_ros2_ws/src/rslidar_sdk/config/config.yaml`。仓库中的参考工作区
通过官方 submodule 和外置 M20 配置保存该版本；仍需实际 ROS 2 环境及依赖才能复现构建。

不要把 `support/rslidar_start.sh` 当作已验证入口：它 source ROS 2 install
workspace，却调用 ROS 1 `roslaunch rslidar_sdk start.launch`，与现场命令不一致。

临时 M20 UDP bridge 的可确认测试命令记录在压缩包 README 中：

```bash
source /opt/ros/noetic/setup.bash
source ~/m20_udp_ws/devel/setup.bash
roslaunch m20_udp_bridge m20_udp_bridge.launch
```

它可将约 10 Hz UDP 状态缓存分别补发为约 200 Hz `/IMU` 和约 500 Hz
`/JOINTS_DATA`。该方式只允许接口联调；最终部署必须由 GOS 直接提供真实 500 Hz
`JOINTS_DATA`。

## 5.3 启动主 RBF-LIO

首先确保输出目录已经存在；TUM 写入器不会创建父目录：

```bash
mkdir -p /absolute/output
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_tron1a_all.launch \
  with_rbf:=false \
  with_robot_tf:=false \
  with_rviz:=true \
  tum_trajectory_path:=/absolute/output/trajectory_tum.txt
```

来源：`LIO-SAM-MID360/launch/run_tron1a_all.launch`。

该历史 launch 仍引用已删除的 Tron1A URDF，且主点云类型仍是 Livox
`CustomMsg`。因此当前不能把上述命令与 Airy 驱动串联成可用 M20 系统；M20 launch、
ROS 1/ROS 2 边界和 PointCloud2/关节适配均为 **TODO**。

这里的 `with_rbf:=false` 只关闭独立 `elevation_rbf`/`fk.py` 辅助链；主地图优化中的轮地 RBF 约束仍由 `paramsLivoxIMU.yaml` 的 `enableRbfConstraint: true` 启用。这是依赖最少的主定位部署方式。

如需独立 CUDA 地形可视化和三项统计文件：

```bash
mkdir -p /absolute/output/rbf_stats
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_tron1a_all.launch \
  with_rbf:=true \
  with_robot_tf:=false \
  with_rviz:=true \
  tum_trajectory_path:=/absolute/output/trajectory_tum.txt \
  rbf_file_path:=/absolute/output/rbf_stats
```

`with_rbf:=true` 还会启动 `rbf_cuda/script/fk.py`。该脚本的默认 URDF 路径是 `/root/code/robot-description/pointfoot/WF_TRON1A/urdf/robot.urdf`，而 `run_rbf.launch` 没有覆盖它；目标机器若不存在该路径，脚本无法按默认值部署。如何在现有综合 launch 中提供可移植路径为 **TODO / 当前未配置**。

此外，`fk.py` 只把输入中的 `joint_0` 至 `joint_7` 映射为 Tron1A 关节名；如果 `/joint_states` 已经使用 `abad_L_Joint` 等具名关节，该脚本不会把这些位置加入其 FK 输入。该 TF 可视化脚本不参与主 `JointStateWheelKinematics` 的轮地约束计算。

## 5.4 可选机器人 TF 与 RViz

- `with_rviz:=true` 启动 `module_rviz.launch` 中的 RViz 配置。
- `with_robot_tf:=true` 启动关节名适配器和 `robot_state_publisher`。
- 关节名适配器默认把 `/joint_states` 复制到 `/joint_states_tron1a`，将 `joint_0..7` 改为 Tron1A 名称，并把时间戳刷新为 `ros::Time::now()`。
- 主 RBF 运动学仍直接订阅原始 `/joint_states`，不订阅 `/joint_states_tron1a`。
- 启用机器人 TF 前必须处理第 3.2 节中的 `base_link` / `base_Link` 图断开问题；仓库没有现成修复参数。

# 6. Runtime Outputs

## 6.1 ROS 输出

- 位姿：`lio_sam/mapping/odometry`、`lio_sam/mapping/odometry_incremental`、`odometry/imu`、`odometry/imu_incremental`。
- 路径：`lio_sam/mapping/path`、`lio_sam/imu/path`。
- 点云：去畸变云、角点、面点、局部地图、全局地图、配准云和回环诊断云，见第 2 节。
- RBF：`lio_sam/rbf/residual_stats`、`lio_sam/lidar_calib/z_offset`；可选 `/rbf_elevation_map`。
- 服务：`/lio_sam/save_map`。

`lio_sam/rbf/residual_stats` 当前数组顺序由 `mapOptmization.cpp` 确认如下：

```text
[mean, mean_abs, ema, lidar_z_offset, constraint_count,
 mean_r1, mean_r2, mean_r3, mean_r4]
```

## 6.2 TF 输出

- `map -> odom`：`TransformFusion` 以恒等变换发布。
- `odom -> lidarFrame`：`mapOptmization` 按 LiDAR 优化位姿发布；当前即 `odom -> base_link`。
- `odom -> baselinkFrame`：仅当 `lidarFrame != baselinkFrame` 时由 `TransformFusion` 发布。
- 机器人 link TF：可选 `robot_state_publisher`。
- 独立轮 link TF：`fk.py` 发布，依赖其 URDF 路径和关节名称处理。

## 6.3 文件输出

TUM 轨迹格式：

```text
timestamp x y z qx qy qz qw
```

可选 `elevation_rbf` 在 `rbf_file_path` 下覆盖创建：

```text
kernel_matrix_time.txt   # timestamp kernel_time_ms
weight_time.txt          # timestamp weight_time_ms
mae.txt                  # timestamp mae
```

调用保存地图服务：

```bash
rosservice call /lio_sam/save_map "resolution: 0.0
destination: '/rbf_lio_result/map'"
```

实现会先删除 `$HOME/rbf_lio_result/map`，再写出：

```text
trajectory.pcd
transformations.pcd
CornerMap.pcd
SurfMap.pcd
GlobalMap.pcd
```

`paramsLivoxIMU.yaml` 还设置 `savePCD: true` 和 `$HOME/Downloads/LOAM/` 默认目录。保存实现包含递归删除目标目录的操作，实机上必须使用隔离路径。

# 7. Parameter Adaptation

## 7.1 从代码确认：更换 LiDAR/IMU 必须核对的接口

这些项目直接决定代码能否解释输入：

- `pointCloudTopic`、`imuTopic` 与消息包版本。
- 输入必须是 Livox `CustomMsg`，并提供 `point_num`、`line`、`offset_time`、`reflectivity` 等字段。
- `N_SCAN`、`Horizon_SCAN`、`downsampleRate`、`lidarMinRange`、`lidarMaxRange`。
- `imuType`、`imuGravity`、IMU 噪声和 bias random walk。
- `extrinsicRot`、`extrinsicRPY`、`extrinsicTrans`。
- `pointCloudTransformEnable`、`pointCloudRot`、`pointCloudTrans`。
- `lidarFrame`、`baselinkFrame`、`odometryFrame`、`mapFrame`。

更换为非 Livox LiDAR不能只把 `sensor` 改成 `velodyne` 或 `ouster`，因为主订阅仍是 Livox `CustomMsg`。当前仓库没有已确认的配置级适配方法。

## 7.2 从代码确认：更换机器人必须核对的接口

- `jointStateTopic` 和 `sensor_msgs/JointState.position` 数量。
- 八个关节名称或数组顺序。
- `wheelRadius`。
- `rbf_wheel_kinematics.cpp` 中写死的左右腿连杆偏移、旋转轴和符号。
- `robot_description` 使用的 URDF，以及 URDF link/joint 名称。
- `lidarFrame`/`baselinkFrame` 与 URDF 根、base link 的名称连接。

当前 `JointStateWheelKinematics` 是 Tron1A 两轮结构，不能仅通过 YAML 切换为其他腿数、轮数或机构。

M20 迁移还必须以 `robot-description/M20/urdf/M20.urdf` 为准核对 16 个关节的
顺序、轴向、零位和限位，生成四个轮心/轮轴，并将轮半径改为 `0.09 m`。测试
`M20JointsData` 中 12 个字段是腿关节角，4 个 wheel 字段是 rad/s 轮速；当前主
接触约束只需要轮几何，并不使用轮速作为里程计因子。

## 7.3 建议检查：部署调参项

以下是基于代码用途的检查建议，不是仓库已经为新平台验证过的事实：

- 建议从保守的距离范围和体素分辨率开始，检查去畸变云与特征云非空后再提高点数。
- 建议用静止数据检查 IMU 转换后的重力方向、量级和角速度零偏，再调整噪声参数。
- 建议在关闭 `enableLidarZCalibration` 时先验证静态外参，避免动态偏移掩盖错误安装变换。
- 建议先确认 `/joint_states` 的角度单位、正方向和左右顺序，再启用 `enableRbfConstraint`。
- 建议分别记录开启/关闭 RBF 约束的轨迹，确认约束确实进入 LM，而不是因缺少关节数据被跳过。
- 建议在生产部署中覆盖所有 `/root/...` 输出路径，并限制地图保存目标，防止删除非实验目录。
- 建议首先关闭驱动侧或 RBF-LIO 侧其中一处点云刚体变换，用静态目标确认只有一处负责 `Airy -> base_link`，再启用完整估计。

# 8. Time Synchronization

## 8.1 LiDAR 时间

- 扫描起始时间 `timeScanCur` 来自 Livox `CustomMsg.header.stamp`。
- 每点相对时间来自 `point.offset_time * 1e-9` 秒。
- 扫描结束时间是 `timeScanCur + laserCloudIn->points.back().time`。
- 代码使用最后一个点的相对时间作为扫描时长，因此输入点是否按时间排列会影响扫描结束时间；驱动排序保证为 **UNKNOWN**。
- 点云队列累计到 3 帧后才处理最早一帧，即 `cloudQueue.size() <= 2` 时返回。这是缓存延迟，不是时间戳修正。

## 8.2 IMU 覆盖与插值

`ImageProjection::deskewInfo()` 只有在 IMU 队列同时覆盖扫描开始和结束时才处理该扫描：

```text
imu.front.stamp <= timeScanCur
imu.back.stamp  >= timeScanEnd
```

它删除早于 `timeScanCur - 0.01 s` 的旧 IMU，积分角速度，并在每点时间上线性插值累计旋转。超过 `timeScanEnd + 0.01 s` 后停止本次积分。

平移去畸变代码当前被注释，`findPosition()` 始终返回零；实际启用的是旋转去畸变。`odometry/imu_incremental` 仍用于扫描初值和可用性判断。

## 8.3 IMU 预积分时间

IMU 预积分使用相邻 `sensor_msgs/Imu.header.stamp` 的差值作为 `dt`。第一次积分没有上一个时间戳时使用固定 `1/500 s`。代码没有把 IMU 频率暴露为配置参数，所以真实 IMU 频率与第一次默认 `dt` 是否一致为 **UNKNOWN**。

## 8.4 GNSS 与关节时间

- GPS 消息只有落在当前 LiDAR 时间 `+-0.2 s` 窗口内才会被考虑，见 `mapOptimization::addGPSFactor()`。
- 主 `JointStateWheelKinematics` 只缓存最新一帧关节位置，不使用 `JointState.header.stamp` 与 LiDAR 时间做关联或插值。
- `joint_state_name_adapter.py` 可刷新输出时间戳，但主轮地约束仍读取原始 `/joint_states`。
- 仓库没有 LiDAR-IMU 时间偏移参数、在线时间标定、硬件同步配置或 GNSS/关节插值器。

因此 README 的“对齐 LiDAR 与 IMU 时间戳”是部署前置条件，而不是软件自动完成的功能。

## 8.5 M20/Airy 时间事实

- 驱动设置 `use_lidar_clock: true`、`ts_first_point: true`，测试 bag 中点
  `timestamp` 为绝对时间。PointCloud2 适配必须将其转换为相对
  `point_time - header.stamp`，并检查非负、单调和扫描时长。
- 前后点云约 10 Hz，前后对应帧 header 的中位差约 `59 us`、观测最大值约
  `5 ms`；第一阶段只使用前路，因此无需双雷达帧融合同步。
- 前 Airy IMU 约 200 Hz。当前主 IMU 预积分第一次 `dt` 默认 `1/500 s`，与该频率
  不同；虽然后续 `dt` 使用消息时间戳，迁移时仍应修正或验证首样本处理。
- 测试 `/JOINTS_DATA` 的 ROS 发布频率约 500 Hz，但 `source_sequence` 仅约 10 Hz
  更新。现有主运动学只缓存最新关节，不做 LiDAR 时刻插值；直接使用重复样本会使
  接触几何产生阶梯状时延。

# 9. Troubleshooting

| Symptom | Confirmed check entry | Code/config source |
|---|---|---|
| 收不到点云 | 核对 `pointCloudTopic` 的 `_123`/`_141` 差异和实际消息 MD5；确认是 Livox `CustomMsg` | `paramsLivoxIMU.yaml`、`utility.h` |
| 节点报告未知传感器并退出 | `sensor` 必须为代码接受的字符串；主回调实际要求 `livox` | `ParamServer`、`ImageProjection::cachePointCloud()` |
| 点云节点报告等待 IMU | 检查 IMU 队列是否覆盖完整扫描 `[timeScanCur,timeScanEnd]` | `ImageProjection::deskewInfo()` |
| 点云节点因非 dense 退出 | 检查驱动输出是否含 NaN；仓库没有驱动侧过滤参数 | `ImageProjection::cachePointCloud()` |
| 去畸变/地图方向错误 | 核对 `pointCloudRot/Trans` 与 `extrinsicRot/RPY/Trans` 是否表达在一致目标 frame | `paramsLivoxIMU.yaml`、`imuConverter()` |
| IMU 加速度量级约 9.8 倍错误 | 核对 `imuType: 0` 的输入单位是否确实为 `g` | `imuConverter()` |
| IMU 里程计频繁重置 | 检查速度与 bias 是否超过 `failureDetection*`；再核对噪声、时间戳和单位 | `IMUPreintegration::failureDetection()`、YAML |
| 特征或局部地图为空 | 检查距离过滤、ring 是否落在 `[0,N_SCAN)`、`Horizon_SCAN` 容量、特征阈值和体素尺寸 | `imageProjection.cpp`、`featureExtraction.cpp`、YAML |
| RBF 约束持续跳过 | 检查 `enableRbfConstraint`、至少 8 个关节位置、`rbfNodeMaxNum`、局部面地图和日志中的 `[lio_sam][rbf-lm][skip]` | `mapOptmization::updateRbfLmConstraints()`、`rbf_wheel_kinematics.cpp` |
| 历史 launch 报 URDF 文件不存在 | `run_tron1a_all.launch` 仍指向已删除的 Tron1A 文件；需使用后续 M20 launch | `run_tron1a_all.launch`、`robot-description/M20` |
| `fk.py` 启动失败或轮 TF 不随关节变化 | 检查硬编码 `/root/code/robot-description/.../robot.urdf`、Python 依赖，以及输入名称是否为脚本实际处理的 `joint_0..7` | `rbf_cuda/script/fk.py` |
| `/rbf_elevation_map` 不更新 | 确认 `with_rbf=true`、`map_local` 和 mapping incremental odometry 非空；节点需平面移动超过 `0.5 m` 才触发后续更新 | `run_rbf.launch`、`elevation_rbf.cpp` |
| GPS 因子未使用 | 检查实际是否有 `nav_msgs/Odometry` 发布到 `odometry/gpsz`；仓库 navsat 链默认名称不匹配 | `paramsLivoxIMU.yaml`、`module_navsat.launch` |
| TUM 文件不存在 | 父目录必须预先存在；检查 launch 覆盖路径和启动日志中的打开失败警告 | `mapOptimization` 构造函数 |
| 保存地图误删目录 | `destination` 会拼到 `$HOME`，保存函数先执行递归删除 | `mapOptimization::saveMapService()` |
| Airy 点云方向/高度明显错误 | 对比驱动 transform 与 M20 URDF；确认 `ENABLE_TRANSFORM`，排除驱动和主代码重复变换 | Airy config、M20 URDF、`moveFromCustomMsg()` |
| Airy 点云与 IMU去畸变方向不一致 | 不能仅检查相同 frame id；还需取得前 Airy DIFOP 内部 q/t，并验证 IMU 三轴是否被驱动旋转 | Airy wrapper/config、`imuConverter()` |
| M20 关节几何跳变或越界 | 检查 `source_sequence/repeated/source_age`，并对每个 APDU 值执行 URDF 符号/零偏/限位验证 | `support/m20_udp_bridge.zip`、M20 URDF |
| ROS 2 Airy topic 在 ROS 1 不可见 | 检查实际 ROS 1/ROS 2 桥接或将输入层移植到 ROS 2；仓库当前无已确认桥 | Airy ROS 2 snapshot、主 ROS 1 package |

# 10. Deployment Checklist

## 10.0 M20 迁移门槛

- [x] 已确认 Airy SDK 版本、两台雷达端口、topic、frame id 和时间戳选项。
- [x] 已取得包含前/后点云、前/后 IMU 和 M20 测试消息的 rosbag。
- [ ] 已取得前 Airy DIFOP LiDAR-IMU 工厂旋转/平移标定；当前 **UNKNOWN**。
- [ ] 已确认驱动 `ENABLE_TRANSFORM` 构建状态，并解释驱动 transform 与 URDF 的差异。
- [ ] 已选择并实现 ROS 1/ROS 2 数据边界。
- [ ] 已用 GOS 原生真实 500 Hz `/JOINTS_DATA` 替换测试重复发布数据。
- [ ] 已建立 APDU 到 URDF joint 的符号、零位、顺序和限位转换测试。

## 10.1 传感器与标定

- [ ] 已确认实际 LiDAR 是当前接口支持的 Livox MID360，并以 `CustomMsg` 发布。
- [ ] 已记录编译时实际选择的 `livox_ros_driver` 或 `livox_ros_driver2` 消息包。
- [ ] 已核对 LiDAR 和 IMU topic，而不是直接沿用 `_141` 默认值。
- [ ] 已确认每点 `line`、`offset_time`、点序和扫描头时间戳有效。
- [ ] 已确认 IMU 是六轴还是九轴，并核对线加速度单位。
- [ ] 已在目标机器人上验证 `extrinsicRot`、`extrinsicRPY`、`extrinsicTrans`。
- [ ] 已验证 `pointCloudRot`、`pointCloudTrans` 与 IMU 外参使用同一目标 frame。
- [ ] 已确认 LiDAR/IMU 时间覆盖满足每帧去畸变要求；软件没有时间偏移参数。
- [ ] 已确认 `/joint_states` 至少包含 8 个位置，名称、顺序、单位和方向正确。
- [ ] 已核对轮半径和 `rbf_wheel_kinematics.cpp` 中的 Tron1A 几何。

## 10.2 ROS 与 TF

- [ ] 已确认 `roscore`/ROS master 和所有机器的网络、时钟配置；仓库配置方法为 **UNKNOWN**。
- [ ] 已启动 Livox/IMU 驱动，并确认消息类型与构建时头文件一致。
- [ ] 已启动机器人关节驱动；具体命令为 **UNKNOWN**。
- [ ] 如使用 GNSS，已提供 `odometry/gpsz` 发布者并验证 covariance；当前仓库链不完整。
- [ ] 已确认 `map -> odom -> base_link` 连通且没有重复发布冲突。
- [ ] 如启用机器人模型，已处理 `base_link` / `base_Link` 差异并检查完整 TF tree。
- [ ] 如启用独立 `fk.py`，已确认 URDF 路径和关节名输入有效。

## 10.3 启动与健康检查

- [ ] 已创建 TUM 和 RBF 统计输出目录，并覆盖 launch 的 `/root/...` 默认路径。
- [ ] 已启动 `run_tron1a_all.launch`，四个 LIO 核心节点均持续运行。
- [ ] 去畸变点云、角点、面点和 `map_local` 均非空。
- [ ] `lio_sam/mapping/odometry` 与 `odometry/imu` 连续发布，时间戳单调且数值有限。
- [ ] 日志没有持续出现 waiting IMU、invalid quaternion、unknown sensor 或 failure reset。
- [ ] RBF 约束启用时，日志没有持续显示缺关节/空节点等 skip 原因。
- [ ] `lio_sam/rbf/residual_stats` 和 `lio_sam/lidar_calib/z_offset` 为有限值。
- [ ] 如启用 `elevation_rbf`，已确认 `/rbf_elevation_map` 和三个统计文件更新。
- [ ] TUM 文件持续追加有效的 `timestamp x y z qx qy qz qw`。
- [ ] 保存地图前已确认目标目录可被安全删除和重建。
- [ ] 已在静止、低速和目标地形上分别验证轨迹与地图；仓库没有规定验收阈值，阈值为 **UNKNOWN**。
