# RBF-LIO Ubuntu 20.04 开发交接

## 1. 文档用途

本文档用于把 Windows 开发阶段已经确认的事实、已完成的 M20 迁移改动、验证证据和下一步工作交给 Ubuntu 20.04 上的新开发者或后续 Codex 会话。

建议按以下顺序阅读：

1. `AGENTS.md`：仓库约束、事实来源和编码规范。
2. 本文档：当前迁移状态、关键决策和接续步骤。
3. `README.md`：项目入口和最短命令。
4. `docs/ARCHITECTURE.md`：真实软件架构和数据流。
5. `docs/DEPLOYMENT.md`：传感器、坐标系和部署检查。
6. `docs/ALGORITHM.md`：数学概念与代码实现映射。
7. `docs/EXPERIMENT.md`：现有实验能力和复现缺口。

不要把聊天记录作为工程事实来源。若本文档与代码冲突，以当前代码为准，并把差异记录为 TODO。

## 2. Git 与数据迁移

- 开发分支：`4wheel_dev`
- 远端：`git@github.com:lolcheng/rbf_lio_src.git`
- 本交接基线：以包含 `docs/HANDOFF_UBUNTU.md` 的提交为准。
- RoboSense 官方代码通过 Git submodule 管理，克隆时必须递归初始化。
- `data/rosbag/*.bag` 被 `.gitignore` 排除，不会随 Git 仓库迁移。

Ubuntu 上克隆：

```bash
git clone --recurse-submodules -b 4wheel_dev \
  git@github.com:lolcheng/rbf_lio_src.git
cd rbf_lio_src
git submodule update --init --recursive
git status --short
```

测试包需通过移动硬盘、局域网或其他大文件渠道单独复制：

```bash
mkdir -p data/rosbag
# 将 m20_upstairs.bag 复制到 data/rosbag/
sha256sum data/rosbag/m20_upstairs.bag
```

已知测试包信息：

| 项目 | 值 |
|---|---|
| 路径 | `data/rosbag/m20_upstairs.bag` |
| 字节数 | `420763439` |
| SHA-256 | `335b47e3c771679478aa56e747e6ad12fc79a405c3cf51998124d426ea138304` |

若校验值不同，必须重新确认包内话题和消息字段，不能沿用本文档中的离线检查结果。

## 3. 已确认的迁移范围

M20 第一阶段主估计链只使用：

- 前置 Airy 点云：`/rslidar_points_front`，`sensor_msgs/PointCloud2`。
- 前置 Airy IMU：`/rslidar_imu_data_front`，`sensor_msgs/Imu`。
- M20 机体关节数据：`/JOINTS_DATA`，经适配器转为 `/joint_states_m20`。

第一阶段明确不使用：

- 后置 Airy 点云。
- 后置 Airy IMU。
- UDP bridge 中另一个 `/IMU` 话题。

M20 关节轴、零位、限位、连杆几何和轮半径以 `robot-description/M20/urdf/M20.urdf` 为唯一代码侧依据。测试 bridge 的 APDU 数值不能覆盖 URDF 约定。

## 4. 当前代码已完成的内容

### 4.1 Airy 点云输入

`LIO-SAM-MID360/src/imageProjection.cpp` 已增加 RoboSense Airy `PointCloud2` 入口：

- 读取 `x/y/z/intensity/ring/timestamp` 字段。
- 校验字段类型和数量。
- 将绝对逐点时间转换为扫描内相对时间。
- 依据 `ring` 和每线顺序组织 96 线点云。
- 在主包内应用 `pointCloudRot` 和 `pointCloudTrans`。
- 保留 Livox 输入支持；Livox 头文件和构建依赖改为可选。

当前 Airy 配置使用 `N_SCAN: 96`、`Horizon_SCAN: 1024`。

### 4.2 M20 四轮运动学

`LIO-SAM-MID360/include/rbf_wheel_kinematics.h` 和 `src/rbf_wheel_kinematics.cpp` 已增加 `M20JointStateWheelKinematics`：

- 使用 FL、FR、HL、HR 四条 URDF 运动链。
- 输入 12 个髋/膝关节位置。
- 输出四个 `WheelGeometry`，供轮地接触残差使用。
- `src/mapOptmization.cpp` 根据 `robotKinematicsModel` 选择 `m20` 或历史 `tron1a` 实现。

### 4.3 关节消息适配

新增：

- `LIO-SAM-MID360/msg/M20JointsData.msg`
- `LIO-SAM-MID360/scripts/m20_joint_state_adapter.py`

本地消息定义的 ROS 1 MD5 为 `fb86770770e356b2075846629adc617b`，与测试 bag 中 `/JOINTS_DATA` 发布者一致。适配器完成：

- 映射到 M20 URDF 中的 16 个标准关节名。
- 支持参数化比例和偏置。
- 对非有限值和陈旧输入做检查。
- 仅在源时间戳或序号变化时积分轮角，避免把重复发布误当成新测量。

重要限制：当前 `support/m20_udp_bridge.zip` 把约 10 Hz 的有效源样本重复发布成约 500 Hz。这只能用于接口测试；最终版本必须由 GOS 直接提供真实 500 Hz `JOINTS_DATA`。

### 4.4 IMU、外参与时间

用户提供的前 Airy DIFOP 内部标定为：

```text
rotation [x,y,z,w] = [-0.703521, 0.710655, -0.00485685, 0.00199339]
translation [x,y,z] = [0.00425, 0.00418, -0.00446]
```

结合 M20 URDF 安装关系后，`LIO-SAM-MID360/config/paramsM20.yaml` 使用：

```text
extRot = [
   0.009667018, -0.004098306, -0.999944875,
  -0.999902180,  0.010069117, -0.009707874,
   0.010108348,  0.999940907, -0.004000567
]
extTrans = [0.315796292, 0.004164198, -0.017240556]
```

点云变换由主包完成，配置为绕 Y 轴 `+pi/2` 和平移 `[0.32028, 0, -0.013]`。参考 Airy 驱动必须保持 `ENABLE_TRANSFORM=OFF`，否则会重复应用安装变换。

`LIO-SAM-MID360/include/utility.h` 新增 `imuFrequency` 参数，`src/imuPreintegration.cpp` 用它替代固定的初始 `1/500 s`。M20 配置设为 `200 Hz`。

### 4.5 启动和构建入口

新增：

- `LIO-SAM-MID360/config/paramsM20.yaml`
- `LIO-SAM-MID360/launch/run_m20.launch`

M20 launch 默认 `enable_rbf_constraint:=false`。原因不是功能缺失，而是测试 bag 中已有若干关节值超出 URDF 限位，尚不能用它验证轮地几何约束。

`robot-description/CMakeLists.txt` 已改为安装现存的 M20 模型。主包的 CMake/package manifest 已加入消息生成、Python 脚本安装和 M20 运行依赖。

## 5. 关键实现文件

| 文件 | 当前作用 |
|---|---|
| `LIO-SAM-MID360/src/imageProjection.cpp` | Livox/Airy 点云接入、缓存、坐标变换和去畸变 |
| `LIO-SAM-MID360/src/imuPreintegration.cpp` | IMU 预积分、偏置估计和高频传播 |
| `LIO-SAM-MID360/src/mapOptmization.cpp` | LOAM LM、M20/Tron1A 运动学选择、RBF 接触残差和因子图 |
| `LIO-SAM-MID360/src/rbf_wheel_kinematics.cpp` | Tron1A 与 M20 轮心/轮轴计算 |
| `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp` | 当前实际使用的 CPU 接触点、残差和数值 Jacobian |
| `LIO-SAM-MID360/scripts/m20_joint_state_adapter.py` | `/JOINTS_DATA` 到标准 `JointState` 的临时适配 |
| `LIO-SAM-MID360/config/paramsM20.yaml` | M20/Airy 主配置和组合外参 |
| `LIO-SAM-MID360/launch/run_m20.launch` | M20 ROS 1 启动入口 |
| `robot-description/M20/urdf/M20.urdf` | M20 运动学和安装几何事实来源 |
| `support/rslidar_ros2_ws` | Airy ROS 2 官方驱动参考，不属于 ROS 1 主估计包 |

主实现是 `LIO-SAM-MID360`。`rbf_cuda/src/lio_sam` 是旧版或实验性副本，不应自动同步修改。

## 6. 已完成的离线验证

这些检查已在 Windows 上完成，但不等于 ROS 运行验证：

- `m20_joint_state_adapter.py` 通过 Python AST 语法解析。
- `paramsM20.yaml` 可被 YAML 解析。
- `run_m20.launch`、`package.xml` 和 M20 URDF 可被 XML 解析。
- 本地 `M20JointsData.msg` MD5 与 bag 发布者完全一致。
- 测试 bag 中前置点云共 165 帧，最大 ring 为 95，每 ring 最大点数为 902。
- 逐点相对时间范围约为 `0.0000343` 至 `0.1005988 s`，未发现超出当前检查范围的点。
- 前置 IMU 共 3304 条，`/JOINTS_DATA` 共 8258 条。
- 组合 IMU 旋转矩阵行列式约为 1，正交误差约为 `1.08e-9`。
- 随机 400 组有效关节样本与独立 URDF FK 对照，最大轮几何位置误差为 0。
- Markdown 本地链接检查通过。
- `git diff --check` 无空白错误，仅出现 Windows CRLF 提示。

当前 Windows 环境没有 ROS Noetic、Catkin 或可用的 Linux 容器，因此尚未完成 C++ 编译、ROS 节点启动或 rosbag 端到端运行。

## 7. Ubuntu 首次构建

仓库根目录 `CMakeLists.txt` 是复制后失效的 Catkin 顶层符号链接文本，不要为此修改它。使用外部 Catkin 工作区：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/rbf_lio_ws/src
cd ~/rbf_lio_ws/src
ln -s /absolute/path/to/rbf_lio_src/LIO-SAM-MID360 lio_sam
ln -s /absolute/path/to/rbf_lio_src/robot-description robot_description
catkin_init_workspace
cd ~/rbf_lio_ws
catkin_make
source devel/setup.bash
```

先只构建 `lio_sam` 和 `robot_description`，以缩小 M20 主链验证范围。只有需要独立 CUDA 高程/RBF 节点时，才把 `rbf_cuda` 链接为 `~/rbf_lio_ws/src/cuda_rbf` 并安装 CUDA 相关依赖后重编译。

仓库可以确认的主要依赖见 `README.md` 和 `docs/ARCHITECTURE.md`。具体 GCC、CUDA、GTSAM 小版本未被仓库锁定，当前为 **UNKNOWN**。

## 8. 测试 bag 首次运行

先关闭 RBF 接触约束，只验证传感器入口、去畸变和基本 LIO：

```bash
# Terminal 1
mkdir -p ~/rbf_lio_results
source ~/rbf_lio_ws/devel/setup.bash
roslaunch lio_sam run_m20.launch \
  use_sim_time:=true \
  with_rviz:=true \
  with_robot_tf:=true \
  enable_rbf_constraint:=false \
  tum_trajectory_path:=$HOME/rbf_lio_results/m20_upstairs_tum.txt
```

```bash
# Terminal 2
source ~/rbf_lio_ws/devel/setup.bash
rosbag play --clock \
  /absolute/path/to/rbf_lio_src/data/rosbag/m20_upstairs.bag
```

至少检查：

```bash
rosnode list
rostopic hz /rslidar_points_front
rostopic hz /rslidar_imu_data_front
rostopic hz /JOINTS_DATA
rostopic hz /joint_states_m20
rostopic hz /lio_sam/mapping/odometry
rostopic echo -n 1 /lio_sam/mapping/odometry
rostopic echo -n 1 /lio_sam/mapping/map_local
```

还需观察控制台是否出现点字段不匹配、IMU 初始化失败、时间回退、空点云、NaN、TF 缺失或节点退出。

测试 bag 通过上述基础检查后，只能说明 ROS 接口与基本估计链可运行，不能证明 M20 轮地约束正确。

## 9. 实机 Airy 启动边界

已确认的 Airy 驱动入口是：

```bash
ros2 launch rslidar_sdk start.py
```

参考日志显示前雷达输出 `/rslidar_points_front`，前 IMU 输出 `/rslidar_imu_data_front`。`support/rslidar_start.sh` source ROS 2 工作区却调用 ROS 1 `roslaunch`，不能作为已验证命令。

主估计器是 ROS 1 Noetic，驱动是 ROS 2。仓库目前没有一套已验证的 ROS 1/ROS 2 bridge 启动文件、QoS 配置或消息桥接验收结果。该跨版本连接仍为 **UNKNOWN / TODO**。不要在未测试的情况下宣称实机链路已接通。

## 10. 下一阶段任务顺序

1. 在 Ubuntu 20.04 + ROS Noetic 上完成 `lio_sam` 与 `robot_description` 的干净编译，修复真实编译错误。
2. 用 `m20_upstairs.bag` 在 `enable_rbf_constraint:=false` 下跑通全部核心节点，确认点云、IMU、TF、里程计和地图非空且有限。
3. 检查 `/JOINTS_DATA` 的原始单位、方向、零位、限位和源时间戳。当前 bag 超限原因未确认，状态为 **UNKNOWN**。
4. 接入 GOS 直接产生的真实 500 Hz 关节数据；不得把 10 Hz 重复方案当成最终性能验证输入。
5. 在有效关节数据下启用 `enable_rbf_constraint:=true`，监控 `lio_sam/rbf/residual_stats`、LM 增量和地图表现。
6. 建立并验证 ROS 2 Airy 到 ROS 1 RBF-LIO 的桥接启动流程，记录 QoS、时间戳和 frame 行为。
7. 用静止、平地、坡面、楼梯等可控数据校验外参方向和重力方向，再调整 IMU 噪声、滤波和 RBF 参数。
8. 补充 ground truth 和评估脚本。仓库当前没有可确认的 ATE、z-ATE 或 RTE 完整评估链。

## 11. 启用 RBF 前的门槛

只有满足以下条件后才建议启用主 RBF 约束：

- 四条腿的关节名称、顺序、单位和方向与 URDF 一致。
- 关节值在 URDF 合理范围内，且不是由旧数据重复伪造高频。
- 轮心和轮轴在 RViz 或离线 FK 检查中符合真实姿态。
- 前 Airy 点云和 IMU 已落在代码预期的统一坐标约定中。
- 基础 LIO 在关闭 RBF 时能稳定运行。

启用方式：

```bash
roslaunch lio_sam run_m20.launch enable_rbf_constraint:=true
```

启用后必须确认 `lio_sam/rbf/residual_stats` 有有效更新，并检查优化增量和输出中不存在非有限值。

## 12. 仍然未知或不可确认的事项

- 当前测试 bag 中关节超出 URDF 限位的根本原因：单位、零位、方向、解析或采集问题均未由仓库证据确认。
- 最终 GOS 500 Hz 消息是否继续保持当前 `/JOINTS_DATA` 字段、MD5 和时间戳语义。
- ROS 2 驱动到 ROS 1 主链的生产级 bridge 方案及 QoS。
- 前 Airy DIFOP 标定值的设备序列号、标定日期和厂商坐标约定原始文档位置。
- M20 实机上 IMU 噪声、偏置、重力方向和时间同步的最终标定结果。
- 论文数据集、ground truth、ATE/z-ATE/RTE 评估脚本和论文图表复现输入。
- 当前参数在 M20 楼梯场景的最优值；现有数值只能作为代码接入初值。

## 13. 交接完成判定

在 Ubuntu 上完成以下事项后，才可认为开发上下文已经真正接续：

- `git status --short` 干净，submodule 均已初始化。
- bag SHA-256 与本文档一致。
- Catkin 干净构建成功并保存完整构建日志。
- M20 launch 在关闭 RBF 时跑通测试 bag。
- 输出话题、TF、点云、轨迹和地图均经过实际检查。
- 所有新发现的代码事实、参数差异和失败命令被更新回本文档或对应专题文档。
