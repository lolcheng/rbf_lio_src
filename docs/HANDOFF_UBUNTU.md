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

当前任务是工作站上的 ROS 1 rosbag 离线处理。bag 已包含所需输入，运行时不需要
Airy 驱动、ROS 1/ROS 2 在线桥接或机载 GPU；下面的在线接口记录供将来采集参考。
保留原代码的接触假设和主 RBF 求值方式，本次迁移不新增逐轮触地判断或关节插值。

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
- 启动时从 `/robot_description` 读取 12 个腿关节限位，直接收到的超限
  `JointState` 也会先按 URDF 截断再进入运动学。
- `src/mapOptmization.cpp` 根据 `robotKinematicsModel` 选择 `m20` 或历史 `tron1a` 实现。

### 4.3 关节消息适配

新增：

- `LIO-SAM-MID360/msg/M20JointsData.msg`
- `LIO-SAM-MID360/scripts/m20_joint_state_adapter.py`

本地消息定义的 ROS 1 MD5 为 `fb86770770e356b2075846629adc617b`，与测试 bag 中 `/JOINTS_DATA` 发布者一致。适配器完成：

- 映射到 M20 URDF 中的 16 个标准关节名。
- 按 `paramsM20.yaml` 中的方向、零偏和角度归一化参数转换到 URDF 坐标。
- 对非有限值和陈旧输入做检查。
- 从 `/robot_description` 读取 12 个腿关节限位，并在发布前按 URDF 截断超限值。
- 仅在源时间戳或序号变化时积分轮角，避免把重复发布误当成新测量。

当前 `support/m20_udp_bridge.zip` 把约 10 Hz 的有效源样本重复发布成约 500 Hz。
离线回放保留这些消息及源时间戳，不把它们解释为 500 Hz 独立测量；将来采集真实
500 Hz GOS 数据属于数据源改进，不是运行现有 bag 的前提。

#### 编码器到 URDF 的转换（2026-11-05）

依据厂家 [M20 SDK 仿真接口的 JOINT_DIR / POS_OFFSET_DEG](https://github.com/DeepRoboticsLab/sdk_deploy/blob/main/src/M20_sdk_deploy/interface/robot/simulation/mujoco_simulation_ros2.py)，
结合本地 URDF 和测试 bag 的起始站姿检查，采用：

```text
q_urdf = atan2(sin(s * q_raw + b), cos(s * q_raw + b))
```

`q_raw`、`b` 均以 rad 输入；下表为便于核对的度数。零偏与厂家数组相差整圈的
写法等价，配置中保留厂家原始零偏。腿顺序为 FL、FR、HL、HR，每腿为 hipx、hipy、knee。

| 腿 | 方向 s | 等效零偏 b（度） | bag 起始 URDF 角（度） |
|---|---|---|---|
| FL | `+1,+1,-1` | `-25,-131,+160` | `2.088,-36.450,69.532` |
| FR | `+1,-1,+1` | `+25,-131,+160` | `-7.714,-38.550,75.830` |
| HL | `-1,+1,-1` | `-25,+131,-160` | `8.645,35.933,-67.846` |
| HR | `-1,-1,+1` | `+25,+131,-160` | `-2.107,29.857,-63.165` |

先转换并归一化，再按 `/robot_description` 限位截断。整包 8258 条消息、166 个
不同源样本的 12 个腿关节转换后均在本地 URDF 限位内，无需截断。起始四轮轮心
`base_link z` 约为 `[-0.4161,-0.4121,-0.4337,-0.4313] m`，与所给站姿外观相符。
轮速只按 `[+1,-1,+1,-1]` 转换方向；连续轮角不归一化。

转换仅在适配器执行一次。直接发布 `/joint_states_m20` 时必须已经使用 URDF
坐标。输入若换成已转换的 URDF 角，应同时设 `position_scale=[1]*12`、
`position_offset=[0]*12`、`wrap_positions=false`，并按新输入约定设置轮速方向。
厂家来源是 SDK 仿真接口，不是该 bag 的 APDU 固件标定记录；当前证据支持此 bag
和站姿的转换，尚无逐关节实机测量或轨迹 ground truth，不据此声称定位精度已验证。

### 4.4 IMU、外参与时间

用户提供的前 Airy DIFOP 内部标定为：

```text
rotation [x,y,z,w] = [-0.703521, 0.710655, -0.00485685, 0.00199339]
translation [x,y,z] = [0.00425, 0.00418, -0.00446]
```

结合 URDF 的 LiDAR 原点位置和 bag 中核对的原始点云轴向后，
`LIO-SAM-MID360/config/paramsM20.yaml` 使用：

```text
extRot = [
   0.009667018, -0.004098306, -0.999944875,
   0.999902180, -0.010069117,  0.009707874,
  -0.010108348, -0.999940907,  0.004000567
]
extTrans = [0.315796292, -0.004164198, -0.008759444]
```

点云变换由主包完成，旋转为 `Rx(pi)*Ry(pi/2)`，即
`base X=raw Z, base Y=-raw Y, base Z=raw X`，平移 `[0.32028,0,-0.013]`。
URDF 的 `front_lidar_link` 描述安装 link，rpy 为零；它没有定义 bag 的原始 Airy
输出轴，不能把该零 rpy 直接作为原始点云旋转。参考驱动必须保持
`ENABLE_TRANSFORM=OFF`，否则会重复应用安装变换。

`LIO-SAM-MID360/include/utility.h` 新增 `imuFrequency` 参数，`src/imuPreintegration.cpp` 用它替代固定的初始 `1/500 s`。M20 配置设为 `200 Hz`。

2026-11-05 检查发现，旧 `Ry(pi/2)` 会把起始地面放在机体上方约 `0.439 m`。
此前只把 `imuAccelerationSign` 设为 `-1` 虽使静止加速度指向 +Z，却未修正
点云及角速度方向。现已统一修正旋转，并恢复 `imuAccelerationSign: +1`。
首条加速度转换后约为 `[-0.501,0.073,9.775] m/s^2`。不能仅凭静止加速度
或轨迹数值有限判断坐标正确，必须同时核对地面方向与动态角速度。

### 4.5 启动和构建入口

新增：

- `LIO-SAM-MID360/config/paramsM20.yaml`
- `LIO-SAM-MID360/launch/run_m20.launch`

M20 launch 保留 `enable_rbf_constraint:=false` 的基础 LIO 默认值。
现有 bag 已配置编码器到 URDF 的转换，可以显式传入 `true` 做离线开关对照；
重复消息不妨碍回放，但限制了可用关节观测的时间分辨率。

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

## 6. 已完成的验证

以下静态/离线检查已完成：

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

以下为关节转换接入前的历史验证记录。2026-09-09 在 Ubuntu 20.04.6 / ROS Noetic 主机上完成了构建与测试 bag
回放。验证环境为 GCC 9.4、GTSAM 4.0.3、PCL 1.10、OpenCV 4.2、CUDA 11.8、
RTX 4070 SUPER（驱动 570.181）：

- 外部 Catkin 工作区中的 `lio_sam`、`cuda_rbf`、`robot_description` 全量构建成功；
  包括 CUDA RBF 库、独立 `elevation_rbf` 和主 LIO 节点。
- 源码定义的 3 个 Python 用例和 1 个 C++ rostest 均通过：覆盖适配器的 URDF
  限位解析/截断，以及四轮运动学的零位轮心/轮轴、四轮数量、有限值、直接输入
  限位截断和 12 组确定性关节姿态的独立 URDF 链 FK 对照。
- 关闭 RBF 回放得到 81 条有限 TUM 位姿；修正 Airy 加速度符号后，末端位置范数
  从错误符号时的约 `215.8 m` 降至约 `4.32 m`。bag 没有 ground truth，因此
  该数值只证明严重发散被消除，不是定位精度结论。
- 开启 RBF 的接口回放收到 129 组有限残差统计，约束数始终为 `24`
  （四轮，每轮 6 项）；同时收到 8258 条适配关节、81 条建图里程计、162 帧
  去畸变点云和 81 帧局部地图，检查字段中没有 NaN/Inf。
- 开启 RBF 的 TUM 轨迹同样为 81 条有限位姿；由于关节坐标未完成实机标定，
  此次只能证明四轮约束已进入 LM 且数值可运行，不能比较约束效果优劣。
- 独立 `elevation_rbf` 也已随同一 bag 完成运行验证：日志显示 CUDA
  核矩阵和权重求解实际执行；收到 18 帧每帧 360,000 点的有限高程图，
  三项统计文件各 18 行且均为有限值。节点中原本无效的 `/root` JSON 死代码已删除，
  统计目录现由 C++ 创建，默认为 `/tmp/rbf_lio_rbf`。

### 6.1 2026-11-05 关节转换接入后、Airy 坐标修正前的记录

在外部工作区 `/tmp/rbf_lio_m20_calibrated.OEZZYo` 完成三个包的全量构建。
8 个 Python 用例和 1 个 C++ 运动学 rostest 通过；Catkin 连同测试包装层汇总为
`11 tests, 0 errors, 0 failures`。新增覆盖实际 bag 起始角、整圈等价角、非法输入、
源时间戳、四轮速度方向和重复样本不重复积分。

使用第 2 节同一 SHA-256 的 bag，在独立 ROS master 上以 1 倍速 `--clock`
回放；启动 `run_m20.launch use_sim_time:=true with_rviz:=false with_robot_tf:=true`，
分别设置 `enable_rbf_constraint:=false/true`，辅助高程节点关闭。验证脚本同时
核对 ROS 参数服务器中的转换参数，确认 launch 未用旧单位比例覆盖 YAML。

| 检查项 | RBF 关闭 | RBF 开启 |
|---|---|---|
| 原始输入点云 / IMU / 关节消息数 | `165 / 3304 / 8258` | `165 / 3304 / 8258` |
| 适配关节消息数 | `8258` | `8258` |
| 关节位置/轮速相对预期转换的最大误差 | `0` | `0` |
| 转换后超限 / 限位截断 | `0 / 0` | `0 / 0` |
| 去畸变 / 角点特征 / 平面特征帧数 | `162 / 162 / 162` | `162 / 162 / 162` |
| 建图里程计 / TUM 位姿数 | `81 / 81` | `81 / 81` |
| 局部地图帧数 | `81`，初始化后 `80` 帧非空 | `81`，初始化后 `80` 帧非空 |
| 有限 RBF 统计 / LiDAR z 偏移消息数 | `0 / 0` | `133 / 133` |
| 每组 RBF 残差行数 | 无 | 恒为 `24`，LM 日志 `used=true` |
| 末端位置 `[x,y,z] m` | `[-3.520378,0.364039,2.167010]` | `[-6.022881,0.190210,-1.688915]` |

关节输出保持原始 `source_stamp`，覆盖约 `1785308857.9133` 至
`1785308874.4766`。去畸变点云每帧 `40963..50734` 点，角点 `48..228`，平面点
`1503..4942`，均非空且坐标有限。关节、里程计、TUM、残差及发布的 z 偏移未发现
NaN/Inf；TUM 时间严格递增。没有节点退出/重启、超限截断或 IMU 速度/偏置重置
警告。内部每次 LM 增量和 IMU 偏置未单独导出，本次有限值检查针对发布结果。
保留的 KDL 根 link 惯量提示不影响运动学发布；关闭 RBF 时的 disabled 提示符合配置。

开启 RBF 后末端 z 与关闭时方向不同，不能据此认定约束改善了楼梯轨迹。
本次完成编码器坐标适配和运行验证；没有轨迹真值或逐关节实机标定记录，
保留 RBF 默认关闭，原算法、权重和接触假设未作调整。
本机日志、两组 TUM 和 `result.json` 位于该外部工作区的
`replay_off_final/`、`replay_on_final/`，整包关节范围记录在 `joint_validation.json`；
这些是临时验证产物，不提交仓库。

### 6.2 2026-11-05 修正 Airy 坐标倒置

用户观察到楼梯上下翻转后，重新提取 bag 原始点云并重放。起始地面平面代表点
`p_raw=[-0.4517166,0.0588006,0.2882193] m`，旧旋转将其放在 base 的
`z=+0.4387166 m`，与 URDF 中车轮位于机体下方矛盾。错误在输入坐标，
不属于 RViz 相机视角或关节方向问题。

旋转修正为 `R_base_lidar=Rx(pi)*Ry(pi/2)`，将该点放到
`z=-0.4647166 m`；同步重算 `extrinsicRot/Trans`，恢复 `imuAccelerationSign=+1`。
起始 0.6 s 静止加速度均值转换后约为 `[-0.0526,0.0198,1.0001] g`。
仅作诊断地对照 bag 的 `/IMU` 机体姿态变化，旧 IMU Y/Z 角速度相关系数约为
`-0.67/-0.78`，修正后为 `+0.67/+0.78`。`/IMU` 未加入主估计输入。
URDF、轮半径、关节转换、RBF 算法和权重均未因该修正改变。

三个包再次构建成功；新增 3 个传感器坐标测试，加上已有 8 个关节适配测试及
C++ 运动学测试均通过，Catkin 汇总 `14 tests, 0 errors, 0 failures`。
新增地面测试在旧配置下明确失败（地面 z 为正），在修正配置下通过；另检查
静止 specific force、DIFOP 旋转/平移组合、右手系和 URDF 安装位置一致性。

同一 bag、1 倍速 `--clock`、`use_sim_time:=true`、RViz/辅助高程节点关闭、
机器人 TF 开启；最终直接使用仓库 YAML，不使用试验参数覆盖：

| 检查项 | RBF 关闭 | RBF 开启 |
|---|---|---|
| 输入点云 / IMU / 关节数 | `165 / 3304 / 8258` | `165 / 3304 / 8258` |
| 适配关节数 / 转换最大误差 | `8258 / 0` | `8258 / 0` |
| 去畸变 / 角点 / 平面特征帧数 | `162 / 162 / 162` | `162 / 162 / 162` |
| 建图里程计 / TUM 位姿数 | `81 / 81` | `81 / 81` |
| 局部地图帧数 | `81`，首帧初始化为空 | `81`，首帧初始化为空 |
| 有限残差统计 / z 偏移数 | `0 / 0` | `112 / 112` |
| 每组 RBF 约束数 | 无 | 恒为 `24`，LM `used=true` |
| 末端位置 `[x,y,z] m` | `[-1.657061,5.503480,3.362785]` | `[-5.976706,-0.301077,1.498529]` |
| 末端 roll/pitch/yaw（度） | `[34.03,30.80,-94.67]` | `[-1.04,-3.45,-13.02]` |

输入、点云和发布结果均为有限值，TUM 时间严格递增，无节点退出/重启、关节
截断、速度/偏置重置警告。开启时 z 偏移保持零，残差统计 mean_abs 约为
`0.135..0.363`，24 行残差持续进入主 LM。地图 X-Z 侧视确认地面在下、楼梯
沿机器人运动方向升高；修正前同次基线回放的 RBF 末端 z 为 `-0.998497 m`。
更早的 6.1 节数值为错误坐标下的历史结果，不能继续作为正确运行的依据。

当前已修复坐标倒置，但关闭 RBF 的基础 LIO 仍明显漂移，最大 roll/pitch 约为
`89.46/62.29` 度；不能把输出有限或末端 z 为正当作稳定运行证据。此问题应继续
独立定位，不能将本次结果表述为定位精度验收。内部逐次 LM 增量和 IMU 偏置
仍未单独导出，本次有限值检查针对发布结果。

本机诊断产物位于 `/tmp/m20_inversion.YV8Dop`：`before_on/` 为旧配置基线，
`final_off/`、`final_on/` 为最终 YAML 回放，含日志、TUM、残差 JSON 和地图数组；
`map_before_after.png` 为同一 X-Z 视角的修正前后地图/轨迹对照。均不提交仓库。

### 6.3 2026-11-05 RViz 轴向与地图清晰度分析

用户提供 M20 截图，并以 `~/2026-07-28 12-02-07.mp4` 为原版参考。已抽取原版
15.834 s 视频的全程和末尾帧；其台阶较清晰，也存在少量拖影。没有与该视频对应
的原始 bag、轨迹和录制时完整配置，不能由像素画面对两个系统作定量精度排名。

**轴向。** 当前 RViz 默认启用的三色轴是 `Reference Frame: base_link`，
`map` 轴默认关闭。M20 URDF 中前髋位于 `x=+0.3141`、后髋位于 `x=-0.3141`，
左髋位于正 y，因此机体约定为 X 前、Y 左、Z 上。bag 的 166 个独立机体状态中，
约 91% 的样本 `LinearX < -0.1 m/s`，中位数为 `-0.4696 m/s`；基线 LIO 轨迹
换算成机体系后的 X 向速度中位数为 `-0.4359 m/s`、Y 向为 `-0.0203 m/s`。
两者支持此包主要沿机体 -X 上楼。观察者沿 -X 看向楼上时，+Y 在画面右侧，
+X 指向观察者，符合截图的“蓝上、绿右、红后”。没有证据要求交换 M20 的 X/Y。

本机原版工程的 RViz 也配置 `base_link`，但 Tron1A URDF 根名为 `base_Link`
（大小写不同），历史 launch 默认关闭机器人 TF。视频中未显示 frame 属性，
不能据“绿沿楼梯”推断那一定是 URDF 机体坐标，也不能确认录制时与当前原版
检出完全一致。绕世界 Z 轴统一旋转地图与轨迹只会改变世界朝向，不会改变
机体轴和行进方向的相对关系，更不会改善几何厚度。

**清晰度。** 使用独立 ROS master `11333`，不影响用户已有 `11311` RViz 会话。
同一 bag 以 1 倍速回放六次，RBF 均开启，辅助高程节点关闭。原始输入均完整
收到 `165 / 3304 / 8258` 条点云/IMU/关节；均产生 162 帧去畸变/特征云、
81 条建图里程计和 81 帧已配准点云，四轮约束始终 24 行。没有节点退出、
速度/偏置重置或非有限输出。此次只运行临时对照，正式代码和配置未改。

为区分单帧噪声与多帧误配准，在初始前方墙面固定 ROI
`x=[0.7,1.6], y=[-0.25,0.25], z=[0.2,1.2] m` 中拟合平面，
以到平面的有符号距离 `P90-P10` 作为点云厚度；各组使用同一 ROI。
该指标衡量这一墙面的相对几何一致性，不是 ATE，也不代表整个场景。

| 临时试验 | 累积墙面厚度 P90-P10 |
|---|---|
| 当前配置，基线第一次 | `31.49 cm` |
| 当前配置，独立重复 | `28.05 cm` |
| 仅 `lidarMinRange: 0.2 -> 1.0` | `23.96 cm` |
| 仅扣除起始 0.5 s 陀螺均值，保持原时间戳 | `27.05 cm` |
| 两个平面体素 `odometrySurfLeafSize/mappingSurfLeafSize: 0.2 -> 0.1` | `26.16 cm` |
| 仅 `rbfConstraintWeight: 0.35 -> 0.35/sqrt(2)` | `25.95 cm` |

基线逐帧墙面厚度中位数约 `3.23 cm`，同一静止墙面的逐帧中心相对初始平面
漂移跨度约 `39.43 cm`；重复基线跨度约 `36.03 cm`。49 个运动帧中，原始
单帧与去畸变单帧的厚度中位数分别约 `3.04/3.13 cm`。因此此 ROI 的主要
增厚发生在多帧累积时，直接问题是帧间位姿/配准不一致。该测量不能单独验证
整幅扫描的去畸变、全部台阶边缘或各个内部优化项。

对照提示：

- M20 起始静止段的 base 角速度均值约 `[-0.02108,+0.02330,-0.00214] rad/s`。
  当前预积分初始偏置为零，`priorBiasNoise=1e-3`；从 IMU 里程计角速度输出
  反推的估计偏置在基线末段已接近 `[-0.01944,+0.02200,-0.00641] rad/s`。
  `imageProjection` 去畸变未接收该估计偏置。初始化/偏置值得优先排查，但扣除
  静止均值的单次试验仍有约 27 cm 累积厚度，尚不能认定它是唯一根因。
- `findPosition()` 在原版和 M20 中都返回零，仅旋转去畸变。bag 速度 P95 约
  `0.723 m/s`，0.1 s 内约移动 `7.2 cm`，可贡献扫描内形变；这不是本次迁移
  新增的差异，不据此直接开启原版未使用的平移补偿。
- 约三分之一输入点位于变换后 base 原点 1 m 内，近距筛选会同时影响地面和
  机身附近观测；不能把全部近点直接视作机身噪声。近距对照的改善仍不充分。
- 原版与 M20 的平面体素同为 0.2 m、角点体素 0.1 m、建图间隔 0.15 s。
  更细体素将 81 帧配准特征总数从约 81,235 增至 231,460，但仍存在明显增厚。
- 主 LM 中权重同时乘 Jacobian 和残差；四轮与两轮的信息量不能只按行数直接
  等同。`1/sqrt(2)` 仅作近似权重对照，结果没有消除漂移，不作为自动改默认值
  的依据。几厘米级的单次改善也应考虑基线重复运行约 3.4 cm 的变动。

下一步优先定位基础 LIO 的 IMU 初始化/偏置、Airy 特征质量与帧间位姿一致性，
再评价 M20 的滤波和四轮权重。单独交换轴、旋转显示或加密点云均不能解决已
测得的累计配准误差；本轮尚未分离出唯一内部根因。

本机临时产物位于 `/tmp/m20_quality.tgSwPL`：六组 `*_on/` 含启动覆盖参数、
完整消息计数、TUM、配准点云和残差；`quality_metrics.json` 为统计，
`quality_summary.png` 为汇总图，`quality_analysis.png` 为墙面/台阶细节，
`original_*png` 为原版视频抽帧。均不提交仓库。

当前登录环境会优先选择 Anaconda CMake/Python/Boost。若不隔离，完整构建会把
Boost 1.82 与 ROS/GTSAM 的系统 Boost 1.71 混链并在链接期失败。构建时应确认
`cmake`、Python 和 Boost 都来自 Ubuntu/ROS 系统环境；至少显式使用：

```bash
export PATH=/usr/local/cuda-11.8/bin:/opt/ros/noetic/bin:/usr/bin:/bin
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

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
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

先只构建 `lio_sam` 和 `robot_description`，以缩小 M20 主链验证范围。只有需要独立 CUDA 高程/RBF 节点时，才把 `rbf_cuda` 链接为 `~/rbf_lio_ws/src/cuda_rbf` 并安装 CUDA 相关依赖后重编译。

仓库可以确认的主要依赖见 `README.md` 和 `docs/ARCHITECTURE.md`。上节版本是
2026-09-09 的成功验证组合，不是仓库锁定的唯一版本。

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
rostopic hz /joint_states_m20
rostopic hz /lio_sam/mapping/odometry
rostopic echo -n 1 /lio_sam/mapping/odometry
rostopic echo -n 1 /lio_sam/mapping/map_local
```

若本机没有安装临时 bridge 包，`rostopic hz /JOINTS_DATA` 会因为无法导入原消息
类型 `m20_udp_bridge/M20JointsData` 而失败，即使本地兼容消息凭相同 MD5 已能让
适配器订阅。此时用 `/joint_states_m20` 验收适配结果，并以适配器连接日志确认
原 topic 已接通。

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

1. 保留当前关节和传感器坐标配置，定位关闭 RBF 时基础 LIO 的严重姿态漂移；
   当前开关回放和坐标修正记录见第 6.2 节。
2. 补充带 ground truth 的数据和评估脚本；当前没有可确认的 ATE、z-ATE 或 RTE
   完整评估链，不能从运行成功推断精度改善。
3. 换用其他机器或固件的数据时，重新核对编码器约定及已知关节姿态。

ROS 2 在线桥接、GOS 原生 500 Hz 采集和机载运行属于后续可选工作，不是当前
离线迁移的阻塞项。

## 11. 启用 RBF 前的门槛

现有 bag 已支持开启主 RBF 约束进行离线对照。换用数据时核对：

- 四条腿的关节名称、顺序、单位和方向与 URDF 一致。
- 转换后关节值在 URDF 合理范围内；重复消息不能被当作独立高频测量。
- 轮心和轮轴在 RViz 或离线 FK 检查中符合真实姿态。
- 前 Airy 点云和 IMU 已落在代码预期的统一坐标约定中。
- 评价 RBF 精度收益前，应先使基础 LIO 在关闭 RBF 时稳定；当前 bag 尚不满足此项，
  仍可开启 RBF 查看地图和验证通路。

启用方式：

```bash
roslaunch lio_sam run_m20.launch use_sim_time:=true enable_rbf_constraint:=true
```

启用后必须确认 `lio_sam/rbf/residual_stats` 有有效更新，并检查优化增量和输出中不存在非有限值。

## 12. 仍然未知或不可确认的事项

- 当前转换与该 bag APDU 固件版本之间尚无厂家标定记录；换固件时应核对已知姿态。
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
