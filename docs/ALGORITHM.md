# RBF-LIO 数学算法与代码映射

本文档建立当前仓库中数学概念、论文公式和实际代码之间的映射。它只陈述能够从以下材料直接确认的事实：

1. 当前主实现 `LIO-SAM-MID360`。
2. 独立 RBF/CUDA 实现 `rbf_cuda`。
3. 参数文件和 launch 文件。
4. `support/RBF_LIO.pdf` 中带编号的公式，仅用于比较论文定义和代码行为。

代码是最终事实来源。论文公式与代码不一致时，本文档同时记录两者，并将不能确认的等价关系标记为 **UNKNOWN**。源码中没有发现对论文公式编号的显式引用，因此下述公式映射是依据表达式和调用关系建立的代码审计结果，不是源码作者留下的声明。

## 1. 算法分层

当前系统包含三个不同的估计/优化层：

1. **当前帧扫描到地图优化**：`mapOptimization::scan2MapOptimization()`；变量是六维欧拉角和平移，残差包括 LiDAR 点线/点面残差和主 RBF 轮地残差。
2. **IMU 预积分图**：`IMUPreintegration`；变量是 GTSAM `Pose3`、速度和 IMU 偏置，地图优化结果作为位姿校正因子。
3. **关键帧地图图优化**：`mapOptimization::saveKeyFramesAndFactor()`；变量是关键帧 `Pose3`，因子包括里程计、GPS 和回环。

主 RBF 约束只直接进入第一层。它不以独立 GTSAM factor 的形式进入 IMU 图或关键帧图；其影响通过当前帧优化位姿和随后生成的关键帧相对位姿间接传递。

独立 `elevation_rbf` 是第四条数值处理链，用于 RBF 拟合、可视化和统计，但其权重/中心当前没有送入上述三个估计层。

## 2. 符号与实现约定

### 2.1 论文位姿

论文式 (1) 将基座位姿写为：

\[
\eta_k = ({}^O R_{B,k}, {}^O t_{B,k}).
\]

### 2.2 扫描匹配位姿

代码使用：

```text
transformTobeMapped = [roll, pitch, yaw, x, y, z]
```

定义和更新位于 `LIO-SAM-MID360/src/mapOptmization.cpp`：

- `updateInitialGuess()` 初始化或传播该数组。
- `LMOptimization()` 对六个分量直接加上 `matX`。
- `trans2Affine3f()` 通过 `pcl::getTransformation()` 将其转为仿射变换。

当前扫描匹配并未使用论文式 (35) 的右扰动 `SO(3)` 局部变量。旋转变量是 Euler RPY，更新是分量相加。

### 2.3 轮地求值位姿

`RbfCudaRequest::pose` 复制 `transformTobeMapped`，顺序不变。`JointStateWheelKinematics::computeWheelGeometry()` 使用：

\[
R = R_z(yaw)R_y(pitch)R_x(roll),\qquad t=[x,y,z]^T.
\]

它将 base 坐标下的轮心和轮轴变换到代码称为 world 的坐标系。默认配置下 `lidarFrame == baselinkFrame == base_link`；非默认帧配置下该位姿是否仍严格表示 base 位姿为 **UNKNOWN**。

M20 迁移尚未改变这段算法代码。`robot-description/M20/urdf/M20.urdf` 是后续四轮
正运动学的几何事实来源；不能把当前 Tron1A 硬编码 offsets 或测试桥中的 APDU
数组直接解释为 M20 的 URDF 关节变量。

## 3. 状态变量

| 数学概念 | 代码变量/类型 | 文件 | 类/函数 | 输入 | 输出/用途 |
| --- | --- | --- | --- | --- | --- |
| 当前 LiDAR 位姿 | `float transformTobeMapped[6]` | `LIO-SAM-MID360/src/mapOptmization.cpp` | `mapOptimization` | IMU 初值、上帧位姿、扫描残差 | 当前帧里程计、关键帧位姿 |
| 扫描匹配增量 | `cv::Mat matX(6,1)` | 同上 | `LMOptimization()` | 线性化矩阵 `matA/matB` | RPY/XYZ 分量增量 |
| 关键帧位姿状态 | GTSAM `Pose3` | 同上 | `addOdomFactor()`、`saveKeyFramesAndFactor()` | 当前扫描匹配位姿 | iSAM2 优化关键帧轨迹 |
| IMU 位姿 | `X(k): gtsam::Pose3` | `LIO-SAM-MID360/src/imuPreintegration.cpp` | `IMUPreintegration` | IMU 预积分、地图校正 | `prevPose_` |
| IMU 速度 | `V(k): gtsam::Vector3` | 同上 | `IMUPreintegration` | IMU factor | `prevVel_` |
| IMU 偏置 | `B(k): ConstantBias` | 同上 | `IMUPreintegration` | 偏置随机游走 factor | `prevBias_` |
| 接触点 | 局部变量 `Eigen::Vector3f p` | `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp` | `solveContactOnWheel()` | 轮几何、地形、轮半径 | 单次残差求值使用，不持久化 |
| RBF 权重（主约束） | `rbf_nodes[i].z()` | 同上 | `evaluateTerrain()` | 局部地图平面点 | 主轮地地形高度/梯度 |
| RBF 权重（独立拟合） | `rbf_weight_`、`roi_rbf_weight_` | `rbf_cuda/include/rbf_fitting/rbf_fit.h` | `VoxelGridRBF` | CUDA 权重求解结果 | 高程拟合和可视化 |
| RBF 权重协方差 | `weight_cov_R_` | 同上 | `updateWeightCuda()` | 旧协方差、观测 | 递归更新；当前活动链未调用 |

### 3.1 论文 IMU 状态式 (29)

论文定义：

\[
s_k=({}^O R_{B,k},{}^O t_{B,k},{}^O v_{B,k},b^a_k,b^g_k).
\]

最接近的代码映射是 `X(k), V(k), B(k)`，由 GTSAM shorthand 在 `imuPreintegration.cpp` 顶部定义。该映射在变量组成上明确，但论文的 base 状态与代码内部 IMU `Pose3` 之间还包含 `lidar2Imu/imu2Lidar` 平移外参。

## 4. LiDAR 特征与约束

### 4.1 特征选择

**数学概念**：从有序扫描中选取局部曲率大的边缘点和曲率小的平面点。

**映射**：

| 项目 | 实现 |
| --- | --- |
| 参数 | `edgeThreshold`、`surfThreshold`、`odometrySurfLeafSize`、`N_SCAN`、`Horizon_SCAN` |
| 文件 | `LIO-SAM-MID360/src/featureExtraction.cpp` |
| 类 | `FeatureExtraction` |
| 函数 | `calculateSmoothness()`、`markOccludedPoints()`、`extractFeatures()` |
| 输入 | `lio_sam/deskew/cloud_info` 中的去畸变点、range、列号和 ring 索引 |
| 输出 | `cloud_corner`、`cloud_surface` 和 `lio_sam/feature/cloud_info` |
| 调用者 | `FeatureExtraction::laserCloudInfoHandler()` |

曲率是五点窗口的 range 差平方。遮挡和近似平行光束会被排除。此处没有统计概率模型或显式 cost function。

### 4.2 点到线残差

论文式 (32) 用 `d_e` 表示边缘点到线残差。代码映射为 `mapOptimization::cornerOptimization()`：

1. 将当前角点按当前位姿变换到地图。
2. 在 `laserCloudCornerFromMapDS` 中取五个最近邻，要求第五近邻平方距离小于 1。
3. 对五点协方差做特征分解，要求最大特征值大于第二特征值三倍。
4. 用主方向两侧的点构造线，计算点到线距离 `ld2 = a012/l12`。
5. 使用启发式尺度 `s = 1 - 0.9 |ld2|`。
6. 将 `s*[la,lb,lc]` 保存为位置方向系数，将 `s*ld2` 保存为残差。

因此当前被线性化的边缘残差可写为：

\[
r_e = s_e d_e,
\]

其中 `s_e` 会随当前残差重新计算。`la/lb/lc` 是代码推导出的距离对变换后点坐标的方向系数。

### 4.3 点到面残差

论文式 (32) 用 `d_p` 表示平面点到面残差。代码映射为 `mapOptimization::surfOptimization()`：

1. 在局部平面地图中取五个最近邻。
2. 求解 `A*[pa,pb,pc]^T = -1`，随后归一化得到平面 `pa*x+pb*y+pc*z+pd=0`。
3. 要求五个邻点到平面的绝对距离均不超过 0.2。
4. 当前点残差为 `pd2 = pa*x+pb*y+pc*z+pd`。
5. 使用距离相关尺度：

\[
s_p=1-0.9\frac{|pd2|}{\sqrt{\sqrt{x_L^2+y_L^2+z_L^2}}}.
\]

6. 保存 `s_p*[pa,pb,pc]` 和 `s_p*pd2`。

当前线性化平面残差为 `r_p=s_p d_p`。这个权重表达式来自代码；其统计意义为 **UNKNOWN**。

### 4.4 LiDAR Jacobian

`LMOptimization()` 根据当前 Euler 位姿、原始点 `pointOri` 和残差空间系数 `coeff`，显式计算 `arx/ary/arz`，形成每行六维 Jacobian：

```text
[d r / d roll, d r / d pitch, d r / d yaw,
 d r / d x,    d r / d y,     d r / d z]
```

代码中存在 lidar/camera 轴顺序交换，因此矩阵赋值顺序为：

```text
[arz, arx, ary, coeff.z, coeff.x, coeff.y]
```

该 Jacobian 是手工展开表达式。仓库没有单元测试或符号推导文件验证其数学正确性。

## 5. IMU 约束

### 5.1 预积分参数

| 数学概念 | 参数 | 文件/类 | 函数 | 作用 |
| --- | --- | --- | --- | --- |
| 加速度白噪声 | `imuAccNoise` | `imuPreintegration.cpp` / `IMUPreintegration` | 构造函数 | `accelerometerCovariance = I*noise^2` |
| 陀螺白噪声 | `imuGyrNoise` | 同上 | 构造函数 | `gyroscopeCovariance = I*noise^2` |
| 加速度偏置随机游走 | `imuAccBiasN` | 同上 | `odometryHandler()` | Bias BetweenFactor sigma |
| 陀螺偏置随机游走 | `imuGyrBiasN` | 同上 | `odometryHandler()` | Bias BetweenFactor sigma |
| 重力 | `imuGravity` | 同上 | `MakeSharedU()`、`imuConverter()` | 预积分重力；六轴配置下也用于加速度单位转换 |
| LiDAR/IMU 平移 | `extrinsicTrans` | 同上 | `lidar2Imu/imu2Lidar` | 校正 pose 与 IMU pose 转换 |

### 5.2 论文传播式 (30a-c)

论文显式给出旋转、速度、位置离散传播。代码没有逐行实现同一公式，而是调用：

```cpp
gtsam::PreintegratedImuMeasurements::integrateMeasurement(...)
gtsam::PreintegratedImuMeasurements::predict(...)
```

因此可以确认概念对应 GTSAM IMU 预积分，但 GTSAM 内部残差、Jacobians 和积分离散化细节不在仓库中，精确公式等价性为 **UNKNOWN**。

### 5.3 IMU 图 cost / factors

`IMUPreintegration::odometryHandler()` 添加：

- `gtsam::ImuFactor(X(k-1),V(k-1),X(k),V(k),B(k-1),preint_imu)`。
- 零均值 `BetweenFactor<ConstantBias>(B(k-1),B(k))`。
- 地图校正 `PriorFactor<Pose3>(X(k),curPose,correctionNoise)`。
- 初始化时的 Pose、Velocity、Bias prior。

优化器是独立的 `gtsam::ISAM2 optimizer`。精确 cost 是上述 GTSAM factors 的加权平方误差之和，但 factor 内部误差坐标和 Jacobian 由外部 GTSAM 实现，仓库内无法展开，标记为 **UNKNOWN**。

### 5.4 IMU 对扫描匹配的作用

IMU 不作为显式残差行加入 `mapOptimization::LMOptimization()`。它通过两种方式影响扫描匹配：

1. `ImageProjection::imuDeskewInfo()` 提供每点旋转去畸变。
2. 高频预积分里程计经 `cloud_info.initialGuess*` 进入 `mapOptimization::updateInitialGuess()`。

`transformUpdate()` 还按 `imuRPYWeight` 对扫描匹配结果的 roll/pitch 与当前 IMU RPY 做 slerp。该步骤位于局部线性求解之后，不是 LM cost 中的显式 IMU residual。

## 6. RBF 地形模型

### 6.1 论文 RBF basis：式 (8)、(9)

论文定义：

\[
f(x;w)=\sum_{i=1}^{N}w_i\phi(\|x-c_i\|_2),
\qquad
\phi(r_i)=\exp\left(-\frac{r_i^2}{2\sigma^2}\right).
\]

当前代码有两种实现。

### 6.2 主轮地约束中的 RBF

`LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp::evaluateTerrain()` 使用各向异性 Gaussian：

\[
\phi_i(x,y)=\exp\left[-\frac12\left(
\frac{(x-c_{x,i})^2}{\sigma_x^2}+
\frac{(y-c_{y,i})^2}{\sigma_y^2}
\right)\right].
\]

并计算：

\[
h=\sum_i c_{z,i}\phi_i,
\quad
h_x=\sum_i c_{z,i}\phi_i\left(-\frac{x-c_{x,i}}{\sigma_x^2}\right),
\quad
h_y=\sum_i c_{z,i}\phi_i\left(-\frac{y-c_{y,i}}{\sigma_y^2}\right).
\]

映射：

| 项目 | 实现 |
| --- | --- |
| 数学概念 | RBF 高程、x/y 一阶导数 |
| 参数 | `rbfSigmaX`、`rbfSigmaY`、`rbfNodeMaxNum` |
| 文件 | `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp` |
| 类/函数 | 匿名命名空间 `evaluateTerrain()` |
| 输入 | `RbfCudaRequest::rbf_nodes`、查询 `(x,y)` |
| 输出 | `h`、`hx`、`hy` |
| 调用者 | `solveContactOnWheel()` |

这里的中心是局部地图平面点 `(x,y)`，权重是同一点的 `z`。这些点没有经过独立 `VoxelGridRBF` 权重拟合。将地图点高度直接解释为核权重的数学依据在代码和注释中均未说明，标记为 **UNKNOWN/TODO**。

主配置 `paramsLivoxIMU.yaml` 使用 `rbfSigmaX=rbfSigmaY=0.07`。论文实验章节报告 `sigma=0.04 m`，二者不一致。

### 6.3 独立 `VoxelGridRBF` basis

`rbf_cuda/include/cuda/rbf_cuda.cuh::calKernelMatrixKernel` 使用同形式的各向异性 Gaussian，并将小于 `1e-6` 的核值截断为零。`RBFCudaCore::visualizeKernel` 使用 `rbf_weight_` 计算高程和。

输入高程在 `VoxelGridRBF::setInputCloud()` 中先减去当前 ROI 点云平均高度 `mean_weight_`。可视化时再把 `mean_weight_` 加回。

## 7. RBF 中心、带宽与分辨率

### 7.1 中心生成

`VoxelGridRBF::filterCenter()` 在矩形 ROI 内按网格生成候选中心：

```text
x = center_x - x_left ... center_x + x_right, step rbf_leaf_size_x
y = center_y - y_backward ... center_y + y_forward, step rbf_leaf_size_y
```

网格坐标会按 leaf size 四舍五入到整数索引再恢复为实际中心坐标。

### 7.2 活跃中心选择

候选中心通过自定义二维 KD 树 `d2_tree::radiusSearch()` 判断附近是否存在点。代码只使用 bool 返回值，因此接受条件是“半径内至少存在一个点”，不是论文描述的“至少三个点”。

新中心搜索半径代码为：

\[
r_{code}=\frac{\sigma_x}{leaf_x}\max(leaf_x,leaf_y).
\]

旧中心使用 `2*r_code`。当 `leaf_x==leaf_y` 时，新中心半径退化为 `sigma_x`；一般各向异性配置下是否具有明确概率意义为 **UNKNOWN**。

### 7.3 参数来源

`elevation_rbf.cpp` 的节点默认值为：

| 参数 | 默认值 | 数学角色 |
| --- | ---: | --- |
| `rbf/sigma_x` | 0.14 | x 方向带宽 |
| `rbf/sigma_y` | 0.14 | y 方向带宽 |
| `rbf/leaf_size_x` | 0.2 | 中心网格 x 分辨率 |
| `rbf/leaf_size_y` | 0.2 | 中心网格 y 分辨率 |
| `rbf/x_left/right` | 3.0 / 3.0 | ROI x 范围 |
| `rbf/y_back/forward` | 3.0 / 3.0 | ROI y 范围 |
| `rbf/cloud_downsapmle_leaf` | 0.05 | PCL 三维体素下采样 leaf，传给 `setEps()` |
| `rbf/cloud_noise` | 0.1 | 代码中协方差初始化相关参数 |
| `rbf/weight_noise` | 0.5 | 类成员参数；当前活动初始拟合未使用 |
| `rbf/margin` | 0.001 | 可视化查询区域边缘留白 |

`run_rbf.launch` 没有加载 `rbf_cuda/config/params.yaml`。由 `run_tron1a_all.launch` 启动时，仓库配置中也没有设置 `rbf/*` 命名空间，因此上述节点默认值是当前可确认的独立拟合参数。

论文实验报告中心分辨率 0.07 m、带宽 0.04 m、测量噪声 0.1。当前独立节点默认值不完全一致。

## 8. RBF 权重估计

### 8.1 初始 ridge regression

论文式 (17)、(18) 为：

\[
\min_w \sum_i (p_{z,i}-m(x_i)^T w)^2+\lambda\|w\|^2,
\qquad
\hat w=(H+\lambda I)^{-1}b.
\]

代码 `RBFCudaCore::initialWeightKernel()` 实际构造 Gaussian kernel matrix `A`，求解：

\[
(A^TA+0.01I)w=A^Tb.
\]

映射：

| 项目 | 实现 |
| --- | --- |
| 参数 | `sigma_x/sigma_y`；代码内固定 `lambda=0.01` |
| 文件 | `rbf_cuda/src/cuda/rbf_cuda.cu` |
| 类 | `RBFCudaCore` |
| 函数 | `calKernelMatrix()`、`initialWeightKernel()` |
| 输入 | 活跃中心、去均值点云 `(x,y,z-mean_z)` |
| 输出 | `output_weight` |
| 调用者 | `VoxelGridRBF::initialWeightCuda()` |
| GPU | Thrust 核矩阵、cuBLAS `A^TA/A^Tb`、cuSOLVER LU |

论文式 (15) 的测量噪声修正核 `m(x)` 包含 `sigma^2/tilde_sigma^2` 缩放和增广带宽。当前 `calKernelMatrixKernel` 只计算原始 Gaussian，没有实现可识别的该缩放。因此代码只与普通 ridge regression 对应，不与论文式 (15)-(18) 完全等价。

### 8.2 递归权重更新

论文式 (19)-(24) 描述活跃权重的递归更新和 Woodbury/block 近似。代码提供：

```text
VoxelGridRBF::updateWeightCuda()
  -> RBFCudaCore::setCovariance()
  -> RBFCudaCore::updateWeightKernel()
```

`updateWeightKernel()` 计算的核心结构为：

\[
K = R W^T(WRW^T+Q)^{-1},
\]

\[
\beta_{new}=\beta+K(y-W\beta),
\]

\[
R_{new}=R-RW^T(WRW^T+Q)^{-1}WR.
\]

该结构与递归线性估计/Kalman 风格更新相似。它是否逐项等价于论文的 `H^{-1}` 活跃块递推为 **UNKNOWN**，因为源码没有公式编号、符号映射或测试。

当前 `elevation_rbf::odomCallback()` 每次调用 `initialWeightCuda()`，没有活动调用点调用 `updateWeightCuda()`。因此递归更新不是当前运行算法的一部分。

### 8.3 拟合误差

论文式 (43) 定义点的垂直残差 `e_i=p_{z,i}-f(x_i;w)`。`VoxelGridRBF::compute_error()` 对当前输入点调用 `visualizeCuda()`，然后使用 `meanAbsoluteError(fit_z,elevation_obs_)` 计算平均绝对误差。这是与式 (43) 最接近的代码映射。

## 9. 轮腿运动学与接触几何

### 9.1 轮心：论文式 (25)

论文：

\[
\xi_j({}^O R_B,{}^O t_B;q)={}^O R_B h_j(q)+{}^O t_B.
\]

代码映射：

| 项目 | 实现 |
| --- | --- |
| 参数 | `/joint_states`、`wheelRadius`、硬编码 Tron1A link offsets |
| 文件 | `LIO-SAM-MID360/src/rbf_wheel_kinematics.cpp` |
| 类 | `JointStateWheelKinematics` |
| 函数 | `computeWheelCenterBase()`、`computeWheelGeometry()` |
| 输入 | 八关节位置、六维 pose |
| 输出 | 左右轮 `center_world` |
| 调用者 | `CudaWheelRbfGradient::computeGradient()` 间接调用 |

代码只使用腿部 abad/hip/knee 角计算轮心，轮关节自转角不影响轮心。

### 9.2 轮轴：论文式 (26)

论文：

\[
u_j({}^O R_B;q)={}^O R_B g_j(q).
\]

代码 `computeWheelAxisBase()` 根据 abad/hip/knee 旋转 wheel joint 的 `[0,1,0]` 轴，再由 `computeWheelGeometry()` 乘以 world rotation。输出归一化 `axis_world`。

该运动学只构造左右两个轮子。论文中的四轮集合没有对应当前代码模型。

### 9.3 M20 四轮迁移边界

主残差接口按 `std::vector<WheelGeometry>` 遍历车轮，因此残差求值和向
`LMOptimization()` 追加行的代码没有固定写死“两轮行数”。但当前唯一活动的
`JointStateWheelKinematics` 只生成 Tron1A 左右两轮几何，M20 四轮正运动学尚未实现。

后续可由代码/模型确认的输入约束为：

| 数学量 | 迁移事实来源 | 当前状态 |
| --- | --- | --- |
| 16 个关节的轴、零位、限位与父子 link | `robot-description/M20/urdf/M20.urdf` | 模型已存在，主代码未调用 |
| 四个轮心和轮轴 | 由上述 URDF 链计算 | **TODO** |
| 轮半径 `rho` | M20 URDF collision geometry，`0.09 m` | 配置/代码尚未切换 |
| 关节观测 | `/JOINTS_DATA` | 测试包直接转发 APDU；符号、零偏和顺序映射 **UNKNOWN** |

测试 bag 中若干关节值超出 URDF 限位，且约 500 Hz topic 主要由约 10 Hz 源样本
重复组成。因此该数据只能用于验证解析器和时间队列，不能用于确认论文式 (25)-(28)
在 M20 上的几何正确性。最终输入应由 GOS 直接提供真实 500 Hz 样本，并在进入
运动学前转换成 URDF 关节约定。

前 Airy IMU 与 LiDAR 的单机 DIFOP 工厂外参仍未获得。该缺失不改变轮地残差的
数学形式，但会影响 `transformTobeMapped` 与 `base_link` 的一致性，因而阻止对
四轮接触 Jacobian 和最终估计结果作标定级验证。

## 10. 轮地 manifold residuals

### 10.1 论文式 (27)、(31)

论文每轮残差为：

\[
r_1=\chi_z-f(\chi_x,\chi_y),
\]

\[
r_2=u^Tn_\omega,
\qquad
r_3=\|\chi-\xi\|^2-r^2,
\qquad
r_4=n_\omega\times n_g.
\]

代码 `evaluateResidualVector()` 使用相同四类表达式，但带有残差截断和独立权重。

### 10.2 接触点局部求解

`solveContactOnWheel()` 先在车轮平面内取“向下”初始点，然后仅使用 `r1/r2/r3` 对接触点 `p` 做最多八次三维 Gauss-Newton：

\[
r(p)=
\begin{bmatrix}
p_z-h(p_x,p_y)\\
a^T(p-c)\\
(p-c)^T(p-c)-\rho^2
\end{bmatrix},
\]

\[
J_p=
\begin{bmatrix}
-h_x & -h_y & 1\\
a_x & a_y & a_z\\
2d_x & 2d_y & 2d_z
\end{bmatrix}.
\]

求解：

\[
\Delta p=-(J_p^TJ_p+10^{-6}I)^{-1}J_p^Tr.
\]

这里使用 Eigen LDLT。`r4` 不参与接触点局部求解，只在接触点求出后计算。

### 10.3 法向：论文式 (28)

代码定义：

\[
n_g=normalize([-h_x,-h_y,1]^T),
\qquad
n_\omega=normalize((p-c)/\rho).
\]

与论文式 (28) 在表达式上对应。若归一化失败，代码提供单位 z 或 `-nGround` 后备值。

### 10.4 残差截断和权重

`evaluateResidualVector()` 对 `r1/r2/r3` 分别按 `max_abs_r*` 截断，对 `r4` 按向量范数 `max_norm_r4` 缩放。随后 `computeGradient()` 分配 `weight_r1...weight_r4`，`updateRbfLmConstraints()` 再乘全局 `rbfConstraintWeight`。

主配置值：

| 参数 | 值 |
| --- | ---: |
| `rbfConstraintWeight` | 0.35 |
| `rbfWeightR1` | 1.0 |
| `rbfWeightR2` | 0.05 |
| `rbfWeightR3` | 0.02 |
| `rbfWeightR4` | 0.05 |
| `rbfMaxAbsR1/R2/R3` | 0.50 / 0.30 / 0.30 |
| `rbfMaxNormR4` | 2.00 |

在 `LMOptimization()` 中 residual 和 Jacobian 都乘以 `c.weight`。因此对应线性最小二乘项是 `(weight*r)^2`，也就是说配置中的 weight 是残差尺度，其对平方 cost 的系数为 `weight^2`。

## 11. Manifold Jacobian

### 11.1 论文式 (35)-(42)

论文将局部增量写为：

\[
\delta x_j=[\delta\theta,\delta t,\delta\chi_j],
\]

并显式推导轮心、轮轴、四类残差对姿态、平移和接触点的 Jacobian。

### 11.2 当前 CPU 实现

当前 `CudaWheelRbfGradient::computeGradient()` 不实现论文式 (35)-(42) 的解析块矩阵。它对六维 pose 做中心有限差分：

\[
\frac{\partial r_i}{\partial x_j}\approx
\frac{r_i(x+\epsilon_j)-r_i(x-\epsilon_j)}{2\epsilon_j}.
\]

代码步长：

- roll/pitch/yaw：`1e-4`。
- x/y/z：`1e-3`。

每次正负扰动都会重新计算轮几何并重新运行接触点局部 Gauss-Newton。因此 Jacobian 包含了接触点重新求解后的数值响应，但没有显式接触点增量列，最终每个 `RbfLmConstraint` 只有六列 pose Jacobian。

残差截断发生在数值差分之前。处于截断区间外时，截断可能使数值导数变为零或不连续；代码没有对此进行单独处理。

### 11.3 CUDA wheel Jacobian

`LIO-SAM-MID360/src/cuda/rbf_cuda_wheel_gradient.cu` 的 `WheelResidualJacobianKernel` 只计算：

\[
r=p_z-h(p_x,p_y),
\]

\[
\frac{\partial r}{\partial pose_j}=
\frac{\partial p_z}{\partial pose_j}
-h_x\frac{\partial p_x}{\partial pose_j}
-h_y\frac{\partial p_y}{\partial pose_j}.
\]

它要求调用者提供每轮 contact point 及其 `3x6` Jacobian。当前仓库没有调用 `ComputeWheelResidualJacobianCuda()`，也没有活动代码构造其 `contact_jacobian` 输入，因此它不参与当前估计。

## 12. Cost Function 与 Optimization

### 12.1 论文式 (7)、(32)-(34)

论文目标：

\[
J_{aug}(\eta_k,\chi_k)=J_{scan}(\eta_k)+J_M(\eta_k,\chi_k),
\]

其中 `J_scan` 是点线/点面平方和，`J_M` 是所有车轮 manifold residual 的平方和。

### 12.2 代码实际线性化目标

每次 `LMOptimization()` 构造：

\[
A=
\begin{bmatrix}
J_{edge/plane}\\
W_MJ_M
\end{bmatrix},
\qquad
b=-
\begin{bmatrix}
r_{edge/plane}\\
W_Mr_M
\end{bmatrix}.
\]

随后求解：

\[
(A^TA)\Delta x=A^Tb.
\]

对应当前一次线性化子问题：

\[
\min_{\Delta x}\|A\Delta x-b\|^2.
\]

函数名为 `LMOptimization()`，论文式 (34) 也说明使用 Levenberg-Marquardt，但当前代码没有构造 `A^TA+\lambda I`、信赖域或阻尼参数。因此实际实现更接近未阻尼 Gauss-Newton 法方程。是否有外部隐式 LM 机制为 **UNKNOWN**；在当前函数内没有。

### 12.3 迭代与退化处理

- `scan2MapOptimization()` 最多迭代 25 次。
- 每轮重新建立 LiDAR 对应关系、残差和 RBF 数值 Jacobian。
- LiDAR 有效残差少于 50 时，函数在构造 RBF 约束前返回；RBF 不能单独驱动该帧优化。
- 首轮对 `A^TA` 做特征分解，六个阈值均为 100。
- 小特征值方向通过 `matP=V^{-1}V_2` 投影。
- 收敛条件来自代码：`deltaR < 0.05` 且 `deltaT < 0.05`；其中 `deltaR` 使用度，`deltaT` 先将米增量乘 100。

### 12.4 关键帧图优化

`mapOptimization` 的第二个优化器 `ISAM2 *isam` 使用：

| Factor | 函数 | 输入 | 作用 |
| --- | --- | --- | --- |
| 首帧 Pose prior | `addOdomFactor()` | 当前扫描位姿 | 初始化关键帧图 |
| Pose BetweenFactor | `addOdomFactor()` | 相邻扫描匹配位姿差 | 关键帧里程计约束 |
| GPSFactor | `addGPSFactor()` | GPS 位置和协方差 | 绝对位置约束 |
| 回环 BetweenFactor | `addLoopFactor()` | ICP/外部回环相对位姿 | 全局一致性 |

RBF residual 没有独立 factor 类型。它先改变当前扫描匹配结果，再通过相邻位姿差进入关键帧图。

## 13. 论文公式到代码映射

| 论文公式 | 数学概念 | 代码映射 | 对应程度 |
| --- | --- | --- | --- |
| (1) | 基座位姿 `R,t` | `transformTobeMapped`、GTSAM `Pose3` | 概念对应；坐标帧需结合配置 |
| (5) | 单值地形 `p_z=f(x;w)` | `evaluateTerrain()`、`VoxelGridRBF` | 对应 |
| (6) | 多轮接触点集合 | 局部 `p`，每次遍历两个 `WheelGeometry` | 只实现两轮；接触点不持久化 |
| (7) | `Jscan + JM` 联合目标 | `LMOptimization()` 拼接 LiDAR/RBF 行 | 位姿 cost 对应；没有联合接触点变量 |
| (8) | RBF 加权和 | 两套 Gaussian 求值器 | 形式对应，权重来源不同 |
| (9) | Gaussian kernel | `evaluateTerrain()`、`calKernelMatrixKernel` | 代码扩展为各向异性带宽 |
| (14)-(16) | moment/least-squares 权重估计 | 无可确认的 `m(x)` 实现 | **UNKNOWN/未完整实现** |
| (17)-(18) | ridge regression | `initialWeightKernel()` | 普通 ridge 对应；缺少式 (15) 噪声修正核 |
| (19)-(24) | 活跃块递归权重更新 | `updateWeightCuda()`、`updateWeightKernel()` | 结构近似；活动运行链未调用，精确等价 **UNKNOWN** |
| (25) | 轮心正运动学 | `computeWheelCenterBase()`、`computeWheelGeometry()` | 两轮对应 |
| (26) | 轮轴正运动学 | `computeWheelAxisBase()` | 两轮对应 |
| (27a) | 接触点位于地形 | `rHeight` / `r1` | 对应，带截断 |
| (27b) | 接触点位于轮平面 | `axis_world.dot(p-center)` | 对应，带截断 |
| (27c) | 轮半径约束 | `squaredNorm-rho^2` | 对应，带截断 |
| (27d) | 法向共线 | `nWheel.cross(nGround)` | 对应，带范数截断 |
| (28a-b) | 车轮径向/地形法向 | `nWheel`、`nGround` | 对应 |
| (29) | IMU pose/velocity/bias 状态 | GTSAM `X,V,B` | 变量对应 |
| (30a-c) | IMU 离散传播 | GTSAM `integrateMeasurement()/predict()` | 概念对应；内部公式 **UNKNOWN** |
| (31) | 每轮六维 manifold residual | `[r1,r2,r3,r4x,r4y,r4z]` | 对应，带权重/截断 |
| (32) | 点线/点面 scan cost | `cornerOptimization()`、`surfOptimization()` | 对应，代码有启发式权重 |
| (33) | manifold 平方 cost | RBF 行进入 `matA/matB` | 对应加权线性化 cost |
| (34) | manifold augmented LM | `LMOptimization()` | 拼接目标对应；无显式 LM 阻尼 |
| (35)-(42) | 解析联合 Jacobian | CPU 中心有限差分 | **未按论文解析形式实现** |
| (43) | RBF 垂直拟合误差 | `compute_error()` / `meanAbsoluteError()` | 残差概念对应，汇总为 MAE |

## 14. 核心算法索引

| Algorithm concept | Source file | Function | Parameters | Role |
| --- | --- | --- | --- | --- |
| LiDAR rotation deskew | `LIO-SAM-MID360/src/imageProjection.cpp` | `imuDeskewInfo()`、`findRotation()`、`projectPointCloud()` | IMU topic、scan time | 将点旋转到扫描起始姿态 |
| LiDAR translation deskew | 同上 | `findPosition()` | odometry | 当前输出零位移，未接通 |
| Feature curvature | `LIO-SAM-MID360/src/featureExtraction.cpp` | `calculateSmoothness()` | `edgeThreshold`、`surfThreshold` | 特征评分 |
| Edge point-to-line residual | `LIO-SAM-MID360/src/mapOptmization.cpp` | `cornerOptimization()` | 5-NN、距离/特征值阈值 | `Jscan` 边缘项 |
| Plane point-to-plane residual | 同上 | `surfOptimization()` | 5-NN、平面阈值 | `Jscan` 平面项 |
| LiDAR analytic pose Jacobian | 同上 | `LMOptimization()` | 当前 RPY/XYZ | 线性化 scan residual |
| Scan pose solve | 同上 | `LMOptimization()` | 特征行、RBF 行、退化阈值 | 六维未阻尼法方程 |
| IMU preintegration | `LIO-SAM-MID360/src/imuPreintegration.cpp` | `odometryHandler()`、`imuHandler()` | IMU noise/bias/gravity | `X,V,B` 状态约束和传播 |
| Keyframe odometry factor | `LIO-SAM-MID360/src/mapOptmization.cpp` | `addOdomFactor()` | 固定 noise | 关键帧图相对位姿 |
| GPS factor | 同上 | `addGPSFactor()` | GPS/pose covariance thresholds | 绝对位置约束 |
| Loop factor | 同上 | `performLoopClosure()`、`addLoopFactor()` | ICP fitness | 回环约束 |
| Main RBF basis | `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp` | `evaluateTerrain()` | `rbfSigmaX/Y` | 主轮地高度和梯度 |
| Wheel kinematics | `LIO-SAM-MID360/src/rbf_wheel_kinematics.cpp` | `computeWheelGeometry()` | joint states、wheel radius | 轮心/轮轴 |
| M20 model kinematics | `robot-description/M20/urdf/M20.urdf` | URDF joint chain（当前无主代码函数） | 16 joints、wheel radius 0.09 | 四轮迁移事实来源，**TODO** 接入 |
| Contact point solve | `LIO-SAM-MID360/src/rbf_cuda_wheel_gradient.cpp` | `solveContactOnWheel()` | r1/r2/r3、8 iterations | 每轮局部接触点 |
| Manifold residual | 同上 | `evaluateResidualVector()` | weights、clamps | 每轮六行轮地残差 |
| Manifold pose Jacobian | 同上 | `computeGradient()` | eps rotation/translation | 中心有限差分六列 Jacobian |
| RBF constraint injection | `LIO-SAM-MID360/src/mapOptmization.cpp` | `updateRbfLmConstraints()`、`LMOptimization()` | RBF 全局/分项权重 | 影响最终扫描位姿 |
| Center grid | `rbf_cuda/include/rbf_fitting/rbf_fit.h` | `filterCenter()`、`generateRBFGrid()` | leaf size、ROI、sigma | 独立 RBF 中心和活跃集 |
| RBF kernel matrix | `rbf_cuda/src/cuda/rbf_cuda.cu` | `calKernelMatrix()` | sigma x/y | GPU Gaussian matrix |
| Ridge weights | 同上 | `initialWeightKernel()` | fixed lambda 0.01 | 独立 RBF 初始权重 |
| Recursive weights | 同上 | `updateWeightKernel()` | covariance R/Q | 已实现但当前未调用 |
| RBF visualization | 同上 | `visualize()` | query grid、weights | `/rbf_elevation_map` |
| RBF fitting MAE | `rbf_cuda/include/rbf_fitting/rbf_fit.h` | `compute_error()` | current observations | 独立拟合诊断 |

## 15. UNKNOWN / TODO

1. **UNKNOWN**：主 RBF 约束把局部地图点 z 当作核权重的理论依据。
2. **UNKNOWN**：`RbfCudaRequest::points_lidar` 的预期数学用途；当前梯度实现不读取它。
3. **UNKNOWN**：论文式 (15) 的噪声修正核是否曾在其他未提交分支实现。
4. **UNKNOWN**：`updateWeightKernel()` 与论文式 (19)-(24) 的逐符号严格对应关系。
5. **TODO**：当前活动独立 RBF 链未调用递归权重更新。
6. **TODO**：主轮地约束未调用已编译的 CUDA wheel kernel。
7. **TODO**：论文式 (35)-(42) 的解析联合 Jacobian 未在主路径实现。
8. **TODO**：论文式 (34) 的位姿/接触点联合优化被“局部接触点求解 + 位姿有限差分”替代。
9. **UNKNOWN**：函数名 `LMOptimization()` 与无显式阻尼法方程之间是否存在未提交的 LM 设计。
10. **UNKNOWN**：非默认 `lidarFrame != baselinkFrame` 时轮地位姿变量的严格 frame 语义。
11. **UNKNOWN**：GTSAM 版本及其 `ImuFactor` 内部使用的精确误差坐标和 Jacobian。
12. **UNKNOWN**：LiDAR 点线/点面启发式权重 `s` 的概率意义。
13. **UNKNOWN**：残差截断阈值和 `rbfWeightR*` 的标定依据。
14. **UNKNOWN**：论文实验参数与当前 YAML/节点默认参数不一致的原因。
15. **UNKNOWN**：仓库没有数值梯度检查、解析 Jacobian 对比或 rosbag 回归测试，无法仅从静态代码确认估计器的数值正确性。
16. **UNKNOWN**：M20 APDU 关节角到 URDF joint variable 的符号、零偏和顺序转换；测试 bag 中存在超限值，不能仅凭变量名推断。
17. **UNKNOWN**：前 Airy DIFOP 中 LiDAR-IMU 工厂旋转/平移标定；通用驱动 transform 参数不能替代该单机标定。
18. **TODO**：M20 四轮运动学尚未生成四个 `WheelGeometry` 并接入现有残差接口。
