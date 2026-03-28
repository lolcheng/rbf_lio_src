#include "cuda/rbf_cuda.cuh"

// 计算矩阵的特征值和条件数
void compute_condition_number_and_eigenvalues(cusolverDnHandle_t& cusolver_handle_,
                                              thrust::device_vector<double> d_A, int m) {
    // 设置参数
    cusolverEigMode_t jobz = CUSOLVER_EIG_MODE_VECTOR; // 计算特征值和特征向量
    cublasFillMode_t uplo = CUBLAS_FILL_MODE_LOWER;   // 矩阵是下三角矩阵

    // 计算特征值分解的工作空间大小
    int lwork = 0;
    thrust::device_vector<double> d_work;  // 工作空间
    thrust::device_vector<double> d_eigenvalues(m);  // 存储特征值
    thrust::device_vector<double> d_eigenvectors(m * m); // 存储特征向量

    cusolverStatus_t status = cusolverDnDsyevd_bufferSize(
        cusolver_handle_, jobz, uplo, m,
        thrust::raw_pointer_cast(d_A.data()), m,
        thrust::raw_pointer_cast(d_eigenvalues.data()), &lwork
    );

    if (status != CUSOLVER_STATUS_SUCCESS) {
        std::cerr << "cusolverDnDsyevd_bufferSize failed!" << std::endl;
        return;
    }

    d_work.resize(lwork);  // 根据返回的 lwork 动态调整工作空间大小

    int* d_info;
    CHECK_CUDA(cudaMalloc(&d_info, sizeof(int)));

    // 执行特征值分解
    status = cusolverDnDsyevd(
        cusolver_handle_, jobz, uplo, m,
        thrust::raw_pointer_cast(d_A.data()), m,
        thrust::raw_pointer_cast(d_eigenvalues.data()),
        thrust::raw_pointer_cast(d_work.data()), lwork,
        d_info
    );

    if (status != CUSOLVER_STATUS_SUCCESS) {
        std::cerr << "cusolverDnDsyevd failed!" << std::endl;
        CHECK_CUDA(cudaFree(d_info));
        return;
    }

    // 获取矩阵的最大和最小特征值
    double max_eigenvalue = *thrust::max_element(d_eigenvalues.begin(), d_eigenvalues.end());
    double min_eigenvalue = *thrust::min_element(d_eigenvalues.begin(), d_eigenvalues.end());
    thrust::host_vector<double> h_eigens = d_eigenvalues;
    std::vector<double> eigens;
    eigens.resize(h_eigens.size());
    thrust::copy(h_eigens.begin(), h_eigens.end(), eigens.begin());
    std::cout << "特征值：" ;
    std::copy(eigens.begin(), eigens.end(), std::ostream_iterator<double>(std::cout, " "));
    std::cout << std::endl;

    std::cout << "最大特征值: " << max_eigenvalue << std::endl;
    std::cout << "最小特征值: " << min_eigenvalue << std::endl;

    // 计算条件数
    double condition_number = max_eigenvalue / min_eigenvalue;
    std::cout << "矩阵条件数: " << condition_number << std::endl;

    // 清理内存
    CHECK_CUDA(cudaFree(d_info));
}


namespace rbf_fit{
    __global__ void add_lambda_kernel(double* matrix, int n, double lambda) {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if (idx < n) {
            matrix[idx * n + idx] += lambda; // 列优先存储的对角线位置
        }
    }

    __global__ void MakeSymmetricKernel(int m, double* A) {
        int row = blockIdx.y * blockDim.y + threadIdx.y;
        int col = blockIdx.x * blockDim.x + threadIdx.x;
        if (row < m && col < m && row < col) {
            A[row * m + col] = A[col * m + row];
        }
    }

    RBFCudaCore::RBFCudaCore(){
        TicTocV2 cudacore_initial;
        cudaDeviceSynchronize();
        cudacore_initial.toc("RBFCudaCore initialization");
    }
    RBFCudaCore::~RBFCudaCore() {}

    void RBFCudaCore::setSampleCloud(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud,
                                    const std::vector<double>& elevation)
    {
        TicTocV2 tictoc_set_sample_cloud;
        // store the points in a thrust::host_vector and then copy it to a device vector
        thrust::host_vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points(cloud.begin(), cloud.end());
        thrust::host_vector<double, thrust::device_allocator<double>> ele(elevation.begin(), elevation.end());
        if (!sample_points_) {
            sample_points_.reset(new DevicePoints());
        }
        *sample_points_ = points;
        if (!sample_points_elevation_){
            sample_points_elevation_.reset(new DevicePointElevation);
        }
        *sample_points_elevation_ = ele;
        std::cout << "sample size: " << cloud.size() << std::endl;
        tictoc_set_sample_cloud.toc("set sample time");
    }

    void RBFCudaCore::setRBFCloud(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud, 
                        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& roi_cloud, 
                        const std::vector<double>& weight,
                        const std::vector<double>& roi_weight,
                        const std::vector<int>& rbf_indices)
    {
        TicTocV2 tictoc_set_rbf_cloud;
        thrust::host_vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points(cloud.begin(), cloud.end());
        thrust::host_vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> roi_points(roi_cloud.begin(), roi_cloud.end());

        thrust::host_vector<double, thrust::device_allocator<double>> temp_weight(weight.begin(), weight.end());
        thrust::host_vector<double, thrust::device_allocator<double>> temp_roi_weight(roi_weight.begin(), roi_weight.end());

        thrust::host_vector<int, thrust::device_allocator<int>> h_rbf_indices(rbf_indices.begin(), rbf_indices.end());

        if (!rbf_points_) {
            rbf_points_.reset(new DevicePoints());
        }
        *rbf_points_ = points;

        if(!roi_rbf_points_){
            roi_rbf_points_.reset(new DevicePoints());
        }
        *roi_rbf_points_ = roi_points;

        if (!rbf_points_weight_){
            rbf_points_weight_.reset(new DevicePointElevation());
        }
        *rbf_points_weight_ = temp_weight;

        if (!roi_rbf_points_weight_){
            roi_rbf_points_weight_.reset(new DevicePointElevation());
        }
        *roi_rbf_points_weight_ = temp_roi_weight;

        rbf_indices_.clear();
        rbf_indices_ = h_rbf_indices;

        std::cout << "activated rbf size: " << cloud.size() << ", roi rbf size: "<< roi_cloud.size() << std::endl;
        tictoc_set_rbf_cloud.toc("set rbf time");
    }

    void RBFCudaCore::calKernelMatrix(std::vector<double>& kernel_matrix, double sigma_x, double sigma_y)
    {
        TicTocV2 cal_kernel_cuda;
        // 获取样本数量和 RBF 数量
        int sample_size = sample_points_->size();
        int rbf_size = rbf_points_->size();
        int roi_rbf_size = roi_rbf_points_->size();
        int total_elements = sample_size * rbf_size;
        int total_roi_elements = sample_size * roi_rbf_size;

        d_kernel_.resize(total_elements);
        d_roi_kernel.resize(total_roi_elements);

        // 构造计数迭代器，其范围为 [0, total_elements)
        thrust::counting_iterator<int> idx_first(0);
        thrust::counting_iterator<int> idx_last = idx_first + total_elements;

        // simulate cpus
        // thrust::transform(thrust::host, idx_first, idx_last, d_kernel_.begin(), 
        //                 calKernelMatrixKernel(*sample_points_, *rbf_points_, sample_size, rbf_size, sigma_x, sigma_y));

        // 利用 Thrust 的 transform 将每个索引映射到对应的核值
        thrust::transform(thrust::device, idx_first, idx_last, d_kernel_.begin(), 
                        calKernelMatrixKernel(*sample_points_, *rbf_points_, sample_size, rbf_size, sigma_x, sigma_y));

        // for roi kernel
        // idx_last = idx_first + total_roi_elements;
        // thrust::transform(thrust::device, idx_first, idx_last, d_roi_kernel.begin(), 
        //                 calKernelMatrixKernel(*sample_points_, *roi_rbf_points_, sample_size, roi_rbf_size, sigma_x, sigma_y));
        
        // 将设备端结果复制到 host 端的 std::vector<float> 中
        kernel_matrix.resize(total_elements);
        thrust::copy(d_kernel_.begin(), d_kernel_.end(), kernel_matrix.begin());
        cal_kernel_cuda.toc("calculate kernel matrix on gpu");
    }

    void RBFCudaCore::initialWeightKernel(std::vector<double>& output_weight)
    {
        int m = sample_points_->size();
        int n = rbf_points_->size();
        assert(d_kernel_.size() == m * n);
        TicTocV2 initial_weight_cuda;

        // 创建 cuBLAS 句柄并检查错误
        cublasHandle_t cublas_handle;
        cublasStatus_t cublasStat = cublasCreate(&cublas_handle);
        if (cublasStat != CUBLAS_STATUS_SUCCESS) {
            throw std::runtime_error("cuBLAS handle creation failed");
        }
        // 创建 cuSOLVER 句柄并检查错误
        cusolverDnHandle_t solver_handle;
        cusolverStatus_t cusolverStat = cusolverDnCreate(&solver_handle);
        if (cusolverStat != CUSOLVER_STATUS_SUCCESS) {
            cublasDestroy(cublas_handle);
            throw std::runtime_error("cuSOLVER handle creation failed");
        }

        // compute_condition_number_and_eigenvalues(solver_handle, d_cov_R_, n);

        // 计算 A^T * A
        thrust::device_vector<double> d_ATA(n * n);
        double alpha = 1.0, beta = 0.0;
        cublasDgemm(cublas_handle, CUBLAS_OP_T, CUBLAS_OP_N,
                    n, n, m,
                    &alpha,
                    thrust::raw_pointer_cast(d_kernel_.data()), m,
                    thrust::raw_pointer_cast(d_kernel_.data()), m,
                    &beta,
                    thrust::raw_pointer_cast(d_ATA.data()), n);

        // Ridge penalty (paper default is 0.01).
        double lambda = 0.01;
        double* d_ATA_ptr = thrust::raw_pointer_cast(d_ATA.data());
        int threads_per_block = 256;
        int blocks = (n + threads_per_block - 1) / threads_per_block;
        add_lambda_kernel<<<blocks, threads_per_block>>>(d_ATA_ptr, n, lambda);
        cudaDeviceSynchronize();

        // compute_condition_number_and_eigenvalues(solver_handle, d_ATA, n);

        // 计算 A^T * b
        thrust::device_vector<double> d_ATb(n);
        cublasDgemv(cublas_handle, CUBLAS_OP_T,
                    m, n,
                    &alpha,
                    thrust::raw_pointer_cast(d_kernel_.data()), m,
                    thrust::raw_pointer_cast(sample_points_elevation_->data()), 1,
                    &beta,
                    thrust::raw_pointer_cast(d_ATb.data()), 1);


        // ------------ Cholesky 分解 ------------
        // int workspace_size = 0;
        // cusolverDnSpotrf_bufferSize(solver_handle, CUBLAS_FILL_MODE_LOWER,
        //                             n, thrust::raw_pointer_cast(d_ATA.data()), n,
        //                             &workspace_size);
        // thrust::device_vector<float> d_workspace(workspace_size);

        // int dev_info = 0;
        // cusolverStatus_t potrfStatus = cusolverDnSpotrf(solver_handle, CUBLAS_FILL_MODE_LOWER,
        //                                                n, thrust::raw_pointer_cast(d_ATA.data()), n,
        //                                                thrust::raw_pointer_cast(d_workspace.data()),
        //                                                workspace_size, &dev_info);
        // if (potrfStatus != CUSOLVER_STATUS_SUCCESS || dev_info != 0) {
        //     cublasDestroy(cublas_handle);
        //     cusolverDnDestroy(solver_handle);
        //     throw std::runtime_error("Cholesky decomposition failed");
        // }

        // // 求解线性系统
        // cusolverStatus_t potrsStatus = cusolverDnSpotrs(solver_handle, CUBLAS_FILL_MODE_LOWER,
        //                                                n, 1,
        //                                                thrust::raw_pointer_cast(d_ATA.data()), n,
        //                                                thrust::raw_pointer_cast(d_ATb.data()), n,
        //                                                &dev_info);
        // if (potrsStatus != CUSOLVER_STATUS_SUCCESS || dev_info != 0) {
        //     cublasDestroy(cublas_handle);
        //     cusolverDnDestroy(solver_handle);
        //     throw std::runtime_error("Cholesky solve failed");
        // }
        // ------------  Cholesky 代码结束 ------------

        // ------------ LU 分解 ------------
        // 备份原始矩阵 A^T A + λI 到 d_A_lu
        thrust::device_vector<double> d_A_lu = d_ATA;

        // 定义主元交换数组和状态变量
        thrust::device_vector<int> d_pivot(n);  // 主元索引（从1开始）
        thrust::device_vector<int> d_info(1);   // 分解状态

        // 查询 LU 分解所需工作空间大小
        int lwork = 0;
        cusolverDnDgetrf_bufferSize(
            solver_handle,
            n,  // 矩阵行数
            n,  // 矩阵列数
            thrust::raw_pointer_cast(d_A_lu.data()),
            n,  // leading dimension
            &lwork
        );

        // 分配工作空间
        thrust::device_vector<double> d_workspace(lwork);

        // 执行 LU 分解
        cusolverDnDgetrf(
            solver_handle,
            n,  // 行数
            n,  // 列数
            thrust::raw_pointer_cast(d_A_lu.data()),
            n,  // leading dimension
            thrust::raw_pointer_cast(d_workspace.data()),
            thrust::raw_pointer_cast(d_pivot.data()),
            thrust::raw_pointer_cast(d_info.data())
        );
        cudaDeviceSynchronize();

        // 检查分解是否成功
        int info;
        thrust::copy(d_info.begin(), d_info.end(), &info);
        if (info != 0) {
            cusolverDnDestroy(solver_handle);
            cublasDestroy(cublas_handle);
            throw std::runtime_error("LU decomposition failed");
        }

        // 利用 LU 分解结果求解线性系统
        cusolverDnDgetrs(
            solver_handle,
            CUBLAS_OP_N,  // 矩阵未转置
            n,             // 矩阵行数
            1,             // 右侧向量数量（nrhs=1）
            thrust::raw_pointer_cast(d_A_lu.data()),
            n,  // leading dimension
            thrust::raw_pointer_cast(d_pivot.data()),
            thrust::raw_pointer_cast(d_ATb.data()),  // 输入为 A^T b，输出为解 x
            n,  // leading dimension of B
            thrust::raw_pointer_cast(d_info.data())
        );
        cudaDeviceSynchronize();

        // 检查求解是否成功
        thrust::copy(d_info.begin(), d_info.end(), &info);
        if (info != 0) {
            cusolverDnDestroy(solver_handle);
            cublasDestroy(cublas_handle);
            throw std::runtime_error("LU solve failed");
        }
        // ------------ LU 分解代码结束 ------------

        // 同步设备并拷贝结果
        cudaDeviceSynchronize();
        *rbf_points_weight_ = d_ATb;
        output_weight.resize(n);
        thrust::copy(d_ATb.begin(), d_ATb.end(), output_weight.begin());

        // 同步后销毁句柄
        cudaDeviceSynchronize(); // 再次确保同步
        cusolverDnDestroy(solver_handle);
        cublasDestroy(cublas_handle);

        initial_weight_cuda.toc("initialize weight using cuda");
    }

    void RBFCudaCore::setCovariance(const Eigen::MatrixXd cov_roi_R, const Eigen::MatrixXd cov_Q)
    {
        // 获取矩阵维度
        const size_t rows_R = cov_roi_R.rows();
        const size_t cols_R = cov_roi_R.cols();
        const size_t rows_Q = cov_Q.rows();
        const size_t cols_Q = cov_Q.cols();


        // 调整设备向量大小（按列优先顺序存储）
        d_roi_R_.resize(rows_R * cols_R);
        d_cov_Q_.resize(rows_Q * cols_Q);

        // 将 Eigen 数据拷贝到设备向量
        // Eigen 默认列优先存储，直接拷贝内存布局即可
        thrust::copy(
            cov_roi_R.data(), 
            cov_roi_R.data() + rows_R * cols_R, 
            d_roi_R_.begin()
        );
        cusolverDnHandle_t cusolver_handle_;
        CHECK_CUSOLVER(cusolverDnCreate(&cusolver_handle_));
        // compute_condition_number_and_eigenvalues(cusolver_handle_, d_roi_R_, rows_R);
        
        std::vector<double> temp_indices;
        temp_indices.resize(rbf_indices_.size());
        thrust::copy(rbf_indices_.begin(), rbf_indices_.end(), temp_indices.begin());
        // std::copy(rbf_indices_.begin(), rbf_indices_.end(), std::ostream_iterator<double>(std::cout, " "));
        // std::cout << std::endl;
        Eigen::MatrixXd cov_R(rbf_indices_.size(), rbf_indices_.size());
        for (int i=0; i<rbf_indices_.size(); i++){
            for (int j=0; j < rbf_indices_.size(); j++){
                cov_R(i,j) = cov_roi_R(temp_indices[i], temp_indices[j]);
                // if (i != j) assert(cov_R(i,j) < 1e-6);
            }
        }
        // for (int i=300; i < 400; i++){
        //     for(int j=300; j< 400; j++){
        //         std::cout << cov_R(i,j) << " ";
        //     }
        //     std::cout << std::endl;
        // }
        d_cov_R_.resize(rbf_indices_.size() * rbf_indices_.size());
        thrust::copy(
            cov_R.data(), 
            cov_R.data() + rbf_indices_.size() * rbf_indices_.size(), 
            d_cov_R_.begin()
        );
        // compute_condition_number_and_eigenvalues(cusolver_handle_, d_cov_R_, rbf_indices_.size());

        thrust::copy(
            cov_Q.data(), 
            cov_Q.data() + rows_Q * cols_Q, 
            d_cov_Q_.begin()
        );
        cusolverDnDestroy(cusolver_handle_);
    }

    void RBFCudaCore::updateWeightKernel(std::vector<double>& output_weight, std::vector<double>& output_cov_R)
    {
        TicTocV2 tictoc_update_weight;
        cublasHandle_t cublas_handle_;
        cusolverDnHandle_t cusolver_handle_;
        CHECK_CUBLAS(cublasCreate(&cublas_handle_));
        CHECK_CUSOLVER(cusolverDnCreate(&cusolver_handle_));

        int m = sample_points_->size();
        int k = rbf_points_->size();
        int roi_k = roi_rbf_points_->size();

        const double alpha = 1.0;
        const double beta = 0.0;
        const double neg_alpha = -1.0;

        // compute_condition_number_and_eigenvalues(cusolver_handle_, d_cov_R_, k);

        auto max_iter = thrust::max_element(d_kernel_.begin(), d_kernel_.end());
        auto min_iter = thrust::min_element(d_kernel_.begin(), d_kernel_.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;
        std::cout << "0" << std::endl;

        // 分配中间矩阵内存
        thrust::device_vector<double> d_WR(m * k);    // W * R (m×k)
        thrust::device_vector<double> d_WRWT(m * m);  // W * R * Wᵀ (m×m)
        thrust::device_vector<double> d_A(m * m);     // A = W*R*Wᵀ + Q
        thrust::device_vector<double> d_RWT(k * m);   // R * W^T (k×m)

        // W * R (m x k) = (m x k) * (k x k)
        CHECK_CUBLAS(cublasDgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N,
                    m, k, k,
                    &alpha,
                    thrust::raw_pointer_cast(d_kernel_.data()), m,
                    thrust::raw_pointer_cast(d_cov_R_.data()), k,
                    &beta,
                    thrust::raw_pointer_cast(d_WR.data()), m));

        max_iter = thrust::max_element(d_WR.begin(), d_WR.end());
        min_iter = thrust::min_element(d_WR.begin(), d_WR.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;

        // d_RWT (k×m)
        CHECK_CUBLAS(cublasDgemm(
            cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_T,
            k, m, k,
            &alpha,
            thrust::raw_pointer_cast(d_cov_R_.data()), k,
            thrust::raw_pointer_cast(d_kernel_.data()), m,
            &beta,
            thrust::raw_pointer_cast(d_RWT.data()), k));
        max_iter = thrust::max_element(d_RWT.begin(), d_RWT.end());
        min_iter = thrust::min_element(d_RWT.begin(), d_RWT.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;
        std::cout << "1" << std::endl;
        
        // W * R * W^T (m x m) = (m x k) * (k x m)
        CHECK_CUBLAS(cublasDgemm(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_T,
                    m, m, k,
                    &alpha,
                    thrust::raw_pointer_cast(d_WR.data()), m,
                    thrust::raw_pointer_cast(d_kernel_.data()), m,
                    &beta,
                    thrust::raw_pointer_cast(d_WRWT.data()), m));

        max_iter = thrust::max_element(d_WRWT.begin(), d_WRWT.end());
        min_iter = thrust::min_element(d_WRWT.begin(), d_WRWT.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;
        std::cout << "2" << std::endl;

        // double lambda = 0.5;
        // double* d_WRWT_ptr = thrust::raw_pointer_cast(d_WRWT.data());
        // int threads_per_block = 256;
        // int blocks = (m + threads_per_block - 1) / threads_per_block;
        // add_lambda_kernel<<<blocks, threads_per_block>>>(d_WRWT_ptr, m, lambda);
        // cudaDeviceSynchronize();
        // std::cout << "2.1" << std::endl;

        // A = WRW^T + Q
        CHECK_CUBLAS(cublasDgeam(cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N,
                    m, m,
                    &alpha, thrust::raw_pointer_cast(d_WRWT.data()), m,
                    &alpha, thrust::raw_pointer_cast(d_cov_Q_.data()), m,
                    thrust::raw_pointer_cast(d_A.data()), m));

        max_iter = thrust::max_element(d_A.begin(), d_A.end());
        min_iter = thrust::min_element(d_A.begin(), d_A.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;
        std::cout << "3" << std::endl;
        
        std::cout << "4" << std::endl;
        // --- 计算 A 的逆 ---
        // compute_condition_number_and_eigenvalues(cusolver_handle_, d_A, m);
        // Cholesky 分解
        int lwork_potrf = 0;
        cusolverDnDpotrf_bufferSize(cusolver_handle_, CUBLAS_FILL_MODE_LOWER, m,
                                    thrust::raw_pointer_cast(d_A.data()), m,
                                    &lwork_potrf);
        thrust::device_vector<double> d_work_potrf(lwork_potrf);
        int* d_info_potrf;
        CHECK_CUDA(cudaMalloc(&d_info_potrf, sizeof(int)));

        // 执行分解
        CHECK_CUSOLVER(cusolverDnDpotrf(cusolver_handle_, CUBLAS_FILL_MODE_LOWER, m,
                                        thrust::raw_pointer_cast(d_A.data()), m,
                                        thrust::raw_pointer_cast(d_work_potrf.data()),
                                        lwork_potrf, d_info_potrf));

        // 检查分解是否成功
        int info_potrf;
        CHECK_CUDA(cudaMemcpy(&info_potrf, d_info_potrf, sizeof(int), cudaMemcpyDeviceToHost));
        if (info_potrf != 0) {
            throw std::runtime_error("Cholesky分解失败: A 非正定");
        }
        std::cout << "4.1" << std::endl;

        // 计算逆矩阵
        CHECK_CUSOLVER(cusolverDnDpotri(cusolver_handle_, CUBLAS_FILL_MODE_LOWER, m,
                                        thrust::raw_pointer_cast(d_A.data()), m,
                                        thrust::raw_pointer_cast(d_work_potrf.data()),
                                        lwork_potrf, d_info_potrf));

        // 检查求逆是否成功
        CHECK_CUDA(cudaMemcpy(&info_potrf, d_info_potrf, sizeof(int), cudaMemcpyDeviceToHost));
        if (info_potrf != 0) {
            throw std::runtime_error("矩阵求逆失败");
            
        }
        std::cout << "4.2" << std::endl;
        
        dim3 block(16, 16);
        dim3 grid((m + block.x - 1) / block.x, (m + block.y - 1) / block.y);
        MakeSymmetricKernel<<<grid, block>>>(m, thrust::raw_pointer_cast(d_A.data()));
        CHECK_CUDA(cudaDeviceSynchronize());

        max_iter = thrust::max_element(d_A.begin(), d_A.end());
        min_iter = thrust::min_element(d_A.begin(), d_A.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;
        std::cout << "5" << std::endl;

        // --- 步骤 3: 计算 b = y - W * β ---
        thrust::device_vector<double> d_Wbeta(m); // W * β (m x 1)
        thrust::device_vector<double> d_b(m);     // b = y - Wβ

        // W * β
        CHECK_CUBLAS(cublasDgemv(cublas_handle_, CUBLAS_OP_N,
                    m, k,
                    &alpha,
                    thrust::raw_pointer_cast(d_kernel_.data()), m,
                    thrust::raw_pointer_cast(rbf_points_weight_->data()), 1,
                    &beta,
                    thrust::raw_pointer_cast(d_Wbeta.data()), 1));
        max_iter = thrust::max_element(d_Wbeta.begin(), d_Wbeta.end());
        min_iter = thrust::min_element(d_Wbeta.begin(), d_Wbeta.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;                    
        std::cout << "6" << std::endl;

        // b = y - Wβ
        CHECK_CUBLAS(cublasDcopy(cublas_handle_, m,
                    thrust::raw_pointer_cast(sample_points_elevation_->data()), 1,
                    thrust::raw_pointer_cast(d_b.data()), 1));
        CHECK_CUBLAS(cublasDaxpy(cublas_handle_, m,
                    &neg_alpha,
                    thrust::raw_pointer_cast(d_Wbeta.data()), 1,
                    thrust::raw_pointer_cast(d_b.data()), 1));

        max_iter = thrust::max_element(d_b.begin(), d_b.end());
        min_iter = thrust::min_element(d_b.begin(), d_b.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;      
        std::cout << "7" << std::endl;

        // --- 步骤 4: 计算最终结果 R * W^T * A^{-1} * b ---
        thrust::device_vector<double> d_Ainvb(m);    // A^{-1} * b (m x 1)
        thrust::device_vector<double> d_RWT_Ainvb(k); // 结果 (k x 1)

        // A^{-1} * b
        CHECK_CUBLAS(cublasDgemv(cublas_handle_, CUBLAS_OP_N,
                    m, m,
                    &alpha,
                    thrust::raw_pointer_cast(d_A.data()), m,
                    thrust::raw_pointer_cast(d_b.data()), 1,
                    &beta,
                    thrust::raw_pointer_cast(d_Ainvb.data()), 1));

        max_iter = thrust::max_element(d_Ainvb.begin(), d_Ainvb.end());
        min_iter = thrust::min_element(d_Ainvb.begin(), d_Ainvb.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;    
        std::cout << "8" << std::endl;

        // R * W^T * A^{-1}b
        CHECK_CUBLAS(cublasDgemv(cublas_handle_, CUBLAS_OP_T,
                    m, k,
                    &alpha,
                    thrust::raw_pointer_cast(d_RWT.data()), m,
                    thrust::raw_pointer_cast(d_Ainvb.data()), 1,
                    &beta,
                    thrust::raw_pointer_cast(d_RWT_Ainvb.data()), 1));       
                    
        max_iter = thrust::max_element(d_RWT_Ainvb.begin(), d_RWT_Ainvb.end());
        min_iter = thrust::min_element(d_RWT_Ainvb.begin(), d_RWT_Ainvb.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;    
        std::cout << "9" << std::endl;

        // --- 更新协方差 ---
        // 步骤 2: 计算 R * W^T * A^{-1}
        thrust::device_vector<double> d_RWT_Ainv(k * m);
        CHECK_CUBLAS(cublasDgemm(
            cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N,
            k, m, m,
            &alpha,
            thrust::raw_pointer_cast(d_RWT.data()), k,
            thrust::raw_pointer_cast(d_A.data()), m,
            &beta,
            thrust::raw_pointer_cast(d_RWT_Ainv.data()), k
        ));
        max_iter = thrust::max_element(d_RWT_Ainv.begin(), d_RWT_Ainv.end());
        min_iter = thrust::min_element(d_RWT_Ainv.begin(), d_RWT_Ainv.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;  
        std::cout << "10" << std::endl;

        // 步骤 3: 计算 R * W^T * A^{-1} * W * R
        thrust::device_vector<double> d_RWT_Ainv_WR(k * k);
        CHECK_CUBLAS(cublasDgemm(
            cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N,
            k, k, m,
            &alpha,
            thrust::raw_pointer_cast(d_RWT_Ainv.data()), k,
            thrust::raw_pointer_cast(d_WR.data()), m,
            &beta,
            thrust::raw_pointer_cast(d_RWT_Ainv_WR.data()), k
        ));
        max_iter = thrust::max_element(d_RWT_Ainv_WR.begin(), d_RWT_Ainv_WR.end());
        min_iter = thrust::min_element(d_RWT_Ainv_WR.begin(), d_RWT_Ainv_WR.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl;  
        std::cout << "11" << std::endl;

        // 步骤 4: 计算 R_new = R - RWT_Ainv_WR
        CHECK_CUBLAS(cublasDgeam(
            cublas_handle_, CUBLAS_OP_N, CUBLAS_OP_N,
            k, k,
            &alpha,
            thrust::raw_pointer_cast(d_cov_R_.data()), k,
            &neg_alpha,
            thrust::raw_pointer_cast(d_RWT_Ainv_WR.data()), k,
            thrust::raw_pointer_cast(d_cov_R_.data()), k
        ));
        max_iter = thrust::max_element(d_cov_R_.begin(), d_cov_R_.end());
        min_iter = thrust::min_element(d_cov_R_.begin(), d_cov_R_.end());
        std::cout << "通过 extrema 计算:\n";
        std::cout << "Max: " << *max_iter << ", Min: " << *min_iter << std::endl; 
        std::cout << "12" << std::endl;

        // --- 结果处理 ---
        // 使用 Thrust 直接在设备端累加
        assert(rbf_points_weight_->size() == d_RWT_Ainvb.size());
        thrust::transform(
            thrust::device_pointer_cast(rbf_points_weight_->data()),
            thrust::device_pointer_cast(rbf_points_weight_->data() + d_RWT_Ainvb.size()),
            d_RWT_Ainvb.begin(),
            thrust::device_pointer_cast(rbf_points_weight_->data()),
            thrust::plus<double>()
        );
        std::cout << "13" << std::endl;

        // 将结果拷贝回主机
        thrust::host_vector<double> h_rbf_weight = *rbf_points_weight_;
        output_weight.resize(h_rbf_weight.size());
        output_weight.assign(h_rbf_weight.begin(), h_rbf_weight.end());

        thrust::host_vector<double> h_cov_R = d_cov_R_;
        output_cov_R.resize(h_cov_R.size());
        output_cov_R.assign(h_cov_R.begin(), h_cov_R.end());

        tictoc_update_weight.toc("update weight on gpu");
        cublasDestroy(cublas_handle_);
        cusolverDnDestroy(cusolver_handle_);
        CHECK_CUDA(cudaFree(d_info_potrf));
    }

    void RBFCudaCore::visualize(std::vector<double>& output_elevation, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud, 
                    double sigma_x, double sigma_y)
    {
        int sample_size = cloud.size();
        int rbf_size = rbf_points_weight_->size();
        
        thrust::device_vector<double> d_output_elevation;
        d_output_elevation.resize(sample_size);

        thrust::device_vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> d_cloud(cloud.begin(), cloud.end());

        thrust::counting_iterator<int> id_first(0);
        thrust::counting_iterator<int> id_last = id_first + sample_size;

        thrust::transform(thrust::device, id_first, id_last, d_output_elevation.begin(),
            visualizeKernel(d_cloud, *rbf_points_, *rbf_points_weight_, sample_size, rbf_size, sigma_x, sigma_y));
        
        output_elevation.resize(sample_size);
        thrust::copy(d_output_elevation.begin(), d_output_elevation.end(), output_elevation.begin());
    }

}