#ifndef RBF_CUDA_CUH
#define RBF_CUDA_CUH

#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform_reduce.h>
#include <thrust/sequence.h>
#include <thrust/fill.h>
#include <thrust/execution_policy.h>

#include <cublas_v2.h>
#include <cusolverDn.h>

#include <memory>
#include <map>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <scancontext/tic_toc.h>

// 错误检查宏
#define CHECK_CUDA(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA Error (" << __FILE__ << ":" << __LINE__ << "): " \
                  << cudaGetErrorString(err) << std::endl; \
        exit(EXIT_FAILURE); \
    } \
}

#define CHECK_CUBLAS(call) { \
    cublasStatus_t status = call; \
    if (status != CUBLAS_STATUS_SUCCESS) { \
        std::cerr << "cuBLAS Error (" << __FILE__ << ":" << __LINE__ << ")" \
        << cublasGetStatusString(status) << std::endl; \
        exit(EXIT_FAILURE); \
    } \
}

#define CHECK_CUSOLVER(call) { \
    cusolverStatus_t status = call; \
    if (status != CUSOLVER_STATUS_SUCCESS) { \
        std::cerr << "cuSOLVER Error (" << __FILE__ << ":" << __LINE__ << ")" \
        << std::endl; \
        exit(EXIT_FAILURE); \
    } \
}



namespace rbf_fit{
    class RBFCudaCore {
        public:
        using DevicePoints = thrust::device_vector<Eigen::Vector2d, thrust::device_allocator<Eigen::Vector2d>>;
        using DevicePointElevation = thrust::device_vector<double, thrust::device_allocator<double>>;
        // using DeviceIndices = thrust::device_vector<int, thrust::device_allocator<int>>;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        RBFCudaCore();
        ~RBFCudaCore();

        // store sample points in GPU
        void setSampleCloud(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud,
                            const std::vector<double>& elevation);

        // store rbf points in GPU
        void setRBFCloud(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud, 
                        const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& roi_cloud, 
                        const std::vector<double>& weight,
                        const std::vector<double>& roi_weight,
                        const std::vector<int>& rbf_indices);

        void setCovariance(const Eigen::MatrixXd cov_R, const Eigen::MatrixXd cov_Q);
        
        void calKernelMatrix(std::vector<double>& kernel_matrix, double sigma_x, double sigma_y);

        void initialWeightKernel(std::vector<double>& output_weight);

        void updateWeightKernel(std::vector<double>& output_weight, std::vector<double>& output_cov_R);

        void visualize(std::vector<double>& output_elevation, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud, 
                    double sigma_x, double sigma_y);

        struct calKernelMatrixKernel
        {
            calKernelMatrixKernel(
                const thrust::device_vector<Eigen::Vector2d>& sample_cloud,
                const thrust::device_vector<Eigen::Vector2d>& rbf_cloud,
                const int sample_size, const int rbf_size,
                const double sigma_x, const double sigma_y)
                : sample_cloud_(sample_cloud.data()),
                rbf_cloud_(rbf_cloud.data()),
                sample_size_(sample_size), rbf_size_(rbf_size),
                sigma_x_(sigma_x), sigma_y_(sigma_y)
                {}

            __host__ __device__ double 
            operator()(const int idx) const {
                // 计算行和列索引，列优先
                // vector[idx] <==> kernel_matrix[idx%sample_size_, idx/sample_size_]
                int i = idx % sample_size_;
                int j = idx / sample_size_;

                // 从设备内存中读取第 i 个样本点和第 j 个 RBF 点
                Eigen::Vector2d sample = thrust::raw_pointer_cast(sample_cloud_)[i];
                Eigen::Vector2d rbf = thrust::raw_pointer_cast(rbf_cloud_)[j];

                // 计算 x 和 y 方向的差值
                double diff_x = sample[0] - rbf[0];
                double diff_y = sample[1] - rbf[1];

                // 计算高斯核值
                double res = exp( -1.0 / 2.0 *(diff_x * diff_x / (sigma_x_ * sigma_x_) + diff_y * diff_y / (sigma_y_ * sigma_y_)));
                return res > 1e-6 ? res : 0.0;
            }

            thrust::device_ptr<const Eigen::Vector2d> sample_cloud_;
            thrust::device_ptr<const Eigen::Vector2d> rbf_cloud_;
            const int sample_size_, rbf_size_;
            const double sigma_x_, sigma_y_;
        };

        struct visualizeKernel
        {
            visualizeKernel(const thrust::device_vector<Eigen::Vector2d>& sample_cloud,
                const thrust::device_vector<Eigen::Vector2d>& rbf_cloud,
                const thrust::device_vector<double>& rbf_weight,
                const int sample_size, const int rbf_size,
                const double sigma_x, const double sigma_y)
                :sample_cloud_(sample_cloud.data()),
                rbf_cloud_(rbf_cloud.data()),
                rbf_weight_(rbf_weight.data()),
                sample_size_(sample_size), rbf_size_(rbf_size),
                sigma_x_(sigma_x), sigma_y_(sigma_y)
                {}

            __host__ __device__ double 
            operator()(const int idx) const {
                double res = 0.0;
                Eigen::Vector2d sample = thrust::raw_pointer_cast(sample_cloud_)[idx];
                for (int i = 0; i < rbf_size_; i++){
                    Eigen::Vector2d rbf = thrust::raw_pointer_cast(rbf_cloud_)[i];
                    // 计算 x 和 y 方向的差值
                    double diff_x = sample[0] - rbf[0];
                    double diff_y = sample[1] - rbf[1];
                    res += thrust::raw_pointer_cast(rbf_weight_)[i] * exp( -1.0 / 2.0 *(diff_x * diff_x / (sigma_x_ * sigma_x_) + diff_y * diff_y / (sigma_y_ * sigma_y_)));
                }
                return res;
            }

            thrust::device_ptr<const Eigen::Vector2d> sample_cloud_;
            thrust::device_ptr<const Eigen::Vector2d> rbf_cloud_;
            thrust::device_ptr<const double> rbf_weight_;
            const int sample_size_, rbf_size_;
            const double sigma_x_, sigma_y_;
        };


        private:
        std::unique_ptr<DevicePoints> rbf_points_, roi_rbf_points_;
        std::unique_ptr<DevicePointElevation> rbf_points_weight_, roi_rbf_points_weight_;
        std::unique_ptr<DevicePoints> sample_points_;
        std::unique_ptr<DevicePointElevation> sample_points_elevation_;
        
        // 列优先
        thrust::device_vector<double> d_kernel_, d_roi_kernel;
        thrust::device_vector<double> d_cov_R_, d_cov_Q_, d_roi_R_;

        // activated rbf indices
        thrust::device_vector<int> rbf_indices_;
    };
}

#endif