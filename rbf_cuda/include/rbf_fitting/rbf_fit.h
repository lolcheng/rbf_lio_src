#ifndef RBF_FIT_H
#define RBF_FIT_H

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>
#include <tuple>
#include <vector>
#include <iostream>
#include <stdexcept>

#include "d2_tree/d2_tree.h"
#include "cuda/rbf_cuda.cuh"

namespace rbf_fit
{
    struct gridHash
    {
        std::size_t operator()(const std::pair<long, long>& key) const {
            return std::hash<long>()(key.first) ^ (std::hash<long>()(key.second) << 1);
        }
    };

    struct pointHash
    {
        std::size_t operator()(const std::tuple<long, long, long>& t) const
        {
            auto h1 = std::hash<long>()(std::get<0>(t));
            auto h2 = std::hash<long>()(std::get<1>(t));
            auto h3 = std::hash<long>()(std::get<2>(t));
            return h1 ^ (h2 << 1) ^ (h3 << 2);
        }
    };

    double meanAbsoluteError(const std::vector<double>& a, const std::vector<double>& b) {
        // 检查向量尺寸是否相同
        if (a.size() != b.size()) {
            throw std::invalid_argument("Vectors must be of the same size.");
        }
        // 处理空向量
        if (a.empty()) {
            return 0.0;
        }
        // 使用std::inner_product计算绝对差之和
        double sum = std::inner_product(
            a.begin(), a.end(), b.begin(), 0.0,
            std::plus<>(),
            [](double x, double y) { return std::abs(x - y); }
        );
        return sum / a.size();
    }

    template <typename PointT>
    class VoxelGridRBF
    {
    public:
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = boost::shared_ptr<pcl::PointCloud<PointT>>;
        using PointCloudConstPtr = boost::shared_ptr<const pcl::PointCloud<PointT>>;
        
    public:
        VoxelGridRBF();

        ~VoxelGridRBF() {};

        void setInputCloud (const PointCloudConstPtr &cloud);

        void generateRBFGrid();

        void initialWeightCuda();

        void updateWeightCuda();

        void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr& visual_cloud, double visual_leaf_x, double visual_leaf_y);

        void compute_error();

        // 获取统计信息
        double getKernelMatrixTime() const { return kernel_matrix_time_; }
        double getWeightTime() const { return weight_time_; }
        double getMAE() const { return mae_; }

        pcl::PointCloud<pcl::PointXYZ>::Ptr visualizeCuda(std::vector<double>& output_elevation, 
            const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud);

        inline void
        setSigma (double sigma_x, double sigma_y)
        {
            sigma_x_ = sigma_x;
            sigma_y_ = sigma_y;
        }
        inline void
        setRbfGridLeafSize(double leaf_size_x, double leaf_size_y)
        {
            rbf_leaf_size_x_ = leaf_size_x;
            rbf_leaf_size_y_ = leaf_size_y;
        }
        inline void 
        setCenter(double center_x, double center_y, double center_z)
        {
            center_x_ = center_x;
            center_y_ = center_y;
            center_z_ = center_z;
        }
        inline void
        setExtent(double x_left, double x_right, double y_back, double y_forward)
        {
            x_left_ = x_left; x_right_ = x_right;
            y_backward_ = y_back; y_forward_ = y_forward;
        }
        inline void
        setEps(double eps)
        {
            eps_ = eps;
        }
        inline void 
        setCloudNoise(double cloud_noise)
        {
            cloud_noise_ = cloud_noise;
        }
        inline void
        setWeightNoise(double weight_noise)
        {
            weight_noise_ = weight_noise;
        }
        inline void
        setMargin(double margin)
        {
            margin_ = margin;
        }
        

    public:
        void filterCloud(PointCloudPtr cloud_filtered, PointCloudConstPtr cloud);
        void filterCenter();

        double sigma_x_, sigma_y_;
        double rbf_leaf_size_x_, rbf_leaf_size_y_;
        double center_x_, center_y_, center_z_;
        double x_left_, x_right_, y_backward_, y_forward_;
        double pre_left_, pre_right_, pre_backward_, pre_forward_;
        double margin_;

        double mean_weight_;

        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> pre_rbf_grid_, cur_rbf_grid_, act_rbf_grid_;
        std::vector<double> elevation_obs_;
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> pre_point_, cur_point_;
        std::vector<double> rbf_weight_,roi_rbf_weight_;
        std::vector<int> rbf_indices_, roi_rbf_indices_;

        // 仅对新增点云点构建kd树
        std::unique_ptr<d2_tree::KDNode> point_kd_tree_ptr_;

        std::unordered_map<std::pair<long, long>, double, gridHash> history_weight_, pre_weight_;
        std::unordered_map<std::pair<long, long>, int, gridHash> pre_weight_indices_;
        std::unordered_set<std::tuple<long,long,long>, pointHash> exist_point_;
        double eps_;

        Eigen::MatrixXd weight_cov_R_, cloud_cov_Q_;
        double weight_noise_, cloud_noise_;

        std::unique_ptr<rbf_fit::RBFCudaCore> rbf_cuda_ptr_;

        // 统计信息
        double kernel_matrix_time_;  // kernel matrix计算时间（毫秒）
        double weight_time_;          // weight计算时间（毫秒）
        double mae_;                  // 平均绝对误差
    };
    
    template <typename PointT>
    VoxelGridRBF<PointT>::VoxelGridRBF():
    sigma_x_(0.1), sigma_y_(0.1),
    rbf_leaf_size_x_(0.1), rbf_leaf_size_y_(0.1),
    center_x_(0.0), center_y_(0.0),
    x_left_(0.5), x_right_(0.5), y_backward_(0.5), y_forward_(0.5),
    pre_left_(0.), pre_right_(0.), pre_backward_(0.), pre_forward_(0.),
    point_kd_tree_ptr_(nullptr),
    eps_(1e-3),
    weight_noise_(1), cloud_noise_(1e-2),
    kernel_matrix_time_(0.0), weight_time_(0.0), mae_(0.0)
    {
        weight_cov_R_.resize(0,0);
        cloud_cov_Q_.resize(0,0);

        rbf_cuda_ptr_.reset(new RBFCudaCore());
    }

    template <typename PointT>
    void VoxelGridRBF<PointT>::filterCloud(PointCloudPtr cloud_filtered, PointCloudConstPtr cloud)
    {
        for (const auto& point : cloud->points) {
            if (point.x >= center_x_-x_left_ && point.x <= center_x_+x_right_ && point.y >= center_y_-y_backward_ && point.y <= center_y_+y_forward_
                && point.z < center_z_ + 0.3 && point.z > center_z_ - 1.5) {
                cloud_filtered->points.push_back(point);
            }
        }
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = true;
        std::cout << "In the ROI: " << cloud->points.size() << " -> " << cloud_filtered->points.size() << std::endl;
    }

    template <typename PointT>
    void VoxelGridRBF<PointT>::filterCenter()
    {
        pre_rbf_grid_ = cur_rbf_grid_;
        std::unordered_map<std::pair<long, long>, int, gridHash> temp_weight_indices;
        std::unordered_map<std::pair<long, long>, double, gridHash> temp_weight;
        cur_rbf_grid_.clear();
        act_rbf_grid_.clear();
        rbf_weight_.clear();
        roi_rbf_weight_.clear();

        double rad = (sigma_x_ / rbf_leaf_size_x_) * std::max(rbf_leaf_size_x_, rbf_leaf_size_y_);
        int num_roi_rbf = 0;
        int num_old_rbf = 0;
        int num_old_rbf_activated = 0;
        int num_old_rbf_stay = 0;
        int num_new_rbf = 0;
        int num_new_rbf_activated = 0;
        int num_new_rbf_stay = 0;

        double left = center_x_-x_left_, right = center_x_+x_right_, backward = center_y_-y_backward_, forward = center_y_+y_forward_;

        for (double x = left; x <= right; x += rbf_leaf_size_x_) {
            for (double y = backward; y <= forward; y += rbf_leaf_size_y_) {
                long i = static_cast<long>(std::round(x / rbf_leaf_size_x_));
                long j = static_cast<long>(std::round(y / rbf_leaf_size_y_));
                Eigen::Vector2d rbf_pt;
                rbf_pt.x() = i * rbf_leaf_size_x_;
                rbf_pt.y() = j * rbf_leaf_size_y_;
                d2_tree::Point2D search_pt(x, y);
                double weight = 0.0;
                // new rbf center point
                if (x <= pre_left_ || x > pre_right_  || y <= pre_backward_ || y > pre_forward_)
                {
                    cur_rbf_grid_.push_back(rbf_pt);
                    roi_rbf_weight_.push_back(weight);
                    // activated
                    if (d2_tree::radiusSearch(point_kd_tree_ptr_, search_pt, rad)){
                        act_rbf_grid_.push_back(rbf_pt);
                        rbf_weight_.push_back(weight);
                        // rbf_indices_ for update of roi_rbf_weight_
                        rbf_indices_.push_back(num_roi_rbf);

                        num_new_rbf_activated += 1;
                    }
                    else{
                        num_new_rbf_stay += 1;
                    }
                    num_new_rbf += 1;
                }
                // old rbf center point
                else{
                    // assert(pre_weight_.find({i,j}) != pre_weight_.end());
                    // assert(pre_weight_indices_.find({i,j}) != pre_weight_indices_.end());
                    // weight = pre_weight_[{i,j}];
                    cur_rbf_grid_.push_back(rbf_pt);
                    roi_rbf_weight_.push_back(weight);
                    // roi_rbf_indices_ for reconstruction of cov R
                    // roi_rbf_indices_.push_back(pre_weight_indices_[{i,j}]);
                    // activated
                    if (d2_tree::radiusSearch(point_kd_tree_ptr_, search_pt, rad * 2)){
                        act_rbf_grid_.push_back(rbf_pt);
                        rbf_weight_.push_back(weight);
                        // rbf_indices_ for update of roi_rbf_weight_
                        rbf_indices_.push_back(num_roi_rbf);

                        num_old_rbf_activated += 1;
                    }
                    else{
                        num_old_rbf_stay += 1;
                    }
                    num_old_rbf += 1;
                }
                temp_weight_indices[{i,j}] = num_roi_rbf;
                temp_weight[{i,j}] = weight;
                num_roi_rbf += 1;
            }
        }
        std::cout << "new rbf: " << num_new_rbf << ", new activated: " << num_new_rbf_activated 
        << "; old rbf: " << num_old_rbf << ", old activated: " << num_old_rbf_activated <<std::endl;
        
        pre_left_ = left;
        pre_right_ = right;
        pre_backward_  = backward;
        pre_forward_ = forward;
        pre_weight_indices_ = temp_weight_indices;
        pre_weight_ = temp_weight;

        // 修改协方差
        // if (num_old_rbf_activated + num_old_rbf_stay == 0){
        //     std::cout << "first rbf grid" << std::endl;
        //     weight_cov_R_.resize(num_roi_rbf, num_roi_rbf);
        //     weight_cov_R_.setIdentity();
        //     weight_cov_R_ *= cloud_noise_ * cloud_noise_;
        // }
        // else{
        //     std::cout << "update rbf grid" << std::endl;
        //     Eigen::MatrixXd temp_cov_R(num_roi_rbf, num_roi_rbf);
        //     temp_cov_R.setIdentity();
        //     temp_cov_R *= weight_noise_ * weight_noise_;
        //     for (const auto i : roi_rbf_indices_){
        //         for(const auto j : roi_rbf_indices_){
        //             temp_cov_R(i,j) = weight_cov_R_(i,j);
        //         }
        //     }
        //     weight_cov_R_ = temp_cov_R;
        // }
        weight_cov_R_.resize(num_roi_rbf, num_roi_rbf);
        weight_cov_R_.setIdentity();
        weight_cov_R_ *= cloud_noise_ * cloud_noise_;
    }


    template <typename PointT>
    void VoxelGridRBF<PointT>::setInputCloud(const PointCloudConstPtr &cloud)
    {
        point_kd_tree_ptr_.reset();
        std::vector<d2_tree::Point2D> kd_point_;
        assert(point_kd_tree_ptr_ == nullptr);
        std::cout << "====================set input cloud=======================" << std::endl;
        TicTocV2 set_input;

        TicTocV2 voxel;
        PointCloudPtr cloud_roi(new PointCloud);
        filterCloud(cloud_roi, cloud);

        PointCloudPtr cloud_filtered(new PointCloud);

        pcl::VoxelGrid<PointT> voxel_filter;
        voxel_filter.setInputCloud(cloud_roi);
        voxel_filter.setLeafSize(eps_, eps_, eps_);
        voxel_filter.filter(*cloud_filtered);
        std::cout << "Downsampled: " << cloud_filtered->points.size() << std::endl;
        voxel.toc("voxel filter");
        mean_weight_ = 0.0;
        if (!cloud_filtered->empty()) {
            double sum_z = 0.0;
            for (const auto& point : *cloud_filtered) {
            sum_z += point.z;
            }
            mean_weight_ = sum_z / cloud_filtered->size();
        }

        if(pre_point_.empty()){
            std::cout << "first input cloud" << std::endl;
            
            pre_point_.clear();
            cur_point_.clear();
            elevation_obs_.clear();
            
            pre_point_.reserve(cloud_filtered->size());
            cur_point_.reserve(cloud_filtered->size());
            elevation_obs_.reserve(cloud_filtered->size());
            kd_point_.reserve(cloud_filtered->size());

            TicTocV2 emplace;
            for (const auto& point : *cloud_filtered)
            {
                pre_point_.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
                cur_point_.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
                elevation_obs_.emplace_back(static_cast<double>(point.z - mean_weight_));
                kd_point_.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
            }
            emplace.toc("emplace back point");

            // const size_t N = cur_point_.size();
            // cloud_cov_Q_.resize(N, N);
            // cloud_cov_Q_.setIdentity();
            // cloud_cov_Q_ *= (cloud_noise_ * cloud_noise_);
        }
        else
        {
            std::cout << "update input cloud" << std::endl;
        
            pre_point_ = cur_point_;
            cur_point_.clear();
            elevation_obs_.clear();

            cur_point_.reserve(cloud_filtered->size());
            elevation_obs_.reserve(cloud_filtered->size());
            kd_point_.reserve(cloud_filtered->size());

            TicTocV2 emplace;
            for (const auto& point : *cloud_filtered)
            {
                cur_point_.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
                elevation_obs_.emplace_back(static_cast<double>(point.z - mean_weight_));
                kd_point_.emplace_back(static_cast<double>(point.x), static_cast<double>(point.y));
            }
            emplace.toc("emplace back point");

            // const size_t N = cur_point_.size();
            // cloud_cov_Q_.resize(N, N);
            // cloud_cov_Q_.setIdentity();
            // cloud_cov_Q_ *= (cloud_noise_ * cloud_noise_);
        }
        TicTocV2 kd_tree;
        point_kd_tree_ptr_ = d2_tree::buildKDTree(kd_point_);
        kd_tree.toc("build kd tree");
        assert(point_kd_tree_ptr_ != nullptr);
        set_input.toc("set input cloud");
        std::cout << "==========================================================" << std::endl;
    }

    template <typename PointT>
    void VoxelGridRBF<PointT>::generateRBFGrid()
    {
        std::cout << "==================generate rbf grid=======================" << std::endl;
        assert(rbf_weight_.size() == act_rbf_grid_.size());
        std::cout<< "before: \nrbf_weights size: " << rbf_weight_.size() << std::endl;
        TicTocV2 gen_rbf;
        filterCenter();
        gen_rbf.toc("generate rbf points");
        assert(rbf_weight_.size() == act_rbf_grid_.size());
        assert(roi_rbf_weight_.size() == cur_rbf_grid_.size());
        assert(weight_cov_R_.cols() == cur_rbf_grid_.size());
        std::cout << "after: \n rbf_weights size: " << rbf_weight_.size() << std::endl;
        // std::cout << "loaded weight: " << std::endl;
        // std::copy(rbf_weight_.begin(), rbf_weight_.end(), std::ostream_iterator<double>(std::cout, " "));
        // std::cout << std::endl;
        std::cout << "==========================================================" << std::endl;
    }

    template <typename PointT>
    void VoxelGridRBF<PointT>::initialWeightCuda()
    {
        std::cout << "==================calculate rbf weight====================" << std::endl;
        rbf_cuda_ptr_->setSampleCloud(cur_point_, elevation_obs_);
        // act_rbf_grid: kd树搜寻到的rbf点
        // cur_rbf_grid_: ROI内的所有rbf点
        // rbf_weight_: act_rbf_grid对应weight
        // roi_rbf_weight_: cur_rbf_grid_对应weight
        // rbf_indices_: act_rbf_grid在cur_rbf_grid_中对应的索引
        rbf_cuda_ptr_->setRBFCloud(act_rbf_grid_, cur_rbf_grid_, rbf_weight_, roi_rbf_weight_, rbf_indices_);
        // rbf_cuda_ptr_->setCovariance(weight_cov_R_, cloud_cov_Q_);
        std::vector<double> kernel_matrix;
        
        // 测量kernel matrix计算时间
        TicTocV2 tictoc_kernel(false);
        rbf_cuda_ptr_->calKernelMatrix(kernel_matrix, sigma_x_, sigma_y_);
        kernel_matrix_time_ = tictoc_kernel.elapsed_ms();
        
        // 测量weight计算时间
        std::vector<double> weight_vector;
        TicTocV2 tictoc_weight(false);
        rbf_cuda_ptr_->initialWeightKernel(weight_vector);
        weight_time_ = tictoc_weight.elapsed_ms();

        assert(rbf_weight_.size() == weight_vector.size());
        assert(pre_weight_.size() == roi_rbf_weight_.size());
        for (int i = 0; i < weight_vector.size(); i++){
            rbf_weight_[i] = weight_vector[i];

            double x = cur_rbf_grid_[i].x();
            double y = cur_rbf_grid_[i].y();

            long i_ = static_cast<long>(std::round(x / rbf_leaf_size_x_));
            long j_ = static_cast<long>(std::round(y / rbf_leaf_size_y_));

            assert(pre_weight_.find({i_,j_}) != pre_weight_.end());
            pre_weight_[{i_, j_}] = weight_vector[i];
            roi_rbf_weight_[rbf_indices_[i]] = weight_vector[i];
        }
        // std::cout << "activated weight: "; 
        // std::copy(rbf_weight_.begin(), rbf_weight_.end(), std::ostream_iterator<double>(std::cout, " "));
        // std::cout << std::endl;
        // std::cout << "roi weight: ";
        // std::copy(roi_rbf_weight_.begin(), roi_rbf_weight_.end(), std::ostream_iterator<double>(std::cout, " "));
        // std::cout << std::endl;
        std::cout << "==========================================================" << std::endl;
    }

    template <typename PointT>
    void VoxelGridRBF<PointT>::updateWeightCuda()
    {
        rbf_cuda_ptr_->setSampleCloud(cur_point_, elevation_obs_);
        rbf_cuda_ptr_->setRBFCloud(act_rbf_grid_, cur_rbf_grid_, rbf_weight_, roi_rbf_weight_, rbf_indices_);
        assert(weight_cov_R_.cols() == cur_rbf_grid_.size());
        rbf_cuda_ptr_->setCovariance(weight_cov_R_, cloud_cov_Q_);
        std::vector<double> kernel_matrix;
        rbf_cuda_ptr_->calKernelMatrix(kernel_matrix, sigma_x_, sigma_y_);
        std::vector<double> weight_vector;
        std::vector<double> vector_cov_R;
        rbf_cuda_ptr_->updateWeightKernel(weight_vector, vector_cov_R);

        assert(rbf_weight_.size() == weight_vector.size());
        for (int i = 0; i < weight_vector.size(); i++){
            rbf_weight_[i] = weight_vector[i];
            double x = cur_rbf_grid_[i].x();
            double y = cur_rbf_grid_[i].y();
            long i_ = static_cast<long>(std::round(x / rbf_leaf_size_x_));
            long j_ = static_cast<long>(std::round(y / rbf_leaf_size_y_));
            pre_weight_[{i_, j_}] = weight_vector[i];
        }
        assert(vector_cov_R.size() == rbf_weight_.size() * rbf_weight_.size());
        MatrixXd temp_cov_R;
        temp_cov_R.resize(rbf_weight_.size(), rbf_weight_.size());
        std::copy(
            vector_cov_R.begin(), 
            vector_cov_R.end(), 
            temp_cov_R.data()
        );
        weight_cov_R_ = temp_cov_R;

        // std::copy(rbf_weight_.begin(), rbf_weight_.end(), std::ostream_iterator<double>(std::cout, " "));
        // std::cout << std::endl;
    }

    template <typename PointT>
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    VoxelGridRBF<PointT>::visualizeCuda(std::vector<double>& output_elevation, const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& cloud)
    {
        size_t size = output_elevation.size();
        rbf_cuda_ptr_->visualize(output_elevation, cloud, sigma_x_, sigma_y_);
        pcl::PointCloud<pcl::PointXYZ>::Ptr fit_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        fit_cloud->reserve(size);
        for (size_t i = 0; i < size; i++){
            pcl::PointXYZ pt;
            pt.x = cloud[i].x();
            pt.y = cloud[i].y();
            pt.z = output_elevation[i] + mean_weight_;
            fit_cloud->push_back(pt);
        }
        return fit_cloud;
    }

    template <typename PointT>
    void VoxelGridRBF<PointT>::visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr& visual_cloud, 
        double visual_leaf_x, double visual_leaf_y)
    {
        TicTocV2 tictoc_visualize;
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> fit_xy;
        for (double x = center_x_-x_left_+margin_; x <= center_x_+x_right_-margin_; x += visual_leaf_x) {
            for (double y = center_y_-y_backward_+margin_; y <= center_y_+y_forward_-margin_; y += visual_leaf_y) {
                fit_xy.emplace_back(x, y);
            }
        }
        std::vector<double> fit_z;
        fit_z.resize(fit_xy.size());

        std::cout << "visualization size: " << fit_xy.size() << std::endl;
        visual_cloud = visualizeCuda(fit_z, fit_xy);
        tictoc_visualize.toc("visualize fitted ground");
    }

    template <typename PointT>
    void VoxelGridRBF<PointT>::compute_error()
    {
        TicTocV2 tictoc_compute_error;
        std::vector<double> fit_z;
        fit_z.resize(cur_point_.size());

        auto fit_cloud = visualizeCuda(fit_z, cur_point_);

        mae_ = meanAbsoluteError(fit_z, elevation_obs_);
        std::cout << "MAE = " << mae_ << std::endl;
        tictoc_compute_error.toc("compute error");
    }
}

#endif