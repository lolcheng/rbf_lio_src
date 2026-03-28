#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/random.h>
#include <pcl/filters/passthrough.h>
#include <boost/thread/thread.hpp>
#include <thread>
#include <math.h>
#include <random>

#include "scancontext/tic_toc.h"
#include "rbf_fitting/rbf_fit.h"
using namespace std;
using namespace rbf_fit;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

template <typename PointT>
void adjustZHeight(typename pcl::PointCloud<PointT>::Ptr& cloud) {
    // 查找最小z值
    float min_z = std::numeric_limits<float>::max();
    for (const auto& point : *cloud) {
        if (point.z < min_z) min_z = point.z;
    }

    // 当最小z值低于0时进行平移
    if (min_z < 0.0f) {
        const float delta_z = -min_z;
        std::cout << "\n检测到点云最低点低于地面（z_min = " << min_z << "），执行z轴平移：" 
                  << delta_z << "米" << std::endl;

        // 执行平移操作
        for (auto& point : *cloud) {
            point.z += delta_z;
        }

        // 验证平移结果
        float new_min_z = std::numeric_limits<float>::max();
        for (const auto& point : *cloud) {
            if (point.z < new_min_z) new_min_z = point.z;
        }
        std::cout << "平移完成，新z范围最小值: " << new_min_z << std::endl;
    }
}

// 通用点云信息输出函数
template <typename PointT>
void printCloudInfo(const typename pcl::PointCloud<PointT>::ConstPtr& cloud, Vector3f& min_p, Vector3f& max_p) {
    if (cloud->empty()) {
        std::cerr << "错误：点云为空！" << std::endl;
        return;
    }

    // 初始化极值
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    // 遍历所有点计算范围
    for (const auto& point : *cloud) {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    min_p[0] = min_x; min_p[1] = min_y; min_p[2] = min_z;
    max_p[0] = max_x; max_p[1] = max_y; max_p[2] = max_z;

    // 打印信息
    std::cout << "======== 点云基本信息 ========\n"
                << "点云类型: " << pcl::getFieldsList(*cloud) << "\n"
                << "总点数: " << cloud->size() << "\n"
                << "组织方式: " << (cloud->isOrganized() ? "有组织" : "无组织") << "\n"
                << "坐标范围:\n"
                << "  X: [" << min_x << " ~ " << max_x << "]\n"
                << "  Y: [" << min_y << " ~ " << max_y << "]\n"
                << "  Z: [" << min_z << " ~ " << max_z << "]\n"
                << "物理尺寸:\n"
                << "  X方向: " << (max_x - min_x) << " m\n"
                << "  Y方向: " << (max_y - min_y) << " m\n"
                << "  Z方向: " << (max_z - min_z) << " m\n"
                << "==============================" << std::endl;
}

template <typename CloudT>
void randomDownsample(typename pcl::PointCloud<CloudT>::Ptr& input, 
                      typename pcl::PointCloud<CloudT>::Ptr& output, 
                      int target_size) {
    if (input->size() <= target_size) {
        output = input;
        return;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(0, input->size()-1);

    output->clear();
    output->reserve(target_size);
    for (int i = 0; i < target_size; ++i) {
        output->push_back(input->points[distrib(gen)]);
    }
}


void generateCurve(typename pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, Vector2f min_p, Vector2f max_p, float sample_size)
{
    for (float x = min_p[0]; x < max_p[0]; x += sample_size){
        for (float y = min_p[1]; y < max_p[1]; y += sample_size){
        pcl::PointXYZ pt;
        pt.x = x;
        pt.y = y;
        pt.z = 2.0 * std::sin(x) + 3.0 * std::cos(y) - 4.0 * std::sin(x + y) * std::cos(x - y);
        cloud->push_back(pt);
        }
    }
}

int main(int argc, char** argv) {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        if (pcl::io::loadPCDFile<PointType>("/root/dataset_baogang/Cloud.pcd", *cloud) == -1) {
        PCL_ERROR("文件加载失败\n");
        return -1;
        }
    Vector3f min_p, max_p;
    printCloudInfo<PointType>(cloud, min_p, max_p);
    adjustZHeight<PointType>(cloud);
    printCloudInfo<PointType>(cloud, min_p, max_p);

    // 首次采样
    pcl::PointCloud<PointType>::Ptr filtered_cloud_0(new pcl::PointCloud<PointType>);
    
    // 创建X轴滤波器
    pcl::PassThrough<PointType> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-9.803, 0.834);
    pass_x.filter(*filtered_cloud_0);

    // 创建Y轴滤波器
    pcl::PassThrough<PointType> pass_y;
    pass_y.setInputCloud(filtered_cloud_0);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-2.63, 1.27);
    pass_y.filter(*filtered_cloud_0);

    pcl::PassThrough<PointType> pass_z;
    pass_z.setInputCloud(filtered_cloud_0);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.85, 3.119);
    pass_z.filter(*filtered_cloud_0);
    std::cout << "裁剪后点数: " << filtered_cloud_0->size() << std::endl;

    adjustZHeight<PointType>(filtered_cloud_0);
    printCloudInfo<PointType>(filtered_cloud_0, min_p, max_p);


    // 第二步：随机下采样
    // pcl::PointCloud<PointType>::Ptr sampled_cloud_0(new pcl::PointCloud<PointType>);
    // randomDownsample<PointType>(filtered_cloud_0, sampled_cloud_0, 8000);

    // std::cout << "下采样后点数: " << sampled_cloud_0->size() << std::endl;

    // 第二帧
    // // 假设前进1m
    // pcl::PointCloud<PointType>::Ptr filtered_cloud_1(new pcl::PointCloud<PointType>);

    // // 创建X轴滤波器
    // pass_x.setInputCloud(cloud);
    // pass_x.setFilterFieldName("x");
    // pass_x.setFilterLimits(-2.0, 2.0);
    // pass_x.filter(*filtered_cloud_1);

    // // 创建Y轴滤波器
    // pass_y.setInputCloud(filtered_cloud_1);
    // pass_y.setFilterFieldName("y");
    // pass_y.setFilterLimits(0.51, 3.51);
    // pass_y.filter(*filtered_cloud_1);

    // std::cout << "裁剪后点数: " << filtered_cloud_1->size() << std::endl;

    // // 第二步：随机下采样
    // pcl::PointCloud<PointType>::Ptr sampled_cloud_1(new pcl::PointCloud<PointType>);
    // randomDownsample<PointType>(filtered_cloud_1, sampled_cloud_1, 2000);

    // std::cout << "下采样后点数: " << sampled_cloud_1->size() << std::endl;

    // 可视化对比
    pcl::visualization::PCLVisualizer viewer("visual");
    
    // 原始点云（红色）
    // viewer.addPointCloud<PointType>(cloud, "original_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "original_cloud");
    
    // 处理后的点云（绿色）
    viewer.addPointCloud<PointType>(filtered_cloud_0, "processed_cloud_0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "processed_cloud_0");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "processed_cloud_0");

    // 处理后的点云（蓝色）
    // viewer.addPointCloud<PointType>(sampled_cloud_1, "processed_cloud_1");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1, "processed_cloud_1");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "processed_cloud_1");

    // 添加坐标轴
    // viewer.addCoordinateSystem(1.0);
    

    // 保持可视化窗口
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

    VoxelGridRBF<PointType> test_rbf;
    // VoxelGridRBF setting
    test_rbf.setCenter(0, 0, 0);
    test_rbf.setSigma(0.14, 0.14);
    test_rbf.setRbfGridLeafSize(0.15, 0.15);
    test_rbf.setExtent(9.803, 0.834, 2.63, 1.27);
    test_rbf.setCloudNoise(0.1);
    test_rbf.setWeightNoise(0.5);
    test_rbf.setEps(0.1);
  // test_rbf.setNormalize(true);

  // simulation for validation
  // pcl::PointCloud<pcl::PointXYZ>::Ptr simu_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // generateCurve(simu_cloud, Eigen::Vector2f(-2.0,0.0), Eigen::Vector2f(2.0,3.0), 0.1);

  // test_rbf.setCenter(0.0, 0.0);
  // test_rbf.setInputCloud(simu_cloud);
  // test_rbf.generateRBFGrid();
  // std::vector<double> kernel_matrix_vector_0;
  // test_rbf.initialWeightCuda();

  // // 可视化
  // pcl::PointCloud<pcl::PointXYZ>::Ptr visual_cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
  // test_rbf.visualize(visual_cloud_0, 0.01, 0.01);
  // pcl::visualization::PCLVisualizer viewer_0("first scan");
  // viewer_0.addPointCloud<PointType>(visual_cloud_0, "original_cloud");
  // viewer_0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "original_cloud");
  
  // // 处理后的点云（绿色）
  // viewer_0.addPointCloud<PointType>(simu_cloud, "processed_cloud_0");
  // viewer_0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "processed_cloud_0");
  // viewer_0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "processed_cloud_0");

  // while (!viewer_0.wasStopped()) {
  //     viewer_0.spinOnce(100);
  // }

    // 第一次拟合
    Vector3f temp_min, temp_max;
    std::cout << "first scan start" << std::endl;
    test_rbf.setCenter(0.0, 0.0, 0.0);
    printCloudInfo<PointType>(filtered_cloud_0, min_p, max_p);
    test_rbf.setInputCloud(filtered_cloud_0);
    test_rbf.generateRBFGrid();

    std::vector<float> kernel_matrix_vector_0;
    // 原始点云拟合
    test_rbf.initialWeightCuda();
    test_rbf.compute_error();
  // 归一化点云拟合
  // test_rbf.normalizeCloudAndGrid(min_p, max_p);
  // test_rbf.CalKernelMatrix(kernel_matrix_vector_0, test_rbf.norm_cloud_, &test_rbf.norm_grid_);

  // std::cout << "kernel_matrix: " << test_rbf.kernel_matrix_.rows() << " " << test_rbf.kernel_matrix_.cols() << std::endl << test_rbf.kernel_matrix_ << std::endl;
  // std::cout << "kernel_matrix: " << test_rbf.kernel_matrix_.rows() << " " << test_rbf.kernel_matrix_.cols() << std::endl;
  // std::cout << "==========================================================" << std::endl;
  //   std::cout << "elevation_obs: " << test_rbf.elevation_obs_.size() << std::endl << test_rbf.elevation_obs_ << std::endl;
  // std::vector<float> vec_elevation(test_rbf.elevation_obs_.data(), test_rbf.elevation_obs_.data()+test_rbf.elevation_obs_.size());
  // std::cout << "elevation_obs: " << test_rbf.elevation_obs_.size() << std::endl;
  // std::cout << "==========================================================" << std::endl;
  // test_rbf.IntialWeight();

    pcl::PointCloud<pcl::PointXYZ>::Ptr visual_cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
    test_rbf.visualize(visual_cloud_0, 0.005, 0.005);
    printCloudInfo<PointType>(visual_cloud_0, temp_min, temp_max);

  // norm可视化
  // pcl::visualization::PCLVisualizer viewer_0_norm("first scan normed");
  // viewer_0_norm.addPointCloud<PointType>(visual_cloud_0_normed, "fit");
  // viewer_0_norm.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "fit");
  
  // viewer_0_norm.addPointCloud<PointType>(test_rbf.norm_cloud_, "original_cloud");
  // viewer_0_norm.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "original_cloud");
  // viewer_0_norm.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");

  // while (!viewer_0_norm.wasStopped()) {
  //     viewer_0_norm.spinOnce(100);
  // }


    pcl::visualization::PCLVisualizer viewer_0("first scan");
    viewer_0.addPointCloud<PointType>(visual_cloud_0, "fit");
    viewer_0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "fit");
    
    viewer_0.addPointCloud<PointType>(filtered_cloud_0, "original_cloud");
    viewer_0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "original_cloud");
    viewer_0.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");

    while (!viewer_0.wasStopped()) {
        viewer_0.spinOnce(100);
    }

    std::cout << "=====================================================" << std::endl;

// //  第二次拟合
//     std::cout << "second scan start" << std::endl;
//     test_rbf.setCenter(0.0, 0.5);
//     test_rbf.setInputCloud(sampled_cloud_1);
//     test_rbf.generateRBFGrid();
//     std::vector<float> kernel_matrix_vector_1;
//     test_rbf.updateWeightCuda();
// //   rbf_size = test_rbf.rbf_weights_.size();
// //   std::cout << "rbf size: " << rbf_size << std::endl;
// //   for (int i = 0; i < rbf_size; i++){
// //     std::cout << test_rbf.rbf_weights_[i] << ' ';
// //   }
// //   std::cout << std::endl;
// //   pcl::PointCloud<PointType>::Ptr visual_cloud_1(new pcl::PointCloud<PointType>);
// //   test_rbf.Visualize(*visual_cloud_1, 0.01, 0.01);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr visual_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
//     test_rbf.visualize(visual_cloud_1, 0.005, 0.005);
//     printCloudInfo<PointType>(visual_cloud_1, temp_min, temp_max);
//     // 可视化
//     pcl::visualization::PCLVisualizer viewer_1("second scan");
//     viewer_1.addPointCloud<PointType>(visual_cloud_1, "original_cloud");
//     viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "original_cloud");
    
//     // 处理后的点云（绿色）
//     viewer_1.addPointCloud<PointType>(sampled_cloud_0, "processed_cloud_0");
//     viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "processed_cloud_0");
//     viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "processed_cloud_0");

//     viewer_1.addPointCloud<PointType>(sampled_cloud_1, "processed_cloud_1");
//     viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "processed_cloud_1");
//     viewer_1.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "processed_cloud_1");

//     while (!viewer_1.wasStopped()) {
//         viewer_1.spinOnce(100);
//     }

//   std::cout << "second prosession done" << std::endl;

  return 0;   
}