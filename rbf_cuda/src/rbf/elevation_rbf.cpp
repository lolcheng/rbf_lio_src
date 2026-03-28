#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include "scancontext/tic_toc.h"
#include "rbf_fitting/rbf_fit.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <iomanip>
using json = nlohmann::json;
using namespace std;
using namespace rbf_fit;

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> pointcloud;
typedef pcl::PointCloud<PointType>::Ptr pointcloud_ptr;

string local_map_topic, odometry_topic;
double sigma_x, sigma_y;
double leaf_size_x, leaf_size_y;
// local map extent size
double x_left, x_right, y_back, y_forward;
double cloud_noise, weight_noise;
double cloud_downsapmle_leaf;
double margin;
double move_threshold = 0.5; // 触发rbf更新的xy平面距离阈值
// rbf_setting
VoxelGridRBF<PointType> test_rbf;
bool first_scan = 0;
// ros publisher
ros::Publisher pub_elevation;
// 统计信息保存路径（绝对路径）
string file_path;
// 统计信息文件流
std::ofstream kernel_matrix_time_file;
std::ofstream weight_time_file;
std::ofstream mae_file;
bool stats_files_opened = false;

Eigen::Matrix4f odomToMatrix(const nav_msgs::Odometry::ConstPtr& odom) {
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    
    // 位置
    matrix(0,3) = odom->pose.pose.position.x;
    matrix(1,3) = odom->pose.pose.position.y;
    matrix(2,3) = odom->pose.pose.position.z;
    
    // 姿态（四元数转旋转矩阵）
    const geometry_msgs::Quaternion& q = odom->pose.pose.orientation;
    Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
    matrix.block<3,3>(0,0) = quat.toRotationMatrix();
    
    return matrix;
}

void syncCallback(
  const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
  const nav_msgs::OdometryConstPtr& odom_msg)
{
    pointcloud_ptr cloud(new pointcloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    Eigen::Matrix4f odom_matrix = odomToMatrix(odom_msg);
    if (first_scan)
    {
        test_rbf.setCenter(odom_matrix(0,3), odom_matrix(1,3), odom_matrix(2,3));
        test_rbf.setInputCloud(cloud);
        test_rbf.generateRBFGrid();
        // 原始点云拟合
        test_rbf.initialWeightCuda();
    }
    else
    {
        test_rbf.setCenter(odom_matrix(0,3), odom_matrix(1,3), odom_matrix(2,3));
        test_rbf.setInputCloud(cloud);
        test_rbf.generateRBFGrid();
        // 原始点云拟合
        test_rbf.initialWeightCuda();
    }
    test_rbf.compute_error();
    pcl::PointCloud<pcl::PointXYZ>::Ptr elevation(new pcl::PointCloud<pcl::PointXYZ>);
    test_rbf.visualize(elevation, 0.01, 0.01);

    if (!elevation->empty()) {
        sensor_msgs::PointCloud2 elevation_msg;
        pcl::toROSMsg(*elevation, elevation_msg);
        elevation_msg.header.frame_id = "map";
        elevation_msg.header.stamp = cloud_msg->header.stamp;
        pub_elevation.publish(elevation_msg);
        ROS_INFO("Elevation map published.");
    } else {
        ROS_WARN("Empty elevation cloud, skipping publish.");
    }
}

// 全局变量保存最新地图和odom
pointcloud_ptr latest_local_map(new pointcloud);
nav_msgs::Odometry last_odom;
bool has_last_odom = false;
bool has_local_map = false;

// 局部地图回调，只保存最新点云
void localMapCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pointcloud_ptr cloud(new pointcloud);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    *latest_local_map = *cloud;
    has_local_map = true;
}


// 里程计回调，判断是否需要更新rbf
void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    if (!has_local_map) return;

    if (!has_last_odom) {
        last_odom = *odom_msg;
        has_last_odom = true;
        return;
    }

    double dx = odom_msg->pose.pose.position.x - last_odom.pose.pose.position.x;
    double dy = odom_msg->pose.pose.position.y - last_odom.pose.pose.position.y;
    double dist = sqrt(dx*dx + dy*dy);

    if (dist < move_threshold) {
        return;
    }

    ROS_INFO_STREAM_THROTTLE(2.0, "RBF update triggered, distance moved: " << dist << " m");

    // RBF 拟合与发布
    Eigen::Matrix4f odom_matrix = odomToMatrix(odom_msg);
    test_rbf.setCenter(odom_matrix(0,3), odom_matrix(1,3), odom_matrix(2,3));
    test_rbf.setInputCloud(latest_local_map);
    test_rbf.generateRBFGrid();
    test_rbf.initialWeightCuda();
    test_rbf.compute_error();

    // 保存统计信息到文件
    if (stats_files_opened) {
        double timestamp = odom_msg->header.stamp.toSec();
        double kernel_time = test_rbf.getKernelMatrixTime();
        double weight_time_val = test_rbf.getWeightTime();
        double mae_val = test_rbf.getMAE();
        
        // 检查文件流状态
        if (kernel_matrix_time_file.good() && weight_time_file.good() && mae_file.good()) {
            kernel_matrix_time_file << std::fixed << std::setprecision(9) << timestamp << " " 
                                   << std::setprecision(6) << kernel_time << std::endl;
            kernel_matrix_time_file.flush();
            
            weight_time_file << std::fixed << std::setprecision(9) << timestamp << " " 
                            << std::setprecision(6) << weight_time_val << std::endl;
            weight_time_file.flush();
            
            mae_file << std::fixed << std::setprecision(9) << timestamp << " " 
                    << std::setprecision(6) << mae_val << std::endl;
            mae_file.flush();
            
            ROS_INFO_STREAM_THROTTLE(2.0, "Statistics saved: kernel=" << kernel_time 
                                   << "ms, weight=" << weight_time_val 
                                   << "ms, mae=" << mae_val);
        } else {
            ROS_WARN("Statistics file streams are not in good state!");
            if (!kernel_matrix_time_file.good()) ROS_WARN("  kernel_matrix_time_file bad");
            if (!weight_time_file.good()) ROS_WARN("  weight_time_file bad");
            if (!mae_file.good()) ROS_WARN("  mae_file bad");
        }
    } else {
        ROS_WARN_THROTTLE(5.0, "Statistics files not opened, cannot save data");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr elevation(new pcl::PointCloud<pcl::PointXYZ>);
    test_rbf.visualize(elevation, 0.01, 0.01);

    if (!elevation->empty()) {
        sensor_msgs::PointCloud2 elevation_msg;
        pcl::toROSMsg(*elevation, elevation_msg);
        elevation_msg.header.frame_id = "map";
        elevation_msg.header.stamp = odom_msg->header.stamp;
        pub_elevation.publish(elevation_msg);
        ROS_INFO("Elevation map published.");
    } else {
        ROS_WARN("Empty elevation cloud, skipping publish.");
    }

    // --- Save data to JSON file named by odom timestamp ---
    std::string out_dir = "/root/code/result/RBF";
    std::string mkdir_cmd = "mkdir -p " + out_dir;
    (void)system(mkdir_cmd.c_str());

    std::ostringstream fname_ss;
    fname_ss << out_dir << "/" << odom_msg->header.stamp.sec << "_" << odom_msg->header.stamp.nsec << ".json";
    std::string filepath = fname_ss.str();

    try {
        json j;

        // timestamp as string
        std::ostringstream ts_ss;
        ts_ss << odom_msg->header.stamp.sec << "." << std::setw(9) << std::setfill('0') << odom_msg->header.stamp.nsec;
        j["timestamp"] = ts_ss.str();

        // pose
        const auto &pos = odom_msg->pose.pose.position;
        const auto &ori = odom_msg->pose.pose.orientation;
        j["pose"]["position"] = { {"x", pos.x}, {"y", pos.y}, {"z", pos.z} };
        j["pose"]["orientation"] = { {"x", ori.x}, {"y", ori.y}, {"z", ori.z}, {"w", ori.w} };
        j["mean_weight"] = test_rbf.mean_weight_;

        // act_rbf_grid_: convert vector<Eigen::Matrix<double,2,1>> to JSON array of [x,y]
        {
            json cur_grid = json::array();
            for (const auto &v : test_rbf.act_rbf_grid_) {
                // v is Eigen::Matrix<double,2,1> (2x1 column vector)
                cur_grid.push_back( { v(0), v(1) } );
            }
            j["cur_rbf_grid"] = std::move(cur_grid);
        }

        // rbf_weight_ (assume vector<double>)
        j["rbf_weight"] = test_rbf.rbf_weight_;

        // 写入文件
        if (false)
        {std::ofstream ofs(filepath);
        if (!ofs.is_open()) {
            ROS_WARN_STREAM("Failed to open JSON file for writing: " << filepath);
        } else {
            ofs << j.dump(2) << std::endl; // indent=2
            ofs.close();
            ROS_INFO_STREAM("Saved RBF data to JSON: " << filepath);
        }}
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("Exception while serializing JSON: " << e.what());
    }

    last_odom = *odom_msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rbf_elevation");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh.param<string>("rbf/local_map", local_map_topic, "/local_map");
    nh.param<string>("rbf/odometry", odometry_topic, "/Odometry");
    nh.param<double>("rbf/sigma_x", sigma_x, 0.14);
    nh.param<double>("rbf/sigma_y", sigma_y, 0.14);
    nh.param<double>("rbf/leaf_size_x", leaf_size_x, 0.2);
    nh.param<double>("rbf/leaf_size_y", leaf_size_y, 0.2);
    nh.param<double>("rbf/x_left", x_left, 3.0);
    nh.param<double>("rbf/x_right", x_right, 3.0);
    nh.param<double>("rbf/y_back", y_back, 3.0);
    nh.param<double>("rbf/y_forward", y_forward, 3.0);
    nh.param<double>("rbf/cloud_noise", cloud_noise, 0.1);
    nh.param<double>("rbf/weight_noise", weight_noise, 0.5);
    nh.param<double>("rbf/cloud_downsapmle_leaf", cloud_downsapmle_leaf, 0.05);
    nh.param<double>("rbf/margin", margin, 1e-3);
    nh_private.param<string>("file_path", file_path, "/root/code/result/staircase2");

    // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/local_map", 1);
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/Odometry", 1);
    // typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
    // message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), cloud_sub, odom_sub);
    // sync.registerCallback(boost::bind(&syncCallback, _1, _2));

    // pub_elevation = nh.advertise<sensor_msgs::PointCloud2>("/elevation_registered", 100000);

    ros::Subscriber sub_local_map = nh.subscribe(local_map_topic, 1, localMapCallback);
    ros::Subscriber sub_odom = nh.subscribe(odometry_topic, 10, odomCallback);

    pub_elevation = nh.advertise<sensor_msgs::PointCloud2>("/rbf_elevation_map", 10);

    // VoxelGridRBF setting
    test_rbf.setSigma(sigma_x, sigma_y);
    test_rbf.setRbfGridLeafSize(leaf_size_x, leaf_size_y);
    test_rbf.setExtent(x_left, x_right, y_back, y_forward);
    test_rbf.setCloudNoise(cloud_noise);
    test_rbf.setWeightNoise(weight_noise);
    test_rbf.setEps(cloud_downsapmle_leaf);
    test_rbf.setMargin(margin);

    // 初始化统计信息保存文件
    std::string mkdir_cmd = "mkdir -p " + file_path;
    (void)system(mkdir_cmd.c_str());

    std::string kernel_matrix_time_path = file_path + "/kernel_matrix_time.txt";
    std::string weight_time_path = file_path + "/weight_time.txt";
    std::string mae_path = file_path + "/mae.txt";

    kernel_matrix_time_file.open(kernel_matrix_time_path, std::ios::out);
    weight_time_file.open(weight_time_path, std::ios::out);
    mae_file.open(mae_path, std::ios::out);

    if (kernel_matrix_time_file.is_open() && weight_time_file.is_open() && mae_file.is_open()) {
        stats_files_opened = true;
        ROS_INFO_STREAM("Statistics output path: " << file_path);
        ROS_INFO_STREAM("Statistics files opened:");
        ROS_INFO_STREAM("  - " << kernel_matrix_time_path);
        ROS_INFO_STREAM("  - " << weight_time_path);
        ROS_INFO_STREAM("  - " << mae_path);
    } else {
        ROS_WARN("Failed to open statistics files, statistics feature will be disabled");
        ROS_WARN_STREAM("  kernel_matrix_time_file: " << (kernel_matrix_time_file.is_open() ? "open" : "failed"));
        ROS_WARN_STREAM("  weight_time_file: " << (weight_time_file.is_open() ? "open" : "failed"));
        ROS_WARN_STREAM("  mae_file: " << (mae_file.is_open() ? "open" : "failed"));
        if (kernel_matrix_time_file.is_open()) kernel_matrix_time_file.close();
        if (weight_time_file.is_open()) weight_time_file.close();
        if (mae_file.is_open()) mae_file.close();
    }

    ros::spin();

    // 关闭文件
    if (kernel_matrix_time_file.is_open()) kernel_matrix_time_file.close();
    if (weight_time_file.is_open()) weight_time_file.close();
    if (mae_file.is_open()) mae_file.close();

    return 0;
}
