#include "lio_sam/utility.h"
#include <pcl/registration/ndt.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include "rbf_fitting/rbf_fit.h"
#include "scancontext/tic_toc.h"

class NDTOdometry : public ParamServer
{
public:
    ros::Subscriber subCloud;
    ros::Subscriber subImuOdom;
    ros::Subscriber subLocalMap;
    ros::Publisher pubNdtOdom;
    ros::Publisher pubNdtPath;
    ros::Publisher pubRBFElevation;
    ros::Publisher pubLocalMap;
    std::deque<nav_msgs::Odometry> imuOdomQueue;
    double lastCloudTime = -1;

    pcl::PointCloud<PointType>::Ptr lastCloud;
    pcl::PointCloud<PointType>::Ptr lastLocalMap;
    pcl::PointCloud<pcl::PointXYZ>::Ptr RBF_elevation;
    bool hasLastCloud = false;

    Eigen::Matrix4f lastPose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f imuGuess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f lastImuPose;

    nav_msgs::Path ndtPath;

    rbf_fit::VoxelGridRBF<PointType> test_rbf;
    double latestOdomX = 0.0;
    double latestOdomY = 0.0;
    double latestOdomZ = 0.0;

    NDTOdometry()
    {
        subCloud = nh.subscribe<sensor_msgs::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 5, &NDTOdometry::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        subImuOdom = nh.subscribe<nav_msgs::Odometry>(
            odomTopic+"_incremental", 2000, &NDTOdometry::imuOdomHandler, this, ros::TransportHints().tcpNoDelay());
        subLocalMap = nh.subscribe<sensor_msgs::PointCloud2>(
            "lio_sam/ndt/local_map", 5, &NDTOdometry::localMapHandler, this, ros::TransportHints().tcpNoDelay());


        pubNdtOdom = nh.advertise<nav_msgs::Odometry>("lio_sam/ndt/odometry", 5);
        pubNdtPath = nh.advertise<nav_msgs::Path>("lio_sam/ndt/path", 1);
        pubRBFElevation = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/ndt/rbf_elevation", 1);
        pubLocalMap = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/ndt/local_map", 1);

        lastCloud.reset(new pcl::PointCloud<PointType>());
        lastLocalMap.reset(new pcl::PointCloud<PointType>());

        test_rbf.setSigma(sigma_x, sigma_y);
        test_rbf.setRbfGridLeafSize(leaf_size_x, leaf_size_y);
        test_rbf.setExtent(x_left, x_right, y_back, y_forward);
        test_rbf.setCloudNoise(cloud_noise);
        test_rbf.setWeightNoise(weight_noise);
        test_rbf.setEps(cloud_downsapmle_leaf);
        test_rbf.setMargin(margin);
    }

    void imuOdomHandler(const nav_msgs::Odometry::ConstPtr& odomMsg)
    {
        imuOdomQueue.push_back(*odomMsg);
        // 保证队列不过大
        while (!imuOdomQueue.empty() && imuOdomQueue.front().header.stamp.toSec() < odomMsg->header.stamp.toSec() - 5.0)
            imuOdomQueue.pop_front();
            
        latestOdomX = odomMsg->pose.pose.position.x;
        latestOdomY = odomMsg->pose.pose.position.y;
        latestOdomZ = odomMsg->pose.pose.position.z;
        test_rbf.setCenter(latestOdomX, latestOdomY, latestOdomZ);
    }

    // 获取指定时间最近的IMU里程计
    bool getImuPose(double stamp, Eigen::Matrix4f& pose)
    {
        if (imuOdomQueue.empty()) return false;
        
        // 清理掉stamp时间之前的数据
        while (!imuOdomQueue.empty() && imuOdomQueue.front().header.stamp.toSec() < stamp)
            imuOdomQueue.pop_front();
            
        if (imuOdomQueue.empty()) return false;
        
        // 找最近的IMU数据
        nav_msgs::Odometry& closest = imuOdomQueue.front();
        double min_diff = fabs(closest.header.stamp.toSec() - stamp);
        
        for (auto& odom : imuOdomQueue)
        {
            double diff = fabs(odom.header.stamp.toSec() - stamp);
            if (diff < min_diff)
            {
                min_diff = diff;
                closest = odom;
            }
        }
        
        if (min_diff > 0.05) return false;
        
        Eigen::Quaternionf q(closest.pose.pose.orientation.w,
                            closest.pose.pose.orientation.x,
                            closest.pose.pose.orientation.y,
                            closest.pose.pose.orientation.z);
        Eigen::Vector3f t(closest.pose.pose.position.x,
                         closest.pose.pose.position.y,
                         closest.pose.pose.position.z);
        pose = Eigen::Matrix4f::Identity();
        pose.block<3,3>(0,0) = q.toRotationMatrix();
        pose.block<3,1>(0,3) = t;
        return true;
    }

    void localMapHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
    {
        pcl::PointCloud<PointType>::Ptr localMapCloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*cloudMsg, *localMapCloud);
        std::cout << "lastLocalMap size: " << localMapCloud->size() << std::endl;
        test_rbf.setInputCloud(localMapCloud);
        test_rbf.generateRBFGrid();
        test_rbf.initialWeightCuda();
        // RBF output
        test_rbf.compute_error();
        pcl::PointCloud<pcl::PointXYZ>::Ptr RBF_elevation(new pcl::PointCloud<pcl::PointXYZ>);
        test_rbf.visualize(RBF_elevation, 0.05, 0.05);

        // publish RBF elevation
        sensor_msgs::PointCloud2 rbf_msg;
        pcl::toROSMsg(*RBF_elevation, rbf_msg);
        rbf_msg.header = cloudMsg->header;
        rbf_msg.header.frame_id = "map";
        pubRBFElevation.publish(rbf_msg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
    {
        pcl::PointCloud<PointType>::Ptr currentCloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*cloudMsg, *currentCloud);
        if (currentCloud->empty())
        {
            ROS_WARN("Received empty point cloud, skipping processing.");
            return;
        }

        Eigen::Matrix4f imuPoseCur, imuPoseLast;
        double curTime = cloudMsg->header.stamp.toSec();

        if (!getImuPose(curTime, imuPoseCur))
            return;

        if (!hasLastCloud)
        {
            *lastCloud = *currentCloud;
            hasLastCloud = true;
            lastPose = imuPoseCur;
            lastCloudTime = curTime;
            // 初始化lastLocalMap为空
            lastLocalMap->clear();
        }
        
        // 1. 计算当前帧相对于世界的变换
        Eigen::Matrix4f T_w_cur = imuPoseCur;
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(0.1f, 0.1f, 0.1f);

        // 2. 将currentCloud变换到世界坐标系
        pcl::PointCloud<PointType>::Ptr currentCloudWorld(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr boxLocalMapCropped(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*currentCloud, *currentCloudWorld, T_w_cur);
        voxel.setInputCloud(currentCloudWorld);
        voxel.filter(*currentCloudWorld);
        // 3. 拼接到lastLocalMap
        *lastLocalMap += *currentCloudWorld;

        // 4. 对lastLocalMap做bounding box和体素降采样
        // 4.1 bounding box裁剪
        pcl::PointCloud<PointType>::Ptr localMapCropped(new pcl::PointCloud<PointType>());
        float min_x = T_w_cur(0,3) - 30, max_x = T_w_cur(0,3) + 30;
        float min_y = T_w_cur(1,3) - 30, max_y = T_w_cur(1,3) + 30;
        float min_z = T_w_cur(2,3) - 5,  max_z = T_w_cur(2,3) + 0.5;
        float box_min_x = T_w_cur(0,3) - 6, box_max_x = T_w_cur(0,3) + 6;
        float box_min_y = T_w_cur(1,3) - 6, box_max_y = T_w_cur(1,3) + 6;
        float box_min_z = T_w_cur(2,3) - 5, box_max_z = T_w_cur(2,3) + 0.5;
        for (const auto& pt : lastLocalMap->points) {
            if (pt.x > min_x && pt.x < max_x &&
                pt.y > min_y && pt.y < max_y &&
                pt.z > min_z && pt.z < max_z) {
                localMapCropped->push_back(pt);
            }
            if (pt.x > box_min_x && pt.x < box_max_x &&
                pt.y > box_min_y && pt.y < box_max_y &&
                pt.z > box_min_z && pt.z < box_max_z) {
                // 仅保留box内的点
                boxLocalMapCropped->push_back(pt);
            }
        }

        // 4.2 体素降采样
        voxel.setInputCloud(localMapCropped);
        voxel.filter(*lastLocalMap);

        voxel.setInputCloud(boxLocalMapCropped);
        voxel.filter(*boxLocalMapCropped);

        // RBF module
        // test_rbf.setCenter(T_w_cur(0,3), T_w_cur(1,3));
        // test_rbf.setInputCloud(lastLocalMap);
        // test_rbf.generateRBFGrid();
        // test_rbf.initialWeightCuda();
        // // RBF output
        // test_rbf.compute_error();
        // pcl::PointCloud<pcl::PointXYZ>::Ptr RBF_elevation(new pcl::PointCloud<pcl::PointXYZ>);
        // test_rbf.visualize(RBF_elevation, 0.05, 0.05);
        // std::cout << "1" << std::endl;  



        // // 计算IMU里程计的相对变换
        // Eigen::Matrix4f initGuess = imuPoseLast.inverse() * imuPoseCur;
        
        // // NDT 配准之前添加点云过滤
        // pcl::PointCloud<PointType>::Ptr filteredCurrentCloud(new pcl::PointCloud<PointType>());
        // pcl::PointCloud<PointType>::Ptr filteredLastCloud(new pcl::PointCloud<PointType>());
        // // 过滤当前帧中的非法点
        // for(const auto& point : currentCloud->points) {
        //     if(pcl::isFinite(point) && 
        //     std::abs(point.x) < 10 && std::abs(point.y) < 10 && std::abs(point.z) < 10) {
        //         filteredCurrentCloud->push_back(point);
        //     }
        // }
        // filteredCurrentCloud->is_dense = false;
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*filteredCurrentCloud, *filteredCurrentCloud, indices);
        // for (const auto& point : *filteredCurrentCloud) {
        //     if (!pcl::isFinite(point)) {
        //         std::cerr << "Invalid point found!" << std::endl;
        //     }
        // }
        // // 过滤上一帧中的非法点
        // for(const auto& point : lastCloud->points) {
        //     if(pcl::isFinite(point) && 
        //     std::abs(point.x) < 10 && std::abs(point.y) < 10 && std::abs(point.z) < 10) {
        //         filteredLastCloud->push_back(point);
        //     }
        // }
        // filteredLastCloud->is_dense = false;
        // pcl::removeNaNFromPointCloud(*filteredLastCloud, *filteredLastCloud, indices);
        // for (const auto& point : *filteredLastCloud) {
        //     if (!pcl::isFinite(point)) {
        //         std::cerr << "Invalid point found!" << std::endl;
        //     }
        // }
        // NDT 配准
        // pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        // ndt.setTransformationEpsilon(0.01);
        // ndt.setStepSize(0.1);
        // ndt.setResolution(1.0);
        // ndt.setMaximumIterations(30);
        // ndt.setInputSource(filteredCurrentCloud);
        // std::cout << "NDT input source size: " << filteredCurrentCloud->size() << std::endl;
        // ndt.setInputTarget(filteredLastCloud);
        // std::cout << "NDT input target size: " << filteredLastCloud->size() << std::endl;

        // pcl::PointCloud<PointType>::Ptr alignedCloud(new pcl::PointCloud<PointType>());
    
        // ndt.align(*alignedCloud, initGuess);
        // std::cout << "2" << std::endl;
        // Eigen::Matrix4f T = ndt.getFinalTransformation();

    
        // ICP 配准
        // pcl::IterativeClosestPoint<PointType, PointType> icp;
        // icp.setMaxCorrespondenceDistance(1.0);      // 最大对应点距离
        // icp.setMaximumIterations(50);               // 最大迭代次数
        // icp.setTransformationEpsilon(1e-6);         // 收敛判据
        // icp.setEuclideanFitnessEpsilon(1e-6);       // 收敛判据
        // icp.setInputSource(filteredCurrentCloud);
        // std::cout << "NDT input source size: " << filteredCurrentCloud->size() << std::endl;
        // icp.setInputTarget(filteredLastCloud);
        // std::cout << "NDT input target size: " << filteredLastCloud->size() << std::endl;

        // pcl::PointCloud<PointType>::Ptr alignedCloud(new pcl::PointCloud<PointType>());
        // icp.align(*alignedCloud, initGuess);
        // std::cout << "2" << std::endl;

        // if (!icp.hasConverged()) {
        //     ROS_WARN("ICP did not converge!");
        //     return;
        // }
        // std::cout << "ICP converged, score: " << icp.getFitnessScore() << std::endl;

        // Eigen::Matrix4f T = icp.getFinalTransformation();

        // // 对齐后的点云再次过滤非法点
        // pcl::PointCloud<PointType>::Ptr filteredAlignedCloud(new pcl::PointCloud<PointType>());
        // for(const auto& point : alignedCloud->points) {
        //     if(pcl::isFinite(point) &&
        //     !std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z) &&
        //     !std::isinf(point.x) && !std::isinf(point.y) && !std::isinf(point.z)) {
        //         filteredAlignedCloud->push_back(point);
        //     }
        // }
        // if(filteredAlignedCloud->empty()) {
        //     ROS_WARN("Aligned cloud is empty after filtering, skipping this frame.");
        //     return;
        // }

        // lastPose = lastPose * T;
        // *lastCloud = *filteredAlignedCloud;

        // 更新上一帧的时间和IMU位姿
        lastCloudTime = curTime;
        lastImuPose = imuPoseCur;

        // // 发布里程计
        // nav_msgs::Odometry odom;
        // odom.header = cloudMsg->header;
        // odom.header.frame_id = "map";
        // odom.child_frame_id = "ndt_odom";
        // odom.pose.pose.position.x = lastPose(0,3);
        // odom.pose.pose.position.y = lastPose(1,3);
        // odom.pose.pose.position.z = lastPose(2,3);
        // Eigen::Quaternionf q(lastPose.block<3,3>(0,0));
        // odom.pose.pose.orientation.x = q.x();
        // odom.pose.pose.orientation.y = q.y();
        // odom.pose.pose.orientation.z = q.z();
        // odom.pose.pose.orientation.w = q.w();
        // pubNdtOdom.publish(odom);

        // // 发布路径
        // geometry_msgs::PoseStamped pose_stamped;
        // pose_stamped.header = odom.header;
        // pose_stamped.pose = odom.pose.pose;
        // ndtPath.header = odom.header;
        // ndtPath.poses.push_back(pose_stamped);
        // pubNdtPath.publish(ndtPath);

        // publish RBF elevation
        // sensor_msgs::PointCloud2 rbf_msg;
        // pcl::toROSMsg(*RBF_elevation, rbf_msg);
        // rbf_msg.header = cloudMsg->header;
        // rbf_msg.header.frame_id = "map";
        // pubRBFElevation.publish(rbf_msg);

        // 发布lastLocalMap
        sensor_msgs::PointCloud2 local_map_msg;
        pcl::toROSMsg(*lastLocalMap, local_map_msg);
        local_map_msg.header = cloudMsg->header;
        local_map_msg.header.frame_id = "map";
        pubLocalMap.publish(local_map_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam_ndt_odometry");
    NDTOdometry ndtOdom;
    ROS_INFO("\033[1;32m----> NDT Odometry Started.\033[0m");
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}