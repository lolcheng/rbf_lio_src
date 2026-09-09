#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Fix missing libflann serialization specialization for std::unordered_map
// on some distros (pulled in by PCL kdtree/flann headers).
#include "flann_unordered_map_serialization_fix.h"

// OpenCV2 legacy headers (e.g. <opencv/cv.h>) were removed in OpenCV3/4.
// Use the unified OpenCV header instead.
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <utility>

// Livox CustomMsg compatibility:
// - Older ROS1 driver:  livox_ros_driver/CustomMsg
// - Newer ROS1 driver:  livox_ros_driver2/CustomMsg
// The selected header must have the same ROS message MD5 as the producer.
#ifndef __has_include
  #define __has_include(x) 0
#endif
#if __has_include(<livox_ros_driver2/CustomMsg.h>)
  #include <livox_ros_driver2/CustomMsg.h>
  namespace livox_msg = livox_ros_driver2;
  #define LIVOX_CUSTOMMSG_PKG "livox_ros_driver2"
  #define LIO_SAM_HAS_LIVOX 1
#elif __has_include(<livox_ros_driver/CustomMsg.h>)
  #include <livox_ros_driver/CustomMsg.h>
  namespace livox_msg = livox_ros_driver;
  #define LIVOX_CUSTOMMSG_PKG "livox_ros_driver"
  #define LIO_SAM_HAS_LIVOX 1
#else
  #define LIVOX_CUSTOMMSG_PKG "not available"
  #define LIO_SAM_HAS_LIVOX 0
#endif
using namespace std;

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX, AIRY };

class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Save TUM trajectory
    bool saveTUMTrajectory;
    string saveTUMTrajectoryPath;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imuType;
    double imuFrequency;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;
    bool enableRbfConstraint;
    double rbfConstraintWeight;
    int rbfConstraintMaxPoints;
    double rbfSigmaX;
    double rbfSigmaY;
    int rbfNodeMaxNum;
    double rbfWeightR1;
    double rbfWeightR2;
    double rbfWeightR3;
    double rbfWeightR4;
    double rbfMaxAbsR1;
    double rbfMaxAbsR2;
    double rbfMaxAbsR3;
    double rbfMaxNormR4;
    std::string jointStateTopic;
    std::string robotKinematicsModel;
    double wheelRadius;
    bool enableLidarZCalibration;
    double lidarZInit;
    double lidarZMin;
    double lidarZMax;
    double lidarZLearningRate;
    double lidarResidualEmaAlpha;
    double lidarResidualDeadband;
    int lidarResidualMinConstraintNum;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    // Debug
    bool debugPrintImu;
    double debugPrintImuPeriod; // seconds, used by ROS_*_THROTTLE

    // Failure detection thresholds (used in imuPreintegration)
    double failureDetectionVelTh;
    double failureDetectionAccBiasTh;
    double failureDetectionGyrBiasTh;

    // Optional point cloud transform (applied to each raw LiDAR point before processing).
    // Use this when the LiDAR is mounted with a rotation relative to the desired lidarFrame axes.
    bool pointCloudTransformEnable;
    vector<double> pointCloudRotV;
    vector<double> pointCloudTransV;
    Eigen::Matrix3d pointCloudRot;
    Eigen::Vector3d pointCloudTrans;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<bool>("lio_sam/debugPrintImu", debugPrintImu, false);
        nh.param<double>("lio_sam/debugPrintImuPeriod", debugPrintImuPeriod, 1.0);

        nh.param<double>("lio_sam/failureDetectionVelTh", failureDetectionVelTh, 30.0);
        nh.param<double>("lio_sam/failureDetectionAccBiasTh", failureDetectionAccBiasTh, 1.0);
        nh.param<double>("lio_sam/failureDetectionGyrBiasTh", failureDetectionGyrBiasTh, 1.0);

        nh.param<bool>("lio_sam/pointCloudTransformEnable", pointCloudTransformEnable, false);
        nh.param<vector<double>>("lio_sam/pointCloudRot", pointCloudRotV, vector<double>());
        nh.param<vector<double>>("lio_sam/pointCloudTrans", pointCloudTransV, vector<double>());
        if (pointCloudRotV.size() == 9 && pointCloudTransV.size() == 3)
        {
            pointCloudRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(pointCloudRotV.data(), 3, 3);
            pointCloudTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(pointCloudTransV.data(), 3, 1);
        }
        else
        {
            // Safe defaults (identity transform)
            pointCloudRot = Eigen::Matrix3d::Identity();
            pointCloudTrans = Eigen::Vector3d::Zero();
        }

        ROS_INFO_STREAM("[lio_sam] Livox CustomMsg package: " << LIVOX_CUSTOMMSG_PKG);
        ROS_INFO_STREAM("[lio_sam] pointCloudTopic: " << pointCloudTopic);
        ROS_INFO_STREAM("[lio_sam] imuTopic: " << imuTopic);
        ROS_INFO_STREAM("[lio_sam] odomTopic: " << odomTopic);
        ROS_INFO_STREAM("[lio_sam] gpsTopic: " << gpsTopic);
        ROS_INFO_STREAM("[lio_sam] debugPrintImu: " << (debugPrintImu ? "true" : "false")
                                                   << ", debugPrintImuPeriod: " << debugPrintImuPeriod << "s");
        ROS_INFO_STREAM("[lio_sam] failureDetectionVelTh: " << failureDetectionVelTh
                         << " failureDetectionAccBiasTh: " << failureDetectionAccBiasTh
                         << " failureDetectionGyrBiasTh: " << failureDetectionGyrBiasTh);
        ROS_INFO_STREAM("[lio_sam] pointCloudTransformEnable: " << (pointCloudTransformEnable ? "true" : "false"));
        if (pointCloudTransformEnable)
        {
            ROS_INFO_STREAM("[lio_sam] pointCloudTrans: [" << pointCloudTrans.x() << ", " << pointCloudTrans.y() << ", " << pointCloudTrans.z() << "]");
            ROS_INFO_STREAM("[lio_sam] pointCloudRot:\n" << pointCloudRot);
        }

        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("lio_sam/savePCD", savePCD, false);
        nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<bool>("lio_sam/saveTUMTrajectory", saveTUMTrajectory, false);
        nh.param<std::string>("lio_sam/saveTUMTrajectoryPath", saveTUMTrajectoryPath, "");

        std::string sensorStr;
        nh.param<std::string>("lio_sam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        }
        else if (sensorStr == "airy" || sensorStr == "robosense")
        {
            sensor = SensorType::AIRY;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be 'velodyne', 'ouster', 'livox' or 'airy'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("lio_sam/imuType", imuType, 0);
        nh.param<double>("lio_sam/imuFrequency", imuFrequency, 500.0);
        if (!std::isfinite(imuFrequency) || imuFrequency <= 0.0)
        {
            ROS_FATAL_STREAM("Invalid lio_sam/imuFrequency: " << imuFrequency);
            ros::shutdown();
            imuFrequency = 500.0;
        }

        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
        nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
        if (extRotV.size() != 9 || extRPYV.size() != 9 || extTransV.size() != 3)
        {
            ROS_FATAL("Invalid IMU extrinsics: extrinsicRot/extrinsicRPY require 9 values and extrinsicTrans requires 3 values.");
            ros::shutdown();
            extRot = Eigen::Matrix3d::Identity();
            extRPY = Eigen::Matrix3d::Identity();
            extTrans = Eigen::Vector3d::Zero();
        }
        else
        {
            extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
            extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
            extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        }
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        ROS_INFO_STREAM("[lio_sam] imuType: " << imuType << " imuFrequency: " << imuFrequency
                        << " imuGravity: " << imuGravity);
        ROS_INFO_STREAM("[lio_sam] extrinsicTrans: [" << extTrans.x() << ", " << extTrans.y() << ", " << extTrans.z() << "]");
        ROS_INFO_STREAM("[lio_sam] extrinsicRot:\n" << extRot);
        ROS_INFO_STREAM("[lio_sam] extrinsicRPY:\n" << extRPY);
        ROS_INFO_STREAM("[lio_sam] extQRPY (wxyz): " << extQRPY.w() << ", " << extQRPY.x() << ", " << extQRPY.y() << ", " << extQRPY.z());

        nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);
        nh.param<bool>("lio_sam/enableRbfConstraint", enableRbfConstraint, false);
        nh.param<double>("lio_sam/rbfConstraintWeight", rbfConstraintWeight, 1.0);
        nh.param<int>("lio_sam/rbfConstraintMaxPoints", rbfConstraintMaxPoints, 2000);
        nh.param<double>("lio_sam/rbfSigmaX", rbfSigmaX, 0.3);
        nh.param<double>("lio_sam/rbfSigmaY", rbfSigmaY, 0.3);
        nh.param<int>("lio_sam/rbfNodeMaxNum", rbfNodeMaxNum, 4000);
        nh.param<double>("lio_sam/rbfWeightR1", rbfWeightR1, 1.0);
        nh.param<double>("lio_sam/rbfWeightR2", rbfWeightR2, 0.3);
        nh.param<double>("lio_sam/rbfWeightR3", rbfWeightR3, 0.2);
        nh.param<double>("lio_sam/rbfWeightR4", rbfWeightR4, 0.2);
        nh.param<double>("lio_sam/rbfMaxAbsR1", rbfMaxAbsR1, 0.25);
        nh.param<double>("lio_sam/rbfMaxAbsR2", rbfMaxAbsR2, 0.10);
        nh.param<double>("lio_sam/rbfMaxAbsR3", rbfMaxAbsR3, 0.08);
        nh.param<double>("lio_sam/rbfMaxNormR4", rbfMaxNormR4, 0.80);
        nh.param<std::string>("lio_sam/jointStateTopic", jointStateTopic, "/joint_states");
        nh.param<std::string>("lio_sam/robotKinematicsModel", robotKinematicsModel, "tron1a");
        nh.param<double>("lio_sam/wheelRadius", wheelRadius, 0.127);
        nh.param<bool>("lio_sam/enableLidarZCalibration", enableLidarZCalibration, false);
        nh.param<double>("lio_sam/lidarZInit", lidarZInit, 0.0);
        nh.param<double>("lio_sam/lidarZMin", lidarZMin, -0.10);
        nh.param<double>("lio_sam/lidarZMax", lidarZMax, 0.10);
        nh.param<double>("lio_sam/lidarZLearningRate", lidarZLearningRate, 0.001);
        nh.param<double>("lio_sam/lidarResidualEmaAlpha", lidarResidualEmaAlpha, 0.10);
        nh.param<double>("lio_sam/lidarResidualDeadband", lidarResidualDeadband, 0.002);
        nh.param<int>("lio_sam/lidarResidualMinConstraintNum", lidarResidualMinConstraintNum, 2);

        nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    inline void maybePrintImuDebug(const sensor_msgs::Imu& raw, const sensor_msgs::Imu& converted, const std::string& from) const
    {
        if (!debugPrintImu)
            return;

        // Throttle in seconds. Guard against invalid values.
        const double period = (debugPrintImuPeriod > 0.0) ? debugPrintImuPeriod : 1.0;

        ROS_INFO_STREAM_THROTTLE(period,
            "[lio_sam][imu][" << from << "]"
            << std::fixed << std::setprecision(6)
            << " t=" << raw.header.stamp.toSec()
            << " frame=" << (raw.header.frame_id.empty() ? "<empty>" : raw.header.frame_id)
            << " raw_acc=(" << raw.linear_acceleration.x << "," << raw.linear_acceleration.y << "," << raw.linear_acceleration.z << ")"
            << " raw_gyr=(" << raw.angular_velocity.x << "," << raw.angular_velocity.y << "," << raw.angular_velocity.z << ")"
            << " raw_q(wxyz)=(" << raw.orientation.w << "," << raw.orientation.x << "," << raw.orientation.y << "," << raw.orientation.z << ")"
            << " | conv_acc=(" << converted.linear_acceleration.x << "," << converted.linear_acceleration.y << "," << converted.linear_acceleration.z << ")"
            << " conv_gyr=(" << converted.angular_velocity.x << "," << converted.angular_velocity.y << "," << converted.angular_velocity.z << ")"
            << " conv_q(wxyz)=(" << converted.orientation.w << "," << converted.orientation.x << "," << converted.orientation.y << "," << converted.orientation.z << ")"
        );
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in) 
    {
    sensor_msgs::Imu imu_out = imu_in;
    // rotate acceleration
    Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
    //livox 内置的六轴imu的加速度单位是g 这里要还原到m/s^2
    if(imuType==0)
        acc*=imuGravity;

    acc = extRot * acc;
    imu_out.linear_acceleration.x = acc.x();
    imu_out.linear_acceleration.y = acc.y();
    imu_out.linear_acceleration.z = acc.z();
    // rotate gyroscope
    Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
    gyr = extRot * gyr;
    imu_out.angular_velocity.x = gyr.x();
    imu_out.angular_velocity.y = gyr.y();
    imu_out.angular_velocity.z = gyr.z();
    // rotate roll pitch yaw
    Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                imu_in.orientation.z);
    Eigen::Quaterniond q_final;

    if (imuType == 0)
    {
        q_final = extQRPY;
    }
    else if (imuType == 1)
    {
        q_final = q_from * extQRPY;
    }
    else
        std::cout << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;

    q_final.normalize();
    imu_out.orientation.x = q_final.x();
    imu_out.orientation.y = q_final.y();
    imu_out.orientation.z = q_final.z();
    imu_out.orientation.w = q_final.w();

    if (sqrt(
            q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() +
            q_final.w() * q_final.w())
        < 0.1) {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}
};

template<typename T>
sensor_msgs::PointCloud2 publishCloud(const ros::Publisher& thisPub, const T& thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub.getNumSubscribers() != 0)
        thisPub.publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
