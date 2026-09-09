#pragma once
#ifndef _LIO_SAM_RBF_WHEEL_KINEMATICS_H_
#define _LIO_SAM_RBF_WHEEL_KINEMATICS_H_

#include "rbf_lm_interface.h"
#include <array>
#include <mutex>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Geometry>

namespace lio_sam_rbf
{
class JointStateWheelKinematics final : public RobotKinematicsModel
{
public:
    JointStateWheelKinematics(const std::string& topicName, float wheelRadius);
    bool computeWheelGeometry(
        const std::array<float, 6>& pose,
        std::vector<WheelGeometry, Eigen::aligned_allocator<WheelGeometry>>& wheels) override;

private:
    void jointStateHandler(const sensor_msgs::JointState::ConstPtr& msg);
    bool getJointVector(std::array<float, 8>& qOut) const;
    Eigen::Vector3f computeWheelCenterBase(bool left, const std::array<float, 8>& q) const;
    Eigen::Vector3f computeWheelAxisBase(bool left, const std::array<float, 8>& q) const;
    static Eigen::Matrix3f rotX(float a);
    static Eigen::Matrix3f rotY(float a);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subJointState_;
    mutable std::mutex mtxJoint_;
    std::array<float, 8> latestQ_;
    bool hasJointState_ = false;
    float wheelRadius_ = 0.127f;
};

class M20JointStateWheelKinematics final : public RobotKinematicsModel
{
public:
    M20JointStateWheelKinematics(const std::string& topicName, float wheelRadius);
    bool computeWheelGeometry(
        const std::array<float, 6>& pose,
        std::vector<WheelGeometry, Eigen::aligned_allocator<WheelGeometry>>& wheels) override;

private:
    void jointStateHandler(const sensor_msgs::JointState::ConstPtr& msg);
    bool getJointVector(std::array<float, 12>& qOut) const;
    Eigen::Vector3f computeWheelCenterBase(int leg, const std::array<float, 12>& q) const;
    Eigen::Vector3f computeWheelAxisBase(int leg, const std::array<float, 12>& q) const;
    static Eigen::Matrix3f rotX(float a);
    static Eigen::Matrix3f rotY(float a);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subJointState_;
    mutable std::mutex mtxJoint_;
    std::array<float, 12> latestQ_;
    bool hasJointState_ = false;
    float wheelRadius_ = 0.09f;
    std::string topicName_;
};
}

#endif
