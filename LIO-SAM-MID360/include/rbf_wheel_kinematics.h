#pragma once
#ifndef _LIO_SAM_RBF_WHEEL_KINEMATICS_H_
#define _LIO_SAM_RBF_WHEEL_KINEMATICS_H_

#include "rbf_lm_interface.h"
#include <array>
#include <mutex>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace lio_sam_rbf
{
class JointStateWheelKinematics final : public RobotKinematicsModel
{
public:
    JointStateWheelKinematics(const std::string& topicName, float wheelRadius);
    bool computeWheelContacts(
        const std::array<float, 6>& pose,
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& contacts,
        std::vector<Eigen::Matrix<float, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<float, 3, 6>>>& jacobians) override;

private:
    void jointStateHandler(const sensor_msgs::JointState::ConstPtr& msg);
    bool getJointVector(std::array<float, 8>& qOut) const;
    Eigen::Vector3f computeWheelCenterBase(bool left, const std::array<float, 8>& q) const;
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
}

#endif
