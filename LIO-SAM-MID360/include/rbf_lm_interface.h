#pragma once
#ifndef _LIO_SAM_RBF_LM_INTERFACE_H_
#define _LIO_SAM_RBF_LM_INTERFACE_H_

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Core>

namespace lio_sam_rbf
{
struct RbfLmConstraint
{
    std::array<float, 6> jacobian;
    float residual = 0.0f;
    float weight = 1.0f;
};

struct RbfCudaRequest
{
    std::array<float, 6> pose;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points_lidar;
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> rbf_nodes;
    float sigma_x = 0.3f;
    float sigma_y = 0.3f;
};

class RobotKinematicsModel
{
public:
    virtual ~RobotKinematicsModel() = default;
    virtual bool computeWheelContacts(
        const std::array<float, 6>& pose,
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& contacts,
        std::vector<Eigen::Matrix<float, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<float, 3, 6>>>& jacobians) = 0;
};

class RbfCudaGradientInterface
{
public:
    virtual ~RbfCudaGradientInterface() = default;
    virtual bool computeGradient(const RbfCudaRequest& request, std::vector<RbfLmConstraint>& constraints) = 0;
};

std::unique_ptr<RbfCudaGradientInterface> CreateCudaWheelRbfGradient(std::unique_ptr<RobotKinematicsModel> kinematics);

class PlaceholderRbfCudaGradient final : public RbfCudaGradientInterface
{
public:
    bool computeGradient(const RbfCudaRequest&, std::vector<RbfLmConstraint>& constraints) override
    {
        constraints.clear();
        return false;
    }
};
}

#endif
