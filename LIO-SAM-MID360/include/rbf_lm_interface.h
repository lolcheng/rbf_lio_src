#pragma once
#ifndef _LIO_SAM_RBF_LM_INTERFACE_H_
#define _LIO_SAM_RBF_LM_INTERFACE_H_

#include <array>
#include <memory>
#include <vector>
#include <Eigen/Core>

namespace lio_sam_rbf
{
struct WheelGeometry
{
    Eigen::Vector3f center_world = Eigen::Vector3f::Zero();
    Eigen::Vector3f axis_world = Eigen::Vector3f::UnitY();
};

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
    float wheel_radius = 0.127f;
    float weight_r1 = 1.0f;
    float weight_r2 = 0.3f;
    float weight_r3 = 0.2f;
    float weight_r4 = 0.2f;
    float max_abs_r1 = 0.25f;
    float max_abs_r2 = 0.10f;
    float max_abs_r3 = 0.08f;
    float max_norm_r4 = 0.80f;
};

class RobotKinematicsModel
{
public:
    virtual ~RobotKinematicsModel() = default;
    virtual bool computeWheelGeometry(
        const std::array<float, 6>& pose,
        std::vector<WheelGeometry, Eigen::aligned_allocator<WheelGeometry>>& wheels) = 0;
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
