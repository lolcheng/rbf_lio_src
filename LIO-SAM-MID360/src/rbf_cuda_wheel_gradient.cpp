#include "rbf_lm_interface.h"
#include "rbf_cuda_wheel_gradient_kernels.cuh"
#include <cmath>

#ifndef LIO_SAM_WITH_CUDA
namespace lio_sam_rbf
{
namespace cuda_kernels
{
bool ComputeWheelResidualJacobianCuda(
    const std::vector<float>& rbf_x,
    const std::vector<float>& rbf_y,
    const std::vector<float>& rbf_w,
    float sigma_x,
    float sigma_y,
    const std::vector<float>& contact_xyz,
    const std::vector<float>& contact_jacobian,
    std::vector<float>& residuals,
    std::vector<float>& jacobians)
{
    residuals.clear();
    jacobians.clear();
    if (rbf_x.empty() || rbf_x.size() != rbf_y.size() || rbf_x.size() != rbf_w.size())
        return false;
    if (contact_xyz.size() % 3 != 0 || contact_jacobian.size() % 18 != 0)
        return false;

    const int wheel_num = static_cast<int>(contact_xyz.size() / 3);
    if (wheel_num == 0 || static_cast<int>(contact_jacobian.size() / 18) != wheel_num)
        return false;
    if (sigma_x <= 1e-6f || sigma_y <= 1e-6f)
        return false;

    const float inv_sigma_x2 = 1.0f / (sigma_x * sigma_x);
    const float inv_sigma_y2 = 1.0f / (sigma_y * sigma_y);
    residuals.resize(wheel_num);
    jacobians.resize(wheel_num * 6);

    for (int i = 0; i < wheel_num; ++i)
    {
        const float px = contact_xyz[i * 3 + 0];
        const float py = contact_xyz[i * 3 + 1];
        const float pz = contact_xyz[i * 3 + 2];

        float h = 0.0f;
        float dhdx = 0.0f;
        float dhdy = 0.0f;
        for (size_t k = 0; k < rbf_x.size(); ++k)
        {
            const float dx = px - rbf_x[k];
            const float dy = py - rbf_y[k];
            const float phi = std::exp(-0.5f * (dx * dx * inv_sigma_x2 + dy * dy * inv_sigma_y2));
            const float wk = rbf_w[k];
            h += wk * phi;
            dhdx += wk * phi * (-dx * inv_sigma_x2);
            dhdy += wk * phi * (-dy * inv_sigma_y2);
        }

        residuals[i] = pz - h;
        const float* J = contact_jacobian.data() + i * 18;
        for (int j = 0; j < 6; ++j)
        {
            const float dpx = J[0 * 6 + j];
            const float dpy = J[1 * 6 + j];
            const float dpz = J[2 * 6 + j];
            jacobians[i * 6 + j] = dpz - dhdx * dpx - dhdy * dpy;
        }
    }
    return true;
}
}
}
#endif

namespace lio_sam_rbf
{
class CudaWheelRbfGradient final : public RbfCudaGradientInterface
{
public:
    explicit CudaWheelRbfGradient(std::unique_ptr<RobotKinematicsModel> kinematics)
        : kinematics_(std::move(kinematics))
    {
    }

    bool computeGradient(const RbfCudaRequest& request, std::vector<RbfLmConstraint>& constraints) override
    {
        constraints.clear();
        if (!kinematics_)
            return false;
        if (request.rbf_nodes.empty())
            return false;

        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> contacts;
        std::vector<Eigen::Matrix<float, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<float, 3, 6>>> jacobians;
        if (!kinematics_->computeWheelContacts(request.pose, contacts, jacobians))
            return false;
        if (contacts.empty() || jacobians.size() != contacts.size())
            return false;

        std::vector<float> rbf_x;
        std::vector<float> rbf_y;
        std::vector<float> rbf_w;
        rbf_x.reserve(request.rbf_nodes.size());
        rbf_y.reserve(request.rbf_nodes.size());
        rbf_w.reserve(request.rbf_nodes.size());
        for (const auto& n : request.rbf_nodes)
        {
            rbf_x.push_back(n.x());
            rbf_y.push_back(n.y());
            rbf_w.push_back(n.z());
        }

        std::vector<float> contact_xyz;
        std::vector<float> contact_jacobian;
        contact_xyz.reserve(contacts.size() * 3);
        contact_jacobian.reserve(contacts.size() * 18);

        for (size_t i = 0; i < contacts.size(); ++i)
        {
            contact_xyz.push_back(contacts[i].x());
            contact_xyz.push_back(contacts[i].y());
            contact_xyz.push_back(contacts[i].z());
            for (int r = 0; r < 3; ++r)
            {
                for (int c = 0; c < 6; ++c)
                    contact_jacobian.push_back(jacobians[i](r, c));
            }
        }

        std::vector<float> residuals;
        std::vector<float> grads;
        bool ok = cuda_kernels::ComputeWheelResidualJacobianCuda(
            rbf_x,
            rbf_y,
            rbf_w,
            request.sigma_x,
            request.sigma_y,
            contact_xyz,
            contact_jacobian,
            residuals,
            grads);

        if (!ok)
            return false;
        if (residuals.size() != contacts.size() || grads.size() != contacts.size() * 6)
            return false;

        constraints.resize(contacts.size());
        for (size_t i = 0; i < contacts.size(); ++i)
        {
            constraints[i].residual = residuals[i];
            constraints[i].weight = 1.0f;
            for (int j = 0; j < 6; ++j)
                constraints[i].jacobian[j] = grads[i * 6 + j];
        }
        return true;
    }

private:
    std::unique_ptr<RobotKinematicsModel> kinematics_;
};

std::unique_ptr<RbfCudaGradientInterface> CreateCudaWheelRbfGradient(std::unique_ptr<RobotKinematicsModel> kinematics)
{
    return std::unique_ptr<RbfCudaGradientInterface>(new CudaWheelRbfGradient(std::move(kinematics)));
}
}
