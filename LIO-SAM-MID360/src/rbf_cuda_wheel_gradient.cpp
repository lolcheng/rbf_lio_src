#include "rbf_lm_interface.h"
#include <cmath>
#include <Eigen/Geometry>

namespace lio_sam_rbf
{
namespace
{
struct ContactResidualEval
{
    std::vector<float> residuals;
    bool valid = false;
};

inline void evaluateTerrain(const RbfCudaRequest& request, float x, float y, float& h, float& hx, float& hy)
{
    h = 0.0f;
    hx = 0.0f;
    hy = 0.0f;
    const float inv_sigma_x2 = 1.0f / std::max(1e-6f, request.sigma_x * request.sigma_x);
    const float inv_sigma_y2 = 1.0f / std::max(1e-6f, request.sigma_y * request.sigma_y);
    for (const auto& n : request.rbf_nodes)
    {
        const float dx = x - n.x();
        const float dy = y - n.y();
        const float phi = std::exp(-0.5f * (dx * dx * inv_sigma_x2 + dy * dy * inv_sigma_y2));
        h += n.z() * phi;
        hx += n.z() * phi * (-dx * inv_sigma_x2);
        hy += n.z() * phi * (-dy * inv_sigma_y2);
    }
}

inline void makeWheelPlaneBasis(const Eigen::Vector3f& axis, Eigen::Vector3f& e1, Eigen::Vector3f& e2)
{
    Eigen::Vector3f ref = std::abs(axis.z()) < 0.9f ? Eigen::Vector3f::UnitZ() : Eigen::Vector3f::UnitX();
    e1 = axis.cross(ref);
    if (e1.norm() < 1e-6f)
        e1 = axis.cross(Eigen::Vector3f::UnitY());
    e1.normalize();
    e2 = axis.cross(e1).normalized();
}

inline bool solveContactOnWheel(
    const RbfCudaRequest& request,
    const WheelGeometry& wheel,
    float rho,
    Eigen::Vector3f& p,
    Eigen::Vector3f& nGround,
    Eigen::Vector3f& nWheel,
    float& rHeight)
{
    if (rho <= 1e-6f)
        return false;

    // Initialize from "closest-down" point on wheel circle in the wheel plane.
    Eigen::Vector3f e1, e2;
    makeWheelPlaneBasis(wheel.axis_world, e1, e2);
    Eigen::Vector3f downProj = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    downProj -= downProj.dot(wheel.axis_world) * wheel.axis_world;
    if (downProj.norm() < 1e-6f)
        downProj = -e2;
    downProj.normalize();
    p = wheel.center_world + rho * downProj;

    // Local GN solve for explicit contact constraints:
    // r1 = pz - h(px,py)
    // r2 = a^T (p - c)
    // r3 = ||p-c||^2 - rho^2
    for (int iter = 0; iter < 8; ++iter)
    {
        float h = 0.0f, hx = 0.0f, hy = 0.0f;
        evaluateTerrain(request, p.x(), p.y(), h, hx, hy);
        const Eigen::Vector3f d = p - wheel.center_world;

        Eigen::Matrix<float, 3, 1> r;
        r(0) = p.z() - h;
        r(1) = wheel.axis_world.dot(d);
        r(2) = d.dot(d) - rho * rho;

        Eigen::Matrix3f J;
        J.row(0) << -hx, -hy, 1.0f;
        J.row(1) = wheel.axis_world.transpose();
        J.row(2) = (2.0f * d).transpose();

        Eigen::Matrix3f H = J.transpose() * J;
        H += 1e-6f * Eigen::Matrix3f::Identity();
        Eigen::Vector3f g = J.transpose() * r;
        Eigen::Vector3f dp = -H.ldlt().solve(g);
        if (!dp.allFinite())
            break;
        p += dp;
        if (dp.norm() < 1e-5f)
            break;
    }

    float h = 0.0f, hx = 0.0f, hy = 0.0f;
    evaluateTerrain(request, p.x(), p.y(), h, hx, hy);
    rHeight = p.z() - h;
    nGround = Eigen::Vector3f(-hx, -hy, 1.0f);
    if (nGround.norm() > 1e-6f)
        nGround.normalize();
    else
        nGround = Eigen::Vector3f::UnitZ();

    nWheel = (p - wheel.center_world) / rho;
    if (nWheel.norm() > 1e-6f)
        nWheel.normalize();
    else
        nWheel = -nGround;
    return true;
}

inline ContactResidualEval evaluateResidualVector(
    RobotKinematicsModel& kinematics,
    const RbfCudaRequest& request,
    const std::array<float, 6>& pose)
{
    ContactResidualEval out;
    std::vector<WheelGeometry, Eigen::aligned_allocator<WheelGeometry>> wheels;
    if (!kinematics.computeWheelGeometry(pose, wheels) || wheels.empty())
        return out;

    out.residuals.reserve(wheels.size() * 6);
    for (const auto& wheel : wheels)
    {
        Eigen::Vector3f p, nGround, nWheel;
        float rHeight = 0.0f;
        if (!solveContactOnWheel(request, wheel, request.wheel_radius, p, nGround, nWheel, rHeight))
        {
            // Keep fixed residual dimensions for numerical Jacobian.
            // If local contact solve fails, provide a neutral block.
            out.residuals.push_back(0.0f);
            out.residuals.push_back(0.0f);
            out.residuals.push_back(0.0f);
            out.residuals.push_back(0.0f);
            out.residuals.push_back(0.0f);
            out.residuals.push_back(0.0f);
            continue;
        }

        auto clampAbs = [](float v, float lim) -> float {
            const float l = std::max(1e-6f, lim);
            return std::max(-l, std::min(l, v));
        };

        float r1 = clampAbs(rHeight, request.max_abs_r1);
        float r2 = clampAbs(wheel.axis_world.dot(p - wheel.center_world), request.max_abs_r2);
        float r3 = clampAbs((p - wheel.center_world).squaredNorm() - request.wheel_radius * request.wheel_radius,
                            request.max_abs_r3);
        // Use sign-invariant normal consistency: n_w x n_g = 0 (parallel or anti-parallel).
        Eigen::Vector3f rNormal = nWheel.cross(nGround);
        const float r4Norm = rNormal.norm();
        const float r4Lim = std::max(1e-6f, request.max_norm_r4);
        if (r4Norm > r4Lim)
            rNormal *= (r4Lim / r4Norm);

        // Explicit contact factor rows:
        // r1: point on terrain
        out.residuals.push_back(r1);
        // r2: point on wheel plane
        out.residuals.push_back(r2);
        // r3: distance-to-center equals wheel radius
        out.residuals.push_back(r3);
        // r4: normal consistency, 3 residual rows
        out.residuals.push_back(rNormal.x());
        out.residuals.push_back(rNormal.y());
        out.residuals.push_back(rNormal.z());
    }
    out.valid = !out.residuals.empty();
    return out;
}
} // namespace

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
        if (!kinematics_ || request.rbf_nodes.empty())
            return false;

        ContactResidualEval base = evaluateResidualVector(*kinematics_, request, request.pose);
        if (!base.valid || base.residuals.empty())
            return false;

        const int rows = static_cast<int>(base.residuals.size());
        std::vector<float> jac(static_cast<size_t>(rows) * 6, 0.0f);

        for (int j = 0; j < 6; ++j)
        {
            std::array<float, 6> posePlus = request.pose;
            std::array<float, 6> poseMinus = request.pose;
            const float eps = (j < 3) ? 1e-4f : 1e-3f;
            posePlus[j] += eps;
            poseMinus[j] -= eps;

            ContactResidualEval plus = evaluateResidualVector(*kinematics_, request, posePlus);
            ContactResidualEval minus = evaluateResidualVector(*kinematics_, request, poseMinus);
            if (!plus.valid || !minus.valid || plus.residuals.size() != base.residuals.size() || minus.residuals.size() != base.residuals.size())
                return false;

            for (int r = 0; r < rows; ++r)
                jac[r * 6 + j] = (plus.residuals[r] - minus.residuals[r]) / (2.0f * eps);
        }

        constraints.resize(rows);
        for (int r = 0; r < rows; ++r)
        {
            constraints[r].residual = base.residuals[r];
            // For each wheel residual block: [r1, r2, r3, r4x, r4y, r4z]
            const int local = r % 6;
            if (local == 0)
                constraints[r].weight = request.weight_r1;   // r1
            else if (local == 1)
                constraints[r].weight = request.weight_r2;   // r2
            else if (local == 2)
                constraints[r].weight = request.weight_r3;   // r3
            else
                constraints[r].weight = request.weight_r4;   // r4
            for (int j = 0; j < 6; ++j)
                constraints[r].jacobian[j] = jac[r * 6 + j];
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
