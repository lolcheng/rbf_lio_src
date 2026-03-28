#include "rbf_wheel_kinematics.h"
#include <cmath>
#include <algorithm>

namespace lio_sam_rbf
{
JointStateWheelKinematics::JointStateWheelKinematics(const std::string& topicName, float wheelRadius)
{
    latestQ_.fill(0.0f);
    wheelRadius_ = wheelRadius > 0.0f ? wheelRadius : 0.127f;
    subJointState_ = nh_.subscribe<sensor_msgs::JointState>(
        topicName, 200, &JointStateWheelKinematics::jointStateHandler, this, ros::TransportHints().tcpNoDelay());
    ROS_INFO_STREAM("[lio_sam][rbf] wheel kinematics use joint topic: " << topicName
                    << ", wheelRadius=" << wheelRadius_);
}

void JointStateWheelKinematics::jointStateHandler(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (!msg || msg->position.size() < 8)
        return;

    std::array<float, 8> q{};
    q.fill(0.0f);

    auto assignFromNamedJoint = [&](const std::string& jointName, int fallbackIdx, int outIdx) {
        int idx = -1;
        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == jointName)
            {
                idx = static_cast<int>(i);
                break;
            }
        }
        if (idx < 0)
        {
            // Compatible with bags that use joint_0 ... joint_7
            const std::string genericName = "joint_" + std::to_string(fallbackIdx);
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                if (msg->name[i] == genericName)
                {
                    idx = static_cast<int>(i);
                    break;
                }
            }
        }
        if (idx < 0 || idx >= static_cast<int>(msg->position.size()))
            idx = fallbackIdx;
        if (idx >= 0 && idx < static_cast<int>(msg->position.size()))
            q[outIdx] = static_cast<float>(msg->position[idx]);
    };

    // Motor order provided by user:
    // 0: abad_L, 1: hip_L, 2: knee_L, 3: wheel_L, 4: abad_R, 5: hip_R, 6: knee_R, 7: wheel_R
    assignFromNamedJoint("abad_L_Joint", 0, 0);
    assignFromNamedJoint("hip_L_Joint", 1, 1);
    assignFromNamedJoint("knee_L_Joint", 2, 2);
    assignFromNamedJoint("wheel_L_Joint", 3, 3);
    assignFromNamedJoint("abad_R_Joint", 4, 4);
    assignFromNamedJoint("hip_R_Joint", 5, 5);
    assignFromNamedJoint("knee_R_Joint", 6, 6);
    assignFromNamedJoint("wheel_R_Joint", 7, 7);

    {
        std::lock_guard<std::mutex> lock(mtxJoint_);
        latestQ_ = q;
        hasJointState_ = true;
    }
}

bool JointStateWheelKinematics::getJointVector(std::array<float, 8>& qOut) const
{
    std::lock_guard<std::mutex> lock(mtxJoint_);
    if (!hasJointState_)
        return false;
    qOut = latestQ_;
    return true;
}

Eigen::Matrix3f JointStateWheelKinematics::rotX(float a)
{
    const float c = std::cos(a);
    const float s = std::sin(a);
    Eigen::Matrix3f R;
    R << 1.0f, 0.0f, 0.0f,
         0.0f, c, -s,
         0.0f, s, c;
    return R;
}

Eigen::Matrix3f JointStateWheelKinematics::rotY(float a)
{
    const float c = std::cos(a);
    const float s = std::sin(a);
    Eigen::Matrix3f R;
    R << c, 0.0f, s,
         0.0f, 1.0f, 0.0f,
         -s, 0.0f, c;
    return R;
}

Eigen::Vector3f JointStateWheelKinematics::computeWheelCenterBase(bool left, const std::array<float, 8>& q) const
{
    // Kinematic chain from WF_TRON1A URDF.
    // Left: q0/q1/q2, Right: q4/q5/q6.
    const int idxAbad = left ? 0 : 4;
    const int idxHip = left ? 1 : 5;
    const int idxKnee = left ? 2 : 6;

    const float qAbad = q[idxAbad];
    const float qHip = q[idxHip];
    const float qKnee = q[idxKnee];

    const Eigen::Vector3f o1(0.05556f, left ? 0.105f : -0.105f, -0.2602f);
    const Eigen::Vector3f o2(-0.077f, left ? 0.0205f : -0.0205f, 0.0f);
    const Eigen::Vector3f o3(-0.1500f, left ? -0.0205f : 0.0205f, -0.25981f);
    const Eigen::Vector3f o4(0.1500f, left ? 0.0435f : -0.0435f, -0.25981f);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f p = Eigen::Vector3f::Zero();

    // base -> abad joint
    p += R * o1;
    R = R * rotX(qAbad);

    // abad -> hip joint
    p += R * o2;
    // URDF axis: left hip [0 1 0], right hip [0 -1 0]
    const float hipSign = left ? 1.0f : -1.0f;
    R = R * rotY(hipSign * qHip);

    // hip -> knee joint
    p += R * o3;
    // URDF axis: left knee [0 -1 0], right knee [0 1 0]
    const float kneeSign = left ? -1.0f : 1.0f;
    R = R * rotY(kneeSign * qKnee);

    // knee -> wheel center
    p += R * o4;
    return p;
}

bool JointStateWheelKinematics::computeWheelContacts(
    const std::array<float, 6>& pose,
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& contacts,
    std::vector<Eigen::Matrix<float, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<float, 3, 6>>>& jacobians)
{
    contacts.clear();
    jacobians.clear();

    std::array<float, 8> q{};
    if (!getJointVector(q))
    {
        ROS_WARN_THROTTLE(2.0, "[lio_sam][rbf] waiting /joint_states for real wheel kinematics.");
        return false;
    }

    const float r = pose[0];
    const float p = pose[1];
    const float y = pose[2];
    const float tx = pose[3];
    const float ty = pose[4];
    const float tz = pose[5];

    const float cr = std::cos(r);
    const float sr = std::sin(r);
    const float cp = std::cos(p);
    const float sp = std::sin(p);
    const float cy = std::cos(y);
    const float sy = std::sin(y);

    Eigen::Matrix3f Rx;
    Rx << 1.0f, 0.0f, 0.0f,
          0.0f, cr, -sr,
          0.0f, sr, cr;

    Eigen::Matrix3f Ry;
    Ry << cp, 0.0f, sp,
          0.0f, 1.0f, 0.0f,
          -sp, 0.0f, cp;

    Eigen::Matrix3f Rz;
    Rz << cy, -sy, 0.0f,
          sy, cy, 0.0f,
          0.0f, 0.0f, 1.0f;

    Eigen::Matrix3f R = Rz * Ry * Rx;

    Eigen::Matrix3f dRx;
    dRx << 0.0f, 0.0f, 0.0f,
           0.0f, -sr, -cr,
           0.0f, cr, -sr;

    Eigen::Matrix3f dRy;
    dRy << -sp, 0.0f, cp,
           0.0f, 0.0f, 0.0f,
           -cp, 0.0f, -sp;

    Eigen::Matrix3f dRz;
    dRz << -sy, -cy, 0.0f,
            cy, -sy, 0.0f,
            0.0f, 0.0f, 0.0f;

    Eigen::Matrix3f dR_dr = Rz * Ry * dRx;
    Eigen::Matrix3f dR_dp = Rz * dRy * Rx;
    Eigen::Matrix3f dR_dy = dRz * Ry * Rx;
    Eigen::Vector3f t(tx, ty, tz);

    const Eigen::Vector3f centerL = computeWheelCenterBase(true, q);
    const Eigen::Vector3f centerR = computeWheelCenterBase(false, q);
    std::array<Eigen::Vector3f, 2> centers = {centerL, centerR};

    contacts.reserve(centers.size());
    jacobians.reserve(centers.size());

    for (const auto& pb : centers)
    {
        Eigen::Vector3f c = t + R * pb;
        // Approximate contact point directly below wheel center in world frame.
        c.z() -= wheelRadius_;
        contacts.push_back(c);

        Eigen::Matrix<float, 3, 6> J;
        J.col(0) = dR_dr * pb;
        J.col(1) = dR_dp * pb;
        J.col(2) = dR_dy * pb;
        J.col(3) = Eigen::Vector3f::UnitX();
        J.col(4) = Eigen::Vector3f::UnitY();
        J.col(5) = Eigen::Vector3f::UnitZ();
        jacobians.push_back(J);
    }
    return true;
}
}
