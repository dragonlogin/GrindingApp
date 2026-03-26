#include "RobotKinematics.h"

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/SVD>

#include "Q.h"

namespace nl {
namespace kinematics {

namespace {

constexpr double kDeg = M_PI / 180.0;

Eigen::Matrix4d DhMatrix(double theta_deg, double d, double a, double alpha_deg)
{
    double t = theta_deg * kDeg;
    double al = alpha_deg * kDeg;
    double ct = cos(t), st = sin(t);
    double ca = cos(al), sa = sin(al);

    Eigen::Matrix4d mat;
    mat <<
        ct,       -st,      0.0,   a,
        st * ca,   ct * ca, -sa,  -d * sa,
        st * sa,   ct * sa,  ca,   d * ca,
        0.0,       0.0,      0.0,  1.0;
    return mat;
}

gp_Trsf EigenToGpTrsf(const Eigen::Matrix4d& m)
{
    gp_Trsf t;
    t.SetValues(
        m(0,0), m(0,1), m(0,2), m(0,3),
        m(1,0), m(1,1), m(1,2), m(1,3),
        m(2,0), m(2,1), m(2,2), m(2,3)
    );
    return t;
}

Eigen::Matrix4d GpTrsfToEigen(const gp_Trsf& trsf)
{
    const gp_Mat& rot = trsf.VectorialPart();
    const gp_XYZ& pos = trsf.TranslationPart();
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    for (int r = 1; r <= 3; ++r)
        for (int c = 1; c <= 3; ++c)
            m(r-1, c-1) = rot.Value(r, c);
    m(0,3) = pos.X();
    m(1,3) = pos.Y();
    m(2,3) = pos.Z();
    return m;
}

// 累积各关节变换（q_full_rad 含 offset），返回 n+1 帧：frames[0]=I
std::vector<Eigen::Matrix4d> FkFrames(const nl::core::RbRobot& robot,
                                       const Eigen::VectorXd& q_full_rad)
{
    int n = static_cast<int>(robot.joints.size());
    std::vector<Eigen::Matrix4d> frames(n + 1);
    frames[0] = Eigen::Matrix4d::Identity();
    for (int i = 0; i < n; ++i) {
        double theta_deg = q_full_rad(i) / kDeg;
        frames[i+1] = frames[i] * DhMatrix(theta_deg, robot.joints[i].d,
                                            robot.joints[i].a, robot.joints[i].alpha_deg);
    }
    return frames;
}

// 几何 Jacobian (6×n)
Eigen::MatrixXd Jacobian(const std::vector<Eigen::Matrix4d>& frames)
{
    int n = static_cast<int>(frames.size()) - 1;
    Eigen::Vector3d p_e = frames[n].block<3,1>(0, 3);
    Eigen::MatrixXd J(6, n);
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d z_i = frames[i].block<3,1>(0, 2);
        Eigen::Vector3d p_i = frames[i].block<3,1>(0, 3);
        J.block<3,1>(0, i) = z_i.cross(p_e - p_i);
        J.block<3,1>(3, i) = z_i;
    }
    return J;
}

Eigen::Vector3d RotError(const Eigen::Matrix3d& R_cur, const Eigen::Matrix3d& R_tgt)
{
    Eigen::Matrix3d R_e = R_tgt * R_cur.transpose();
    return 0.5 * Eigen::Vector3d(R_e(2,1) - R_e(1,2),
                                  R_e(0,2) - R_e(2,0),
                                  R_e(1,0) - R_e(0,1));
}

} // namespace

// ---------------------------------------------------------------------------

std::vector<gp_Trsf> ComputeFk(const nl::core::RbRobot& robot, const nl::utils::Q& joint_angles)
{
    int n = static_cast<int>(robot.joints.size());
    Eigen::VectorXd q(n);
    for (int i = 0; i < n; ++i)
        q(i) = (robot.joints[i].offset_deg + (i < 6 ? joint_angles[i] : 0.0)) * kDeg;

    std::vector<Eigen::Matrix4d> frames = FkFrames(robot, q);
    std::vector<gp_Trsf> result(n);
    for (int i = 0; i < n; ++i)
        result[i] = EigenToGpTrsf(frames[i+1]);
    return result;
}

bool ComputeIk(const nl::core::RbRobot& robot, const gp_Trsf& target,
               const nl::utils::Q& init_angles, nl::utils::Q& out_angles)
{
    constexpr double kPosTol   = 0.01;
    constexpr double kRotTol   = 1e-4;
    constexpr int    kMaxIter  = 300;
    constexpr double kMaxDqRad = 5.0 * kDeg;
    constexpr double kPosScale = 1e-3;
    constexpr double kSvThresh = 1e-3;

    int n = static_cast<int>(robot.joints.size());

    Eigen::VectorXd q(n);
    for (int i = 0; i < n; ++i)
        q(i) = (robot.joints[i].offset_deg + (i < 6 ? init_angles[i] : 0.0)) * kDeg;

    Eigen::Matrix4d T_tgt = GpTrsfToEigen(target);
    Eigen::Vector3d p_tgt = T_tgt.block<3,1>(0, 3);
    Eigen::Matrix3d R_tgt = T_tgt.block<3,3>(0, 0);

    for (int iter = 0; iter < kMaxIter; ++iter) {
        std::vector<Eigen::Matrix4d> frames = FkFrames(robot, q);

        Eigen::Vector3d p_cur = frames[n].block<3,1>(0, 3);
        Eigen::Matrix3d R_cur = frames[n].block<3,3>(0, 0);

        Eigen::Vector3d dp = p_tgt - p_cur;
        Eigen::Vector3d dr = RotError(R_cur, R_tgt);

        if (dp.norm() < kPosTol && dr.norm() < kRotTol) {
            for (int i = 0; i < 6; ++i)
                out_angles[i] = q(i) / kDeg - robot.joints[i].offset_deg;
            return true;
        }

        Eigen::VectorXd dx(6);
        dx << dp * kPosScale, dr;

        Eigen::MatrixXd J = Jacobian(frames);
        J.topRows(3) *= kPosScale;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J,
            Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::VectorXd& sv = svd.singularValues();
        double sv_max = sv(0);
        Eigen::VectorXd sv_inv(sv.size());
        for (int i = 0; i < sv.size(); ++i)
            sv_inv(i) = (sv(i) > kSvThresh * sv_max) ? 1.0 / sv(i) : 0.0;

        Eigen::VectorXd dq = svd.matrixV() * sv_inv.asDiagonal()
                           * svd.matrixU().transpose() * dx;

        double dq_max = dq.cwiseAbs().maxCoeff();
        if (dq_max > kMaxDqRad)
            dq *= kMaxDqRad / dq_max;

        q += dq;
    }

    for (int i = 0; i < 6; ++i)
        out_angles[i] = q(i) / kDeg - robot.joints[i].offset_deg;
    return false;
}

} // namespace kinematics
} // namespace nl
