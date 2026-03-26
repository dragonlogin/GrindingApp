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

double NormalizeDeg(double deg)
{
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

double JointDistance(const nl::utils::Q& a, const nl::utils::Q& b)
{
    double sum = 0.0;
    for (int i = 0; i < 6; ++i) {
        double d = NormalizeDeg(a[i] - b[i]);
        sum += d * d;
    }
    return sum;
}

// Craig DH rotation matrix (3×3 only)
Eigen::Matrix3d DhRotation(double theta_rad, double alpha_deg)
{
    double ct = cos(theta_rad), st = sin(theta_rad);
    double ca = cos(alpha_deg * kDeg), sa = sin(alpha_deg * kDeg);
    Eigen::Matrix3d R;
    R << ct, -st * ca,  st * sa,
         st,  ct * ca, -ct * sa,
         0.0,      sa,       ca;
    return R;
}

bool IsSphericalWrist(const nl::core::RbRobot& robot)
{
    // 球型腕条件：a4=0, a5=0, d5=0
    if (robot.joints.size() != 6) return false;
    return (fabs(robot.joints[3].a) < 1e-6 &&
            fabs(robot.joints[4].a) < 1e-6 &&
            fabs(robot.joints[4].d) < 1e-6);
}

bool JacobianIk(const nl::core::RbRobot& robot, const gp_Trsf& target,
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

//解析解
/*

IRB140 球型腕解耦法：

1. 腕心位置 = TCP 位置 - d6 × z轴方向
2. θ1 从腕心 XY 平面求解（2 解）
3. θ3 从余弦定理（2 解）
4. θ2 从三角方程组直接求解
5. θ4/θ5/θ6 从 R_36 = R_03^T × R_06 提取（2 解）
6. 最多 8 组解，选与初始角最近的

步骤	公式
腕心	p_wc = p_06 - d6 * R_06[:,2]
θ1	atan2(wy, wx), 第二解 +180°
θ3_act	sin(θ3) = (D² - a2² - d4²) / (2·a2·d4), cos(θ3) = ±√(1-sin²)
θ2_act	A = a2 + d4·sin θ3, B = d4·cos θ3, θ2 = atan2((An+Bm)/(A²+B²), (Am-Bn)/(A²+B²))
θ5	atan2(±√(r13²+r23²), r33)
θ4	atan2(±r23, ±r13) (符号同 θ5)
θ6	atan2(±r32, ∓r31) (符号同 θ5)
jog角	θ_jog = θ_actual - offset

*/
bool AnalyticalIk(const nl::core::RbRobot& robot, const gp_Trsf& target,
                  const nl::utils::Q& init_angles, nl::utils::Q& out_angles)
{
    int n = static_cast<int>(robot.joints.size());
    if (n != 6) return false;

    // DH parameters
    double a1 = robot.joints[0].a;    // 70
    double d1 = robot.joints[0].d;    // 352
    double a2 = robot.joints[1].a;    // 360
    double d4 = robot.joints[3].d;    // 380
    double d6 = robot.joints[5].d;    // 65

    Eigen::Matrix4d T_target = GpTrsfToEigen(target);
    Eigen::Vector3d p_06 = T_target.block<3,1>(0, 3);
    Eigen::Matrix3d R_06 = T_target.block<3,3>(0, 0);

    // --- Step 1: Wrist center ---
    Eigen::Vector3d p_wc = p_06 - d6 * R_06.col(2);
    double wx = p_wc(0), wy = p_wc(1), wz = p_wc(2);

    // --- Step 2: θ1 candidates (2 solutions) ---
    double theta1_deg[2];
    theta1_deg[0] = atan2(wy, wx) / kDeg;
    theta1_deg[1] = NormalizeDeg(theta1_deg[0] + 180.0);

    struct Solution { nl::utils::Q q; };
    std::vector<Solution> solutions;

    for (int i1 = 0; i1 < 2; ++i1) {
        double t1 = theta1_deg[i1] * kDeg;
        double c1 = cos(t1), s1 = sin(t1);

        // Wrist center in arm plane
        double r_wc = c1 * wx + s1 * wy;
        double m = r_wc - a1;
        double nv = d1 - wz;
        double D2 = m * m + nv * nv;

        // --- Step 3: θ3_act (2 solutions: elbow up/down) ---
        double sin_t3 = (D2 - a2 * a2 - d4 * d4) / (2.0 * a2 * d4);
        if (sin_t3 < -1.0 - 1e-9 || sin_t3 > 1.0 + 1e-9) continue;
        sin_t3 = std::clamp(sin_t3, -1.0, 1.0);

        double cos_t3_abs = sqrt(1.0 - sin_t3 * sin_t3);
        double theta3_act_rad[2] = {
            atan2(sin_t3,  cos_t3_abs),
            atan2(sin_t3, -cos_t3_abs)
        };

        for (int i3 = 0; i3 < 2; ++i3) {
            double t3 = theta3_act_rad[i3];
            double ct3 = cos(t3), st3 = sin(t3);

            // --- Step 4: θ2_act ---
            double A = a2 + d4 * st3;
            double B = d4 * ct3;
            double denom = A * A + B * B;
            if (denom < 1e-10) continue;

            double ct2 = (A * m - B * nv) / denom;
            double st2 = (A * nv + B * m) / denom;
            double t2 = atan2(st2, ct2);

            // --- Step 5: Compute R_03 from DH ---
            Eigen::Matrix3d R_01 = DhRotation(t1, robot.joints[0].alpha_deg);
            Eigen::Matrix3d R_12 = DhRotation(t2, robot.joints[1].alpha_deg);
            Eigen::Matrix3d R_23 = DhRotation(t3, robot.joints[2].alpha_deg);
            Eigen::Matrix3d R_03 = R_01 * R_12 * R_23;

            // --- Step 6: R_36 = R_03^T * R_06 ---
            Eigen::Matrix3d R_36 = R_03.transpose() * R_06;

            // Extract θ4, θ5, θ6 from R_36:
            // R_36[2][2] = cos(θ5)
            // R_36[0][2] = cos(θ4)*sin(θ5)
            // R_36[1][2] = sin(θ4)*sin(θ5)
            // R_36[2][0] = -sin(θ5)*cos(θ6)
            // R_36[2][1] =  sin(θ5)*sin(θ6)
            double r13 = R_36(0, 2);
            double r23 = R_36(1, 2);
            double r33 = R_36(2, 2);
            double r31 = R_36(2, 0);
            double r32 = R_36(2, 1);

            // θ5 has 2 solutions (±sin)
            for (int i5 = 0; i5 < 2; ++i5) {
                double sign5 = (i5 == 0) ? 1.0 : -1.0;
                double s5 = sign5 * sqrt(r13 * r13 + r23 * r23);
                double t5 = atan2(s5, r33);

                double t4, t6;
                if (fabs(s5) > 1e-6) {
                    t4 = atan2(sign5 * r23, sign5 * r13);
                    t6 = atan2(sign5 * r32, -sign5 * r31);
                } else {
                    // Wrist singularity: keep θ4 from init
                    t4 = (init_angles[3] + robot.joints[3].offset_deg) * kDeg;
                    if (r33 > 0) {
                        double sum46 = atan2(R_36(1, 0), R_36(0, 0));
                        t6 = sum46 - t4;
                    } else {
                        double diff46 = atan2(-R_36(1, 0), R_36(0, 0));
                        t6 = t4 - diff46;
                    }
                }

                // Convert DH actual angles to jog angles
                nl::utils::Q sol({
                    theta1_deg[i1] - robot.joints[0].offset_deg,
                    t2 / kDeg      - robot.joints[1].offset_deg,
                    t3 / kDeg      - robot.joints[2].offset_deg,
                    t4 / kDeg      - robot.joints[3].offset_deg,
                    t5 / kDeg      - robot.joints[4].offset_deg,
                    t6 / kDeg      - robot.joints[5].offset_deg
                });

                for (int j = 0; j < 6; ++j)
                    sol[j] = NormalizeDeg(sol[j]);

                solutions.push_back({sol});
            }
        }
    }

    if (solutions.empty()) {
        out_angles = init_angles;
        return false;
    }

    // Pick closest to init
    double best_dist = 1e30;
    int best_idx = 0;
    for (size_t i = 0; i < solutions.size(); ++i) {
        double d = JointDistance(solutions[i].q, init_angles);
        if (d < best_dist) {
            best_dist = d;
            best_idx = static_cast<int>(i);
        }
    }

    out_angles = solutions[best_idx].q;
    return true;
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
    if (IsSphericalWrist(robot)) {
        if (AnalyticalIk(robot, target, init_angles, out_angles))
            return true;
    }
    // Fallback
    return JacobianIk(robot, target, init_angles, out_angles);
}

} // namespace kinematics
} // namespace nl
