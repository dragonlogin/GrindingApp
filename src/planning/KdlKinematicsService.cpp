#include "KdlKinematicsService.h"

#include <algorithm>
#include <cmath>

#include <gp_Trsf.hxx>

#include "kinematics/KdlSolver.h"
#include "Q.h"
#include "foundation/Conversions.h"

// [Pimpl 精神] KDL/OCCT 类型及内部 helper 全部在匿名 namespace，对外不可见
namespace {

double NormalizeDeg(double deg)
{
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

double JointDistance(const nl::utils::Q& a, const nl::utils::Q& b)
{
    double sum = 0.0;
    int n = std::min(a.size(), b.size());
    for (int i = 0; i < n; ++i) {
        double d = NormalizeDeg(a[i] - b[i]);
        sum += d * d;
    }
    return sum;
}

void NormalizeJointAngles(nl::utils::Q& angles)
{
    for (int i = 0; i < angles.size(); ++i)
        angles[i] = NormalizeDeg(angles[i]);
}

bool PoseNear(const gp_Trsf& a, const gp_Trsf& b,
              double pos_tol_mm = 0.1, double rot_tol = 1e-4)
{
    const gp_XYZ da = a.TranslationPart() - b.TranslationPart();
    if (std::sqrt(da.X()*da.X() + da.Y()*da.Y() + da.Z()*da.Z()) > pos_tol_mm)
        return false;

    const gp_Mat& Ra = a.VectorialPart();
    const gp_Mat& Rb = b.VectorialPart();
    double r12 = Ra.Value(1,1)*Rb.Value(1,2) + Ra.Value(2,1)*Rb.Value(2,2) + Ra.Value(3,1)*Rb.Value(3,2)
               - Ra.Value(1,2)*Rb.Value(1,1) - Ra.Value(2,2)*Rb.Value(2,1) - Ra.Value(3,2)*Rb.Value(3,1);
    double r20 = Ra.Value(1,2)*Rb.Value(1,3) + Ra.Value(2,2)*Rb.Value(2,3) + Ra.Value(3,2)*Rb.Value(3,3)
               - Ra.Value(1,3)*Rb.Value(1,2) - Ra.Value(2,3)*Rb.Value(2,2) - Ra.Value(3,3)*Rb.Value(3,2);
    double r01 = Ra.Value(1,3)*Rb.Value(1,1) + Ra.Value(2,3)*Rb.Value(2,1) + Ra.Value(3,3)*Rb.Value(3,1)
               - Ra.Value(1,1)*Rb.Value(1,3) - Ra.Value(2,1)*Rb.Value(2,3) - Ra.Value(3,1)*Rb.Value(3,3);
    double rot_err = 0.5 * std::sqrt(r12*r12 + r20*r20 + r01*r01);
    return rot_err <= rot_tol;
}

bool SolutionMatchesTarget(const domain::Robot& robot, const gp_Trsf& target,
                           const nl::utils::Q& joint_angles)
{
    nl::kinematics::KdlSolver solver;
    std::vector<gp_Trsf> fk = solver.ComputeFk(robot, joint_angles);
    if (fk.empty()) return false;
    return PoseNear(fk.back(), target);
}

bool AppendUniqueSolution(const domain::Robot& robot, const gp_Trsf& target,
                          const nl::utils::Q& candidate,
                          std::vector<nl::utils::Q>& out_solutions)
{
    nl::utils::Q normalized = candidate;
    NormalizeJointAngles(normalized);

    if (!SolutionMatchesTarget(robot, target, normalized))
        return false;

    for (const nl::utils::Q& existing : out_solutions) {
        if (JointDistance(existing, normalized) < 1e-4)
            return false;
    }

    out_solutions.push_back(normalized);
    return true;
}

std::vector<nl::utils::Q> BuildIkSeedVariants(const nl::utils::Q& init_angles)
{
    std::vector<nl::utils::Q> seeds;

    auto add_seed = [&](nl::utils::Q seed) {
        NormalizeJointAngles(seed);
        for (const nl::utils::Q& existing : seeds) {
            if (JointDistance(existing, seed) < 1e-4)
                return;
        }
        seeds.push_back(seed);
    };

    auto wrist_flip = [](const nl::utils::Q& q) {
        nl::utils::Q flipped = q;
        flipped[3] += 180.0;
        flipped[4] = -flipped[4];
        flipped[5] += 180.0;
        return flipped;
    };

    auto shoulder_flip = [](const nl::utils::Q& q) {
        nl::utils::Q flipped = q;
        flipped[0] += 180.0;
        flipped[1] = -flipped[1];
        flipped[2] = -flipped[2];
        return flipped;
    };

    add_seed(init_angles);
    add_seed(wrist_flip(init_angles));
    add_seed(shoulder_flip(init_angles));
    add_seed(wrist_flip(shoulder_flip(init_angles)));

    std::vector<nl::utils::Q> neutral_seeds = {
        nl::utils::Q({0.0, -45.0, -45.0, 0.0,  45.0, 0.0}),
        nl::utils::Q({0.0,  45.0, -90.0, 0.0, -45.0, 0.0}),
        nl::utils::Q({180.0, -45.0, -45.0, 180.0,  45.0, 180.0}),
        nl::utils::Q({180.0,  45.0, -90.0, 180.0, -45.0, 180.0}),
        nl::utils::Q({90.0,  -30.0, -60.0, 90.0,  60.0, -90.0}),
        nl::utils::Q({-90.0,  30.0, -60.0, -90.0, -60.0, 90.0}),
    };

    for (const nl::utils::Q& seed : neutral_seeds) {
        add_seed(seed);
        add_seed(wrist_flip(seed));
    }

    return seeds;
}

bool NumericalIkAll(const domain::Robot& robot, const gp_Trsf& target,
                    const nl::utils::Q& init_angles,
                    std::vector<nl::utils::Q>& out_solutions)
{
    out_solutions.clear();
    nl::kinematics::KdlSolver solver;

    for (const nl::utils::Q& seed : BuildIkSeedVariants(init_angles)) {
        nl::utils::Q candidate(6, 0.0);
        if (!solver.ComputeIk(robot, target, seed, candidate))
            continue;
        AppendUniqueSolution(robot, target, candidate, out_solutions);
    }

    if (out_solutions.empty()) return false;

    std::sort(out_solutions.begin(), out_solutions.end(),
        [&](const nl::utils::Q& a, const nl::utils::Q& b) {
            return JointDistance(a, init_angles) < JointDistance(b, init_angles);
        });

    return true;
}

} // namespace

namespace planning {

foundation::Result<std::vector<foundation::Pose>> KdlKinematicsService::ComputeFk(
    const domain::Robot& robot,
    const foundation::JointState& joints)
{
    nl::kinematics::KdlSolver solver;
    std::vector<gp_Trsf> fk = solver.ComputeFk(robot, foundation::ToQ(joints));
    if (fk.empty()) {
        return foundation::Result<std::vector<foundation::Pose>>::Fail(
            foundation::Error{ foundation::ErrorCode::kInternalError, "FK failed: empty result" });
    }

    std::vector<foundation::Pose> poses;
    poses.reserve(fk.size());
    for (const gp_Trsf& t : fk)
        poses.push_back(foundation::ToPose(t));

    return foundation::Result<std::vector<foundation::Pose>>::Ok(std::move(poses));
}

// ComputeIk 取 ComputeIkAll 第 0 个解，不重复实现
foundation::Result<foundation::JointState> KdlKinematicsService::ComputeIk(
    const domain::Robot& robot,
    const foundation::Pose& target,
    const foundation::JointState& seed)
{
    auto all = ComputeIkAll(robot, target, seed);
    if (!all) {
        return foundation::Result<foundation::JointState>::Fail(all.error());
    }
    return foundation::Result<foundation::JointState>::Ok(all.value()[0]);
}

foundation::Result<std::vector<foundation::JointState>> KdlKinematicsService::ComputeIkAll(
    const domain::Robot& robot,
    const foundation::Pose& target,
    const foundation::JointState& seed)
{
    gp_Trsf target_trsf = foundation::ToGpTrsf(target);
    nl::utils::Q seed_q = foundation::ToQ(seed);

    std::vector<nl::utils::Q> solutions;
    if (!NumericalIkAll(robot, target_trsf, seed_q, solutions)) {
        return foundation::Result<std::vector<foundation::JointState>>::Fail(
            foundation::Error{ foundation::ErrorCode::kInternalError, "IK failed: no solution found" });
    }

    std::vector<foundation::JointState> result;
    result.reserve(solutions.size());
    for (const nl::utils::Q& q : solutions)
        result.push_back(foundation::ToJointState(q));

    return foundation::Result<std::vector<foundation::JointState>>::Ok(std::move(result));
}

} // namespace planning
