#include "CartesianTrajectoryPlanner.h"

#include <cmath>

// OCCT 类型只在 .cpp 中引入 [Pimpl 精神]
#include <gp_Vec.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>

#include "Q.h"
#include "foundation/Conversions.h"

using domain::Trajectory;
using domain::TrajectoryPoint;
using domain::Waypoint;

// [Pimpl 精神] 依赖 gp_Trsf 的 helper 放匿名 namespace，对外不可见
namespace {

gp_Trsf ComputeApproachPose(const gp_Trsf& first_wp_pose, double approach_dist)
{
    gp_XYZ z_col(first_wp_pose.Value(1, 3),
        first_wp_pose.Value(2, 3),
        first_wp_pose.Value(3, 3));
    gp_XYZ origin = first_wp_pose.TranslationPart();
    gp_XYZ approach_origin = origin - z_col * approach_dist;

    gp_Trsf result = first_wp_pose;
    result.SetTranslationPart(gp_Vec(approach_origin));
    return result;
}

bool HasJointJump(const nl::utils::Q& a, const nl::utils::Q& b, double threshold)
{
    int n = std::min(a.size(), b.size());
    for (int i = 0; i < n; ++i) {
        if (std::abs(a[i] - b[i]) > threshold)
            return true;
    }
    return false;
}

std::vector<TrajectoryPoint> InterpolateMoveJ(
    const nl::utils::Q& from,
    const nl::utils::Q& to,
    const gp_Trsf& target_pose,
    int steps)
{
    std::vector<TrajectoryPoint> result;
    if (steps < 2) steps = 2;

    int n = std::min(from.size(), to.size());
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;

        TrajectoryPoint pt;
        pt.move_type = domain::MoveType::kMoveJ;
        nl::utils::Q q(n);
        for (int j = 0; j < n; ++j) {
            q[j] = from[j] * (1.0 - t) + to[j] * t;
        }
        pt.joint_state = foundation::ToJointState(q);
        pt.tcp_pose = foundation::ToPose(target_pose);
        pt.status = domain::TrajectoryPointStatus::kOk;
        result.push_back(pt);
    }
    return result;
}

gp_Trsf InterpolatePose(const gp_Trsf& from, const gp_Trsf& to, double t)
{
    gp_XYZ p0 = from.TranslationPart();
    gp_XYZ p1 = to.TranslationPart();
    gp_XYZ pos = p0 * (1.0 - t) + p1 * t;

    gp_Quaternion q0 = from.GetRotation();
    gp_Quaternion q1 = to.GetRotation();

    gp_Quaternion q0_inv = q0.Inverted();
    gp_Quaternion q_diff = q0_inv * q1;

    double dot = q0.X() * q1.X() + q0.Y() * q1.Y() +
        q0.Z() * q1.Z() + q0.W() * q1.W();
    if (dot < 0.0) {
        q1.Set(-q1.X(), -q1.Y(), -q1.Z(), -q1.W());
        q_diff = q0_inv * q1;
    }

    double angle = 2.0 * std::acos(std::min(1.0, std::abs(q_diff.W())));
    gp_Quaternion qi;
    if (angle < 1e-10) {
        qi = q0;
    }
    else {
        double sin_half = std::sin(angle / 2.0);
        double ax = q_diff.X() / sin_half;
        double ay = q_diff.Y() / sin_half;
        double az = q_diff.Z() / sin_half;
        double scaled_angle = angle * t;
        double sin_s = std::sin(scaled_angle / 2.0);
        double cos_s = std::cos(scaled_angle / 2.0);
        gp_Quaternion q_part(ax * sin_s, ay * sin_s, az * sin_s, cos_s);
        qi = q0 * q_part;
    }

    gp_Trsf result;
    result.SetRotation(qi);
    result.SetTranslationPart(gp_Vec(pos));
    return result;
}

std::vector<TrajectoryPoint> InterpolateMoveL(
    const gp_Trsf& from_pose,
    const gp_Trsf& to_pose,
    const domain::Robot& robot,
    planning::IKinematicsService& kin,
    const nl::utils::Q& seed_angles,
    int steps,
    int waypoint_index)
{
    std::vector<TrajectoryPoint> result;
    if (steps < 1) steps = 1;

    nl::utils::Q prev_angles = seed_angles;

    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;

        TrajectoryPoint pt;
        pt.move_type = domain::MoveType::kMoveL;
        gp_Trsf tcp_pose = InterpolatePose(from_pose, to_pose, t);
        pt.tcp_pose = foundation::ToPose(tcp_pose);
        if (i == steps) {
            pt.waypoint_index = waypoint_index;
        }

        auto ik_result = kin.ComputeIk(
            robot, pt.tcp_pose, foundation::ToJointState(prev_angles));

        if (!ik_result) {
            pt.status = domain::TrajectoryPointStatus::kIkFailed;
            pt.joint_state = foundation::ToJointState(prev_angles);
        }
        else {
            pt.joint_state = ik_result.value();
            pt.status = domain::TrajectoryPointStatus::kOk;
            prev_angles = foundation::ToQ(ik_result.value());
        }

        result.push_back(pt);
    }
    return result;
}

} // anonymous namespace

namespace planning {

// [Facade] 对外一个入口，内部封装 5 步规划流程
// [Result 惯用法] 失败时返回 Fail，成功时 Ok 携带结果
foundation::Result<domain::Trajectory> CartesianTrajectoryPlanner::Plan(
    const domain::Robot& robot,
    const foundation::JointState& current_angles,
    const domain::WaypointSet& waypoints,
    const PlanningRequest& request)
{
    if (waypoints.points.empty()) {
        return foundation::Result<domain::Trajectory>::Fail(
            foundation::Error{ foundation::ErrorCode::kInvalidArgument,
                "CartesianTrajectoryPlanner: no waypoints" });
    }

    Trajectory traj;

    gp_Trsf approach_pose = ComputeApproachPose(
        foundation::ToGpTrsf(waypoints.points[0].pose), request.approach_dist);

    auto approach_result = kin_.ComputeIk(
        robot, foundation::ToPose(approach_pose), current_angles);

    nl::utils::Q current_q = foundation::ToQ(current_angles);
    nl::utils::Q approach_q;
    bool approach_ok = approach_result.operator bool();

    if (approach_ok) {
        approach_q = foundation::ToQ(approach_result.value());
        auto movej_pts = InterpolateMoveJ(
            current_q, approach_q, approach_pose, request.movej_steps);
        for (auto& pt : movej_pts)
            traj.points.push_back(std::move(pt));
    }
    else {
        TrajectoryPoint pt;
        pt.tcp_pose = foundation::ToPose(approach_pose);
        pt.joint_state = current_angles;
        pt.move_type = domain::MoveType::kMoveJ;
        pt.status = domain::TrajectoryPointStatus::kIkFailed;
        traj.points.push_back(pt);
    }

    nl::utils::Q prev_angles = approach_ok ? approach_q : current_q;
    gp_Trsf prev_pose = approach_pose;

    for (size_t i = 0; i < waypoints.points.size(); ++i) {
        auto movel_pts = InterpolateMoveL(
            prev_pose,
            foundation::ToGpTrsf(waypoints.points[i].pose),
            robot,
            kin_,
            prev_angles,
            request.movel_steps_per_seg,
            static_cast<int>(i));

        for (auto& pt : movel_pts)
            traj.points.push_back(std::move(pt));

        if (!traj.points.empty()) {
            const auto& last = traj.points.back();
            if (last.status == domain::TrajectoryPointStatus::kOk)
                prev_angles = foundation::ToQ(last.joint_state);
            prev_pose = foundation::ToGpTrsf(waypoints.points[i].pose);
        }
    }

    // Post-process: detect joint jumps
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& pt = traj.points[i];
        if (pt.status != domain::TrajectoryPointStatus::kOk) continue;
        const auto& prev = traj.points[i - 1];
        if (prev.status != domain::TrajectoryPointStatus::kOk) continue;
        if (HasJointJump(
            foundation::ToQ(prev.joint_state),
            foundation::ToQ(pt.joint_state),
            request.joint_jump_threshold)) {
            pt.status = domain::TrajectoryPointStatus::kJointJump;
        }
    }

    return foundation::Result<domain::Trajectory>::Ok(std::move(traj));
}

bool CartesianTrajectoryPlanner::ResolveSinglePoint(
    domain::Trajectory& traj,
    int index,
    const domain::Robot& robot,
    int solution_index)
{
    if (index < 0 || index >= static_cast<int>(traj.points.size()))
        return false;

    auto& pt = traj.points[index];

    foundation::JointState seed;
    if (index > 0 && traj.points[index - 1].status ==
        domain::TrajectoryPointStatus::kOk) {
        seed = traj.points[index - 1].joint_state;
    }
    else {
        seed = pt.joint_state;
    }

    auto solutions_result = kin_.ComputeIkAll(robot, pt.tcp_pose, seed);
    if (!solutions_result)
        return false;

    const auto& solutions = solutions_result.value();
    if (solution_index < 0 ||
        solution_index >= static_cast<int>(solutions.size())) {
        return false;
    }

    pt.joint_state = solutions[solution_index];
    pt.status = domain::TrajectoryPointStatus::kOk;

    if (index > 0) {
        const auto& prev = traj.points[index - 1];
        if (prev.status == domain::TrajectoryPointStatus::kOk &&
            HasJointJump(
                foundation::ToQ(prev.joint_state),
                foundation::ToQ(pt.joint_state),
                30.0)) {
            pt.status = domain::TrajectoryPointStatus::kJointJump;
        }
    }

    return true;
}

} // namespace planning
