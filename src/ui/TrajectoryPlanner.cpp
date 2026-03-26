#include "TrajectoryPlanner.h"

#include <cmath>

#include <gp_Vec.hxx>
#include <gp_Quaternion.hxx>

#include "RobotKinematics.h"

using nl::occ::Trajectory;
using nl::occ::TrajectoryPoint;
using nl::occ::Waypoint;

namespace nl {
namespace ui {

gp_Trsf TrajectoryPlanner::ComputeApproachPose(
    const gp_Trsf& first_wp_pose, double approach_dist)
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

bool TrajectoryPlanner::HasJointJump(
    const nl::utils::Q& a, const nl::utils::Q& b, double threshold)
{
    int n = std::min(a.size(), b.size());
    for (int i = 0; i < n; ++i) {
        if (std::abs(a[i] - b[i]) > threshold)
            return true;
    }
    return false;
}

std::vector<TrajectoryPoint> TrajectoryPlanner::InterpolateMoveJ(
    const nl::utils::Q& from, const nl::utils::Q& to,
    const gp_Trsf& target_pose, int steps)
{
    std::vector<TrajectoryPoint> result;
    if (steps < 2) steps = 2;

    int n = std::min(from.size(), to.size());
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;

        TrajectoryPoint pt;
        pt.move_type = TrajectoryPoint::MoveType::kMoveJ;
        pt.joint_angles = nl::utils::Q(n);
        for (int j = 0; j < n; ++j) {
            pt.joint_angles[j] = from[j] * (1.0 - t) + to[j] * t;
        }
        pt.tcp_pose = target_pose;
        pt.status = TrajectoryPoint::Status::kOk;
        result.push_back(pt);
    }
    return result;
}

static gp_Trsf InterpolatePose(const gp_Trsf& from, const gp_Trsf& to, double t)
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
    } else {
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

std::vector<TrajectoryPoint> TrajectoryPlanner::InterpolateMoveL(
    const gp_Trsf& from_pose, const gp_Trsf& to_pose,
    const nl::core::RbRobot& robot, const nl::utils::Q& seed_angles,
    int steps, int waypoint_index)
{
    std::vector<TrajectoryPoint> result;
    if (steps < 1) steps = 1;

    nl::utils::Q prev_angles = seed_angles;

    for (int i = 1; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;

        TrajectoryPoint pt;
        pt.move_type = TrajectoryPoint::MoveType::kMoveL;
        pt.tcp_pose = InterpolatePose(from_pose, to_pose, t);

        if (i == steps) {
            pt.waypoint_index = waypoint_index;
        }

        nl::utils::Q ik_result;
        bool ok = nl::kinematics::ComputeIk(
            robot, pt.tcp_pose, prev_angles, ik_result);

        if (!ok) {
            pt.status = TrajectoryPoint::Status::kIkFailed;
            pt.joint_angles = prev_angles;
        } else {
            pt.joint_angles = ik_result;
            pt.status = TrajectoryPoint::Status::kOk;
            prev_angles = ik_result;
        }

        result.push_back(pt);
    }
    return result;
}

Trajectory TrajectoryPlanner::Plan(
    const std::vector<Waypoint>& waypoints,
    const nl::core::RbRobot& robot,
    const nl::utils::Q& current_angles,
    const Config& config)
{
    Trajectory traj;
    if (waypoints.empty()) return traj;

    gp_Trsf approach_pose = ComputeApproachPose(
        waypoints[0].pose, config.approach_dist);

    nl::utils::Q approach_q;
    bool approach_ok = nl::kinematics::ComputeIk(
        robot, approach_pose, current_angles, approach_q);

    if (approach_ok) {
        auto movej_pts = InterpolateMoveJ(
            current_angles, approach_q, approach_pose, config.movej_steps);
        for (auto& pt : movej_pts) {
            traj.points.push_back(std::move(pt));
        }
    } else {
        TrajectoryPoint pt;
        pt.tcp_pose = approach_pose;
        pt.joint_angles = current_angles;
        pt.move_type = TrajectoryPoint::MoveType::kMoveJ;
        pt.status = TrajectoryPoint::Status::kIkFailed;
        traj.points.push_back(pt);
    }

    nl::utils::Q prev_angles = approach_ok ? approach_q : current_angles;
    gp_Trsf prev_pose = approach_pose;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        auto movel_pts = InterpolateMoveL(
            prev_pose, waypoints[i].pose,
            robot, prev_angles,
            config.movel_steps_per_seg,
            static_cast<int>(i));

        for (auto& pt : movel_pts) {
            traj.points.push_back(std::move(pt));
        }

        if (!traj.points.empty()) {
            const auto& last = traj.points.back();
            if (last.status == TrajectoryPoint::Status::kOk) {
                prev_angles = last.joint_angles;
            }
            prev_pose = waypoints[i].pose;
        }
    }

    // Post-process: detect joint jumps
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& pt = traj.points[i];
        if (pt.status != TrajectoryPoint::Status::kOk) continue;

        const auto& prev = traj.points[i - 1];
        if (prev.status != TrajectoryPoint::Status::kOk) continue;

        if (HasJointJump(prev.joint_angles, pt.joint_angles,
                         config.joint_jump_threshold)) {
            pt.status = TrajectoryPoint::Status::kJointJump;
        }
    }

    return traj;
}

bool TrajectoryPlanner::ResolveSinglePoint(
    Trajectory& traj, int index,
    const nl::core::RbRobot& robot, int solution_index)
{
    if (index < 0 || index >= static_cast<int>(traj.points.size()))
        return false;

    auto& pt = traj.points[index];

    nl::utils::Q seed;
    if (index > 0 && traj.points[index - 1].status ==
        TrajectoryPoint::Status::kOk) {
        seed = traj.points[index - 1].joint_angles;
    } else {
        seed = pt.joint_angles;
    }

    std::vector<nl::utils::Q> solutions;
    if (!nl::kinematics::ComputeIkAllSolutions(
            robot, pt.tcp_pose, seed, solutions)) {
        return false;
    }

    if (solution_index < 0 ||
        solution_index >= static_cast<int>(solutions.size())) {
        return false;
    }

    pt.joint_angles = solutions[solution_index];
    pt.status = TrajectoryPoint::Status::kOk;

    if (index > 0) {
        const auto& prev = traj.points[index - 1];
        if (prev.status == TrajectoryPoint::Status::kOk &&
            HasJointJump(prev.joint_angles, pt.joint_angles, 30.0)) {
            pt.status = TrajectoryPoint::Status::kJointJump;
        }
    }

    return true;
}

} // namespace ui
} // namespace nl
