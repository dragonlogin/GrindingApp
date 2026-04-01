#ifndef GRINDINGAPP_SRC_UI_TRAJECTORY_PLANNER_H_
#define GRINDINGAPP_SRC_UI_TRAJECTORY_PLANNER_H_

#include <vector>
#include <gp_Trsf.hxx>

#include "domain/Trajectory.h"
#include "domain/Waypoint.h"
#include "Q.h"
#include "RbXmlParser.h"
#include "GrindingUIExport.h"

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT TrajectoryPlanner {
public:
    struct Config {
        double approach_dist = 50.0;
        int movej_steps = 50;
        int movel_steps_per_seg = 10;
        double joint_jump_threshold = 30.0;
    };

    domain::Trajectory Plan(
        const std::vector<domain::Waypoint>& waypoints,
        const nl::core::RbRobot& robot,
        const nl::utils::Q& current_angles,
        const Config& config);

    bool ResolveSinglePoint(
        domain::Trajectory& traj, int index,
        const nl::core::RbRobot& robot,
        int solution_index);

private:
    std::vector<domain::TrajectoryPoint> InterpolateMoveJ(
        const nl::utils::Q& from,
        const nl::utils::Q& to,
        const gp_Trsf& target_pose,
        int steps);

    std::vector<domain::TrajectoryPoint> InterpolateMoveL(
        const gp_Trsf& from_pose,
        const gp_Trsf& to_pose,
        const nl::core::RbRobot& robot,
        const nl::utils::Q& seed_angles,
        int steps,
        int waypoint_index);

    gp_Trsf ComputeApproachPose(
        const gp_Trsf& first_wp_pose,
        double approach_dist);

    bool HasJointJump(
        const nl::utils::Q& a,
        const nl::utils::Q& b,
        double threshold);
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_TRAJECTORY_PLANNER_H_
