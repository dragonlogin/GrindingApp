#pragma once

#include "GrindingPlanningExport.h"
#include "ITrajectoryPlanner.h"

namespace planning {

// [Strategy] 笛卡尔轨迹规划实现：封装 approach + MoveJ + MoveL + IK + jump 检查
class GRINDING_PLANNING_EXPORT CartesianTrajectoryPlanner : public ITrajectoryPlanner {
public:
    explicit CartesianTrajectoryPlanner(IKinematicsService& kin)
        : ITrajectoryPlanner(kin) {}

    // [Facade] 对外一个入口，内部封装 5 步规划流程
    foundation::Result<domain::Trajectory> Plan(
        const domain::Robot& robot,
        const foundation::JointState& current_angles,
        const domain::WaypointSet& waypoints,
        const PlanningRequest& request) override;

    // 接口外：UI 交互用，为指定点重新选择 IK 解
    bool ResolveSinglePoint(
        domain::Trajectory& traj,
        int index,
        const domain::Robot& robot,
        int solution_index);
};

} // namespace planning
