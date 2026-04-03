#pragma once

#include "GrindingPlanningExport.h"
#include "ITrajectoryPlanner.h"

namespace planning {

// [Strategy] 具体策略：笛卡尔插值规划（approach + MoveJ + MoveL + IK + jump 检查）
class GRINDING_PLANNING_EXPORT CartesianTrajectoryPlanner : public ITrajectoryPlanner {
public:
    // [Facade] 对外一个入口，内部封装 5 步规划流程
    foundation::Result<domain::Trajectory> Plan(
        const nl::core::RbRobot& robot,
        const nl::utils::Q& current_angles,
        const domain::WaypointSet& waypoints,
        const PlanningRequest& request) override;

    // 不在接口内：UI 交互行为，用于面板重选 IK 解
    bool ResolveSinglePoint(
        domain::Trajectory& traj,
        int index,
        const nl::core::RbRobot& robot,
        int solution_index);
};

} // namespace planning
