#pragma once

#include "domain/Robot.h"
#include "domain/Trajectory.h"
#include "domain/WaypointSet.h"
#include "foundation/JointState.h"
#include "foundation/Result.h"
#include "IKinematicsService.h"
#include "PlanningTypes.h"

namespace planning {

// [Strategy] 规定接口契约，不含任何算法实现
class ITrajectoryPlanner {
public:
    // [Strategy] 基类持有运动学服务，所有子类共用
    explicit ITrajectoryPlanner(IKinematicsService& kin) : kin_(kin) {}
    virtual ~ITrajectoryPlanner() = default;

    // [Result 惯用法] 返回 Result<T>，错误显式传递
    virtual foundation::Result<domain::Trajectory> Plan(
        const domain::Robot& robot,
        const foundation::JointState& current_angles,
        const domain::WaypointSet& waypoints,
        const PlanningRequest& request) = 0;

protected:
    IKinematicsService& kin_;
};

} // namespace planning
