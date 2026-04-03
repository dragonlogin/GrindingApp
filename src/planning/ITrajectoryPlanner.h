#pragma once

#include "domain/Trajectory.h"
#include "domain/WaypointSet.h"
#include "foundation/Result.h"
#include "PlanningTypes.h"
#include "RbXmlParser.h"
#include "Q.h"

namespace planning {

    // [Strategy] 抽象策略：规定接口契约，不含任何算法实现
    class ITrajectoryPlanner {
    public:
        virtual ~ITrajectoryPlanner() = default;

        // [Result 惯用法] 返回 Result<T> 而非抛异常或返回空对象，
        //   调用方被迫显式处理失败，错误信息有类型、有描述
        virtual foundation::Result<domain::Trajectory> Plan(
            const nl::core::RbRobot& robot,
            const nl::utils::Q& current_angles,
            const domain::WaypointSet& waypoints,
            const PlanningRequest& request) = 0;
    };

} // namespace planning
