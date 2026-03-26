#ifndef GRINDINGAPP_SRC_OCC_TRAJECTORY_H_
#define GRINDINGAPP_SRC_OCC_TRAJECTORY_H_

#include <vector>
#include <gp_Trsf.hxx>
#include "Q.h"

namespace nl {
namespace occ {

struct TrajectoryPoint {
    gp_Trsf tcp_pose;
    nl::utils::Q joint_angles;

    enum class MoveType { kMoveJ, kMoveL };
    MoveType move_type = MoveType::kMoveL;

    enum class Status { kOk, kIkFailed, kJointJump };
    Status status = Status::kOk;

    int waypoint_index = -1;  // Source waypoint index (-1 = interpolated)
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;

    bool HasErrors() const {
        for (const auto& p : points)
            if (p.status != TrajectoryPoint::Status::kOk) return true;
        return false;
    }

    int ErrorCount() const {
        int count = 0;
        for (const auto& p : points)
            if (p.status != TrajectoryPoint::Status::kOk) ++count;
        return count;
    }
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_TRAJECTORY_H_
