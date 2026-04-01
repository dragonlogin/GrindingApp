#pragma once

#include "domain/Waypoint.h"

#include <gp_Trsf.hxx>
namespace foundation {

inline domain::Waypoint ToWaypoint(const gp_Trsf& pose, double speed_ratio = 1.0)
{
    return domain::Waypoint{ToPose(pose), speed_ratio};
}

inline gp_Trsf ToGpTrsf(const domain::Waypoint& waypoint)
{
    return ToGpTrsf(waypoint.pose);
}

} // namespace foundation