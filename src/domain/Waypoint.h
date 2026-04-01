#pragma once

#include "Pose.h"

namespace domain {

struct Waypoint {
    foundation::Pose pose;
    double speed_ratio = 1.0;
};

} // namespace domain