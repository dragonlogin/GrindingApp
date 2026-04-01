#pragma once

#include <string>
#include <vector>

#include "Ids.h"
#include "Waypoint.h"

namespace domain {

struct WaypointSet {
    foundation::WaypointSetId id;
    std::vector<Waypoint> points;
    std::string source_face_token; // source_face_token 先用 std::string 过渡，后面再换成正式 FaceRef
};

} // namespace domain
