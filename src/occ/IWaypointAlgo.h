#ifndef GRINDINGAPP_SRC_OCC_IWAYPOINT_ALGO_H_
#define GRINDINGAPP_SRC_OCC_IWAYPOINT_ALGO_H_

#include <vector>
#include <gp_Dir.hxx>
#include <TopoDS_Face.hxx>
#include "Waypoint.h"

namespace nl {
namespace occ {

struct WaypointConfig {
    int u_steps = 10;
    int v_steps = 5;
    int num_slices = 10;
    gp_Dir cut_normal{0, 0, 1};
    double approach_dist = 0.0;
};

class IWaypointAlgo {
public:
    virtual ~IWaypointAlgo() = default;
    virtual std::vector<Waypoint> Generate(
        const TopoDS_Face& face,
        const WaypointConfig& config) = 0;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_IWAYPOINT_ALGO_H_
