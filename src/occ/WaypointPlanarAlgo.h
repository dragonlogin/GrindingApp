#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_PLANAR_ALGO_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_PLANAR_ALGO_H_

#include "IWaypointAlgo.h"
#include "GrindingOccExport.h"

namespace nl {
namespace occ {

class GRINDING_OCC_EXPORT WaypointPlanarAlgo : public IWaypointAlgo {
public:
    std::vector<Waypoint> Generate(
        const TopoDS_Face& face,
        const WaypointConfig& config) override;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_WAYPOINT_PLANAR_ALGO_H_
