#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_GRID_ALGO_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_GRID_ALGO_H_

#include "IWaypointAlgo.h"
#include "GrindingOccExport.h"

namespace nl {
namespace occ {

class GRINDING_OCC_EXPORT WaypointGridAlgo : public IWaypointAlgo {
public:
    std::vector<Waypoint> Generate(
        const TopoDS_Face& face,
        const WaypointConfig& config) override;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_WAYPOINT_GRID_ALGO_H_
