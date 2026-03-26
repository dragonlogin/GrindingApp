#ifndef GRINDINGAPP_SRC_OCC_WAYPOINT_GENERATOR_H_
#define GRINDINGAPP_SRC_OCC_WAYPOINT_GENERATOR_H_

#include <memory>
#include "IWaypointAlgo.h"
#include "GrindingOccExport.h"

namespace nl {
namespace occ {

class GRINDING_OCC_EXPORT WaypointGenerator {
public:
    void SetFace(const TopoDS_Face& face);
    void SetAlgorithm(std::unique_ptr<IWaypointAlgo> algo);
    std::vector<Waypoint> Generate(const WaypointConfig& config);

private:
    TopoDS_Face face_;
    std::unique_ptr<IWaypointAlgo> algo_;
};

} // namespace occ
} // namespace nl

#endif // GRINDINGAPP_SRC_OCC_WAYPOINT_GENERATOR_H_
