#include "WaypointGenerator.h"

namespace nl {
namespace occ {

void WaypointGenerator::SetFace(const TopoDS_Face& face)
{
    face_ = face;
}

void WaypointGenerator::SetAlgorithm(std::unique_ptr<IWaypointAlgo> algo)
{
    algo_ = std::move(algo);
}

std::vector<Waypoint> WaypointGenerator::Generate(const WaypointConfig& config)
{
    if (face_.IsNull() || !algo_)
        return {};

    return algo_->Generate(face_, config);
}

} // namespace occ
} // namespace nl
