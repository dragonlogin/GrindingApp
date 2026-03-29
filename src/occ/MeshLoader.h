#ifndef GRINDINGAPP_SRC_OCC_MESH_LOADER_H_
#define GRINDINGAPP_SRC_OCC_MESH_LOADER_H_

#include <string>

#include <TopoDS_Shape.hxx>

#include "GrindingOccExport.h"

namespace nl {
namespace occ {

class GRINDING_OCC_EXPORT MeshLoader {
public:
    // Returns Null shape on failure or unsupported formats.
    static TopoDS_Shape Load(const std::string& mesh_path);
};

} // namespace occ
} // namespace nl

#endif  // GRINDINGAPP_SRC_OCC_MESH_LOADER_H_
