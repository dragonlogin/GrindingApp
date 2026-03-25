#ifndef GRINDINGAPP_SRC_OCC_STL_LOADER_H_
#define GRINDINGAPP_SRC_OCC_STL_LOADER_H_

#include <string>
#include <TopoDS_Shape.hxx>

#include "GrindingAppExport.h"

class GRINDINGAPP_EXPORT StlLoader {
public:
    // Returns Null shape on failure.
    static TopoDS_Shape Load(const std::string& stl_path);
};

#endif  // GRINDINGAPP_SRC_OCC_STL_LOADER_H_
