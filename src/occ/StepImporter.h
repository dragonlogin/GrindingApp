#ifndef GRINDINGAPP_SRC_OCC_STEP_IMPORTER_H_
#define GRINDINGAPP_SRC_OCC_STEP_IMPORTER_H_

#include <string>
#include <TopoDS_Shape.hxx>

#include "GrindingAppExport.h"

class GRINDINGAPP_EXPORT StepImporter {
public:
    // Load a STEP file and triangulate. face_count may be nullptr.
    static TopoDS_Shape Load(const std::string& file_path, int* face_count = nullptr);
};

#endif  // GRINDINGAPP_SRC_OCC_STEP_IMPORTER_H_
