#ifndef GRINDINGAPP_SRC_STEP_IMPORTER_H_
#define GRINDINGAPP_SRC_STEP_IMPORTER_H_

#include <QString>
#include <TopoDS_Shape.hxx>

class StepImporter {
public:
    // Load a STEP file and triangulate. face_count may be nullptr.
    static TopoDS_Shape Load(const QString& file_path, int* face_count = nullptr);
};

#endif  // GRINDINGAPP_SRC_STEP_IMPORTER_H_
