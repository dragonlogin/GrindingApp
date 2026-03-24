#ifndef GRINDINGAPP_SRC_STL_LOADER_H_
#define GRINDINGAPP_SRC_STL_LOADER_H_

#include <QString>
#include <TopoDS_Shape.hxx>

class StlLoader {
public:
    // Returns Null shape on failure.
    static TopoDS_Shape Load(const QString& stl_path);
};

#endif  // GRINDINGAPP_SRC_STL_LOADER_H_
