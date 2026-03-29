#include "MeshLoader.h"

#include <QDebug>
#include <QFileInfo>
#include <QString>

#include "StlLoader.h"

namespace nl {
namespace occ {

TopoDS_Shape MeshLoader::Load(const std::string& mesh_path)
{
    const QString path = QString::fromStdString(mesh_path);
    const QString suffix = QFileInfo(path).suffix().toLower();

    if (suffix == QStringLiteral("stl"))
        return StlLoader::Load(mesh_path);

    qWarning() << "MeshLoader: unsupported mesh format:" << path;
    return TopoDS_Shape();
}

} // namespace occ
} // namespace nl
