#include "UrdfToolImporter.h"

#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QSet>
#include <QString>

#include "foundation/Error.h"

namespace model_import {

namespace {

nl::utils::Vector3d ParseXyz(const std::string& s)
{
    auto p = QString::fromStdString(s).split(' ', Qt::SkipEmptyParts);
    if (p.size() == 3)
        return {p[0].toDouble(), p[1].toDouble(), p[2].toDouble()};
    return {};
}

nl::utils::Vector3d ParseUrdfRpyDegrees(const QString& s)
{
    const auto parts = s.split(' ', Qt::SkipEmptyParts);
    if (parts.size() != 3)
        return {};
    const double roll_deg  = parts[0].toDouble();
    const double pitch_deg = parts[1].toDouble();
    const double yaw_deg   = parts[2].toDouble();
    return {yaw_deg, pitch_deg, roll_deg};
}

nl::utils::Vector3d ParseScaleTriple(const QString& s)
{
    const auto parts = s.split(' ', Qt::SkipEmptyParts);
    if (parts.size() != 3)
        return {1.0, 1.0, 1.0};
    return {parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble()};
}

QString ResolveRelativePath(const QString& base_dir, const QString& relative_path)
{
    return QDir(base_dir).filePath(relative_path);
}

QString FindUrdfRootLink(const QDomElement& robot_el)
{
    QSet<QString> child_links;
    const QDomNodeList joint_nodes = robot_el.elementsByTagName("joint");
    for (int i = 0; i < joint_nodes.count(); ++i) {
        const QString child_link =
            joint_nodes.at(i).toElement().firstChildElement("child").attribute("link");
        if (!child_link.isEmpty())
            child_links.insert(child_link);
    }
    const QDomNodeList link_nodes = robot_el.elementsByTagName("link");
    for (int i = 0; i < link_nodes.count(); ++i) {
        const QDomElement link_el = link_nodes.at(i).toElement();
        const QString link_name = link_el.attribute("name");
        if (!child_links.contains(link_name))
            return link_name;
    }
    return {};
}

QDomElement FindUrdfLinkByName(const QDomElement& robot_el, const QString& link_name)
{
    const QDomNodeList link_nodes = robot_el.elementsByTagName("link");
    for (int i = 0; i < link_nodes.count(); ++i) {
        const QDomElement link_el = link_nodes.at(i).toElement();
        if (link_el.attribute("name") == link_name)
            return link_el;
    }
    return {};
}

} // namespace

foundation::Result<ToolDefinition>
UrdfToolImporter::Import(const std::string& file_path)
{
    const QString urdf_path = QString::fromStdString(file_path);
    QFile file(urdf_path);
    if (!file.open(QIODevice::ReadOnly))
        return foundation::Result<ToolDefinition>::Fail(
            foundation::Error{foundation::ErrorCode::kImportFailed, "Cannot open tool file: " + file_path});

    QDomDocument doc;
    QString err_msg;
    int err_line = 0, err_col = 0;
    if (!doc.setContent(file.readAll(), &err_msg, &err_line, &err_col))
        return foundation::Result<ToolDefinition>::Fail(
            foundation::Error{foundation::ErrorCode::kImportFailed, "XML parse error: " + err_msg.toStdString()});

    const QDomElement robot_el = doc.documentElement();
    const QString base_dir = QFileInfo(urdf_path).absoluteDir().absolutePath();
    const QString root_link_name = FindUrdfRootLink(robot_el);
    if (root_link_name.isEmpty())
        return foundation::Result<ToolDefinition>::Fail(
            foundation::Error{foundation::ErrorCode::kImportFailed, "Cannot find root link in: " + file_path});

    const QDomElement root_link_el = FindUrdfLinkByName(robot_el, root_link_name);
    if (root_link_el.isNull())
        return foundation::Result<ToolDefinition>::Fail(
            foundation::Error{foundation::ErrorCode::kImportFailed, "Root link element missing in: " + file_path});

    const QDomElement visual_el = root_link_el.firstChildElement("visual");
    const QDomElement origin_el = visual_el.firstChildElement("origin");
    const QDomElement mesh_el =
        visual_el.firstChildElement("geometry").firstChildElement("mesh");
    if (mesh_el.isNull())
        return foundation::Result<ToolDefinition>::Fail(
            foundation::Error{foundation::ErrorCode::kImportFailed, "No mesh element found in: " + file_path});

    ToolDefinition def;
    def.name         = QFileInfo(urdf_path).baseName().toStdString();
    def.source_path  = file_path;
    def.mesh_file    = ResolveRelativePath(base_dir, mesh_el.attribute("filename")).toStdString();
    def.mesh_scale   = ParseScaleTriple(mesh_el.attribute("scale"));
    def.base_pos_mm  = ParseXyz(origin_el.attribute("xyz").toStdString());
    def.base_rpy_deg = ParseUrdfRpyDegrees(origin_el.attribute("rpy"));

    const QDomNodeList joint_nodes = robot_el.elementsByTagName("joint");
    for (int i = 0; i < joint_nodes.count(); ++i) {
        const QDomElement joint_el = joint_nodes.at(i).toElement();
        const QString child_link = joint_el.firstChildElement("child").attribute("link");
        if (child_link != QStringLiteral("tool0") && child_link != QStringLiteral("tcp0"))
            continue;
        const QDomElement tcp_origin = joint_el.firstChildElement("origin");
        def.tcp_pos_mm  = ParseXyz(tcp_origin.attribute("xyz").toStdString());
        def.tcp_rpy_deg = ParseUrdfRpyDegrees(tcp_origin.attribute("rpy"));
        break;
    }

    return foundation::Result<ToolDefinition>::Ok(def);
}

} // namespace model_import