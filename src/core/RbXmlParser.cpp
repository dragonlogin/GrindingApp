#include "RbXmlParser.h"

#include <cmath>
#include <unordered_map>

#include <QDebug>
#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QString>

namespace nl {
namespace core {

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;

utils::Vector3d ParseTriple(const QString& text)
{
    utils::Vector3d out;
    const auto parts = text.split(' ', Qt::SkipEmptyParts);
    if (parts.size() == 3) {
        out[0] = parts[0].toDouble();
        out[1] = parts[1].toDouble();
        out[2] = parts[2].toDouble();
    }
    return out;
}

QString CanonicalJointName(const QString& raw_name)
{
    if (raw_name.startsWith("joint", Qt::CaseInsensitive)) {
        bool ok = false;
        const int index = raw_name.mid(5).toInt(&ok);
        if (ok) return QStringLiteral("Joint%1").arg(index);
    }
    return raw_name;
}

QString ResolveMeshPath(const QString& base_dir, const QString& relative_path)
{
    const QString full = QDir(base_dir).filePath(relative_path);
    if (QFile::exists(full)) return full;

    const QFileInfo info(full);
    if (info.suffix().isEmpty()) return full;

    const QString alt_suffix =
        info.suffix().toLower() == info.suffix() ? info.suffix().toUpper() : info.suffix().toLower();
    const QString alt = info.path() + QDir::separator() + info.completeBaseName() + "." + alt_suffix;
    if (QFile::exists(alt)) return alt;
    return full;
}

bool MeshIsRenderableNow(const QString& mesh_path)
{
    return QFileInfo(mesh_path).suffix().compare(QStringLiteral("stl"), Qt::CaseInsensitive) == 0;
}

bool TryFillDrawableFromUrdfElement(const QDomElement& element,
                                    const QString& base_dir,
                                    const std::string& ref_joint,
                                    const std::string& drawable_name,
                                    RbDrawable& drawable)
{
    if (element.isNull()) return false;

    const QDomElement mesh_el =
        element.firstChildElement("geometry").firstChildElement("mesh");
    if (mesh_el.isNull()) return false;

    const QString resolved_mesh =
        ResolveMeshPath(base_dir, mesh_el.attribute("filename"));
    if (!MeshIsRenderableNow(resolved_mesh))
        return false;

    const QDomElement origin_el = element.firstChildElement("origin");
    const utils::Vector3d xyz = ParseTriple(origin_el.attribute("xyz"));
    const utils::Vector3d rpy_rad = ParseTriple(origin_el.attribute("rpy"));

    drawable.name = drawable_name;
    drawable.ref_joint = ref_joint;
    drawable.pos = xyz;
    // URDF stores roll-pitch-yaw in radians; RobotDisplay expects yaw-pitch-roll in degrees.
    drawable.rpy[0] = rpy_rad[2] * kRadToDeg;
    drawable.rpy[1] = rpy_rad[1] * kRadToDeg;
    drawable.rpy[2] = rpy_rad[0] * kRadToDeg;
    drawable.mesh_file = resolved_mesh.toStdString();
    return true;
}

void ParseSerialDevice(const QDomElement& root, const QString& base_dir, RbRobot& robot)
{
    const QDomNodeList dh_nodes = root.elementsByTagName("DHJoint");
    for (int i = 0; i < dh_nodes.count(); ++i) {
        const QDomElement el = dh_nodes.at(i).toElement();
        RbJoint joint;
        joint.name = el.attribute("name").toStdString();
        joint.alpha_deg = el.attribute("alpha").toDouble();
        joint.a = el.attribute("a").toDouble();
        joint.d = el.attribute("d").toDouble();
        joint.offset_deg = el.attribute("offset").toDouble();
        robot.joints.push_back(joint);
    }

    const QDomNodeList draw_nodes = root.elementsByTagName("Drawable");
    for (int i = 0; i < draw_nodes.count(); ++i) {
        const QDomElement el = draw_nodes.at(i).toElement();
        RbDrawable drawable;
        drawable.name = el.attribute("name").toStdString();
        drawable.ref_joint = el.attribute("refframe").toStdString();

        const utils::Vector3d rpy = ParseTriple(el.firstChildElement("RPY").text());
        const utils::Vector3d pos = ParseTriple(el.firstChildElement("Pos").text());
        drawable.rpy = rpy;
        drawable.pos = pos;

        const QDomElement poly_el = el.firstChildElement("Polytope");
        if (!poly_el.isNull()) {
            const QString relative_path = poly_el.attribute("file") + ".stl";
            drawable.mesh_file = ResolveMeshPath(base_dir, relative_path).toStdString();
        }

        robot.drawables.push_back(drawable);
    }
}

void ParseUrdf(const QDomElement& root, const QString& base_dir, RbRobot& robot)
{
    std::unordered_map<std::string, std::string> child_link_to_joint;

    const QDomNodeList joint_nodes = root.elementsByTagName("joint");
    for (int i = 0; i < joint_nodes.count(); ++i) {
        const QDomElement joint_el = joint_nodes.at(i).toElement();
        const QString type = joint_el.attribute("type");
        const QString joint_name = CanonicalJointName(joint_el.attribute("name"));
        const QString child_link = joint_el.firstChildElement("child").attribute("link");

        if (type != QStringLiteral("fixed")) {
            const QDomElement origin_el = joint_el.firstChildElement("origin");
            const utils::Vector3d xyz = ParseTriple(origin_el.attribute("xyz"));
            const utils::Vector3d rpy_rad = ParseTriple(origin_el.attribute("rpy"));

            RbJoint joint;
            joint.name = joint_name.toStdString();
            joint.alpha_deg = rpy_rad[0] * kRadToDeg;
            joint.a = xyz[0];
            joint.d = xyz[2];
            joint.offset_deg = joint_el.attribute("offset_deg").toDouble();
            robot.joints.push_back(joint);
        }

        if (!child_link.isEmpty())
            child_link_to_joint[child_link.toStdString()] = joint_name.toStdString();
    }

    const QDomNodeList link_nodes = root.elementsByTagName("link");
    for (int i = 0; i < link_nodes.count(); ++i) {
        const QDomElement link_el = link_nodes.at(i).toElement();
        const QString link_name = link_el.attribute("name");

        std::string ref_joint = "Robot_Base";
        const auto mapping = child_link_to_joint.find(link_name.toStdString());
        if (mapping != child_link_to_joint.end())
            ref_joint = mapping->second;

        int visual_index = 0;
        for (QDomElement visual_el = link_el.firstChildElement("visual");
             !visual_el.isNull();
             visual_el = visual_el.nextSiblingElement("visual")) {
            RbDrawable drawable;
            const std::string drawable_name = QStringLiteral("%1Visual%2")
                                                  .arg(link_name)
                                                  .arg(visual_index)
                                                  .toStdString();
            if (TryFillDrawableFromUrdfElement(
                    visual_el, base_dir, ref_joint, drawable_name, drawable)) {
                robot.drawables.push_back(drawable);
                ++visual_index;
            }
        }

        if (visual_index > 0)
            continue;

        int collision_index = 0;
        for (QDomElement collision_el = link_el.firstChildElement("collision");
             !collision_el.isNull();
             collision_el = collision_el.nextSiblingElement("collision")) {
            RbDrawable drawable;
            const std::string drawable_name = QStringLiteral("%1Collision%2")
                                                  .arg(link_name)
                                                  .arg(collision_index)
                                                  .toStdString();
            if (TryFillDrawableFromUrdfElement(
                    collision_el, base_dir, ref_joint, drawable_name, drawable)) {
                robot.drawables.push_back(drawable);
                ++collision_index;
            }
        }
    }
}

} // namespace

RbRobot RbXmlParser::Parse(const std::string& xml_path)
{
    RbRobot robot;
    QFile file(QString::fromStdString(xml_path));
    if (!file.open(QIODevice::ReadOnly)) {
        qWarning() << "RbXmlParser: cannot open" << xml_path.c_str();
        return robot;
    }

    const QByteArray raw = file.readAll();
    file.close();

    QDomDocument doc;
    QString err_msg;
    int err_line = 0;
    int err_col = 0;
    if (!doc.setContent(raw, &err_msg, &err_line, &err_col)) {
        qWarning() << "RbXmlParser: XML parse error at line" << err_line
                   << "col" << err_col << ":" << err_msg;
        return robot;
    }

    const QString base_dir = QFileInfo(QString::fromStdString(xml_path))
                                 .absoluteDir().absolutePath();
    const QDomElement root = doc.documentElement();
    robot.name = root.attribute("name").toStdString();
    robot.source_path = xml_path;

    if (root.tagName() == QStringLiteral("robot")) {
        ParseUrdf(root, base_dir, robot);
        return robot;
    }

    ParseSerialDevice(root, base_dir, robot);
    return robot;
}

} // namespace core
} // namespace nl
