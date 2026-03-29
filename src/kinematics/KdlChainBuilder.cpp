#include "KdlChainBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QString>

#include <kdl/frames.hpp>
#include <kdl/joint.hpp>
#include <kdl/segment.hpp>

#include <urdf_model/link.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace nl {
namespace kinematics {

namespace {

constexpr double kDeg = M_PI / 180.0;

struct UrdfChainSpec {
    std::string base_link;
    std::string flange_link;
    std::string tool_link;
    std::unordered_map<std::string, double> joint_offset_rad;
};

QString ResolveUrdfPath(const nl::core::RbRobot& robot)
{
    if (robot.source_path.empty()) return {};

    const QString source_path = QString::fromStdString(robot.source_path);
    if (source_path.endsWith(".urdf", Qt::CaseInsensitive))
        return QDir::cleanPath(source_path);

    const QFileInfo xml_info(source_path);
    const QString urdf_path =
        xml_info.absoluteDir().filePath(QStringLiteral("urdf/%1.urdf")
                                        .arg(QString::fromStdString(robot.name)));
    return QDir::cleanPath(urdf_path);
}

QString TripleDegreesToRadians(const QString& text)
{
    const auto parts = text.split(' ', Qt::SkipEmptyParts);
    if (parts.size() != 3)
        return text;

    return QStringLiteral("%1 %2 %3")
        .arg(parts[0].toDouble() * kDeg, 0, 'g', 16)
        .arg(parts[1].toDouble() * kDeg, 0, 'g', 16)
        .arg(parts[2].toDouble() * kDeg, 0, 'g', 16);
}

QString ScalarDegreesToRadians(const QString& text)
{
    bool ok = false;
    const double value_deg = text.toDouble(&ok);
    if (!ok)
        return text;
    return QString::number(value_deg * kDeg, 'g', 16);
}

bool JointUsesAngularLimits(const QString& joint_type)
{
    return joint_type == QStringLiteral("revolute") ||
           joint_type == QStringLiteral("continuous");
}

QString LoadRobotUrdfRadiansXml(const QString& urdf_path)
{
    QFile file(urdf_path);
    if (!file.open(QIODevice::ReadOnly)) return {};

    QDomDocument doc;
    QString err_msg;
    int err_line = 0;
    int err_col = 0;
    if (!doc.setContent(file.readAll(), &err_msg, &err_line, &err_col))
        return {};

    const QDomNodeList joint_nodes = doc.documentElement().elementsByTagName(QStringLiteral("joint"));
    for (int i = 0; i < joint_nodes.count(); ++i) {
        QDomElement joint_el = joint_nodes.at(i).toElement();
        const QString joint_type = joint_el.attribute(QStringLiteral("type")).toLower();

        // Robot URDF files in this project keep all origin RPY values in
        // degrees for readability. urdfdom/KDL still expect radians, so we
        // rewrite only the XML that enters the runtime kinematics pipeline.
        QDomElement origin_el = joint_el.firstChildElement(QStringLiteral("origin"));
        if (!origin_el.hasAttribute(QStringLiteral("rpy")))
            origin_el = QDomElement();
        if (!origin_el.isNull()) {
            origin_el.setAttribute(QStringLiteral("rpy"),
                                   TripleDegreesToRadians(origin_el.attribute(QStringLiteral("rpy"))));
        }

        // Only angular joints use degree-based limit fields. Prismatic limits
        // stay in linear units and must not be converted.
        if (!JointUsesAngularLimits(joint_type))
            continue;

        QDomElement limit_el = joint_el.firstChildElement(QStringLiteral("limit"));
        if (limit_el.isNull())
            continue;
        if (limit_el.hasAttribute(QStringLiteral("lower"))) {
            limit_el.setAttribute(QStringLiteral("lower"),
                                  ScalarDegreesToRadians(limit_el.attribute(QStringLiteral("lower"))));
        }
        if (limit_el.hasAttribute(QStringLiteral("upper"))) {
            limit_el.setAttribute(QStringLiteral("upper"),
                                  ScalarDegreesToRadians(limit_el.attribute(QStringLiteral("upper"))));
        }
        if (limit_el.hasAttribute(QStringLiteral("velocity"))) {
            limit_el.setAttribute(QStringLiteral("velocity"),
                                  ScalarDegreesToRadians(limit_el.attribute(QStringLiteral("velocity"))));
        }
    }

    return doc.toString(-1);
}

bool LoadUrdfChainSpec(const QString& urdf_path, UrdfChainSpec& spec)
{
    spec = {};

    QFile file(urdf_path);
    if (!file.open(QIODevice::ReadOnly)) return false;

    QDomDocument doc;
    QString err_msg;
    int err_line = 0;
    int err_col = 0;
    if (!doc.setContent(file.readAll(), &err_msg, &err_line, &err_col))
        return false;

    const QDomNodeList joint_nodes = doc.documentElement().elementsByTagName("joint");
    // This project currently assumes a single serial chain and derives the
    // runtime chain endpoints from the ordered URDF joint list.
    std::string fallback_base_link;
    std::string preferred_tool_link;
    std::string trailing_fixed_link;
    for (int i = 0; i < joint_nodes.count(); ++i) {
        const QDomElement joint_el = joint_nodes.at(i).toElement();
        const std::string joint_name = joint_el.attribute("name").toStdString();
        const std::string joint_type = joint_el.attribute("type").toStdString();
        const std::string parent_link =
            joint_el.firstChildElement("parent").attribute("link").toStdString();
        const std::string child_link =
            joint_el.firstChildElement("child").attribute("link").toStdString();

        if (fallback_base_link.empty() && !parent_link.empty())
            fallback_base_link = parent_link;

        if (joint_type != "fixed") {
            if (spec.base_link.empty())
                spec.base_link = parent_link;
            spec.flange_link = child_link;

            bool ok = false;
            const double offset_deg = joint_el.attribute("offset_deg").toDouble(&ok);
            if (ok)
                spec.joint_offset_rad[joint_name] = offset_deg * kDeg;
        } else {
            trailing_fixed_link = child_link;
            if (joint_name == "tool0" || joint_name == "tool_mount" ||
                child_link == "tool0" || child_link == "tool_mount") {
                preferred_tool_link = child_link;
            }
        }
    }

    if (spec.base_link.empty())
        spec.base_link = fallback_base_link;
    if (spec.tool_link.empty())
        spec.tool_link = !preferred_tool_link.empty() ? preferred_tool_link : trailing_fixed_link;

    return !spec.base_link.empty() && !spec.flange_link.empty();
}

KDL::Frame UrdfPoseToKdlFrame(const urdf::Pose& pose)
{
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    pose.rotation.getRPY(roll, pitch, yaw);
    return KDL::Frame(
        KDL::Rotation::RPY(roll, pitch, yaw),
        KDL::Vector(pose.position.x, pose.position.y, pose.position.z));
}

bool CollectUrdfJointPath(const urdf::ModelInterfaceSharedPtr& model,
                          const std::string& base_link,
                          const std::string& tip_link,
                          std::vector<urdf::JointSharedPtr>& joints)
{
    joints.clear();
    if (!model) return false;

    urdf::LinkConstSharedPtr link = model->getLink(tip_link);
    while (link) {
        if (link->name == base_link) {
            std::reverse(joints.begin(), joints.end());
            return true;
        }

        urdf::JointSharedPtr joint = link->parent_joint;
        if (!joint) return false;
        joints.push_back(joint);
        link = link->getParent();
    }

    return false;
}

bool CollectUrdfLimits(const std::vector<urdf::JointSharedPtr>& joints,
                       KDL::JntArray& q_min,
                       KDL::JntArray& q_max)
{
    int movable_count = 0;
    for (const auto& joint : joints) {
        if (!joint) return false;
        if (joint->type != urdf::Joint::FIXED) ++movable_count;
    }

    q_min.resize(movable_count);
    q_max.resize(movable_count);

    int idx = 0;
    for (const auto& joint : joints) {
        if (joint->type == urdf::Joint::FIXED) continue;

        if (joint->type == urdf::Joint::CONTINUOUS || !joint->limits) {
            q_min(idx) = -std::numeric_limits<double>::infinity();
            q_max(idx) = std::numeric_limits<double>::infinity();
        } else {
            q_min(idx) = joint->limits->lower;
            q_max(idx) = joint->limits->upper;
        }
        ++idx;
    }

    return true;
}

KDL::Segment MakeKdlSegment(const urdf::Joint& joint,
                            const std::unordered_map<std::string, double>& joint_offset_rad)
{
    const KDL::Frame origin_frame = UrdfPoseToKdlFrame(joint.parent_to_joint_origin_transform);
    if (joint.type == urdf::Joint::FIXED) {
        return KDL::Segment(
            joint.child_link_name,
            KDL::Joint(KDL::Joint::Fixed),
            origin_frame);
    }

    const KDL::Vector joint_origin = origin_frame.p;
    const KDL::Vector axis_in_joint_frame(joint.axis.x, joint.axis.y, joint.axis.z);
    const KDL::Vector axis_in_parent_frame = origin_frame.M * axis_in_joint_frame;

    double joint_offset = 0.0;
    const auto found = joint_offset_rad.find(joint.name);
    if (found != joint_offset_rad.end())
        joint_offset = found->second;

    // Mirror kdl_parser's URDF mapping:
    // - joint origin is the axis point in the parent frame
    // - joint axis is rotated into the parent frame
    // - the full origin frame stays on the segment tip
    // This yields parent->child = T_origin * R_axis(q + offset).
    KDL::Joint kdl_joint;
    switch (joint.type) {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
        kdl_joint = KDL::Joint(
            joint.name,
            joint_origin,
            axis_in_parent_frame,
            KDL::Joint::RotAxis,
            1.0,
            joint_offset);
        break;
    case urdf::Joint::PRISMATIC:
        kdl_joint = KDL::Joint(
            joint.name,
            joint_origin,
            axis_in_parent_frame,
            KDL::Joint::TransAxis,
            1.0,
            joint_offset);
        break;
    default:
        kdl_joint = KDL::Joint(KDL::Joint::Fixed);
        break;
    }

    return KDL::Segment(
        joint.child_link_name,
        kdl_joint,
        origin_frame);
}

KDL::Chain BuildKdlChainFromModel(const urdf::ModelInterfaceSharedPtr& model,
                                  const std::string& base_link,
                                  const std::string& tip_link,
                                  const std::unordered_map<std::string, double>& joint_offset_rad)
{
    KDL::Chain chain;
    if (!model) return chain;

    std::vector<urdf::JointSharedPtr> joints;
    if (!CollectUrdfJointPath(model, base_link, tip_link, joints))
        return chain;

    for (const auto& joint : joints) {
        chain.addSegment(MakeKdlSegment(*joint, joint_offset_rad));
    }

    return chain;
}

} // namespace

KDL::Chain BuildKdlChain(const nl::core::RbRobot& robot, bool include_tool_mount)
{
    const QString urdf_path = ResolveUrdfPath(robot);
    if (urdf_path.isEmpty()) return {};

    UrdfChainSpec spec;
    if (!LoadUrdfChainSpec(urdf_path, spec))
        return {};

    const std::string tip_link =
        (include_tool_mount && !spec.tool_link.empty()) ? spec.tool_link : spec.flange_link;
    return BuildKdlChainFromUrdfFile(urdf_path.toStdString(), spec.base_link, tip_link);
}

bool BuildKdlJointLimits(const nl::core::RbRobot& robot,
                         KDL::JntArray& q_min,
                         KDL::JntArray& q_max)
{
    const QString urdf_path = ResolveUrdfPath(robot);
    if (urdf_path.isEmpty()) return false;

    UrdfChainSpec spec;
    if (!LoadUrdfChainSpec(urdf_path, spec))
        return false;

    return BuildKdlJointLimitsFromUrdfFile(
        urdf_path.toStdString(), spec.base_link, spec.flange_link, q_min, q_max);
}

KDL::Chain BuildKdlChainFromUrdfFile(const std::string& urdf_path,
                                     const std::string& base_link,
                                     const std::string& tip_link)
{
    const QString urdf_path_q = QString::fromStdString(urdf_path);
    UrdfChainSpec spec;
    LoadUrdfChainSpec(urdf_path_q, spec);

    const QString xml_radians = LoadRobotUrdfRadiansXml(urdf_path_q);
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(xml_radians.toStdString());
    return BuildKdlChainFromModel(model, base_link, tip_link, spec.joint_offset_rad);
}

bool BuildKdlJointLimitsFromUrdfFile(const std::string& urdf_path,
                                     const std::string& base_link,
                                     const std::string& tip_link,
                                     KDL::JntArray& q_min,
                                     KDL::JntArray& q_max)
{
    const QString xml_radians = LoadRobotUrdfRadiansXml(QString::fromStdString(urdf_path));
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(xml_radians.toStdString());
    if (!model) return false;

    std::vector<urdf::JointSharedPtr> joints;
    if (!CollectUrdfJointPath(model, base_link, tip_link, joints))
        return false;

    return CollectUrdfLimits(joints, q_min, q_max);
}

} // namespace kinematics
} // namespace nl
