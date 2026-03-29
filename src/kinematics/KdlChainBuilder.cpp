#include "KdlChainBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
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

struct UrdfJoint {
    std::string name;
    std::string type;
    nl::utils::Vector3d xyz;
    nl::utils::Vector3d rpy_rad;
    nl::utils::Vector3d axis;
    double offset_rad = 0.0;
    double lower_rad = -std::numeric_limits<double>::infinity();
    double upper_rad = std::numeric_limits<double>::infinity();
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

nl::utils::Vector3d ParseTriple(const QString& text)
{
    nl::utils::Vector3d out;
    const auto parts = text.split(' ', Qt::SkipEmptyParts);
    if (parts.size() == 3) {
        out[0] = parts[0].toDouble();
        out[1] = parts[1].toDouble();
        out[2] = parts[2].toDouble();
    }
    return out;
}

bool LoadUrdfJoints(const nl::core::RbRobot& robot, std::vector<UrdfJoint>& joints)
{
    joints.clear();
    const QString urdf_path = ResolveUrdfPath(robot);
    if (urdf_path.isEmpty()) return false;

    QFile file(urdf_path);
    if (!file.open(QIODevice::ReadOnly)) return false;

    QDomDocument doc;
    QString err_msg;
    int err_line = 0;
    int err_col = 0;
    if (!doc.setContent(file.readAll(), &err_msg, &err_line, &err_col))
        return false;

    const QDomNodeList joint_nodes = doc.documentElement().elementsByTagName("joint");
    int movable_index = 0;
    for (int i = 0; i < joint_nodes.count(); ++i) {
        const QDomElement el = joint_nodes.at(i).toElement();
        UrdfJoint joint;
        joint.name = el.attribute("name").toStdString();
        joint.type = el.attribute("type").toStdString();

        const QDomElement origin_el = el.firstChildElement("origin");
        if (!origin_el.isNull()) {
            joint.xyz = ParseTriple(origin_el.attribute("xyz"));
            joint.rpy_rad = ParseTriple(origin_el.attribute("rpy"));
        }

        const QDomElement axis_el = el.firstChildElement("axis");
        if (!axis_el.isNull())
            joint.axis = ParseTriple(axis_el.attribute("xyz"));
        else
            joint.axis = {0.0, 0.0, 1.0};

        if (joint.type != "fixed" &&
            movable_index < static_cast<int>(robot.joints.size())) {
            joint.offset_rad = robot.joints[movable_index].offset_deg * kDeg;
            ++movable_index;
        }

        const QDomElement limit_el = el.firstChildElement("limit");
        if (!limit_el.isNull()) {
            joint.lower_rad = limit_el.attribute("lower").toDouble();
            joint.upper_rad = limit_el.attribute("upper").toDouble();
        }

        joints.push_back(joint);
    }

    return !joints.empty();
}

KDL::Frame MakeOriginFrame(const UrdfJoint& joint)
{
    KDL::Frame frame(
        KDL::Rotation::RPY(joint.rpy_rad[0], joint.rpy_rad[1], joint.rpy_rad[2]),
        KDL::Vector(joint.xyz[0], joint.xyz[1], joint.xyz[2]));
    if (std::abs(joint.offset_rad) > 1e-12) {
        frame = frame * KDL::Frame(
            KDL::Rotation::Rot(
                KDL::Vector(joint.axis[0], joint.axis[1], joint.axis[2]),
                joint.offset_rad));
    }
    return frame;
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

KDL::Joint MakeKdlJoint(const urdf::Joint& joint)
{
    const KDL::Vector origin(0.0, 0.0, 0.0);
    const KDL::Vector axis(joint.axis.x, joint.axis.y, joint.axis.z);

    switch (joint.type) {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS:
        return KDL::Joint(joint.name, origin, axis, KDL::Joint::RotAxis);
    case urdf::Joint::PRISMATIC:
        return KDL::Joint(joint.name, origin, axis, KDL::Joint::TransAxis);
    case urdf::Joint::FIXED:
        return KDL::Joint(KDL::Joint::Fixed);
    default:
        return KDL::Joint(KDL::Joint::Fixed);
    }
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

} // namespace

KDL::Chain BuildKdlChain(const nl::core::RbRobot& robot, bool include_tool_mount)
{
    KDL::Chain chain;
    std::vector<UrdfJoint> urdf_joints;
    if (!LoadUrdfJoints(robot, urdf_joints))
        return chain;

    for (const UrdfJoint& joint : urdf_joints) {
        if (joint.type == "fixed") {
            if (!include_tool_mount && joint.name == "tool_mount")
                continue;

            chain.addSegment(KDL::Segment(
                joint.name,
                KDL::Joint(KDL::Joint::Fixed),
                MakeOriginFrame(joint)));
            continue;
        }

        chain.addSegment(KDL::Segment(
            joint.name + "_origin",
            KDL::Joint(KDL::Joint::Fixed),
            MakeOriginFrame(joint)));

        chain.addSegment(KDL::Segment(
            joint.name,
            KDL::Joint(
                joint.name,
                KDL::Vector(0.0, 0.0, 0.0),
                KDL::Vector(joint.axis[0], joint.axis[1], joint.axis[2]),
                KDL::Joint::RotAxis),
            KDL::Frame::Identity()));
    }

    return chain;
}

bool BuildKdlJointLimits(const nl::core::RbRobot& robot,
                         KDL::JntArray& q_min,
                         KDL::JntArray& q_max)
{
    std::vector<UrdfJoint> urdf_joints;
    if (!LoadUrdfJoints(robot, urdf_joints))
        return false;

    int movable_count = 0;
    for (const UrdfJoint& joint : urdf_joints) {
        if (joint.type != "fixed") ++movable_count;
    }

    q_min.resize(movable_count);
    q_max.resize(movable_count);

    int idx = 0;
    for (const UrdfJoint& joint : urdf_joints) {
        if (joint.type == "fixed") continue;
        q_min(idx) = joint.lower_rad;
        q_max(idx) = joint.upper_rad;
        ++idx;
    }

    return true;
}

KDL::Chain BuildKdlChainFromUrdfFile(const std::string& urdf_path,
                                     const std::string& base_link,
                                     const std::string& tip_link)
{
    KDL::Chain chain;
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_path);
    if (!model) return chain;

    std::vector<urdf::JointSharedPtr> joints;
    if (!CollectUrdfJointPath(model, base_link, tip_link, joints))
        return chain;

    for (const auto& joint : joints) {
        chain.addSegment(KDL::Segment(
            joint->child_link_name,
            MakeKdlJoint(*joint),
            UrdfPoseToKdlFrame(joint->parent_to_joint_origin_transform)));
    }

    return chain;
}

bool BuildKdlJointLimitsFromUrdfFile(const std::string& urdf_path,
                                     const std::string& base_link,
                                     const std::string& tip_link,
                                     KDL::JntArray& q_min,
                                     KDL::JntArray& q_max)
{
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_path);
    if (!model) return false;

    std::vector<urdf::JointSharedPtr> joints;
    if (!CollectUrdfJointPath(model, base_link, tip_link, joints))
        return false;

    return CollectUrdfLimits(joints, q_min, q_max);
}

} // namespace kinematics
} // namespace nl
