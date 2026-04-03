#include "RobotController.h"

#include <QDebug>
#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QDomDocument>
#include <QSet>

#include <Geom_Axis2Placement.hxx>
#include <BRepBuilderAPI_GTransform.hxx>
#include <gp_GTrsf.hxx>

#include "MeshLoader.h"
#include "RobotDisplay.h"
#include "KdlSolver.h"
#include "KdlKinematicsService.h"
#include "domain/Robot.h"
#include "foundation/Conversions.h"

using namespace nl::occ;
using namespace nl::core;

namespace nl {
namespace ui {

namespace {

// 临时转换：Phase 6/7 统一迁移后删除
domain::Robot ToDomainRobot(const nl::core::RbRobot& rb)
{
    domain::Robot r;
    r.name = rb.name;
    r.source_path = rb.source_path;
    for (const auto& j : rb.joints) {
        domain::RobotJoint dj;
        dj.name = j.name;
        dj.alpha_deg = j.alpha_deg;
        dj.a_mm = j.a;
        dj.d_mm = j.d;
        dj.offset_deg = j.offset_deg;
        r.joints.push_back(dj);
    }
    return r;
}

constexpr double kTcpFrameSizeM = 0.04;
constexpr double kBaseFrameSizeM = 0.08;
constexpr double kJointFrameSizeM = 0.05;

} // namespace

static nl::utils::Vector3d ParseXyz(const std::string& s)
{
    auto p = QString::fromStdString(s).split(' ', Qt::SkipEmptyParts);
    if (p.size() == 3)
        return {p[0].toDouble(), p[1].toDouble(), p[2].toDouble()};
    return {};
}

static nl::utils::Vector3d ParseUrdfRpyDegrees(const QString& s)
{
    const auto parts = s.split(' ', Qt::SkipEmptyParts);
    if (parts.size() != 3)
        return {};

    const double roll_deg = parts[0].toDouble();
    const double pitch_deg = parts[1].toDouble();
    const double yaw_deg = parts[2].toDouble();
    // RobotDisplay::RpyPosTrsf expects yaw, pitch, roll in degrees.
    return {yaw_deg, pitch_deg, roll_deg};
}

static nl::utils::Vector3d ParseScaleTriple(const QString& s)
{
    const auto parts = s.split(' ', Qt::SkipEmptyParts);
    if (parts.size() != 3)
        return {1.0, 1.0, 1.0};
    return {parts[0].toDouble(), parts[1].toDouble(), parts[2].toDouble()};
}

static QString ResolveRelativePath(const QString& base_dir, const QString& relative_path)
{
    return QDir(base_dir).filePath(relative_path);
}

static QString FindUrdfRootLink(const QDomElement& robot_el)
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

static QDomElement FindUrdfLinkByName(const QDomElement& robot_el, const QString& link_name)
{
    const QDomNodeList link_nodes = robot_el.elementsByTagName("link");
    for (int i = 0; i < link_nodes.count(); ++i) {
        const QDomElement link_el = link_nodes.at(i).toElement();
        if (link_el.attribute("name") == link_name)
            return link_el;
    }
    return {};
}

static bool ParseToolUrdf(const QString& urdf_path,
                          QString& mesh_path,
                          nl::utils::Vector3d& mesh_scale,
                          nl::utils::Vector3d& tool_base_rpy_deg,
                          nl::utils::Vector3d& tool_base_pos,
                          nl::utils::Vector3d& tcp_rpy_deg,
                          nl::utils::Vector3d& tcp_pos)
{
    QFile file(urdf_path);
    if (!file.open(QIODevice::ReadOnly)) return false;

    QDomDocument doc;
    QString err_msg;
    int err_line = 0;
    int err_col = 0;
    if (!doc.setContent(file.readAll(), &err_msg, &err_line, &err_col))
        return false;

    const QDomElement robot_el = doc.documentElement();
    const QString base_dir = QFileInfo(urdf_path).absoluteDir().absolutePath();
    const QString root_link_name = FindUrdfRootLink(robot_el);
    if (root_link_name.isEmpty()) return false;

    const QDomElement root_link_el = FindUrdfLinkByName(robot_el, root_link_name);
    if (root_link_el.isNull()) return false;

    const QDomElement visual_el = root_link_el.firstChildElement("visual");
    const QDomElement origin_el = visual_el.firstChildElement("origin");
    const QDomElement mesh_el =
        visual_el.firstChildElement("geometry").firstChildElement("mesh");
    if (mesh_el.isNull()) return false;

    mesh_path = ResolveRelativePath(base_dir, mesh_el.attribute("filename"));
    mesh_scale = ParseScaleTriple(mesh_el.attribute("scale"));
    tool_base_pos = ParseXyz(origin_el.attribute("xyz").toStdString());
    // Tool URDF files follow the same authoring convention as robot URDF:
    // RPY values are written in degrees and converted only when building the
    // OCC transforms used at runtime.
    tool_base_rpy_deg = ParseUrdfRpyDegrees(origin_el.attribute("rpy"));

    const QDomNodeList joint_nodes = robot_el.elementsByTagName("joint");
    for (int i = 0; i < joint_nodes.count(); ++i) {
        const QDomElement joint_el = joint_nodes.at(i).toElement();
        const QString child_link = joint_el.firstChildElement("child").attribute("link");
        if (child_link != QStringLiteral("tool0") && child_link != QStringLiteral("tcp0"))
            continue;

        const QDomElement tcp_origin_el = joint_el.firstChildElement("origin");
        tcp_pos = ParseXyz(tcp_origin_el.attribute("xyz").toStdString());
        tcp_rpy_deg = ParseUrdfRpyDegrees(tcp_origin_el.attribute("rpy"));
        return true;
    }

    tcp_pos = {};
    tcp_rpy_deg = {};
    return true;
}

static Handle(AIS_Trihedron) MakeTrihedron(const gp_Trsf& trsf, double size)
{
    gp_Pnt origin(trsf.TranslationPart());
    gp_Mat rot = trsf.VectorialPart();
    gp_Dir z_dir(rot.Column(3));
    gp_Dir x_dir(rot.Column(1));
    Handle(Geom_Axis2Placement) pl =
        new Geom_Axis2Placement(gp_Ax2(origin, z_dir, x_dir));
    Handle(AIS_Trihedron) tri = new AIS_Trihedron(pl);
    tri->SetSize(size);
    return tri;
}

RobotController::RobotController(Handle(AIS_InteractiveContext) context, QObject* parent)
    : QObject(parent), context_(context)
{
}

bool RobotController::LoadRobot(const std::string& xml_path)
{
    RbRobot robot = RbXmlParser::Parse(xml_path);
    if (robot.name.empty() || robot.joints.empty()) return false;

    for (auto& m : robot_meshes_)
        context_->Remove(m.ais, Standard_False);
    robot_meshes_.clear();

    current_robot_ = robot;
    joint_angles_ = nl::utils::Q(static_cast<int>(current_robot_.joints.size()), 0.0);

    for (const RbDrawable& drw : robot.drawables) {
        TopoDS_Shape shape = MeshLoader::Load(drw.mesh_file);
        if (shape.IsNull()) continue;

        Handle(AIS_Shape) ais = new AIS_Shape(shape);
        context_->Display(ais, AIS_Shaded, 0, Standard_False);
        robot_meshes_.push_back({drw, shape, ais});
    }

    emit RobotLoaded(QString::fromStdString(current_robot_.name));
    
    UpdateRobotDisplay();
    return true;
}

bool RobotController::LoadTool(const std::string& path_str)
{
    QString mesh_path;
    nl::utils::Vector3d mesh_scale;
    nl::utils::Vector3d base_pos;
    nl::utils::Vector3d base_rpy;
    nl::utils::Vector3d tcp_pos;
    nl::utils::Vector3d tcp_rpy;
    const QString urdf_path = QString::fromStdString(path_str);
    if (!ParseToolUrdf(urdf_path, mesh_path, mesh_scale, base_rpy, base_pos, tcp_rpy, tcp_pos))
        return false;

    tool_base_trsf_ = RpyPosTrsf(base_rpy, base_pos);
    tool_tcp_trsf_ = RpyPosTrsf(tcp_rpy, tcp_pos);

    TopoDS_Shape shape = MeshLoader::Load(mesh_path.toStdString());
    if (shape.IsNull()) return false;

    if (std::abs(mesh_scale[0] - 1.0) > 1e-12 ||
        std::abs(mesh_scale[1] - 1.0) > 1e-12 ||
        std::abs(mesh_scale[2] - 1.0) > 1e-12) {
        gp_GTrsf scale_trsf;
        scale_trsf.SetValue(1, 1, mesh_scale[0]);
        scale_trsf.SetValue(2, 2, mesh_scale[1]);
        scale_trsf.SetValue(3, 3, mesh_scale[2]);
        shape = BRepBuilderAPI_GTransform(shape, scale_trsf, Standard_True).Shape();
    }

    if (!tool_ais_.IsNull())
        context_->Remove(tool_ais_, Standard_False);
    if (!tool_tcp_frame_.IsNull())
        context_->Remove(tool_tcp_frame_, Standard_False);

    tool_ais_ = new AIS_Shape(shape);
    context_->Display(tool_ais_, AIS_Shaded, 0, Standard_False);

    emit ToolLoaded(QFileInfo(urdf_path).baseName());

    UpdateRobotDisplay();
    return true;
}

void RobotController::SetJointAngles(const nl::utils::Q& angles)
{
    joint_angles_ = angles;
    emit JointAnglesChanged(angles);
    UpdateRobotDisplay();
}

void RobotController::SetBaseTrsf(const gp_Trsf& trsf)
{
    base_trsf_ = trsf;
    UpdateRobotDisplay();
    emit BaseTrsfChanged(trsf);
}

void RobotController::UpdateRobotDisplay()
{
    if (robot_meshes_.empty()) return;

    nl::kinematics::KdlSolver kin_solver;
    std::vector<gp_Trsf> fk = kin_solver.ComputeFk(ToDomainRobot(current_robot_), joint_angles_);

    auto joint_idx = [&](const std::string& name) -> int {
        for (int i = 0; i < static_cast<int>(current_robot_.joints.size()); ++i)
            if (current_robot_.joints[i].name == name) return i;
        return -1;
    };

    for (auto& m : robot_meshes_) {
        gp_Trsf local = RpyPosTrsf(m.drawable.rpy, m.drawable.pos);
        int idx = joint_idx(m.drawable.ref_joint);
        if (idx >= 0) {
            gp_Trsf world = base_trsf_;
            world.Multiply(fk[idx]);
            world.Multiply(local);
            local = world;
        }
        else {
            gp_Trsf world = base_trsf_;
            world.Multiply(local);
            local = world;
        }
        context_->SetLocation(m.ais, TopLoc_Location(local));
    }
    UpdateCoordinateFrames(fk);

    if (!tool_ais_.IsNull() && !fk.empty()) {
        gp_Trsf tool_world = base_trsf_;
        tool_world.Multiply(fk.back());
        tool_world.Multiply(tool_base_trsf_);
        context_->SetLocation(tool_ais_, TopLoc_Location(tool_world));

        if (!tool_tcp_frame_.IsNull())
            context_->Remove(tool_tcp_frame_, Standard_False);
        gp_Trsf tcp_world = base_trsf_;
        tcp_world.Multiply(fk.back());
        tcp_world.Multiply(tool_tcp_trsf_);
        tool_tcp_frame_ = MakeTrihedron(tcp_world, kTcpFrameSizeM);
        context_->Display(tool_tcp_frame_, Standard_False);
    }

    emit DisplayUpdated();
}

void RobotController::UpdateCoordinateFrames(const std::vector<gp_Trsf>& fk)
{
    if (!base_frame_.IsNull())
        context_->Remove(base_frame_, Standard_False);
    for (auto& t : joint_frames_)
        context_->Remove(t, Standard_False);
    joint_frames_.clear();

    base_frame_ = MakeTrihedron(base_trsf_, kBaseFrameSizeM);
    context_->Display(base_frame_, Standard_False);

    int n = static_cast<int>(fk.size());
    for (int i = 0; i < n; ++i) {
        gp_Trsf joint_world = base_trsf_;
        joint_world.Multiply(fk[i]);
        Handle(AIS_Trihedron) tri = MakeTrihedron(joint_world, kJointFrameSizeM);
        context_->Display(tri, Standard_False);

        if (i < n - 1)
            context_->Erase(tri, Standard_False);

        joint_frames_.push_back(tri);
    }
}

void RobotController::ToggleJointFrame(int index)
{
    if (index < 0 || index >= static_cast<int>(joint_frames_.size())) return;
    Handle(AIS_Trihedron) tri = joint_frames_[index];
    if (context_->IsDisplayed(tri))
        context_->Erase(tri, Standard_True);
    else
        context_->Display(tri, Standard_True);
}

bool RobotController::SetTcpPose(const gp_Trsf& target_pose, int tcp_ref_mode, std::vector<nl::utils::Q>& out_solutions)
{
    gp_Trsf target = target_pose;
    // If Tool TCP mode, compute flange target = target * tool_tcp^-1
    if (tcp_ref_mode == 1) {
        gp_Trsf tcp_inv = tool_tcp_trsf_.Inverted();
        target.Multiply(tcp_inv);
    }

    {
        planning::KdlKinematicsService kin_svc;
        auto ik_result = kin_svc.ComputeIkAll(
            ToDomainRobot(current_robot_),
            foundation::ToPose(target),
            foundation::ToJointState(joint_angles_));
        if (!ik_result) return false;
        for (const auto& js : ik_result.value())
            out_solutions.push_back(foundation::ToQ(js));
    }

    if (!out_solutions.empty()) {
        SetJointAngles(out_solutions[0]);
    }
    return true;
}

std::vector<gp_Trsf> RobotController::GetCurrentFk() const
{
    if (robot_meshes_.empty()) return {};
    nl::kinematics::KdlSolver fk_solver;
    return fk_solver.ComputeFk(ToDomainRobot(current_robot_), joint_angles_);
}

} // namespace ui
} // namespace nl
