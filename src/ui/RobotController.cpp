#include "RobotController.h"

#include <QDebug>
#include <QFileInfo>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDomDocument>

#include <Geom_Axis2Placement.hxx>

#include "StlLoader.h"
#include "RobotDisplay.h"
#include "RobotKinematics.h"

using namespace nl::occ;
using namespace nl::kinematics;
using namespace nl::core;

namespace nl {
namespace ui {

static nl::utils::Vector3d ParseXyz(const std::string& s)
{
    auto p = QString::fromStdString(s).split(' ', Qt::SkipEmptyParts);
    if (p.size() == 3)
        return {p[0].toDouble(), p[1].toDouble(), p[2].toDouble()};
    return {};
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
    joint_angles_ = nl::utils::Q(6, 0.0);

    for (const RbDrawable& drw : robot.drawables) {
        TopoDS_Shape shape = StlLoader::Load(drw.mesh_file);
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
    QString path = QString::fromStdString(path_str);
    QString xml_path = path;
    if (path.endsWith(".tool", Qt::CaseInsensitive)) {
        QFile f(path);
        if (!f.open(QIODevice::ReadOnly)) return false;
        QJsonDocument jdoc = QJsonDocument::fromJson(f.readAll());
        QString rw = jdoc.object().value("RWFile").toString();
        xml_path = QFileInfo(path).absoluteDir().filePath(rw);
    }

    QString base_dir = QFileInfo(xml_path).absoluteDir().absolutePath();

    QFile xf(xml_path);
    if (!xf.open(QIODevice::ReadOnly)) return false;
    QDomDocument doc;
    doc.setContent(&xf);
    QDomElement root = doc.documentElement();

    QString stl_rel  = root.elementsByTagName("Polytope").at(0)
                           .toElement().attribute("file");
    QString stl_path = QDir(base_dir).filePath(stl_rel);

    QDomElement base_frame_el = root.elementsByTagName("Frame").at(0).toElement();
    nl::utils::Vector3d base_pos = ParseXyz(base_frame_el.firstChildElement("Pos").text().toStdString());
    nl::utils::Vector3d base_rpy = ParseXyz(base_frame_el.firstChildElement("RPY").text().toStdString());
    tool_base_trsf_ = RpyPosTrsf(base_rpy, base_pos);

    nl::utils::Vector3d tcp_pos, tcp_rpy;
    QDomNodeList frames = root.elementsByTagName("Frame");
    for (int i = 0; i < frames.count(); ++i) {
        QDomElement fe = frames.at(i).toElement();
        if (fe.attribute("type") == "EndEffector") {
            tcp_pos = ParseXyz(fe.firstChildElement("Pos").text().toStdString());
            tcp_rpy = ParseXyz(fe.firstChildElement("RPY").text().toStdString());
            break;
        }
    }
    tool_tcp_trsf_ = RpyPosTrsf(tcp_rpy, tcp_pos);

    TopoDS_Shape shape = StlLoader::Load(stl_path.toStdString());
    if (shape.IsNull()) return false;

    if (!tool_ais_.IsNull())
        context_->Remove(tool_ais_, Standard_False);
    if (!tool_tcp_frame_.IsNull())
        context_->Remove(tool_tcp_frame_, Standard_False);

    tool_ais_ = new AIS_Shape(shape);
    context_->Display(tool_ais_, AIS_Shaded, 0, Standard_False);

    emit ToolLoaded(QFileInfo(xml_path).baseName());

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

    std::vector<gp_Trsf> fk = ComputeFk(current_robot_, joint_angles_);

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
        tool_tcp_frame_ = MakeTrihedron(tcp_world, 40.0);
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

    base_frame_ = MakeTrihedron(base_trsf_, 80.0);
    context_->Display(base_frame_, Standard_False);

    int n = static_cast<int>(fk.size());
    for (int i = 0; i < n; ++i) {
        gp_Trsf joint_world = base_trsf_;
        joint_world.Multiply(fk[i]);
        Handle(AIS_Trihedron) tri = MakeTrihedron(joint_world, 50.0);
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

    if (!ComputeIkAllSolutions(current_robot_, target, joint_angles_, out_solutions)) {
        return false;
    }

    if (!out_solutions.empty()) {
        SetJointAngles(out_solutions[0]);
    }
    return true;
}

std::vector<gp_Trsf> RobotController::GetCurrentFk() const
{
    if (robot_meshes_.empty()) return {};
    return ComputeFk(current_robot_, joint_angles_);
}

} // namespace ui
} // namespace nl
