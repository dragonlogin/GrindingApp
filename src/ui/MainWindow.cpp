#include "MainWindow.h"

#include <string>
#include <vector>

#include <QDebug>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>
#include <QApplication>
#include <QSettings>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>
#include <QTreeWidget>
#include <QHeaderView>
#include <QMenu>
#include <QDomDocument>
#include <QJsonDocument>
#include <QJsonObject>

#include <AIS_Shape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>
#include <Geom_Axis2Placement.hxx>
#include <AIS_Trihedron.hxx>

#include "OcctViewWidget.h"
#include "JogPanel.h"
#include "StepImporter.h"
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

static QTreeWidgetItem* FindNode(QTreeWidgetItem* parent, const std::string& role)
{
    for (int i = 0; i < parent->childCount(); ++i) {
        auto child = parent->child(i);
        if (child->data(0, Qt::UserRole).toString().toStdString() == role)
            return child;
        if (auto found = FindNode(child, role))
            return found;
    }
    return nullptr;
}

static QTreeWidgetItem* FindNode(QTreeWidget* tree, const std::string& role)
{
    for (int i = 0; i < tree->topLevelItemCount(); ++i) {
        auto item = tree->topLevelItem(i);
        if (item->data(0, Qt::UserRole).toString().toStdString() == role)
            return item;
        if (auto found = FindNode(item, role))
            return found;
    }
    return nullptr;
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

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("GrindingApp");

    SetupMenuBar();
    SetupToolBar();
    SetupStatusBar();
    SetupCentralWidget();
    SetupJogPanel();
    SetupSceneTree();
}

void MainWindow::SwitchLanguage(const std::string& lang)
{
    QSettings settings("GrindingApp", "GrindingApp");
    settings.setValue("language", QString::fromStdString(lang));
    QMessageBox::information(this, tr("Language"),
        tr("Language changed. Please restart the application."));
}

void MainWindow::SetupMenuBar()
{
    QMenu* fileMenu = menuBar()->addMenu(tr("File(&F)"));
    fileMenu->addAction(tr("Load Robot(&R)"), this, &MainWindow::OnLoadRobot,
        QKeySequence("Ctrl+R"));
    fileMenu->addAction(tr("Load Tool(&T)"), this, &MainWindow::OnLoadTool,
        QKeySequence("Ctrl+T"));
    fileMenu->addAction(tr("Import Workpiece(&I)"), this,
        &MainWindow::OnImportWorkpiece, QKeySequence("Ctrl+O"));
    fileMenu->addSeparator();
    fileMenu->addAction(tr("Exit(&Q)"), qApp, &QApplication::quit,
        QKeySequence("Ctrl+Q"));

    QMenu* viewMenu = menuBar()->addMenu(tr("View(&V)"));
    viewMenu->addAction(tr("Front View"), this, &MainWindow::OnViewFront);
    viewMenu->addAction(tr("Top View"),   this, &MainWindow::OnViewTop);
    viewMenu->addAction(tr("Side View"),  this, &MainWindow::OnViewSide);
    viewMenu->addAction(tr("Isometric"),  this, &MainWindow::OnViewIsometric);
    viewMenu->addSeparator();
    viewMenu->addAction(tr("Wireframe"), this, &MainWindow::OnViewWireframe);
    viewMenu->addAction(tr("Shaded"),    this, &MainWindow::OnViewShaded);
    viewMenu->addSeparator();
    viewMenu->addAction(tr("Fit All"), this, &MainWindow::OnFitAll);
}

void MainWindow::SetupToolBar()
{
    QToolBar* tb = addToolBar(tr("Main Toolbar"));
    tb->setMovable(false);
    tb->addAction(tr("Import Workpiece"), this, &MainWindow::OnImportWorkpiece);
    tb->addSeparator();
    tb->addAction(tr("Front View"), this, &MainWindow::OnViewFront);
    tb->addAction(tr("Top View"),   this, &MainWindow::OnViewTop);
    tb->addAction(tr("Side View"),  this, &MainWindow::OnViewSide);
    tb->addAction(tr("Isometric"),  this, &MainWindow::OnViewIsometric);
    tb->addSeparator();
    tb->addAction(tr("Wireframe"), this, &MainWindow::OnViewWireframe);
    tb->addAction(tr("Shaded"),    this, &MainWindow::OnViewShaded);
    tb->addAction(tr("Fit All"),   this, &MainWindow::OnFitAll);
}

void MainWindow::SetupStatusBar()
{
    model_info_  = new QLabel(tr("No model loaded"));
    coord_label_ = new QLabel("X: - Y: - Z: -");

    statusBar()->addWidget(model_info_, 1);
    statusBar()->addPermanentWidget(coord_label_);
}

void MainWindow::SetupCentralWidget()
{
    viewer_ = new OcctViewWidget(this);
    setCentralWidget(viewer_);
}

void MainWindow::SetupJogPanel()
{
    jog_panel_ = new JogPanel(this);
    connect(jog_panel_, &JogPanel::JointAnglesChanged, this, [this](const nl::utils::Q& angles) {
        joint_angles_ = angles;
        UpdateRobotDisplay();
    });
    addDockWidget(Qt::RightDockWidgetArea, jog_panel_);
}

void MainWindow::UpdateRobotDisplay()
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
            gp_Trsf world = fk[idx];
            world.Multiply(local);
            local = world;
        }
        viewer_->Context()->SetLocation(m.ais, TopLoc_Location(local));
    }
    UpdateCoordinateFrames(fk);
    UpdateRobotJoints();

    if (!tool_ais_.IsNull() && !fk.empty()) {
        gp_Trsf tool_world = fk.back();
        tool_world.Multiply(tool_base_trsf_);
        viewer_->Context()->SetLocation(tool_ais_, TopLoc_Location(tool_world));

        if (!tool_tcp_frame_.IsNull())
            viewer_->Context()->Remove(tool_tcp_frame_, Standard_False);
        gp_Trsf tcp_world = fk.back();
        tcp_world.Multiply(tool_tcp_trsf_);
        tool_tcp_frame_ = MakeTrihedron(tcp_world, 40.0);
        viewer_->Context()->Display(tool_tcp_frame_, Standard_False);
    }

    viewer_->Context()->UpdateCurrentViewer();
}

void MainWindow::SetupSceneTree()
{
    auto* dock = new QDockWidget(tr("Station Manager"), this);
    scene_tree_ = new QTreeWidget;
    scene_tree_->setColumnCount(2);
    scene_tree_->setHeaderHidden(true);
    scene_tree_->header()->setSectionResizeMode(0, QHeaderView::Stretch);
    scene_tree_->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);

    auto* station = new QTreeWidgetItem({tr("Station-1"), ""});
    station->setData(0, Qt::UserRole, "station");
    scene_tree_->addTopLevelItem(station);
    station->setExpanded(true);

    scene_tree_->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(scene_tree_, &QWidget::customContextMenuRequested,
            this, &MainWindow::OnSceneTreeContextMenu);

    dock->setWidget(scene_tree_);
    addDockWidget(Qt::LeftDockWidgetArea, dock);
}

void MainWindow::AddRobot(const std::string& name)
{
    auto* station = FindNode(scene_tree_, "station");
    if (!station) return;

    auto* old = FindNode(scene_tree_, "robot");
    if (old) delete old;

    auto* robot = new QTreeWidgetItem({QString::fromStdString(name), tr("Robot")});
    robot->setData(0, Qt::UserRole, "robot");
    station->insertChild(0, robot);
    robot->setExpanded(true);

    UpdateRobotJoints();
}

void MainWindow::UpdateRobotJoints()
{
    auto* robot = FindNode(scene_tree_, "robot");
    if (!robot) return;

    for (int i = robot->childCount() - 1; i >= 0; --i) {
        if (robot->child(i)->data(0, Qt::UserRole).toString() == "joint")
            delete robot->takeChild(i);
    }

    int n = static_cast<int>(current_robot_.joints.size());
    for (int i = 0; i < n; ++i) {
        double angle = (i < 6) ? joint_angles_[i] : 0.0;
        bool is_tcp = (i == n - 1);
        QString display_name = is_tcp
            ? QString::fromStdString(current_robot_.joints[i].name) + tr(" (TCP)")
            : QString::fromStdString(current_robot_.joints[i].name);

        auto* item = new QTreeWidgetItem(
            {display_name, QString("%1°").arg(angle, 0, 'f', 1)});
        item->setData(0, Qt::UserRole, "joint");
        item->setData(0, Qt::UserRole + 1, i);
        robot->insertChild(i, item);
    }
}

void MainWindow::AddTool(const std::string& name, const std::string& parent_role)
{
    auto* parent_node = FindNode(scene_tree_, parent_role);
    if (!parent_node) return;

    auto* tool_node = new QTreeWidgetItem({QString::fromStdString(name), tr("Tool")});
    tool_node->setData(0, Qt::UserRole, "tool");
    parent_node->addChild(tool_node);
    tool_node->setExpanded(true);

    auto* tcp = new QTreeWidgetItem(
        {QString::fromStdString(name) + ".TCP0", tr("Frame")});
    tcp->setData(0, Qt::UserRole, "tcp");
    tool_node->addChild(tcp);
}

void MainWindow::AddWorkpiece(const std::string& name, const std::string& parent_role)
{
    auto* parent = FindNode(scene_tree_, parent_role);
    if (!parent) return;

    auto* wp = new QTreeWidgetItem({QString::fromStdString(name), tr("Workpiece")});
    wp->setData(0, Qt::UserRole, "workpiece");
    parent->addChild(wp);
}

void MainWindow::UpdateCoordinateFrames(const std::vector<gp_Trsf>& fk)
{
    if (!base_frame_.IsNull())
        viewer_->Context()->Remove(base_frame_, Standard_False);
    for (auto& t : joint_frames_)
        viewer_->Context()->Remove(t, Standard_False);
    joint_frames_.clear();

    base_frame_ = MakeTrihedron(gp_Trsf(), 80.0);
    viewer_->Context()->Display(base_frame_, Standard_False);

    int n = static_cast<int>(fk.size());
    for (int i = 0; i < n; ++i) {
        Handle(AIS_Trihedron) tri = MakeTrihedron(fk[i], 50.0);
        viewer_->Context()->Display(tri, Standard_False);

        if (i < n - 1)
            viewer_->Context()->Erase(tri, Standard_False);

        joint_frames_.push_back(tri);
    }
}

void MainWindow::OnSceneTreeContextMenu(const QPoint& pos)
{
    QTreeWidgetItem* item = scene_tree_->itemAt(pos);
    if (!item) return;
    if (item->data(0, Qt::UserRole).toString() != "joint") return;

    int idx = item->data(0, Qt::UserRole + 1).toInt();
    if (idx < 0 || idx >= static_cast<int>(joint_frames_.size())) return;

    Handle(AIS_Trihedron) tri = joint_frames_[idx];
    bool visible = viewer_->Context()->IsDisplayed(tri);

    QMenu menu(this);
    menu.addAction(visible ? tr("Hide Frame") : tr("Show Frame"),
                   [this, tri, visible]() {
        if (visible)
            viewer_->Context()->Erase(tri, Standard_True);
        else
            viewer_->Context()->Display(tri, Standard_True);
    });
    menu.exec(scene_tree_->mapToGlobal(pos));
}

void MainWindow::OnLoadRobot()
{
    QString path = QFileDialog::getOpenFileName(
        this, tr("Open Robot File"), "",
        tr("Robot Files (*.xml)"));
    if (path.isEmpty()) return;

    RbRobot robot = RbXmlParser::Parse(path.toStdString());
    qDebug() << "Robot:" << robot.name.c_str()
             << "joints:" << robot.joints.size()
             << "drawables:" << robot.drawables.size();

    for (auto& m : robot_meshes_)
        viewer_->Context()->Remove(m.ais, Standard_False);
    robot_meshes_.clear();

    current_robot_ = robot;
    joint_angles_ = nl::utils::Q(6, 0.0);
    if (jog_panel_) jog_panel_->SetJointAngles(joint_angles_);

    for (const RbDrawable& drw : robot.drawables) {
        TopoDS_Shape shape = StlLoader::Load(drw.mesh_file);
        if (shape.IsNull()) { qDebug() << drw.name.c_str() << "not found"; continue; }

        Handle(AIS_Shape) ais = new AIS_Shape(shape);
        viewer_->Context()->Display(ais, AIS_Shaded, 0, Standard_False);
        robot_meshes_.push_back({drw, shape, ais});
    }

    AddRobot(current_robot_.name);
    viewer_->Context()->UpdateCurrentViewer();
    UpdateRobotDisplay();
    viewer_->View()->FitAll();
}

void MainWindow::OnLoadTool()
{
    QString path = QFileDialog::getOpenFileName(
        this, tr("Load Tool"), "",
        tr("Tool Files (*.tool *.xml)"));
    if (path.isEmpty()) return;

    QString xml_path = path;
    if (path.endsWith(".tool", Qt::CaseInsensitive)) {
        QFile f(path);
        f.open(QIODevice::ReadOnly);
        QJsonDocument jdoc = QJsonDocument::fromJson(f.readAll());
        QString rw = jdoc.object().value("RWFile").toString();
        xml_path = QFileInfo(path).absoluteDir().filePath(rw);
    }

    QString base_dir = QFileInfo(xml_path).absoluteDir().absolutePath();

    QFile xf(xml_path);
    xf.open(QIODevice::ReadOnly);
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
    if (shape.IsNull()) {
        qDebug() << "Tool STL not found:" << stl_path;
        return;
    }

    if (!tool_ais_.IsNull())
        viewer_->Context()->Remove(tool_ais_, Standard_False);
    if (!tool_tcp_frame_.IsNull())
        viewer_->Context()->Remove(tool_tcp_frame_, Standard_False);

    tool_ais_ = new AIS_Shape(shape);
    viewer_->Context()->Display(tool_ais_, AIS_Shaded, 0, Standard_False);

    AddTool(QFileInfo(xml_path).baseName().toStdString(), "robot");

    viewer_->Context()->UpdateCurrentViewer();
    UpdateRobotDisplay();
}

void MainWindow::OnImportWorkpiece()
{
    QString path = QFileDialog::getOpenFileName(
        this, tr("Open STEP File"), "", tr("STEP Files (*.step *.stp)"));
    if (path.isEmpty()) return;

    int face_count = 0;
    auto shape = occ::StepImporter::Load(path.toStdString(), &face_count);
    if (shape.IsNull()) return;

    Handle(AIS_Shape) ais_shape = new AIS_Shape(shape);
    viewer_->Context()->Display(ais_shape, Standard_True);
    viewer_->View()->FitAll();

    model_info_->setText(
        tr("File: %1 | Face: %2").arg(QFileInfo(path).fileName()).arg(face_count));
}

void MainWindow::OnViewFront()
{
    viewer_->View()->SetProj(V3d_Yneg);
    viewer_->View()->FitAll();
}

void MainWindow::OnViewTop()
{
    viewer_->View()->SetProj(V3d_Zpos);
    viewer_->View()->FitAll();
}

void MainWindow::OnViewSide()
{
    viewer_->View()->SetProj(V3d_Xpos);
    viewer_->View()->FitAll();
}

void MainWindow::OnViewIsometric()
{
    viewer_->View()->SetProj(V3d_XposYnegZpos);
    viewer_->View()->FitAll();
}

void MainWindow::OnViewWireframe()
{
    viewer_->Context()->SetDisplayMode(AIS_WireFrame, Standard_True);
}

void MainWindow::OnViewShaded()
{
    viewer_->Context()->SetDisplayMode(AIS_Shaded, Standard_True);
}

void MainWindow::OnFitAll()
{
    viewer_->View()->FitAll();
}

}
}