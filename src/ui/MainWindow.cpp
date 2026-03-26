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

#include "OcctViewWidget.h"
#include "JogPanel.h"
#include "RobotController.h"
#include "Commands.h"
#include "StepImporter.h"
#include "SurfaceWaypointGen.h"
#include "RobotKinematics.h"
#include "RobotDisplay.h"

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <Quantity_Color.hxx>

using namespace nl::occ;

namespace nl {
namespace ui {

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

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("GrindingApp");

    undo_stack_ = new QUndoStack(this);

    SetupMenuBar();
    SetupToolBar();
    SetupStatusBar();
    SetupCentralWidget();
    
    // Initialize controller after central widget is ready
    controller_ = new RobotController(viewer_->Context(), this);
    connect(controller_, &RobotController::RobotLoaded, this, &MainWindow::AddRobot);
    connect(controller_, &RobotController::ToolLoaded, this, [this](const QString& name) {
        AddTool(name, "robot");
    });
    connect(controller_, &RobotController::DisplayUpdated, this, [this]() {
        viewer_->Context()->UpdateCurrentViewer();
        UpdateTcpPoseDisplay();
        UpdateRobotJoints();
    });
    connect(controller_, &RobotController::JointAnglesChanged, this, &MainWindow::OnControllerJointAnglesChanged);

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
    fileMenu->addAction(tr("Generate Waypoints(&G)"), this,
        &MainWindow::OnGenerateWaypoints, QKeySequence("Ctrl+G"));
    fileMenu->addSeparator();
    fileMenu->addAction(tr("Exit(&Q)"), qApp, &QApplication::quit,
        QKeySequence("Ctrl+Q"));

    QMenu* editMenu = menuBar()->addMenu(tr("Edit(&E)"));
    QAction* undoAction = undo_stack_->createUndoAction(this, tr("Undo(&U)"));
    undoAction->setShortcut(QKeySequence::Undo);
    editMenu->addAction(undoAction);
    
    QAction* redoAction = undo_stack_->createRedoAction(this, tr("Redo(&R)"));
    redoAction->setShortcut(QKeySequence::Redo);
    editMenu->addAction(redoAction);

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
        // Create an undo command instead of directly setting angles
        nl::utils::Q old_angles = controller_->GetJointAngles();
        // If the angles are the same, don't push a command
        bool changed = false;
        for (int i = 0; i < angles.size() && i < old_angles.size(); ++i) {
            if (std::abs(angles[i] - old_angles[i]) > 1e-4) {
                changed = true;
                break;
            }
        }
        if (changed) {
            undo_stack_->push(new CmdJogJoints(controller_, old_angles, angles));
        }
    });
    connect(jog_panel_, &JogPanel::PoseEdited,
            this, &MainWindow::OnPoseEdited);
    connect(jog_panel_, &JogPanel::IkSolutionSelected,
            this, &MainWindow::OnIkSolutionSelected);
    connect(jog_panel_, &JogPanel::ReferenceFrameChanged,
            this, &MainWindow::OnReferenceFrameChanged);
    addDockWidget(Qt::RightDockWidgetArea, jog_panel_);
}

void MainWindow::OnControllerJointAnglesChanged(const nl::utils::Q& angles)
{
    if (jog_panel_) {
        jog_panel_->SetJointAngles(angles);
    }
}

void MainWindow::UpdateTcpPoseDisplay()
{
    std::vector<gp_Trsf> fk = controller_->GetCurrentFk();
    if (fk.empty()) return;

    gp_Trsf pose_trsf = fk.back();
    if (tcp_ref_mode_ == 1)
        pose_trsf.Multiply(controller_->GetToolTcpTrsf());

    nl::utils::Vector3d rpy, pos;
    // We need TrsfToRpyPos logic, which was previously in RobotKinematics or MainWindow
    // To avoid duplication, let's use the occ::TrsfToRpyPos from StlLoader/RobotDisplay if available,
    // or we can implement a local helper since it was removed.
    // Wait, earlier it was in MainWindow, let's check where it came from.
    // I will temporarily add a dummy or re-implement it if it's missing, but it should be in RobotKinematics.
    nl::occ::TrsfToRpyPos(pose_trsf, rpy, pos);
    jog_panel_->SetTcpPose(pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]);
}

void MainWindow::OnPoseEdited(double x, double y, double z,
                               double rx, double ry, double rz)
{
    nl::utils::Vector3d rpy(rx, ry, rz);
    nl::utils::Vector3d pos(x, y, z);
    gp_Trsf target = RpyPosTrsf(rpy, pos);

    std::vector<nl::utils::Q> solutions;
    // We can use controller's helper if we adjust it, or calculate here:
    // If Tool TCP mode, compute flange target = target * tool_tcp^-1
    if (tcp_ref_mode_ == 1) {
        gp_Trsf tcp_inv = controller_->GetToolTcpTrsf().Inverted();
        target.Multiply(tcp_inv);
    }

    if (!nl::kinematics::ComputeIkAllSolutions(controller_->GetRobot(), target, controller_->GetJointAngles(), solutions)) {
        statusBar()->showMessage(tr("IK: No solution found"), 3000);
        return;
    }

    current_ik_solutions_ = solutions;
    jog_panel_->SetIkSolutions(solutions);

    undo_stack_->push(new CmdJogJoints(controller_, controller_->GetJointAngles(), solutions[0]));
}

void MainWindow::OnIkSolutionSelected(int index)
{
    if (index < 0 || index >= static_cast<int>(current_ik_solutions_.size()))
        return;
    undo_stack_->push(new CmdJogJoints(controller_, controller_->GetJointAngles(), current_ik_solutions_[index]));
}

void MainWindow::OnReferenceFrameChanged(int index)
{
    tcp_ref_mode_ = index;
    UpdateTcpPoseDisplay();
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

void MainWindow::AddRobot(const QString& name)
{
    auto* station = FindNode(scene_tree_, "station");
    if (!station) return;

    auto* old = FindNode(scene_tree_, "robot");
    if (old) delete old;

    auto* robot = new QTreeWidgetItem({name, tr("Robot")});
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

    const auto& current_robot = controller_->GetRobot();
    const auto& joint_angles = controller_->GetJointAngles();

    int n = static_cast<int>(current_robot.joints.size());
    for (int i = 0; i < n; ++i) {
        double angle = (i < 6) ? joint_angles[i] : 0.0;
        bool is_tcp = (i == n - 1);
        QString display_name = is_tcp
            ? QString::fromStdString(current_robot.joints[i].name) + tr(" (TCP)")
            : QString::fromStdString(current_robot.joints[i].name);

        auto* item = new QTreeWidgetItem(
            {display_name, QString("%1°").arg(angle, 0, 'f', 1)});
        item->setData(0, Qt::UserRole, "joint");
        item->setData(0, Qt::UserRole + 1, i);
        robot->insertChild(i, item);
    }
}

void MainWindow::AddTool(const QString& name, const std::string& parent_role)
{
    auto* parent_node = FindNode(scene_tree_, parent_role);
    if (!parent_node) return;

    auto* tool_node = new QTreeWidgetItem({name, tr("Tool")});
    tool_node->setData(0, Qt::UserRole, "tool");
    parent_node->addChild(tool_node);
    tool_node->setExpanded(true);

    auto* tcp = new QTreeWidgetItem(
        {name + ".TCP0", tr("Frame")});
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

void MainWindow::OnSceneTreeContextMenu(const QPoint& pos)
{
    QTreeWidgetItem* item = scene_tree_->itemAt(pos);
    if (!item) return;

    QString role = item->data(0, Qt::UserRole).toString();
    QMenu menu(this);

    if (role == "joint") {
        int idx = item->data(0, Qt::UserRole + 1).toInt();
        menu.addAction(tr("Toggle Frame Visibility"), [this, idx]() {
            controller_->ToggleJointFrame(idx);
        });
    } else if (role == "workpiece") {
        menu.addAction(tr("Generate Waypoints"), this, &MainWindow::OnGenerateWaypoints);
    }

    if (!menu.isEmpty())
        menu.exec(scene_tree_->mapToGlobal(pos));
}

void MainWindow::OnLoadRobot()
{
    QString path = QFileDialog::getOpenFileName(
        this, tr("Open Robot File"), "",
        tr("Robot Files (*.xml)"));
    if (path.isEmpty()) return;

    if (controller_->LoadRobot(path.toStdString())) {
        undo_stack_->clear(); // clear history for new robot
        viewer_->View()->FitAll();
    } else {
        QMessageBox::warning(this, tr("Error"), tr("Failed to load robot file."));
    }
}

void MainWindow::OnLoadTool()
{
    QString path = QFileDialog::getOpenFileName(
        this, tr("Load Tool"), "",
        tr("Tool Files (*.tool *.xml)"));
    if (path.isEmpty()) return;

    if (!controller_->LoadTool(path.toStdString())) {
        QMessageBox::warning(this, tr("Error"), tr("Failed to load tool."));
    }
}

void MainWindow::OnImportWorkpiece()
{
    QString path = QFileDialog::getOpenFileName(
        this, tr("Open STEP File"), "", tr("STEP Files (*.step *.stp)"));
    if (path.isEmpty()) return;

    int face_count = 0;
    TopoDS_Shape shape = occ::StepImporter::Load(path.toStdString(), &face_count);
    if (shape.IsNull()) return;

    // 移除旧工件和路径点
    if (!workpiece_ais_.IsNull()) {
        viewer_->Context()->Remove(workpiece_ais_, Standard_False);
        workpiece_ais_.Nullify();
    }
    if (!waypoints_ais_.IsNull()) {
        viewer_->Context()->Remove(waypoints_ais_, Standard_False);
        waypoints_ais_.Nullify();
        waypoints_.clear();
    }

    workpiece_shape_ = shape;
    workpiece_ais_   = new AIS_Shape(shape);
    viewer_->Context()->Display(workpiece_ais_, AIS_Shaded, 0, Standard_False);
    viewer_->Context()->UpdateCurrentViewer();
    viewer_->View()->FitAll();

    std::string file_name = QFileInfo(path).baseName().toStdString();
    AddWorkpiece(file_name, "station");

    model_info_->setText(
        tr("File: %1 | Face: %2").arg(QFileInfo(path).fileName()).arg(face_count));
}

void MainWindow::OnGenerateWaypoints()
{
    if (workpiece_shape_.IsNull()) {
        statusBar()->showMessage(tr("No workpiece loaded"), 3000);
        return;
    }

    // 移除旧路径点显示
    if (!waypoints_ais_.IsNull()) {
        viewer_->Context()->Remove(waypoints_ais_, Standard_False);
        waypoints_ais_.Nullify();
        waypoints_.clear();
    }

    // 第一版：取面积最大的 Face，10x5 网格，贴面
    TopoDS_Face face = occ::LargestFace(workpiece_shape_);
    if (face.IsNull()) {
        statusBar()->showMessage(tr("No valid face found"), 3000);
        return;
    }

    waypoints_ = occ::GenerateGridWaypoints(face, 10, 5, 0.0);
    if (waypoints_.empty()) {
        statusBar()->showMessage(tr("Waypoint generation failed"), 3000);
        return;
    }

    // 将路径点位置连成折线并显示
    BRepBuilderAPI_MakePolygon poly;
    for (const auto& wp : waypoints_) {
        const gp_XYZ& t = wp.pose.TranslationPart();
        poly.Add(gp_Pnt(t.X(), t.Y(), t.Z()));
    }
    if (poly.IsDone()) {
        waypoints_ais_ = new AIS_Shape(poly.Shape());
        waypoints_ais_->SetColor(Quantity_NOC_YELLOW);
        waypoints_ais_->SetWidth(2.0);
        viewer_->Context()->Display(waypoints_ais_, AIS_WireFrame, 0, Standard_False);
        viewer_->Context()->UpdateCurrentViewer();
    }

    statusBar()->showMessage(
        tr("Generated %1 waypoints on largest face").arg(static_cast<int>(waypoints_.size())),
        5000);
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