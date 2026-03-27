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
#include <StdSelect_BRepOwner.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <QActionGroup>

#include "OcctViewWidget.h"
#include "JogPanel.h"
#include "RobotController.h"
#include "Commands.h"
#include "StepImporter.h"
#include "SurfaceWaypointGen.h"
#include "WaypointGenerator.h"
#include "WaypointGridAlgo.h"
#include "WaypointPlanarAlgo.h"
#include "RobotKinematics.h"
#include "RobotDisplay.h"
#include "TrajectoryPanel.h"
#include "TrajectoryPlayer.h"
#include "MovementPanel.h"

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <Quantity_Color.hxx>

using nl::occ::LargestFace;
using nl::occ::StepImporter;
using nl::occ::TrsfToRpyPos;
using nl::occ::RpyPosTrsf;

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
    SetupWaypointMenu();
    SetupTrajectoryPanel();
    SetupTrajectoryPlayer();

    connect(viewer_, &OcctViewWidget::ShapeSelected,
            this, &MainWindow::OnFacePicked);
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

    gp_Trsf pose_trsf = controller_->GetBaseTrsf();
    pose_trsf.Multiply(fk.back());
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

    // We can use controller's helper if we adjust it, or calculate here:
    // If Tool TCP mode, compute flange target = target * tool_tcp^-1
    if (tcp_ref_mode_ == 1) {
        gp_Trsf tcp_inv = controller_->GetToolTcpTrsf().Inverted();
        target.Multiply(tcp_inv);
    }

    // 世界坐标 → 机器人 base 坐标
    gp_Trsf base_inv = controller_->GetBaseTrsf().Inverted();
    gp_Trsf target_base = base_inv;
    target_base.Multiply(target);

    std::vector<nl::utils::Q> solutions;
    if (!nl::kinematics::ComputeIkAllSolutions(controller_->GetRobot(), target_base, controller_->GetJointAngles(), solutions)) {
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
    }
    else if (role == "robot") {
        menu.addAction(tr("Move"), this, &MainWindow::OnMoveRobot);
    }
    else if (role == "workpiece") {
        menu.addAction(tr("Move"), this, &MainWindow::OnMoveWorkpiece);
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
    viewer_->Context()->SetColor(workpiece_ais_, Quantity_NOC_GRAY89, Standard_False);
    viewer_->Context()->UpdateCurrentViewer();
    viewer_->View()->FitAll();

    std::string file_name = QFileInfo(path).baseName().toStdString();
    AddWorkpiece(file_name, "station");

    model_info_->setText(
        tr("File: %1 | Face: %2").arg(QFileInfo(path).fileName()).arg(face_count));
}

void MainWindow::OnGenerateWaypoints()
{
    if (selected_face_.IsNull()) {
        QMessageBox::warning(this, tr("Warning"), tr("Please select a surface first"));
        return;
    }

    // Remove old waypoints display
    if (!waypoints_ais_.IsNull()) {
        viewer_->Context()->Remove(waypoints_ais_, Standard_False);
        waypoints_ais_.Nullify();
        waypoints_.clear();
    }

    waypoint_gen_ = std::make_unique<nl::occ::WaypointGenerator>();
    waypoint_gen_->SetFace(selected_face_);

    if (waypoint_mode_ == WaypointMode::kGrid) {
        waypoint_gen_->SetAlgorithm(std::make_unique<nl::occ::WaypointGridAlgo>());
    } else {
        waypoint_gen_->SetAlgorithm(std::make_unique<nl::occ::WaypointPlanarAlgo>());
    }

    waypoints_ = waypoint_gen_->Generate(waypoint_config_);
    // 工件局部坐标 → 世界坐标
    for (auto& wp : waypoints_) {
        gp_Trsf world_pose = workpiece_trsf_;
        world_pose.Multiply(wp.pose);
        wp.pose = world_pose;
    }

    if (waypoints_.empty()) {
        statusBar()->showMessage(tr("No waypoints generated for this surface"), 3000);
        return;
    }

    DisplayWaypoints();
    statusBar()->showMessage(
        tr("%1 waypoints generated").arg(static_cast<int>(waypoints_.size())), 5000);
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

void MainWindow::OnFacePicked()
{
    if (selection_mode_ != SelectionMode::kSelectFace)
        return;

    auto ctx = viewer_->Context();

    for (ctx->InitSelected(); ctx->MoreSelected(); ctx->NextSelected()) {
        Handle(StdSelect_BRepOwner) owner =
            Handle(StdSelect_BRepOwner)::DownCast(ctx->SelectedOwner());

        if (owner.IsNull() || owner->Shape().ShapeType() != TopAbs_FACE)
            continue;

        selected_face_ = TopoDS::Face(owner->Shape());

        // Highlight selected face in yellow
        if (!selected_face_ais_.IsNull()) {
            ctx->Remove(selected_face_ais_, Standard_False);
        }
        selected_face_ais_ = new AIS_Shape(selected_face_);
        ctx->Display(selected_face_ais_, AIS_Shaded, 0, Standard_False);
        ctx->SetLocation(selected_face_ais_, TopLoc_Location(workpiece_trsf_));
        ctx->SetColor(selected_face_ais_, Quantity_NOC_CYAN1, Standard_False);
        ctx->SetTransparency(selected_face_ais_, 0.4, Standard_False);
        ctx->UpdateCurrentViewer();

        // Restore default selection mode on workpiece
        if (!workpiece_ais_.IsNull()) {
            ctx->Deactivate(workpiece_ais_);
            ctx->Activate(workpiece_ais_, 0);
        }

        selection_mode_ = SelectionMode::kNone;
        statusBar()->showMessage(tr("Surface selected, ready to generate waypoints"));
        return;
    }

    statusBar()->showMessage(tr("Please select a valid surface"), 3000);
}

void MainWindow::SetupWaypointMenu()
{
    QMenu* path_menu = menuBar()->addMenu(tr("Path(&P)"));

    path_menu->addAction(tr("Select Face"), this, &MainWindow::OnSelectFaceMode);
    path_menu->addSeparator();

    // Generation Mode submenu with radio behavior
    QMenu* mode_menu = path_menu->addMenu(tr("Generation Mode"));
    auto* mode_group = new QActionGroup(this);

    QAction* grid_action = mode_menu->addAction(tr("Grid"));
    grid_action->setCheckable(true);
    grid_action->setChecked(true);
    mode_group->addAction(grid_action);
    connect(grid_action, &QAction::triggered, this, &MainWindow::OnSetGridMode);

    QAction* planar_action = mode_menu->addAction(tr("Planar Cut"));
    planar_action->setCheckable(true);
    mode_group->addAction(planar_action);
    connect(planar_action, &QAction::triggered, this, &MainWindow::OnSetPlanarMode);

    path_menu->addSeparator();
    path_menu->addAction(tr("Generate Waypoints"), this,
        &MainWindow::OnGenerateWaypoints, QKeySequence("Ctrl+G"));
    path_menu->addAction(tr("Clear Waypoints"), this,
        &MainWindow::OnClearWaypoints);
    path_menu->addSeparator();
    path_menu->addAction(tr("Plan Trajectory"), this, [this]() {
        OnPlanTrajectory(planner_config_.approach_dist);
    }, QKeySequence("Ctrl+P"));
    path_menu->addAction(tr("Clear Trajectory"), this,
        &MainWindow::OnClearTrajectory);
}

void MainWindow::OnClearWaypoints()
{
    if (waypoints_ais_)
        viewer_->Context()->Remove(waypoints_ais_, Standard_False);
    waypoints_.clear();
    viewer_->Context()->UpdateCurrentViewer();
}

void MainWindow::OnSelectFaceMode()
{
    if (workpiece_shape_.IsNull()) {
        statusBar()->showMessage(tr("No workpiece loaded"), 3000);
        return;
    }
    // Activate face-level selection on workpiece
    auto ctx = viewer_->Context();
    if (!workpiece_ais_.IsNull()) {
        ctx->Deactivate(workpiece_ais_);
        ctx->Activate(workpiece_ais_, AIS_Shape::SelectionMode(TopAbs_FACE));
    }
    selection_mode_ = SelectionMode::kSelectFace;
    statusBar()->showMessage(tr("Click on a surface to select it"));
}

void MainWindow::OnSetGridMode()
{
    waypoint_mode_ = WaypointMode::kGrid;
}

void MainWindow::OnSetPlanarMode()
{
    waypoint_mode_ = WaypointMode::kPlanarCut;
}

void MainWindow::DisplayWaypoints()
{
    if (waypoints_.empty())
        return;

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
}

void MainWindow::SetupTrajectoryPanel()
{
    traj_panel_ = new TrajectoryPanel(this);
    addDockWidget(Qt::RightDockWidgetArea, traj_panel_);

    connect(traj_panel_, &TrajectoryPanel::PlanRequested,
            this, &MainWindow::OnPlanTrajectory);
    connect(traj_panel_, &TrajectoryPanel::PointSelected,
            this, &MainWindow::OnTrajectoryPointSelected);
    connect(traj_panel_, &TrajectoryPanel::IkSolutionChanged,
            this, &MainWindow::OnIkSolutionChanged);
}

void MainWindow::SetupTrajectoryPlayer()
{
    traj_player_ = new TrajectoryPlayer(this);
    addDockWidget(Qt::BottomDockWidgetArea, traj_player_);

    connect(traj_player_, &TrajectoryPlayer::FrameChanged,
            this, &MainWindow::OnPlaybackFrame);
    connect(traj_player_, &TrajectoryPlayer::PlaybackFinished,
            this, &MainWindow::OnPlaybackFinished);
}

void MainWindow::OnPlanTrajectory(double approach_dist)
{
    if (waypoints_.empty()) {
        QMessageBox::warning(this, tr("Warning"),
            tr("Generate waypoints first"));
        return;
    }

    planner_config_.approach_dist = approach_dist;

    // 根据 waypoint 数量动态调整插值步数，总点数 ≤ 300
    constexpr int kMaxPoints = 300;
    int n_wp = static_cast<int>(waypoints_.size());
    int movej = std::min(planner_config_.movej_steps, kMaxPoints / 4);
    int movel = std::max(1, (kMaxPoints - movej) / std::max(n_wp, 1));
    planner_config_.movej_steps = movej;
    planner_config_.movel_steps_per_seg = movel;

    // 世界坐标 → 机器人 base 坐标, 再 TCP → flange
    gp_Trsf base_inv = controller_->GetBaseTrsf().Inverted();
    gp_Trsf tcp_inv = controller_->GetToolTcpTrsf().Inverted();
    std::vector<nl::occ::Waypoint> base_wps = waypoints_;
    for (auto& wp : base_wps) {
        gp_Trsf t = base_inv;
        t.Multiply(wp.pose);
        t.Multiply(tcp_inv);
        wp.pose = t;
    }

    TrajectoryPlanner planner;
    trajectory_ = planner.Plan(
        base_wps,
        controller_->GetRobot(),
        controller_->GetJointAngles(),
        planner_config_);

    traj_panel_->SetDisplayTransforms(
        controller_->GetBaseTrsf(), controller_->GetToolTcpTrsf());
    traj_panel_->SetTrajectory(trajectory_);
    traj_player_->SetFrameCount(
        static_cast<int>(trajectory_.points.size()));

    if (trajectory_.HasErrors()) {
        statusBar()->showMessage(
            tr("Trajectory planned with %1 errors")
                .arg(trajectory_.ErrorCount()), 5000);
    } else {
        statusBar()->showMessage(
            tr("Trajectory planned: %1 points, ready to play")
                .arg(static_cast<int>(trajectory_.points.size())), 5000);
    }
}

void MainWindow::OnTrajectoryPointSelected(int index)
{
    if (index < 0 || index >= static_cast<int>(trajectory_.points.size()))
        return;

    const auto& pt = trajectory_.points[index];
    if (pt.status != nl::occ::TrajectoryPoint::Status::kIkFailed) {
        controller_->SetJointAngles(pt.joint_angles);
    }

    // Show IK alternatives for this point
    std::vector<nl::utils::Q> solutions;
    nl::kinematics::ComputeIkAllSolutions(
        controller_->GetRobot(), pt.tcp_pose,
        pt.joint_angles, solutions);
    traj_panel_->SetIkSolutions(solutions);
}

void MainWindow::OnIkSolutionChanged(int point_index, int solution_index)
{
    TrajectoryPlanner planner;
    if (planner.ResolveSinglePoint(trajectory_, point_index,
                                    controller_->GetRobot(), solution_index)) {
        traj_panel_->UpdatePoint(point_index, trajectory_.points[point_index]);
        controller_->SetJointAngles(trajectory_.points[point_index].joint_angles);
    }
}

void MainWindow::OnPlaybackFrame(int index)
{
    if (index < 0 || index >= static_cast<int>(trajectory_.points.size()))
        return;

    const auto& pt = trajectory_.points[index];
    if (pt.status == nl::occ::TrajectoryPoint::Status::kIkFailed)
        return;

    controller_->SetJointAngles(pt.joint_angles);
}

void MainWindow::OnPlaybackFinished()
{
    statusBar()->showMessage(tr("Playback finished"), 3000);
}

void MainWindow::OnClearTrajectory()
{
    trajectory_.points.clear();
    traj_panel_->Clear();
    traj_player_->SetFrameCount(0);
    statusBar()->showMessage(tr("Trajectory cleared"), 3000);
}

void MainWindow::SetWorkpieceTrsf(const gp_Trsf& trsf)
{
    workpiece_trsf_ = trsf;
    auto ctx = viewer_->Context();
    if (!workpiece_ais_.IsNull())
        ctx->SetLocation(workpiece_ais_, TopLoc_Location(trsf));
    if (!selected_face_ais_.IsNull())
        ctx->SetLocation(selected_face_ais_, TopLoc_Location(trsf));
    ctx->UpdateCurrentViewer();
}

void MainWindow::OnMoveRobot()
{
    if (movement_panel_) return;  // 已有面板打开

    move_target_ = MoveTarget::kRobot;
    gp_Trsf current = controller_->GetBaseTrsf();

    movement_panel_ = new MovementPanel(
        QString::fromStdString(controller_->GetRobot().name), current, this);
    addDockWidget(Qt::RightDockWidgetArea, movement_panel_);

    connect(movement_panel_, &MovementPanel::PreviewRequested,
        this, &MainWindow::OnMovePreview);
    connect(movement_panel_, &MovementPanel::Accepted,
        this, &MainWindow::OnMoveAccepted);
    connect(movement_panel_, &MovementPanel::Cancelled,
        this, &MainWindow::OnMoveCancelled);
}

void MainWindow::OnMoveWorkpiece()
{
    if (movement_panel_) return;

    move_target_ = MoveTarget::kWorkpiece;

    movement_panel_ = new MovementPanel(tr("Workpiece"), workpiece_trsf_, this);
    addDockWidget(Qt::RightDockWidgetArea, movement_panel_);

    connect(movement_panel_, &MovementPanel::PreviewRequested,
        this, &MainWindow::OnMovePreview);
    connect(movement_panel_, &MovementPanel::Accepted,
        this, &MainWindow::OnMoveAccepted);
    connect(movement_panel_, &MovementPanel::Cancelled,
        this, &MainWindow::OnMoveCancelled);
}

void MainWindow::OnMovePreview(const gp_Trsf& trsf)
{
    if (move_target_ == MoveTarget::kRobot) {
        controller_->SetBaseTrsf(trsf);
    }
    else if (move_target_ == MoveTarget::kWorkpiece) {
        SetWorkpieceTrsf(trsf);
    }
}

void MainWindow::OnMoveAccepted(const gp_Trsf& trsf)
{
    gp_Trsf old_trsf = movement_panel_->GetInitialTrsf();

    if (move_target_ == MoveTarget::kRobot) {
        undo_stack_->push(new CmdMoveBase(controller_, old_trsf, trsf));
        if (!trajectory_.points.empty())
            ReplanTrajectory();
    }
    else if (move_target_ == MoveTarget::kWorkpiece) {
        auto apply = [this](const gp_Trsf& t) { SetWorkpieceTrsf(t); };
        undo_stack_->push(new CmdMoveWorkpiece(apply, old_trsf, trsf));
        TransformWaypointsAndTrajectory(old_trsf, trsf);
    }

    movement_panel_->deleteLater();
    movement_panel_ = nullptr;
    move_target_ = MoveTarget::kNone;
}

void MainWindow::OnMoveCancelled()
{
    gp_Trsf old_trsf = movement_panel_->GetInitialTrsf();

    if (move_target_ == MoveTarget::kRobot)
        controller_->SetBaseTrsf(old_trsf);
    else if (move_target_ == MoveTarget::kWorkpiece)
        SetWorkpieceTrsf(old_trsf);

    movement_panel_->deleteLater();
    movement_panel_ = nullptr;
    move_target_ = MoveTarget::kNone;
}

void MainWindow::TransformWaypointsAndTrajectory(
    const gp_Trsf& old_trsf, const gp_Trsf& new_trsf)
{
    if (waypoints_.empty()) return;

    gp_Trsf delta = new_trsf;
    delta.Multiply(old_trsf.Inverted());

    for (auto& wp : waypoints_) {
        gp_Trsf t = delta;
        t.Multiply(wp.pose);
        wp.pose = t;
    }
    DisplayWaypoints();

    if (!trajectory_.points.empty())
        ReplanTrajectory();
}

void MainWindow::ReplanTrajectory()
{
    if (waypoints_.empty()) return;
    OnPlanTrajectory(planner_config_.approach_dist);
}

}
}