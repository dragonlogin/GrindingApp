#ifndef GRINDINGAPP_SRC_UI_MAIN_WINDOW_H_
#define GRINDINGAPP_SRC_UI_MAIN_WINDOW_H_

#include <string>
#include <vector>

#include <QMainWindow>
#include <QLabel>
#include <QDockWidget>
#include <QTreeWidget>

#include <AIS_Trihedron.hxx>
#include <AIS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <QUndoStack>
#include <memory>

#include "GrindingUIExport.h"
#include "Q.h"
#include "RbXmlParser.h"
#include "domain/Waypoint.h"
#include "domain/Trajectory.h"
#include "IWaypointAlgo.h"
#include "WaypointGenerator.h"
#include "TrajectoryPlanner.h"

namespace nl {
namespace ui {

class OcctViewWidget;
class JogPanel;
class RobotController;
class TrajectoryPanel;
class TrajectoryPlayer;
class MovementPanel;

class GRINDING_UI_EXPORT MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
    void OnImportWorkpiece();
    void OnGenerateWaypoints();
    void OnClearWaypoints();
    void OnSelectFaceMode();
    void OnFacePicked();
    void OnSetGridMode();
    void OnSetPlanarMode();
    void OnViewFront();
    void OnViewTop();
    void OnViewSide();
    void OnViewIsometric();
    void OnViewWireframe();
    void OnViewShaded();
    void OnFitAll();
    void OnLoadRobot();
    void OnLoadTool();
    void OnSceneTreeContextMenu(const QPoint& pos);

private:
    void SwitchLanguage(const std::string& lang);

    void SetupMenuBar();
    void SetupToolBar();
    void SetupStatusBar();
    void SetupCentralWidget();

    void SetupJogPanel();
    void UpdateTcpPoseDisplay();

    void OnPoseEdited(double x, double y, double z, double rx, double ry, double rz);
    void OnIkSolutionSelected(int index);
    void OnReferenceFrameChanged(int index);
    void OnControllerJointAnglesChanged(const nl::utils::Q& angles);

    void SetupSceneTree();
    void AddRobot(const QString& name);
    void UpdateRobotJoints();
    void AddTool(const QString& name, const std::string& parent_role);
    void AddWorkpiece(const std::string& name, const std::string& parent_role);

    void SetupWaypointMenu();
    void DisplayWaypoints();

    void SetupTrajectoryPanel();
    void SetupTrajectoryPlayer();
    void OnPlanTrajectory(double approach_dist);
    void OnTrajectoryPointSelected(int index);
    void OnIkSolutionChanged(int point_index, int solution_index);
    void OnPlaybackFrame(int index);
    void OnPlaybackFinished();
    void OnClearTrajectory();

    // Movement
    void SetWorkpieceTrsf(const gp_Trsf& trsf);
    void OnMoveRobot();
    void OnMoveWorkpiece();
    void OnMovePreview(const gp_Trsf& trsf);
    void OnMoveAccepted(const gp_Trsf& trsf);
    void OnMoveCancelled();
    void TransformWaypointsAndTrajectory(const gp_Trsf& old_trsf, const gp_Trsf& new_trsf);
    void ReplanTrajectory();

    QLabel*          model_info_;
    QLabel*          coord_label_;
    OcctViewWidget*  viewer_ = nullptr;
    RobotController* controller_ = nullptr;
    QUndoStack*      undo_stack_ = nullptr;

    JogPanel*               jog_panel_ = nullptr;
    QTreeWidget*            scene_tree_   = nullptr;

    int tcp_ref_mode_ = 0;  // 0=Flange, 1=Tool TCP
    std::vector<nl::utils::Q> current_ik_solutions_;

    Handle(AIS_Shape)               workpiece_ais_;
    TopoDS_Shape                    workpiece_shape_;
    Handle(AIS_Shape)               waypoints_ais_;
    std::vector<domain::Waypoint>  waypoints_;

    // Surface picking
    enum class SelectionMode { kNone, kSelectFace };
    SelectionMode selection_mode_ = SelectionMode::kNone;
    TopoDS_Face selected_face_;
    Handle(AIS_Shape) selected_face_ais_;

    // Waypoint generation
    enum class WaypointMode { kGrid, kPlanarCut };
    WaypointMode waypoint_mode_ = WaypointMode::kGrid;
    std::unique_ptr<nl::occ::WaypointGenerator> waypoint_gen_;
    nl::occ::WaypointConfig waypoint_config_;

    // Trajectory
    TrajectoryPanel*  traj_panel_ = nullptr;
    TrajectoryPlayer* traj_player_ = nullptr;
    domain::Trajectory trajectory_;
    TrajectoryPlanner::Config planner_config_;

    // Movement
    gp_Trsf workpiece_trsf_;
    MovementPanel* movement_panel_ = nullptr;
    enum class MoveTarget { kNone, kRobot, kWorkpiece };
    MoveTarget move_target_ = MoveTarget::kNone;
};

} // namespace ui
} // namespace nl

#endif  // GRINDINGAPP_SRC_UI_MAIN_WINDOW_H_
