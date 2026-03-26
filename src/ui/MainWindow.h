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

#include "GrindingUIExport.h"
#include "Q.h"
#include "RbXmlParser.h"
#include "Waypoint.h"

namespace nl {
namespace ui {

class OcctViewWidget;
class JogPanel;
class RobotController;

class GRINDING_UI_EXPORT MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
    void OnImportWorkpiece();
    void OnGenerateWaypoints();
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
    std::vector<nl::occ::Waypoint>  waypoints_;
};

} // namespace ui
} // namespace nl

#endif  // GRINDINGAPP_SRC_UI_MAIN_WINDOW_H_
