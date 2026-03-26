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

#include "GrindingUIExport.h"
#include "Q.h"
#include "RbXmlParser.h"

namespace nl {
namespace ui {

class OcctViewWidget;
class JogPanel;

class GRINDING_UI_EXPORT MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
    void OnImportWorkpiece();
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
    void UpdateRobotDisplay();
    void UpdateTcpPoseDisplay();
    void UpdateTcpPoseDisplay(const std::vector<gp_Trsf>& fk);
    void UpdateCoordinateFrames(const std::vector<gp_Trsf>& fk);

    void OnPoseEdited(double x, double y, double z, double rx, double ry, double rz);
    void OnIkSolutionSelected(int index);
    void OnReferenceFrameChanged(int index);

    void SetupSceneTree();
    void AddRobot(const std::string& name);
    void UpdateRobotJoints();
    void AddTool(const std::string& name, const std::string& parent_role);
    void AddWorkpiece(const std::string& name, const std::string& parent_role);

    QLabel*          model_info_;
    QLabel*          coord_label_;
    OcctViewWidget*  viewer_ = nullptr;

    struct RobotMesh {
        nl::core::RbDrawable  drawable;
        TopoDS_Shape          original;
        Handle(AIS_Shape)     ais;
    };
    std::vector<RobotMesh>  robot_meshes_;
    nl::core::RbRobot       current_robot_;
    nl::utils::Q            joint_angles_;
    JogPanel*               jog_panel_ = nullptr;

    QTreeWidget*                        scene_tree_   = nullptr;
    std::vector<Handle(AIS_Trihedron)>  joint_frames_;
    Handle(AIS_Trihedron)               base_frame_;

    Handle(AIS_Shape)     tool_ais_;
    Handle(AIS_Trihedron) tool_tcp_frame_;
    gp_Trsf               tool_base_trsf_;
    gp_Trsf               tool_tcp_trsf_;

    int tcp_ref_mode_ = 0;  // 0=Flange, 1=Tool TCP
    std::vector<nl::utils::Q> current_ik_solutions_;
};

} // namespace ui
} // namespace nl

#endif  // GRINDINGAPP_SRC_UI_MAIN_WINDOW_H_
