#ifndef GRINDINGAPP_SRC_UI_ROBOT_CONTROLLER_H_
#define GRINDINGAPP_SRC_UI_ROBOT_CONTROLLER_H_

#include <memory>
#include <vector>
#include <string>
#include <QObject>

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Trihedron.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>

#include "GrindingUIExport.h"
#include "Q.h"
#include "RobotDisplay.h"
#include "model_import/IRobotImporter.h"
#include "model_import/IToolImporter.h"

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT RobotController : public QObject
{
    Q_OBJECT

public:
    explicit RobotController(
        Handle(AIS_InteractiveContext) context,
        std::unique_ptr<model_import::IRobotImporter> robot_importer,
        std::unique_ptr<model_import::IToolImporter>  tool_importer,
        QObject* parent = nullptr);
    ~RobotController() override = default;

    bool LoadRobot(const std::string& xml_path);
    bool LoadTool(const std::string& path);

    void SetJointAngles(const nl::utils::Q& angles);
    nl::utils::Q GetJointAngles() const { return joint_angles_; }

    const domain::Robot& GetRobot() const { return current_robot_; }

    void ToggleJointFrame(int index);

    bool SetTcpPose(const gp_Trsf& target_pose, int tcp_ref_mode, std::vector<nl::utils::Q>& out_solutions);

    void SetBaseTrsf(const gp_Trsf& trsf);
    gp_Trsf GetBaseTrsf() const { return base_trsf_; }

    std::vector<gp_Trsf> GetCurrentFk() const;
    gp_Trsf GetToolTcpTrsf() const { return tool_tcp_trsf_; }

signals:
    void BaseTrsfChanged(const gp_Trsf& trsf);
    void RobotLoaded(const QString& name);
    void ToolLoaded(const QString& name);
    void JointAnglesChanged(const nl::utils::Q& angles);
    void DisplayUpdated();

private:
    void UpdateRobotDisplay();
    void UpdateCoordinateFrames(const std::vector<gp_Trsf>& fk);

    Handle(AIS_InteractiveContext) context_;

    std::unique_ptr<model_import::IRobotImporter> robot_importer_;
    std::unique_ptr<model_import::IToolImporter>  tool_importer_;

    struct RobotMesh {
        model_import::MeshRef drawable;
        TopoDS_Shape          original;
        Handle(AIS_Shape)     ais;
    };

    std::vector<RobotMesh>             robot_meshes_;
    domain::Robot                      current_robot_;
    nl::utils::Q                       joint_angles_;

    std::vector<Handle(AIS_Trihedron)> joint_frames_;
    Handle(AIS_Trihedron)              base_frame_;

    Handle(AIS_Shape)                  tool_ais_;
    Handle(AIS_Trihedron)              tool_tcp_frame_;
    gp_Trsf                            tool_base_trsf_;
    gp_Trsf                            tool_tcp_trsf_;
    gp_Trsf                            base_trsf_;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_ROBOT_CONTROLLER_H_