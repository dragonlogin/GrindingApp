#ifndef GRINDINGAPP_SRC_UI_ROBOT_CONTROLLER_H_
#define GRINDINGAPP_SRC_UI_ROBOT_CONTROLLER_H_

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
#include "RbXmlParser.h"

namespace nl {
namespace ui {

/*
- 机器人与工具的加载 ：接管了 RbXmlParser::Parse 和 StlLoader::Load 等逻辑，并在内部维护 robot_meshes_ 。
- 3D 渲染控制 ：内部持有 AIS_InteractiveContext ，统一负责调用 context_->Display() 和 context_->SetLocation() 来渲染机器人网格和坐标系。
- 数据驱动 ：通过 JointAnglesChanged 和 DisplayUpdated 信号向外通知状态变更， MainWindow 现在只需要绑定这些信号，并专注于刷新 UI（如更新右侧的 TCP 坐标和左侧的场景树）。
*/
class GRINDING_UI_EXPORT RobotController : public QObject
{
    Q_OBJECT

public:
    explicit RobotController(Handle(AIS_InteractiveContext) context, QObject* parent = nullptr);
    ~RobotController() override = default;

    bool LoadRobot(const std::string& xml_path);
    bool LoadTool(const std::string& path);

    void SetJointAngles(const nl::utils::Q& angles);
    nl::utils::Q GetJointAngles() const { return joint_angles_; }

    const nl::core::RbRobot& GetRobot() const { return current_robot_; }
    
    // Toggle joint frame visibility
    void ToggleJointFrame(int index);

    // Compute IK and update
    bool SetTcpPose(const gp_Trsf& target_pose, int tcp_ref_mode, std::vector<nl::utils::Q>& out_solutions);

    // Provide FK data for UI (e.g. TCP display)
    std::vector<gp_Trsf> GetCurrentFk() const;
    gp_Trsf GetToolTcpTrsf() const { return tool_tcp_trsf_; }

signals:
    void RobotLoaded(const QString& name);
    void ToolLoaded(const QString& name);
    void JointAnglesChanged(const nl::utils::Q& angles);
    void DisplayUpdated();

private:
    void UpdateRobotDisplay();
    void UpdateCoordinateFrames(const std::vector<gp_Trsf>& fk);

    Handle(AIS_InteractiveContext) context_;

    struct RobotMesh {
        nl::core::RbDrawable  drawable;
        TopoDS_Shape          original;
        Handle(AIS_Shape)     ais;
    };
    
    std::vector<RobotMesh>              robot_meshes_;
    nl::core::RbRobot                   current_robot_;
    nl::utils::Q                        joint_angles_;

    std::vector<Handle(AIS_Trihedron)>  joint_frames_;
    Handle(AIS_Trihedron)               base_frame_;

    Handle(AIS_Shape)                   tool_ais_;
    Handle(AIS_Trihedron)               tool_tcp_frame_;
    gp_Trsf                             tool_base_trsf_;
    gp_Trsf                             tool_tcp_trsf_;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_ROBOT_CONTROLLER_H_
