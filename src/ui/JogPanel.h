#ifndef GRINDINGAPP_SRC_UI_JOG_PANEL_H_
#define GRINDINGAPP_SRC_UI_JOG_PANEL_H_

#include <vector>

#include <QDockWidget>
#include <QSlider>
#include <QDoubleSpinBox>

#include "GrindingUIExport.h"
#include "Q.h"

class QComboBox;
class QLabel;
class QGroupBox;

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT JogPanel : public QDockWidget
{
    Q_OBJECT

public:
    explicit JogPanel(QWidget* parent = nullptr);
    ~JogPanel() override = default;

    void SetJointAngles(const nl::utils::Q& angles);
    void SetTcpPose(double x, double y, double z, double rx, double ry, double rz);
    void SetIkSolutions(const std::vector<nl::utils::Q>& solutions);

signals:
    void JointAnglesChanged(const nl::utils::Q& angles);
    void PoseEdited(double x, double y, double z, double rx, double ry, double rz);
    void IkSolutionSelected(int index);
    void ReferenceFrameChanged(int index);

private:
    void SetupUI();
    QGroupBox* SetupJointJogSection();
    QGroupBox* SetupTcpPoseSection();
    QGroupBox* SetupIkSolutionsSection();

    void UpdateIkDisplay();

    // Joint Jog
    nl::utils::Q    joint_angles_;
    QSlider*        joint_sliders_[6]   = {};
    QDoubleSpinBox* joint_spinboxes_[6] = {};
    QComboBox*      joint_unit_combo_   = nullptr;

    // TCP Pose
    QDoubleSpinBox* pose_spinboxes_[6] = {};
    QComboBox*      tcp_ref_combo_     = nullptr;
    QComboBox*      tcp_unit_combo_    = nullptr;
    bool            pose_updating_     = false;

    // IK Solutions
    QComboBox*      ik_solution_combo_ = nullptr;
    QComboBox*      ik_unit_combo_     = nullptr;
    QLabel*         ik_labels_[6]      = {};
    std::vector<nl::utils::Q> ik_solutions_;
};

} // namespace ui
} // namespace nl

#endif  // GRINDINGAPP_SRC_UI_JOG_PANEL_H_
