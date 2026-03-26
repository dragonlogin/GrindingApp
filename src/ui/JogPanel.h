#ifndef GRINDINGAPP_SRC_UI_JOG_PANEL_H_
#define GRINDINGAPP_SRC_UI_JOG_PANEL_H_

#include <QDockWidget>
#include <QSlider>
#include <QDoubleSpinBox>

#include "GrindingUIExport.h"
#include "Q.h"

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT JogPanel : public QDockWidget
{
    Q_OBJECT

public:
    explicit JogPanel(QWidget* parent = nullptr);
    ~JogPanel() override = default;

    void SetJointAngles(const nl::utils::Q& angles);

signals:
    void JointAnglesChanged(const nl::utils::Q& angles);

private:
    void SetupUI();

    nl::utils::Q    joint_angles_;
    QSlider*        joint_sliders_[6]   = {};
    QDoubleSpinBox* joint_spinboxes_[6] = {};
};

} // namespace ui
} // namespace nl

#endif  // GRINDINGAPP_SRC_UI_JOG_PANEL_H_