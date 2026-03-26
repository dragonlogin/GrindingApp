#include "JogPanel.h"

#include <QWidget>
#include <QGridLayout>
#include <QLabel>

namespace nl {
namespace ui {

JogPanel::JogPanel(QWidget* parent)
    : QDockWidget(tr("Joint Jog"), parent)
{
    SetupUI();
}

void JogPanel::SetJointAngles(const nl::utils::Q& angles)
{
    joint_angles_ = angles;
    int n = std::min(6, angles.size());
    for (int i = 0; i < n; ++i) {
        if (joint_spinboxes_[i]) {
            joint_spinboxes_[i]->blockSignals(true);
            joint_spinboxes_[i]->setValue(angles[i]);
            joint_spinboxes_[i]->blockSignals(false);
        }
        if (joint_sliders_[i]) {
            joint_sliders_[i]->blockSignals(true);
            joint_sliders_[i]->setValue(qRound(angles[i] * 10));
            joint_sliders_[i]->blockSignals(false);
        }
    }
}

void JogPanel::SetupUI()
{
    auto widget = new QWidget;
    auto layout = new QGridLayout(widget);

    const char* labels[6] = {"J1", "J2", "J3", "J4", "J5", "J6"};

    for (int i = 0; i < 6; ++i) {
        layout->addWidget(new QLabel(labels[i]), i, 0);

        auto* slider = new QSlider(Qt::Horizontal);
        slider->setRange(-1800, 1800);  // ±180° × 10
        slider->setValue(0);
        joint_sliders_[i] = slider;
        layout->addWidget(slider, i, 1);

        auto* spin = new QDoubleSpinBox;
        spin->setRange(-180.0, 180.0);
        spin->setDecimals(1);
        spin->setSuffix("°");
        spin->setValue(0.0);
        joint_spinboxes_[i] = spin;
        layout->addWidget(spin, i, 2);

        connect(slider, &QSlider::valueChanged, this, [this, i, spin](int v) {
            spin->blockSignals(true);
            spin->setValue(v / 10.0);
            spin->blockSignals(false);
            joint_angles_[i] = v / 10.0;
            emit JointAnglesChanged(joint_angles_);
        });
        connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this, i, slider](double v) {
                slider->blockSignals(true);
                slider->setValue(qRound(v * 10));
                slider->blockSignals(false);
                joint_angles_[i] = v;
                emit JointAnglesChanged(joint_angles_);
            });
    }

    setWidget(widget);
}

} // namespace ui
} // namespace nl