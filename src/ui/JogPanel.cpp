#include "JogPanel.h"

#include <cmath>

#include <QWidget>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QGroupBox>
#include <QComboBox>

namespace nl {
namespace ui {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;
constexpr double kRad2Deg = 180.0 / M_PI;
} // namespace

JogPanel::JogPanel(QWidget* parent)
    : QDockWidget(tr("Jog Panel"), parent)
{
    SetupUI();
}

void JogPanel::SetupUI()
{
    auto* widget = new QWidget;
    auto* main_layout = new QVBoxLayout(widget);
    main_layout->setContentsMargins(4, 4, 4, 4);
    main_layout->setSpacing(4);

    main_layout->addWidget(SetupJointJogSection());
    main_layout->addWidget(SetupTcpPoseSection());
    main_layout->addWidget(SetupIkSolutionsSection());
    main_layout->addStretch();

    setWidget(widget);
}

QGroupBox* JogPanel::SetupJointJogSection()
{
    auto* group = new QGroupBox(tr("Joint Jog"));
    auto* layout = new QVBoxLayout(group);

    // Unit combo
    auto* combo_row = new QHBoxLayout;
    joint_unit_combo_ = new QComboBox;
    joint_unit_combo_->addItems({"DEG", "RAD"});
    joint_unit_combo_->setFixedWidth(70);
    combo_row->addWidget(joint_unit_combo_);
    combo_row->addStretch();
    layout->addLayout(combo_row);

    // Joint sliders + spinboxes
    auto* grid = new QGridLayout;
    const char* labels[6] = {"J1", "J2", "J3", "J4", "J5", "J6"};

    for (int i = 0; i < 6; ++i) {
        grid->addWidget(new QLabel(labels[i]), i, 0);

        auto* slider = new QSlider(Qt::Horizontal);
        slider->setRange(-1800, 1800);
        slider->setValue(0);
        joint_sliders_[i] = slider;
        grid->addWidget(slider, i, 1);

        auto* spin = new QDoubleSpinBox;
        spin->setRange(-180.0, 180.0);
        spin->setDecimals(1);
        spin->setSuffix(QString::fromUtf8("\xc2\xb0"));
        spin->setValue(0.0);
        joint_spinboxes_[i] = spin;
        grid->addWidget(spin, i, 2);

        connect(slider, &QSlider::valueChanged, this, [this, i, spin](int v) {
            bool is_rad = (joint_unit_combo_->currentIndex() == 1);
            double display_val = is_rad ? v / 10000.0 : v / 10.0;
            double deg_val = is_rad ? v / 10000.0 * kRad2Deg : v / 10.0;
            spin->blockSignals(true);
            spin->setValue(display_val);
            spin->blockSignals(false);
            joint_angles_[i] = deg_val;
            emit JointAnglesChanged(joint_angles_);
        });
        connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this, i, slider](double v) {
                bool is_rad = (joint_unit_combo_->currentIndex() == 1);
                double deg_val = is_rad ? v * kRad2Deg : v;
                int slider_val = is_rad ? qRound(v * 10000) : qRound(v * 10);
                slider->blockSignals(true);
                slider->setValue(slider_val);
                slider->blockSignals(false);
                joint_angles_[i] = deg_val;
                emit JointAnglesChanged(joint_angles_);
            });
    }
    layout->addLayout(grid);

    // Unit switch: convert displayed values
    connect(joint_unit_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int idx) {
            bool is_rad = (idx == 1);
            for (int i = 0; i < 6; ++i) {
                joint_spinboxes_[i]->blockSignals(true);
                double deg_val = joint_angles_[i];
                if (is_rad) {
                    joint_spinboxes_[i]->setRange(-M_PI, M_PI);
                    joint_spinboxes_[i]->setDecimals(4);
                    joint_spinboxes_[i]->setSuffix(" rad");
                    joint_spinboxes_[i]->setValue(deg_val * kDeg2Rad);
                    joint_sliders_[i]->blockSignals(true);
                    joint_sliders_[i]->setRange(-31416, 31416);
                    joint_sliders_[i]->setValue(qRound(deg_val * kDeg2Rad * 10000));
                    joint_sliders_[i]->blockSignals(false);
                } else {
                    joint_spinboxes_[i]->setRange(-180.0, 180.0);
                    joint_spinboxes_[i]->setDecimals(1);
                    joint_spinboxes_[i]->setSuffix(QString::fromUtf8("\xc2\xb0"));
                    joint_spinboxes_[i]->setValue(deg_val);
                    joint_sliders_[i]->blockSignals(true);
                    joint_sliders_[i]->setRange(-1800, 1800);
                    joint_sliders_[i]->setValue(qRound(deg_val * 10));
                    joint_sliders_[i]->blockSignals(false);
                }
                joint_spinboxes_[i]->blockSignals(false);
            }
        });

    return group;
}

QGroupBox* JogPanel::SetupTcpPoseSection()
{
    auto* group = new QGroupBox(tr("TCP Pose"));
    auto* layout = new QVBoxLayout(group);

    // Combos row: [Flange/Tool TCP] [DEG/RAD]
    auto* combo_row = new QHBoxLayout;
    tcp_ref_combo_ = new QComboBox;
    tcp_ref_combo_->addItems({tr("Flange"), tr("Tool TCP")});
    tcp_ref_combo_->setFixedWidth(90);
    combo_row->addWidget(tcp_ref_combo_);

    tcp_unit_combo_ = new QComboBox;
    tcp_unit_combo_->addItems({"DEG", "RAD"});
    tcp_unit_combo_->setFixedWidth(70);
    combo_row->addWidget(tcp_unit_combo_);
    combo_row->addStretch();
    layout->addLayout(combo_row);

    // Pose grid: X/Y/Z mm + Rx/Ry/Rz deg
    auto* grid = new QGridLayout;
    const char* pos_labels[3] = {"X", "Y", "Z"};
    const char* rot_labels[3] = {"Rx", "Ry", "Rz"};

    for (int i = 0; i < 3; ++i) {
        grid->addWidget(new QLabel(pos_labels[i]), i, 0);
        auto* spin = new QDoubleSpinBox;
        spin->setRange(-9999.0, 9999.0);
        spin->setDecimals(2);
        spin->setSuffix(" mm");
        spin->setValue(0.0);
        pose_spinboxes_[i] = spin;
        grid->addWidget(spin, i, 1);

        grid->addWidget(new QLabel(rot_labels[i]), i, 2);
        auto* rspin = new QDoubleSpinBox;
        rspin->setRange(-180.0, 180.0);
        rspin->setDecimals(2);
        rspin->setSuffix(QString::fromUtf8("\xc2\xb0"));
        rspin->setValue(0.0);
        pose_spinboxes_[3 + i] = rspin;
        grid->addWidget(rspin, i, 3);
    }
    layout->addLayout(grid);

    // Pose spinbox edit → emit PoseEdited (always in degrees)
    for (int i = 0; i < 6; ++i) {
        connect(pose_spinboxes_[i], qOverload<double>(&QDoubleSpinBox::valueChanged),
            this, [this](double) {
                if (pose_updating_) return;
                double vals[6];
                bool is_rad = (tcp_unit_combo_->currentIndex() == 1);
                for (int k = 0; k < 3; ++k)
                    vals[k] = pose_spinboxes_[k]->value();  // mm, always
                for (int k = 3; k < 6; ++k)
                    vals[k] = is_rad ? pose_spinboxes_[k]->value() * kRad2Deg
                                     : pose_spinboxes_[k]->value();
                emit PoseEdited(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
            });
    }

    // Reference frame combo → emit
    connect(tcp_ref_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int idx) { emit ReferenceFrameChanged(idx); });

    // Unit switch for Rx/Ry/Rz display
    connect(tcp_unit_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int idx) {
            pose_updating_ = true;
            bool is_rad = (idx == 1);
            for (int i = 3; i < 6; ++i) {
                double cur = pose_spinboxes_[i]->value();
                pose_spinboxes_[i]->blockSignals(true);
                if (is_rad) {
                    double rad_val = cur * kDeg2Rad;
                    pose_spinboxes_[i]->setRange(-M_PI, M_PI);
                    pose_spinboxes_[i]->setDecimals(4);
                    pose_spinboxes_[i]->setSuffix(" rad");
                    pose_spinboxes_[i]->setValue(rad_val);
                } else {
                    double deg_val = cur * kRad2Deg;
                    pose_spinboxes_[i]->setRange(-180.0, 180.0);
                    pose_spinboxes_[i]->setDecimals(2);
                    pose_spinboxes_[i]->setSuffix(QString::fromUtf8("\xc2\xb0"));
                    pose_spinboxes_[i]->setValue(deg_val);
                }
                pose_spinboxes_[i]->blockSignals(false);
            }
            pose_updating_ = false;
        });

    return group;
}

QGroupBox* JogPanel::SetupIkSolutionsSection()
{
    auto* group = new QGroupBox(tr("IK Solutions"));
    auto* layout = new QVBoxLayout(group);

    // Combos row: [Sol 1] [DEG/RAD]
    auto* combo_row = new QHBoxLayout;
    ik_solution_combo_ = new QComboBox;
    ik_solution_combo_->setFixedWidth(90);
    combo_row->addWidget(ik_solution_combo_);

    ik_unit_combo_ = new QComboBox;
    ik_unit_combo_->addItems({"DEG", "RAD"});
    ik_unit_combo_->setFixedWidth(70);
    combo_row->addWidget(ik_unit_combo_);
    combo_row->addStretch();
    layout->addLayout(combo_row);

    // 2 rows x 3 cols of labels: J1-J3, J4-J6
    auto* grid = new QGridLayout;
    for (int i = 0; i < 6; ++i) {
        ik_labels_[i] = new QLabel(QString("J%1: --").arg(i + 1));
        grid->addWidget(ik_labels_[i], i / 3, i % 3);
    }
    layout->addLayout(grid);

    // Solution combo → emit
    connect(ik_solution_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int idx) {
            if (idx >= 0 && idx < static_cast<int>(ik_solutions_.size())) {
                UpdateIkDisplay();
                emit IkSolutionSelected(idx);
            }
        });

    // Unit switch → refresh display
    connect(ik_unit_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
        this, [this](int) { UpdateIkDisplay(); });

    return group;
}

void JogPanel::SetJointAngles(const nl::utils::Q& angles)
{
    joint_angles_ = angles;
    bool is_rad = (joint_unit_combo_ && joint_unit_combo_->currentIndex() == 1);
    int n = std::min(6, angles.size());
    for (int i = 0; i < n; ++i) {
        if (joint_spinboxes_[i]) {
            joint_spinboxes_[i]->blockSignals(true);
            joint_spinboxes_[i]->setValue(is_rad ? angles[i] * kDeg2Rad : angles[i]);
            joint_spinboxes_[i]->blockSignals(false);
        }
        if (joint_sliders_[i]) {
            joint_sliders_[i]->blockSignals(true);
            if (is_rad)
                joint_sliders_[i]->setValue(qRound(angles[i] * kDeg2Rad * 10000));
            else
                joint_sliders_[i]->setValue(qRound(angles[i] * 10));
            joint_sliders_[i]->blockSignals(false);
        }
    }
}

void JogPanel::SetTcpPose(double x, double y, double z,
                           double rx, double ry, double rz)
{
    pose_updating_ = true;
    bool is_rad = (tcp_unit_combo_ && tcp_unit_combo_->currentIndex() == 1);
    double vals[6] = {x, y, z, rx, ry, rz};
    for (int i = 0; i < 6; ++i) {
        if (!pose_spinboxes_[i]) continue;
        pose_spinboxes_[i]->blockSignals(true);
        if (i >= 3 && is_rad)
            pose_spinboxes_[i]->setValue(vals[i] * kDeg2Rad);
        else
            pose_spinboxes_[i]->setValue(vals[i]);
        pose_spinboxes_[i]->blockSignals(false);
    }
    pose_updating_ = false;
}

void JogPanel::SetIkSolutions(const std::vector<nl::utils::Q>& solutions)
{
    ik_solutions_ = solutions;
    ik_solution_combo_->blockSignals(true);
    ik_solution_combo_->clear();
    for (int i = 0; i < static_cast<int>(solutions.size()); ++i)
        ik_solution_combo_->addItem(QString("Sol %1").arg(i + 1));
    if (!solutions.empty())
        ik_solution_combo_->setCurrentIndex(0);
    ik_solution_combo_->blockSignals(false);
    UpdateIkDisplay();
}

void JogPanel::UpdateIkDisplay()
{
    int idx = ik_solution_combo_->currentIndex();
    if (idx < 0 || idx >= static_cast<int>(ik_solutions_.size())) {
        for (int i = 0; i < 6; ++i)
            ik_labels_[i]->setText(QString("J%1: --").arg(i + 1));
        return;
    }

    const nl::utils::Q& sol = ik_solutions_[idx];
    bool is_rad = (ik_unit_combo_->currentIndex() == 1);
    for (int i = 0; i < 6; ++i) {
        double val = is_rad ? sol[i] * kDeg2Rad : sol[i];
        QString suffix = is_rad ? "rad" : QString::fromUtf8("\xc2\xb0");
        int decimals = is_rad ? 4 : 1;
        ik_labels_[i]->setText(
            QString("J%1: %2%3").arg(i + 1)
                .arg(val, 0, 'f', decimals).arg(suffix));
    }
}

} // namespace ui
} // namespace nl
