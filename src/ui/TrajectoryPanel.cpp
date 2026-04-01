#include "TrajectoryPanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTableWidget>
#include <QHeaderView>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>

#include "RobotDisplay.h"
#include "Conversions.h"

using domain::TrajectoryPoint;

namespace nl {
namespace ui {

TrajectoryPanel::TrajectoryPanel(QWidget* parent)
    : QDockWidget(tr("Trajectory Editor"), parent)
{
    SetupUi();
}

void TrajectoryPanel::SetupUi()
{
    auto* widget = new QWidget;
    auto* layout = new QVBoxLayout(widget);

    // Top row: approach distance + plan button
    auto* top_row = new QHBoxLayout;
    top_row->addWidget(new QLabel(tr("Approach dist:")));
    approach_spin_ = new QDoubleSpinBox;
    approach_spin_->setRange(0.0, 500.0);
    approach_spin_->setValue(50.0);
    approach_spin_->setSuffix(" mm");
    top_row->addWidget(approach_spin_);

    plan_btn_ = new QPushButton(tr("Plan"));
    connect(plan_btn_, &QPushButton::clicked, this, &TrajectoryPanel::OnPlanClicked);
    top_row->addWidget(plan_btn_);
    top_row->addStretch();
    layout->addLayout(top_row);

    // Table
    table_ = new QTableWidget;
    table_->setColumnCount(10);
    table_->setHorizontalHeaderLabels(
        {"#", "Type", "X", "Y", "Z", "Rx", "Ry", "Rz", "Status", "WP"});
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->verticalHeader()->setVisible(false);
    table_->setAlternatingRowColors(true);

    connect(table_, &QTableWidget::cellClicked,
            this, &TrajectoryPanel::OnTableRowClicked);
    layout->addWidget(table_);

    // IK solution selector
    auto* ik_row = new QHBoxLayout;
    ik_row->addWidget(new QLabel(tr("IK Solution:")));
    ik_combo_ = new QComboBox;
    connect(ik_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TrajectoryPanel::OnIkComboChanged);
    ik_row->addWidget(ik_combo_);
    ik_row->addStretch();
    layout->addLayout(ik_row);

    // Summary
    summary_label_ = new QLabel;
    layout->addWidget(summary_label_);

    setWidget(widget);
}

void TrajectoryPanel::SetDisplayTransforms(const gp_Trsf& base_trsf,
                                            const gp_Trsf& tool_tcp_trsf)
{
    base_trsf_ = base_trsf;
    tool_tcp_trsf_ = tool_tcp_trsf;
}

gp_Trsf TrajectoryPanel::FlangeBaseToTcpWorld(const gp_Trsf& flange_base) const
{
    // tcp_world = base_trsf * flange_base * tool_tcp_trsf
    gp_Trsf result = base_trsf_;
    result.Multiply(flange_base);
    result.Multiply(tool_tcp_trsf_);
    return result;
}

void TrajectoryPanel::SetTrajectory(const domain::Trajectory& traj)
{
    trajectory_ = traj;
    selected_row_ = -1;
    ik_combo_->clear();
    PopulateTable();

    int total = static_cast<int>(traj.points.size());
    int errors = traj.ErrorCount();
    if (errors > 0) {
        summary_label_->setText(
            tr("Total: %1 points | Errors: %2").arg(total).arg(errors));
    } else {
        summary_label_->setText(
            tr("Total: %1 points | Ready to play").arg(total));
    }
}

void TrajectoryPanel::PopulateTable()
{
    table_->setRowCount(0);
    int row_count = static_cast<int>(trajectory_.points.size());
    table_->setRowCount(row_count);

    for (int i = 0; i < row_count; ++i) {
        const auto& pt = trajectory_.points[i];

        // Index
        table_->setItem(i, 0, new QTableWidgetItem(QString::number(i)));

        // Move type
        QString type_str = (pt.move_type == domain::MoveType::kMoveJ)
            ? "MJ" : "ML";
        table_->setItem(i, 1, new QTableWidgetItem(type_str));

        // TCP pose in world coordinates
        gp_Trsf tcp_world = FlangeBaseToTcpWorld(foundation::ToGpTrsf(pt.tcp_pose));
        nl::utils::Vector3d rpy, pos;
        nl::occ::TrsfToRpyPos(tcp_world, rpy, pos);
        table_->setItem(i, 2, new QTableWidgetItem(
            QString::number(pos.x, 'f', 1)));
        table_->setItem(i, 3, new QTableWidgetItem(
            QString::number(pos.y, 'f', 1)));
        table_->setItem(i, 4, new QTableWidgetItem(
            QString::number(pos.z, 'f', 1)));
        table_->setItem(i, 5, new QTableWidgetItem(
            QString::number(rpy.x, 'f', 1)));
        table_->setItem(i, 6, new QTableWidgetItem(
            QString::number(rpy.y, 'f', 1)));
        table_->setItem(i, 7, new QTableWidgetItem(
            QString::number(rpy.z, 'f', 1)));

        // Status
        QString status_str;
        switch (pt.status) {
        case domain::TrajectoryPointStatus::kOk:        status_str = "OK"; break;
        case domain::TrajectoryPointStatus::kIkFailed:   status_str = "IK FAIL"; break;
        case domain::TrajectoryPointStatus::kJointJump:  status_str = "JUMP"; break;
        }
        table_->setItem(i, 8, new QTableWidgetItem(status_str));

        // Waypoint index
        QString wp_str = (pt.waypoint_index >= 0)
            ? QString("wp%1").arg(pt.waypoint_index) : "";
        table_->setItem(i, 9, new QTableWidgetItem(wp_str));

        HighlightRow(i, pt.status);

        // Bold waypoint rows
        if (pt.waypoint_index >= 0) {
            QFont bold_font = table_->font();
            bold_font.setBold(true);
            for (int c = 0; c < table_->columnCount(); ++c) {
                if (auto* item = table_->item(i, c))
                    item->setFont(bold_font);
            }
        }
    }

    table_->resizeColumnsToContents();
}

void TrajectoryPanel::HighlightRow(int row,
    domain::TrajectoryPointStatus status)
{
    QColor bg;
    switch (status) {
    case domain::TrajectoryPointStatus::kOk:
        return;  // Default background
    case domain::TrajectoryPointStatus::kIkFailed:
        bg = QColor(255, 180, 180);  // Light red
        break;
    case domain::TrajectoryPointStatus::kJointJump:
        bg = QColor(255, 240, 180);  // Light yellow
        break;
    }

    for (int c = 0; c < table_->columnCount(); ++c) {
        if (auto* item = table_->item(row, c))
            item->setBackground(bg);
    }
}

void TrajectoryPanel::UpdatePoint(int index,
    const domain::TrajectoryPoint& point)
{
    if (index < 0 || index >= static_cast<int>(trajectory_.points.size()))
        return;

    trajectory_.points[index] = point;

    // Refresh the single row — display as TCP world coordinates
    gp_Trsf tcp_world = FlangeBaseToTcpWorld(foundation::ToGpTrsf(point.tcp_pose));
    nl::utils::Vector3d rpy, pos;
    nl::occ::TrsfToRpyPos(tcp_world, rpy, pos);

    table_->item(index, 2)->setText(QString::number(pos.x, 'f', 1));
    table_->item(index, 3)->setText(QString::number(pos.y, 'f', 1));
    table_->item(index, 4)->setText(QString::number(pos.z, 'f', 1));
    table_->item(index, 5)->setText(QString::number(rpy.x, 'f', 1));
    table_->item(index, 6)->setText(QString::number(rpy.y, 'f', 1));
    table_->item(index, 7)->setText(QString::number(rpy.z, 'f', 1));

    QString status_str;
    switch (point.status) {
    case domain::TrajectoryPointStatus::kOk:        status_str = "OK"; break;
    case domain::TrajectoryPointStatus::kIkFailed:   status_str = "IK FAIL"; break;
    case domain::TrajectoryPointStatus::kJointJump:  status_str = "JUMP"; break;
    }
    table_->item(index, 8)->setText(status_str);

    // Reset row background then re-highlight
    for (int c = 0; c < table_->columnCount(); ++c) {
        if (auto* item = table_->item(index, c))
            item->setBackground(QColor());
    }
    HighlightRow(index, point.status);

    // Update summary
    int total = static_cast<int>(trajectory_.points.size());
    int errors = trajectory_.ErrorCount();
    summary_label_->setText(
        tr("Total: %1 points | Errors: %2").arg(total).arg(errors));
}

void TrajectoryPanel::SetIkSolutions(
    const std::vector<nl::utils::Q>& solutions)
{
    ik_combo_->blockSignals(true);
    ik_combo_->clear();
    for (size_t i = 0; i < solutions.size(); ++i) {
        QString label = tr("Solution %1").arg(i + 1);
        ik_combo_->addItem(label);
    }
    if (!solutions.empty())
        ik_combo_->setCurrentIndex(0);
    ik_combo_->blockSignals(false);
}

void TrajectoryPanel::Clear()
{
    trajectory_.points.clear();
    table_->setRowCount(0);
    ik_combo_->clear();
    selected_row_ = -1;
    summary_label_->setText("");
}

void TrajectoryPanel::OnTableRowClicked(int row, int /*column*/)
{
    selected_row_ = row;
    emit PointSelected(row);
}

void TrajectoryPanel::OnPlanClicked()
{
    emit PlanRequested(approach_spin_->value());
}

void TrajectoryPanel::OnIkComboChanged(int index)
{
    if (selected_row_ >= 0 && index >= 0) {
        emit IkSolutionChanged(selected_row_, index);
    }
}

} // namespace ui
} // namespace nl
