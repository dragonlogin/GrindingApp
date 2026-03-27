#ifndef GRINDINGAPP_SRC_UI_TRAJECTORY_PANEL_H_
#define GRINDINGAPP_SRC_UI_TRAJECTORY_PANEL_H_

#include <vector>

#include <QDockWidget>
#include <gp_Trsf.hxx>

#include "GrindingUIExport.h"
#include "Trajectory.h"

class QTableWidget;
class QDoubleSpinBox;
class QPushButton;
class QComboBox;
class QLabel;

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT TrajectoryPanel : public QDockWidget {
    Q_OBJECT

public:
    explicit TrajectoryPanel(QWidget* parent = nullptr);

    void SetDisplayTransforms(const gp_Trsf& base_trsf, const gp_Trsf& tool_tcp_trsf);
    void SetTrajectory(const nl::occ::Trajectory& traj);
    void UpdatePoint(int index, const nl::occ::TrajectoryPoint& point);
    void SetIkSolutions(const std::vector<nl::utils::Q>& solutions);
    void Clear();

signals:
    void PlanRequested(double approach_dist);
    void PointSelected(int index);
    void IkSolutionChanged(int point_index, int solution_index);

private:
    void SetupUi();
    void PopulateTable();
    void HighlightRow(int row, nl::occ::TrajectoryPoint::Status status);
    void OnTableRowClicked(int row, int column);
    void OnPlanClicked();
    void OnIkComboChanged(int index);

    QTableWidget*    table_ = nullptr;
    QDoubleSpinBox*  approach_spin_ = nullptr;
    QPushButton*     plan_btn_ = nullptr;
    QComboBox*       ik_combo_ = nullptr;
    QLabel*          summary_label_ = nullptr;

    gp_Trsf FlangeBaseToTcpWorld(const gp_Trsf& flange_base) const;

    nl::occ::Trajectory trajectory_;
    int selected_row_ = -1;

    gp_Trsf base_trsf_;
    gp_Trsf tool_tcp_trsf_;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_TRAJECTORY_PANEL_H_
