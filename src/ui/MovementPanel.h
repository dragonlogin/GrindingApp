#ifndef GRINDINGAPP_SRC_UI_MOVEMENT_PANEL_H_
#define GRINDINGAPP_SRC_UI_MOVEMENT_PANEL_H_

#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QPushButton>

#include <gp_Trsf.hxx>

#include "GrindingUIExport.h"

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT MovementPanel : public QDockWidget
{
    Q_OBJECT

public:
    explicit MovementPanel(const QString& target_name,
                           const gp_Trsf& initial_trsf,
                           QWidget* parent = nullptr);

    gp_Trsf GetInitialTrsf() const { return initial_trsf_; }

signals:
    void PreviewRequested(const gp_Trsf& trsf);
    void Accepted(const gp_Trsf& trsf);
    void Cancelled();

private:
    void SetupUi(const QString& target_name);
    void OnSpinBoxEditingFinished();
    gp_Trsf ComputeTrsf() const;

    QDoubleSpinBox* spinboxes_[6] = {};
    QPushButton*    ok_btn_ = nullptr;
    QPushButton*    cancel_btn_ = nullptr;

    gp_Trsf initial_trsf_;
};

} // namespace ui
} // namespace nl

#endif // GRINDINGAPP_SRC_UI_MOVEMENT_PANEL_H_
