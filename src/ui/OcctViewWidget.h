#ifndef GRINDINGAPP_SRC_UI_OCCT_VIEW_WIDGET_H_
#define GRINDINGAPP_SRC_UI_OCCT_VIEW_WIDGET_H_

#include <QWidget>
#include <Standard_Handle.hxx>

#include "GrindingUIExport.h"

// Forward declarations to avoid pulling large OCCT headers.
class V3d_Viewer;
class V3d_View;
class AIS_InteractiveContext;
class WNT_Window;
class OpenGl_GraphicDriver;

namespace nl {
namespace ui {

class GRINDING_UI_EXPORT OcctViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit OcctViewWidget(QWidget* parent = nullptr);
    ~OcctViewWidget() override = default;

    Handle(AIS_InteractiveContext) Context() const { return context_; }
    Handle(V3d_View)               View()    const { return view_; }
    Handle(V3d_Viewer)             Viewer()  const { return viewer_; }

protected:
    QPaintEngine* paintEngine() const override { return nullptr; }
    void paintEvent(QPaintEvent* e) override;
    void resizeEvent(QResizeEvent* e) override;
    void showEvent(QShowEvent* e) override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void mouseReleaseEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

signals:
    void ShapeSelected();

private:
    void InitOcct();

    Handle(V3d_Viewer)             viewer_;
    Handle(V3d_View)               view_;
    Handle(AIS_InteractiveContext) context_;
    Handle(WNT_Window)             occtWindow_;
    bool                           initialized_ = false;

    QPoint last_pos_;
    bool   rotating_ = false;
    bool   panning_  = false;
};

} // namespace ui
} // namespace nl

#endif  // GRINDINGAPP_SRC_UI_OCCT_VIEW_WIDGET_H_
