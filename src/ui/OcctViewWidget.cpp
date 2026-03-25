#include "OcctViewWidget.h"

// Windows 必须在 OCCT Window 头之前
#include <windows.h>
#include <WNT_Window.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <V3d_Viewer.hxx>
#include <V3d_View.hxx>
#include <AIS_InteractiveContext.hxx>
#include <Quantity_Color.hxx>
#include <Aspect_TypeOfTriedronPosition.hxx>

#include <QShowEvent>
#include <QResizeEvent>
#include <QPaintEvent>

namespace nl {
namespace ui {

OcctViewWidget::OcctViewWidget(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_PaintOnScreen, true);
    setAttribute(Qt::WA_NoSystemBackground, true);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    setMinimumSize(200, 200);
}

void OcctViewWidget::showEvent(QShowEvent* e)
{
    QWidget::showEvent(e);
    if (!initialized_) InitOcct();
}

void OcctViewWidget::mousePressEvent(QMouseEvent* e)
{
    last_pos_ = e->pos();
    if (e->button() == Qt::RightButton) {
        rotating_ = true;
        view_->StartRotation(e->pos().x(), e->pos().y());
    }

    if (e->button() == Qt::MidButton)
        panning_ = true;
}

void OcctViewWidget::mouseMoveEvent(QMouseEvent* e)
{
    if (!initialized_)
        return;

    const int dx = e->pos().x() - last_pos_.x();
    const int dy = e->pos().y() - last_pos_.y();

    if (rotating_) {
        view_->Rotation(e->pos().x(), e->pos().y());
    }
    else if (panning_) {
        view_->Pan(dx, -dy);
    }

    last_pos_ = e->pos();
    view_->Redraw();
}

void OcctViewWidget::mouseReleaseEvent(QMouseEvent* e)
{
    if (e->button() == Qt::RightButton)
        rotating_ = false;
    if (e->button() == Qt::MidButton)
        panning_ = false;

    view_->Redraw();
}

void OcctViewWidget::wheelEvent(QWheelEvent* e)
{
    if (!initialized_)
        return;

    const double factor = e->angleDelta().y() > 0 ? 1.1 : 0.9;
    view_->SetZoom(factor, true);
    view_->Redraw();
}

void OcctViewWidget::InitOcct()
{
    // 1. Graphic driver（Windows 传空 DisplayConnection）
    Handle(Aspect_DisplayConnection) disp;
    Handle(OpenGl_GraphicDriver) driver =
        new OpenGl_GraphicDriver(disp, Standard_False);

    // 2. Viewer
    viewer_ = new V3d_Viewer(driver);
    viewer_->SetDefaultViewSize(1000.0);
    viewer_->SetDefaultLights();
    viewer_->SetLightOn();
	// 灰色背景
    viewer_->SetDefaultBackgroundColor(
        Quantity_Color(0.3, 0.3, 0.3, Quantity_TOC_RGB));

    // 3. View
    view_ = viewer_->CreateView();

    // 4. 绑定到 Qt Widget 的 HWND
    HWND hwnd = reinterpret_cast<HWND>(winId());
    occtWindow_ = new WNT_Window(hwnd);
    view_->SetWindow(occtWindow_);
    if (!occtWindow_->IsMapped()) occtWindow_->Map();

    // 5. AIS 交互上下文
    context_ = new AIS_InteractiveContext(viewer_);

    // 6. 首次渲染
    view_->MustBeResized();
    view_->TriedronDisplay(Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.2, V3d_ZBUFFER);
    view_->Redraw();

    initialized_ = true;
}

void OcctViewWidget::paintEvent(QPaintEvent*)
{
    if (!initialized_) return;
    view_->Redraw();
}

void OcctViewWidget::resizeEvent(QResizeEvent* e)
{
    QWidget::resizeEvent(e);
    if (initialized_) view_->MustBeResized();
}

} // namespace ui
} // namespace nl
