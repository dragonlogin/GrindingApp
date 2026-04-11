#include "OcctViewWidget.h"

#ifdef _WIN32
#include <windows.h>
#include <WNT_Window.hxx>
#elif defined(__APPLE__)
#include <Cocoa_Window.hxx>
#else
#include <Xw_Window.hxx>
#include <Aspect_DisplayConnection.hxx>
#endif
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

    InitOcct();
}

void OcctViewWidget::showEvent(QShowEvent* e)
{
    QWidget::showEvent(e);
    //if (!initialized_) InitOcct();
}

void OcctViewWidget::mousePressEvent(QMouseEvent* e)
{
    last_pos_ = e->pos();
    drag_started_ = false;

    const bool alt = e->modifiers() & Qt::AltModifier;

    if (e->button() == Qt::LeftButton) {
        if (alt) {
            panning_ = true;
        } else {
            rotating_ = true;
            view_->StartRotation(e->pos().x(), e->pos().y());
        }
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

    if (dx != 0 || dy != 0)
        drag_started_ = true;

    if (rotating_) {
        view_->Rotation(e->pos().x(), e->pos().y());
    } else if (panning_) {
        view_->Pan(dx, -dy);
    }

    last_pos_ = e->pos();
    view_->Redraw();
}

void OcctViewWidget::mouseReleaseEvent(QMouseEvent* e)
{
    if (e->button() == Qt::LeftButton) {
        if (rotating_ || panning_) {
            rotating_ = false;
            panning_  = false;
        } else if (!drag_started_) {
            // 没有拖拽 → 选择
            context_->MoveTo(e->pos().x(), e->pos().y(), view_, Standard_True);
            context_->Select(Standard_True);
            emit ShapeSelected();
        }
    }

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
        Quantity_Color(0.2, 0.2, 0.2, Quantity_TOC_RGB));

    // 3. View
    view_ = viewer_->CreateView();

    // 4. 绑定到 Qt Widget 的 native handle（跨平台）
#ifdef _WIN32
    occtWindow_ = new WNT_Window(reinterpret_cast<HWND>(winId()));
#elif defined(__APPLE__)
    occtWindow_ = new Cocoa_Window(reinterpret_cast<NSView*>(winId()));
#else
    Handle(Aspect_DisplayConnection) xdisp = new Aspect_DisplayConnection();
    occtWindow_ = new Xw_Window(xdisp, (Window)winId());
#endif
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
