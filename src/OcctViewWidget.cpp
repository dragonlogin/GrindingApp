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

#include <QShowEvent>
#include <QResizeEvent>
#include <QPaintEvent>

OcctViewWidget::OcctViewWidget(QWidget* parent)
	: QWidget(parent)
{
	// 必须在构造时设置，否则 Qt 会干预 OpenGL 绘制
	setAttribute(Qt::WA_PaintOnScreen, true);
	setAttribute(Qt::WA_NoSystemBackground, true);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setMinimumSize(200, 200);
}

void OcctViewWidget::showEvent(QShowEvent* e)
{
	QWidget::showEvent(e);
	// winId() 在 show 之后才有效，所以 InitOcct 放在这里
	if (!initialized_) InitOcct();
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