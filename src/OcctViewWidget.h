#pragma once

#include <QWidget>
#include <Standard_Handle.hxx>

//// 前向声明（避免在头文件引入大量 OCCT 头）
class V3d_Viewer;
class V3d_View;
class AIS_InteractiveContext;
class WNT_Window;
class OpenGl_GraphicDriver;

class OcctViewWidget : public QWidget
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

private:
	void InitOcct();

	Handle(V3d_Viewer)             viewer_;
	Handle(V3d_View)               view_;
	Handle(AIS_InteractiveContext) context_;
	Handle(WNT_Window)             occtWindow_;
	bool                           initialized_ = false;


	// 右键旋转 / 左键平移 / 滚轮缩放
	QPoint   last_pos_;
	bool     rotating_ = false;
	bool     panning_ = false;
};