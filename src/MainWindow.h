#pragma once

#include <QMainWindow>
#include <QLabel>


class OcctViewWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override = default;

private slots:
	void OnImportWorkpiece();
	void OnViewFront();
	void OnViewTop();
	void OnViewSide();
	void OnViewIsometric();
	void OnViewWireframe();
	void OnViewShaded();
	void OnFitAll();
private:
    // MainWindow.h 新增
    void SwitchLanguage(const QString& lang);

    void SetupMenuBar();
    void SetupToolBar();
    void SetupStatusBar();
    void SetupCentralWidget();

    QLabel* model_info_;   // 状态栏：模型信息
    QLabel* coord_label_;  // 状态栏：坐标显示
    OcctViewWidget* viewer_ = nullptr;
};
