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

private:
    // MainWindow.h 新增
    void SwitchLanguage(const QString& lang);

    void SetupMenuBar();
    void SetupToolBar();
    void SetupStatusBar();
    void SetupCentralWidget();

    QLabel* modelInfo_;   // 状态栏：模型信息
    QLabel* coordLabel_;  // 状态栏：坐标显示
    OcctViewWidget* viewer_ = nullptr;
};
