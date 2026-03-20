#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QVector>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QDockWidget>
#include <QTreeWidget>

#include <AIS_Trihedron.hxx>
#include <AIS_Shape.hxx>
#include <gp_Trsf.hxx>

#include "RbXmlParser.h"

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
	void OnLoadRobot();
	void OnSceneTreeContextMenu(const QPoint& pos);
private:
    // MainWindow.h 新增
    void SwitchLanguage(const QString& lang);

    void SetupMenuBar();
    void SetupToolBar();
    void SetupStatusBar();
    void SetupCentralWidget();

	void SetupJogPanel();
	void UpdateRobotDisplay();
	void UpdateCoordinateFrames(const QVector<gp_Trsf>& fk);

	void SetupSceneTree();
	void AddRobot(const QString& name);
	void UpdateRobotJoints();                      // 更新关节角度，从 UpdateRobotDisplay 调用
	void AddTool(const QString& name,
		const QString& parent_role);      // parent_role: "robot" 或 "station"
	void AddWorkpiece(const QString& name,
		const QString& parent_role); // parent_role: "robot" 或 "station"

    QLabel* model_info_;   // 状态栏：模型信息
    QLabel* coord_label_;  // 状态栏：坐标显示
    OcctViewWidget* viewer_ = nullptr;

	// Jog 状态
	struct RobotMesh {
		RbDrawable   drawable;
		TopoDS_Shape original;
		Handle(AIS_Shape) ais;
	};
	QVector<RobotMesh>   robot_meshes_;
	RbRobot              current_robot_;
	double               joint_angles_[6] = {};  // 当前关节角，单位：度

	QSlider* joint_sliders_[6] = {};
	QDoubleSpinBox* joint_spinboxes_[6] = {};

	QTreeWidget*                    scene_tree_   = nullptr;
	QVector<Handle(AIS_Trihedron)>  joint_frames_;  // index = joint index
	Handle(AIS_Trihedron)           base_frame_;    // robot base (always visible)
};
