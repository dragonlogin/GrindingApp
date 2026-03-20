#include "MainWindow.h"

#include <QDebug>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>
#include <QApplication>
#include <QSettings>
#include <QMessageBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QGridLayout>

#include <AIS_Shape.hxx>
#include <AIS_InteractiveContext.hxx>
#include <V3d_View.hxx>

#include "OcctViewWidget.h"
#include "StepImporter.h"
#include "StlLoader.h"
#include "RobotDisplay.h"

MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
{
	setWindowTitle("GrindingApp");

	SetupMenuBar();
	SetupToolBar();
	SetupStatusBar();
	SetupCentralWidget();
	SetupJogPanel();
}



void MainWindow::SwitchLanguage(const QString& lang)
{
	QSettings settings("GrindingApp", "GrindingApp");
	settings.setValue("language", lang);
	QMessageBox::information(this, tr("Language"),
		tr("Language changed. Please restart the application."));
}

void MainWindow::SetupMenuBar()
{
	// 文件菜单

	QMenu* fileMenu = menuBar()->addMenu(tr("File(&F)"));
	fileMenu->addAction(tr("Load Robot(&R)"), this, &MainWindow::OnLoadRobot,
		QKeySequence("Ctrl+R"));

	fileMenu->addAction(tr("Import Workpiece(&I)"), this, &MainWindow::OnImportWorkpiece, QKeySequence("Ctrl+O"));
	fileMenu->addSeparator();
	fileMenu->addAction(tr("Exit(&Q)"), qApp, &QApplication::quit,
		QKeySequence("Ctrl+Q"));

	// 视图菜单
	QMenu* viewMenu = menuBar()->addMenu(tr("View(&V)"));
	viewMenu->addAction(tr("Front View"), this, &MainWindow::OnViewFront);
	viewMenu->addAction(tr("Top View"), this, &MainWindow::OnViewTop);
	viewMenu->addAction(tr("Side View"), this, &MainWindow::OnViewSide);
	viewMenu->addAction(tr("Isometric"), this, &MainWindow::OnViewIsometric);
	viewMenu->addSeparator();
	viewMenu->addAction(tr("Wireframe"), this, &MainWindow::OnViewWireframe);
	viewMenu->addAction(tr("Shaded"), this, &MainWindow::OnViewShaded);
	viewMenu->addSeparator();
	viewMenu->addAction(tr("Fit All"), this, &MainWindow::OnFitAll);
}

void MainWindow::SetupToolBar()
{
	QToolBar* tb = addToolBar("主工具栏");
	tb->setMovable(false);
	tb->addAction(tr("Import Workpiece"), this, &MainWindow::OnImportWorkpiece);
	tb->addSeparator();
	tb->addAction(tr("Front View"), this, &MainWindow::OnViewFront);
	tb->addAction(tr("Top View"), this, &MainWindow::OnViewTop);
	tb->addAction(tr("Side View"), this, &MainWindow::OnViewSide);
	tb->addAction(tr("Isometric"), this, &MainWindow::OnViewIsometric);
	tb->addSeparator();
	tb->addAction(tr("Wireframe"), this, &MainWindow::OnViewWireframe);
	tb->addAction(tr("Shaded"), this, &MainWindow::OnViewShaded);
	tb->addAction(tr("Fit All"), this, &MainWindow::OnFitAll);
}

void MainWindow::SetupStatusBar()
{
	model_info_ = new QLabel("No model loaded");
	coord_label_ = new QLabel("X: - Y: - Z: -");

	statusBar()->addWidget(model_info_, 1);
	statusBar()->addPermanentWidget(coord_label_);
}

void MainWindow::SetupCentralWidget()
{
	viewer_ = new OcctViewWidget(this);
	setCentralWidget(viewer_);
}

void MainWindow::SetupJogPanel()
{
	auto dock = new QDockWidget(tr("Joint Jog"), this);
	auto widget = new QWidget;
	auto layout = new QGridLayout(widget);

	const QString labels[6] = { "J1","J2","J3","J4","J5","J6" };

	for (int i = 0; i < 6; ++i) {
		layout->addWidget(new QLabel(labels[i]), i, 0);

		auto* slider = new QSlider(Qt::Horizontal);
		slider->setRange(-1800, 1800);   // ±180° × 10
		slider->setValue(0);
		joint_sliders_[i] = slider;
		layout->addWidget(slider, i, 1);

		auto* spin = new QDoubleSpinBox;
		spin->setRange(-180.0, 180.0);
		spin->setDecimals(1);
		spin->setSuffix("°");
		spin->setValue(0.0);
		joint_spinboxes_[i] = spin;
		layout->addWidget(spin, i, 2);

		// 滑块 → spinbox
		connect(slider, &QSlider::valueChanged, this, [this, i, spin](int v) {
			spin->blockSignals(true);
			spin->setValue(v / 10.0);
			spin->blockSignals(false);
			joint_angles_[i] = v / 10.0;
			UpdateRobotDisplay();
			});
		// spinbox → 滑块
		connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
			this, [this, i, slider](double v) {
				slider->blockSignals(true);
				slider->setValue(qRound(v * 10));
				slider->blockSignals(false);
				joint_angles_[i] = v;
				UpdateRobotDisplay();
			});
	}

	dock->setWidget(widget);
	addDockWidget(Qt::RightDockWidgetArea, dock);
}

void MainWindow::UpdateRobotDisplay()
{
	if (robot_meshes_.isEmpty()) return;

	// 重算 FK（home offset + jog 角度）
	QVector<gp_Trsf> fk(current_robot_.joints.size());
	for (int i = 0; i < current_robot_.joints.size(); ++i) {
		const RbJoint& j = current_robot_.joints[i];
		double theta = j.offset_deg + (i < 6 ? joint_angles_[i] : 0.0);
		gp_Trsf dh = DhTrsf(theta, j.d, j.a, j.alpha_deg);
		fk[i] = (i == 0) ? dh : fk[i - 1];
		if (i > 0) fk[i].Multiply(dh);
	}

	auto joint_idx = [&](const QString& name) -> int {
		for (int i = 0; i < current_robot_.joints.size(); ++i)
			if (current_robot_.joints[i].name == name) return i;
		return -1;
		};

	for (auto& m : robot_meshes_) {
		gp_Trsf local = RpyPosTrsf(m.drawable.rpy, m.drawable.pos);
		int idx = joint_idx(m.drawable.ref_joint);
		if (idx >= 0) {
			gp_Trsf world = fk[idx];
			world.Multiply(local);
			local = world;
		}
		viewer_->Context()->SetLocation(m.ais, TopLoc_Location(local));
	}
	viewer_->Context()->UpdateCurrentViewer();
}

void MainWindow::OnLoadRobot()
{
	QString path = QFileDialog::getOpenFileName(
		this, tr("Open Robot File"), "",
		tr("Robot Files (*.xml)"));
	if (path.isEmpty())
		return;

	RbRobot robot = RbXmlParser::Parse(path);
	qDebug() << "Robot:" << robot.name
		<< "joints:" << robot.joints.size()
		<< "drawables:" << robot.drawables.size();

	for (auto& m : robot_meshes_)
		viewer_->Context()->Remove(m.ais, Standard_False);
	robot_meshes_.clear();

	current_robot_ = robot;
	memset(joint_angles_, 0, sizeof(joint_angles_));
	for (int i = 0; i < 6; ++i) {
		if (joint_sliders_[i]) joint_sliders_[i]->setValue(0);
		if (joint_spinboxes_[i]) joint_spinboxes_[i]->setValue(0.0);
	}

	for (const RbDrawable& drw : robot.drawables) {
		TopoDS_Shape shape = StlLoader::Load(drw.mesh_file);
		if (shape.IsNull()) { qDebug() << drw.name << "not found"; continue; }

		Handle(AIS_Shape) ais = new AIS_Shape(shape);
		viewer_->Context()->Display(ais, AIS_Shaded, 0, Standard_False);
		robot_meshes_.append({ drw, shape, ais });
	}
	viewer_->Context()->UpdateCurrentViewer();
	UpdateRobotDisplay();   // 应用 home 姿态
	viewer_->View()->FitAll();
}


void MainWindow::OnImportWorkpiece()
{
	QString path = QFileDialog::getOpenFileName(
		this, tr("Open STEP File"), "", tr("STEP Files (*.step *.stp)"));
	if (path.isEmpty())
		return;
	int face_count = 0;
	auto shape = StepImporter::Load(path, &face_count);
	if (shape.IsNull())
		return;

	Handle(AIS_Shape) ais_shape = new AIS_Shape(shape);
	viewer_->Context()->Display(ais_shape, Standard_True);
	viewer_->View()->FitAll();

	QString file_name = QFileInfo(path).fileName();
	model_info_->setText(tr("File: %1 | Face: %2").arg(file_name).arg(face_count));
}

void MainWindow::OnViewFront() 
{ 
	viewer_->View()->SetProj(V3d_Yneg);          
	viewer_->View()->FitAll(); 
}

void MainWindow::OnViewTop() 
{ 
	viewer_->View()->SetProj(V3d_Zpos);          
	viewer_->View()->FitAll(); 
}

void MainWindow::OnViewSide() 
{ 
	viewer_->View()->SetProj(V3d_Xpos);          
	viewer_->View()->FitAll(); 
}

void MainWindow::OnViewIsometric() 
{ 
	viewer_->View()->SetProj(V3d_XposYnegZpos);  
	viewer_->View()->FitAll(); 
}

void MainWindow::OnViewWireframe()
{
	viewer_->Context()->SetDisplayMode(AIS_WireFrame, Standard_True);
}

void MainWindow::OnViewShaded()
{
	viewer_->Context()->SetDisplayMode(AIS_Shaded, Standard_True);
}

void MainWindow::OnFitAll()
{
	viewer_->View()->FitAll();
}
