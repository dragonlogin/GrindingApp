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

	for (auto& s : robot_shapes_)
		viewer_->Context()->Remove(s, Standard_False);
	robot_shapes_.clear();

	QVector<gp_Trsf> fk = ComputeFkHome(robot);

	auto joint_index = [&](const QString& name) -> int {
		for (int i = 0; i < robot.joints.size(); ++i)
			if (robot.joints[i].name == name)
				return i;
		return -1;
	};

	for (const RbDrawable& drw : robot.drawables) {
		TopoDS_Shape shape = StlLoader::Load(drw.mesh_file);
		if (shape.IsNull()) {
			qDebug() << " -" << drw.name << "(mesh not found)";
			continue;
		}
		gp_Trsf mesh_trsf = RpyPosTrsf(drw.rpy, drw.pos);
		int idx = joint_index(drw.ref_joint);
		if (idx >= 0) {
			gp_Trsf world = fk[idx];
			world.Multiply(mesh_trsf);
			mesh_trsf = world;
		}
		Handle(AIS_Shape) ais = new AIS_Shape(shape);
		ais->SetLocalTransformation(mesh_trsf);
		viewer_->Context()->Display(ais, AIS_Shaded, 0, Standard_False);
		robot_shapes_.append(ais);
		qDebug() << " -" << drw.name << "(ok)";
	}

	viewer_->Context()->UpdateCurrentViewer();
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
