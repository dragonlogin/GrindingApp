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
#include "UrdfParser.h"
#include "StlLoader.h"

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
		this, tr("Open URDF File"), "", tr("URDF Files (*.urdf)"));
	if (path.isEmpty())
		return;

	QVector<UrdfLink> links = UrdfParser::Parse(path);
	qDebug() << "Robot links:" << links.size();
	for (const UrdfLink& link : links) {
		TopoDS_Shape shape = link.mesh_path.isEmpty()
			? TopoDS_Shape()
			: StlLoader::Load(link.mesh_path);
		qDebug() << " -" << link.name
			<< (shape.IsNull() ? "(no mesh)" : "(mesh OK)");
	}
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
