#include "MainWindow.h"

#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QLabel>
#include <QApplication>
#include <QSettings>
#include <QMessageBox>

#include "OcctViewWidget.h"

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
	fileMenu->addAction(tr("Import Workpiece(&I)"), this, [] { /* 后续实现 */ },
		QKeySequence("Ctrl+O"));
	fileMenu->addSeparator();
	fileMenu->addAction(tr("Exit(&Q)"), qApp, &QApplication::quit,
		QKeySequence("Ctrl+Q"));

	// 视图菜单
	QMenu* viewMenu = menuBar()->addMenu(tr("View(&V)"));
	viewMenu->addAction(tr("Front View"));
	viewMenu->addAction(tr("Top View"));
	viewMenu->addAction(tr("Side View"));
	viewMenu->addAction(tr("Isometric"));
	viewMenu->addSeparator();
	viewMenu->addAction(tr("Wireframe"));
	viewMenu->addAction(tr("Shaded"));
	viewMenu->addSeparator();
	viewMenu->addAction(tr("Fit All"));
}

void MainWindow::SetupToolBar()
{
	QToolBar* tb = addToolBar("主工具栏");
	tb->setMovable(false);
	tb->addAction(tr("Import Workpiece(&I)"));
	tb->addSeparator();
	tb->addAction(tr("Front View"));
	tb->addAction(tr("Top View"));
	tb->addAction(tr("Side View"));
	tb->addAction(tr("Isometric"));
	tb->addSeparator();
	tb->addAction(tr("Wireframe"));
	tb->addAction(tr("Shaded"));
	tb->addAction(tr("Fit All"));
}

void MainWindow::SetupStatusBar()
{
	modelInfo_ = new QLabel("No model loaded");
	coordLabel_ = new QLabel("X: - Y: - Z: -");

	statusBar()->addWidget(modelInfo_, 1);
	statusBar()->addPermanentWidget(coordLabel_);
}

void MainWindow::SetupCentralWidget()
{
	viewer_ = new OcctViewWidget(this);
	setCentralWidget(viewer_);
}
