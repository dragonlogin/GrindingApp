#include <QApplication>
#include <QTranslator>
#include <QSettings>
#include <QLocale>
#include "MainWindow.h"

int main(int argc, char* argv[])
{
	QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
	QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
	QApplication app(argc, argv);
	app.setApplicationName("GrindingApp");

	// 读取语言设置
	QSettings settings("GrindingApp", "GrindingApp");
	QString lang = settings.value("language", "en").toString();

	QTranslator translator;
	if (lang == "zh") {
		// 从资源文件加载
		translator.load(":/translations/zh_CN.qm");
		app.installTranslator(&translator);
	}

	MainWindow w;
	w.resize(1280, 800);
	w.show();
	return app.exec();
}
