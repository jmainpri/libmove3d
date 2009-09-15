#include "Graphic-pkg.h"

#include "qtMainWindow.hpp"

#include <iostream>
#include <fcntl.h>
#include "cppToQt.hpp"

using namespace std;

/**
 * @defgroup qtWindow The Qt Window
 */

extern int main_old(int argc, char** argv);

/**
 * @ingroup qtWindow
 * @brief Main double thread class (X-Forms Thread)
 */
class Fl_thread: public QThread {

	Q_OBJECT;

public:
	int _argc;
	char** _argv;

	Fl_thread(QObject* parent = 0) :
		QThread(parent) {
	}

	Fl_thread(int argc, char** argv, QObject* parent = 0) :
		QThread(parent) {
		_argc = argc;
		_argv = argv;
	};

protected:
	void run() {
		main_old(_argc,_argv);
		cout << "Ends main_old" << endl;
		terminate();
		wait();
	}


public slots:

void drawAllWindowActive() {
		cout << "g3d_draw_allwin_active()" << endl;
		g3d_draw_allwin_active();
	}

};

/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
class MainProgram : public QObject {

	Q_OBJECT;

	QApplication* app;
	MainWidget* widget;


public:

	MainProgram() {

	}

	~MainProgram() {
		//delete (widget);
		//delete (app);
	}

public:
	int run(int argc, char** argv) {

		app = new QApplication(argc,argv);
		app->setStyle(new QCleanlooksStyle());

		widget = new MainWidget();

		Fl_thread move3dthread(argc, argv);
		connect(&move3dthread,SIGNAL(terminated()),this,SLOT(exit()));
		//connect(ENV.getObject(Env::drawAll), SIGNAL(valueChanged(bool)),&move3dthread,SLOT(drawAllWindowActive()));
		move3dthread.start();

		widget->show();

		return app->exec();
	}

private slots :

	void exit() {
		cout<< "Ends all threads" << endl;
		app->quit ();
	}

};

int qt_fl_pipe[2];

/**
 * @ingroup qtWindow
 * @brief Main function of Move3D
 */
int main(int argc, char *argv[]) {

	bool qt_flag = true;

	if(qt_flag){
		pipe(qt_fl_pipe);
		fcntl(qt_fl_pipe[0], F_SETFL, O_NONBLOCK);
		MainProgram main;
		return main.run(argc,argv);
	}
	else{
		return main_old(argc,argv);
	}
}

#include "main.moc"
