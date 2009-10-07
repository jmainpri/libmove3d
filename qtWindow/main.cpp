#include "Graphic-pkg.h"

#include "qtMainWindow.hpp"
#ifdef QT_GL
#include "qtOpenGL/qtGLWindow.hpp"
#endif

#include <iostream>
#include <fcntl.h>
#include "cppToQt.hpp"

#ifdef QT_GL
QSystemSemaphore sem("market", 0, QSystemSemaphore::Create);
Move3D2OpenGl* pipe2openGl;
#endif

using namespace std;

/**
 * @defgroup qtWindow The Qt Window
 */

extern int main_old(int argc, char** argv);

/**
 * @ingroup qtWindow
 * @brief Main double thread class (X-Forms Thread)
 */
class Fl_thread: public QThread
{

Q_OBJECT
	;

public:
	int _argc;
	char** _argv;

	Fl_thread(QObject* parent = 0) :
		QThread(parent)
	{
	}

	Fl_thread(int argc, char** argv, QObject* parent = 0) :
		QThread(parent)
	{
		_argc = argc;
		_argv = argv;
	}
	;

protected:
	void run()
	{
		main_old(_argc, _argv);
		cout << "Ends main_old" << endl;
		terminate();
		wait();
	}

public slots:

	void drawAllWindowActive()
	{
		cout << "g3d_draw_allwin_active()" << endl;
		g3d_draw_allwin_active();
	}

};

/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
class MainProgram: public QObject
{

Q_OBJECT
	;

	QApplication* 	app;

	MainWidget* 	sideWin;
#ifdef QT_GL
	qtGLWindow* 	g3dWin;
#endif

public:

	MainProgram()
	{

	}

	~MainProgram()
	{

	}

public:
	int run(int argc, char** argv)
	{
		app = new QApplication(argc, argv);
		app->setStyle(new QCleanlooksStyle());

		Fl_thread move3dthread(argc, argv);
		connect(&move3dthread, SIGNAL(terminated()), this, SLOT(exit()));

		move3dthread.start();

#ifdef QT_GL
		sem.acquire();
		g3dWin = new qtGLWindow();
		g3dWin->show();
		pipe2openGl = new Move3D2OpenGl(g3dWin->getOpenGLWidget());
#endif
		sideWin = new MainWidget();
		sideWin->show();

		return app->exec();
	}

private slots :

	void exit()
	{
		cout << "Ends all threads" << endl;
		app->quit();
	}

};

int qt_fl_pipe[2];

/**
 * @ingroup qtWindow
 * @brief Main function of Move3D
 */
int main(int argc, char *argv[])
{

	bool qt_flag = true;

	if (qt_flag)
	{
		pipe(qt_fl_pipe);
		fcntl(qt_fl_pipe[0], F_SETFL, O_NONBLOCK);
		MainProgram main;
		return main.run(argc, argv);
	}
	else
	{
		return main_old(argc, argv);
	}
}

#include "main.moc"
