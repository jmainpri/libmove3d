#ifdef QT_UI_XML_FILES
#include "qtMainInterface/mainwindow.hpp"
#endif

#include "main.hpp"

#include <iostream>
#include <fcntl.h>
#include "cppToQt.hpp"
#include <QDesktopWidget>

#ifdef QT_GL
QSemaphore* sem;
GLWidget* openGlWidget;
extern int mainMhp(int argc, char ** argv);
#endif


//extern int mainMhp(int argc, char** argv);

using namespace std;


/**
 * @ingroup qtWindow
 * @brief Main double thread class (X-Forms Thread)
 */
Fl_thread::Fl_thread(QObject* parent) :
        QThread(parent)
{
}

Fl_thread::Fl_thread(int argc, char** argv, QObject* parent) :
        QThread(parent)
{
    _argc = argc;
    _argv = argv;
}

void Fl_thread::run()
{
    mainMhp(_argc, _argv);
    cout << "Ends main_old" << endl;
//    terminate();
//    wait();
}


/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
Main_threads::Main_threads()
{
#ifdef QT_GL
    sem = new QSemaphore(0);
#endif
}

Main_threads::~Main_threads()
{

}

// Temporary mechanism to redraw the opengl scene.
// Not elegant, but it works.
MainWindow* global_w(NULL);
// void draw_opengl()
// {
//   if(global_w != NULL)
//   {
//     QMetaObject::invokeMethod(global_w->getOpenGL(),
// 															"updateGL",
// 															Qt::BlockingQueuedConnection);
//   }
// }


int Main_threads::run(int argc, char** argv)
{
    app = new QApplication(argc, argv);
		app->setWindowIcon(QIcon::QIcon(QPixmap::QPixmap(molecule_xpm)));
//    app->setStyle(new QCleanlooksStyle());
//    app->setStyle(new QWindowsStyle());
//    app->setStyle(new QMacStyle());

    Fl_thread move3dthread(argc, argv);
    connect(&move3dthread, SIGNAL(terminated()), this, SLOT(exit()));
    move3dthread.start();

#ifdef QT_GL
		cout << "Waiting end of parser to draw OpenGL and create Qt Forms ..."<< endl;
		//sem->acquire();
    waitDrawAllWin = new QWaitCondition();
    lockDrawAllWin = new QMutex();
#endif


#ifdef QT_UI_XML_FILES
	MainWindow w;
	global_w = &w;
	w.showMaximized();
	w.show();
	w.raise();
// 	QRect g = QApplication::desktop()->screenGeometry();
// 	cout << " x = " << g.x() << " y = " << g.y() << endl;
// 	cout << " width = " << g.width() << " height = " << g.height() << endl;
// 	
// 	QRect g_window = w.geometry();
// 	g_window.setWidth( g.width() );
// 	g_window.setHeight( 0.707*g.height() ); // sqrt(2) / 2
// 	g_window.moveTo( 0, 0 );
	
// 	w.setGeometry( g_window );
#endif

		return app->exec();
}


void Main_threads::exit()
{
    cout << "Ends all threads" << endl;
    app->quit();
}


/**
 * @defgroup qtWindow The Qt Window
 * The qt Module implements the interface in a separate thread so that it is necessary to use
 * such things as semaphore, locks and pipes to have everything behaving nicely. The maine function is as follows
 * \code
    app = new QApplication(argc, argv);

    Fl_thread move3dthread(argc, argv);
    connect(&move3dthread, SIGNAL(terminated()), this, SLOT(exit()));
    move3dthread.start();

    sem->acquire();
    cout << "Waiting"<< endl;
    waitDrawAllWin = new QWaitCondition();
    lockDrawAllWin = new QMutex();

    MainWindow w;
    w.show();

    return app->exec();
    \endcode
 */

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
#ifdef WITH_XFORMS
        pipe(qt_fl_pipe);
        fcntl(qt_fl_pipe[0], F_SETFL, O_NONBLOCK);
#endif
		
        Main_threads main;
        cout << "main.run(argc, argv)"  << endl;
        return main.run(argc, argv);
    }
    else
    {
        //return mainMhp(argc, argv);
    }
}
