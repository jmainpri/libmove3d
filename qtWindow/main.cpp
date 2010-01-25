#include "qtMainWindow.hpp"

#ifdef QT_UI_XML_FILES
#include "qtMainInterface/mainwindow.hpp"
#endif

#include "main.hpp"

#include <iostream>
#include <fcntl.h>
#include "cppToQt.hpp"

#ifdef QT_GL
QSemaphore* sem;
GLWidget* openGlWidget;
#endif

#ifdef QT_OPENGL_SIDE
Move3D2OpenGl* pipe2openGl;
#endif


extern int main_old(int argc, char** argv);

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
    main_old(_argc, _argv);
    cout << "Ends main_old" << endl;
    terminate();
    wait();
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

int Main_threads::run(int argc, char** argv)
{

    app = new QApplication(argc, argv);
//    app->setStyle(new QCleanlooksStyle());
//    app->setStyle(new QWindowsStyle());
//    app->setStyle(new QMacStyle());

    Fl_thread move3dthread(argc, argv);
    connect(&move3dthread, SIGNAL(terminated()), this, SLOT(exit()));
    move3dthread.start();

#ifdef QT_GL
    sem->acquire();
    cout << "Waiting"<< endl;
    waitDrawAllWin = new QWaitCondition();
    lockDrawAllWin = new QMutex();

#ifdef QT_OPENGL_SIDE
    g3dWin = new qtGLWindow();
    g3dWin->show();
    pipe2openGl = new Move3D2OpenGl(g3dWin->getOpenGLWidget());
#endif

#endif

#ifdef QT_OPENGL_SIDE
    sideWin = new MainWidget();
    sideWin->show();
#endif

#ifdef QT_UI_XML_FILES
    MainWindow w;
    w.show();
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

    g3dWin = new qtGLWindow();
    g3dWin->show();
    pipe2openGl = new Move3D2OpenGl(g3dWin->getOpenGLWidget());

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
        pipe(qt_fl_pipe);
        fcntl(qt_fl_pipe[0], F_SETFL, O_NONBLOCK);
        Main_threads main;
        return main.run(argc, argv);
    }
    else
    {
        return main_old(argc, argv);
    }
}
