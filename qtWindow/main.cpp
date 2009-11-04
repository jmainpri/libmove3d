#include "qtMainWindow.hpp"
#include "qtMainInterface/mainwindow.hpp"

#include "main.hpp"

#include <iostream>
#include <fcntl.h>
#include "cppToQt.hpp"

#ifdef QT_GL
QSystemSemaphore* sem;
//Move3D2OpenGl* pipe2openGl;
GLWidget* openGlWidget;
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
MainProgram::MainProgram()
{
#ifdef QT_GL
    sem = new QSystemSemaphore("market", 0, QSystemSemaphore::Create);
#endif
}

MainProgram::~MainProgram()
{

}

int MainProgram::run(int argc, char** argv)
{

    app = new QApplication(argc, argv);
//    app->setStyle(new QCleanlooksStyle());
//    app->setStyle(new QWindowsStyle());
    app->setStyle(new QMacStyle());

    Fl_thread move3dthread(argc, argv);
    connect(&move3dthread, SIGNAL(terminated()), this, SLOT(exit()));
    move3dthread.start();

#ifdef QT_GL

    sem->acquire();

    cout << "Waiting"<< endl;

    waitDrawAllWin = new QWaitCondition();
    lockDrawAllWin = new QMutex();

/*
    g3dWin = new qtGLWindow();
    g3dWin->show();
    pipe2openGl = new Move3D2OpenGl(g3dWin->getOpenGLWidget());
*/
#endif

//    sideWin = new MainWidget();
//    sideWin->show();

#ifdef QT_UI_XML_FILES
    MainWindow w;
    w.show();
#endif

    return app->exec();

}


void MainProgram::exit()
{
    cout << "Ends all threads" << endl;
    app->quit();
}

/**
 * @defgroup qtWindow The Qt Window
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
        MainProgram main;
        return main.run(argc, argv);
    }
    else
    {
        return main_old(argc, argv);
    }
}

#include "moc_main.cpp"
