#ifndef MAIN_HPP
#define MAIN_HPP

#ifdef QT_GL
#include "qtOpenGL/qtGLWindow.hpp"
#endif

/**
 * @ingroup qtWindow
 * @brief Main double thread class (X-Forms Thread)
 */
class Fl_thread: public QThread
{

Q_OBJECT

public:
        int _argc;
        char** _argv;

        Fl_thread(QObject* parent = 0);
        Fl_thread(int argc, char** argv, QObject* parent = 0);

protected:
        void run();

};


/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
class Main_threads: public QObject
{

Q_OBJECT
#ifdef QT_GL
        qtGLWindow* 	g3dWin;
#endif
        MainWidget* 	sideWin;
        QApplication* 	app;

public:
        Main_threads();
        ~Main_threads();

public:
        int run(int argc, char** argv);

private slots :
        void exit();

};

#endif // MAIN_HPP
