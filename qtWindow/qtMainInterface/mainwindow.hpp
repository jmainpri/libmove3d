#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "p3d_sys.h"
#include "kcdpropertieswindow.hpp"
#include "../../p3d/env.hpp"

/**
 * @ingroup qtWindow
 * @defgroup qtMainWindow
 * The Qt Main window is fired in a different thread as the worker thread in which is located the xform thread.
 * It then makes an object of the MainWindow class which contains other small widgets like the Side Window.
 \code
    MainWindow::MainWindow(QWidget *parent)
            : QMainWindow(parent), ui(new Ui::MainWindow)
    {
        ui->setupUi(this);
        ui->sidePannel->setMainWindow(this);

        mKCDpropertiesWindow = new KCDpropertiesWindow();

        ui->OpenGL->setWinSize(G3D_WIN->size);
        pipe2openGl = new Move3D2OpenGl(ui->OpenGL);

        QPalette pal(ui->centralWidget->palette()); // copy widget's palette to non const QPalette
        QColor myColor(Qt::darkGray);
        pal.setColor(QPalette::Window,myColor);
        ui->centralWidget->setPalette( pal );        // set the widget's palette

        cout << "pipe2openGl = new Move3D2OpenGl(ui->OpenGL)" << endl;
        ...
  \endcode
  The Ui file edited is setup by ui->setupUi(this);
 */

namespace Ui
{
    class MainWindow;
}

/**
  * @ingroup qtMainWindow
  * @brief Qt Main Window container
  */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void openScenario();
    void isPlanning();
    void planningFinished();

protected:
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

private slots:
    void changeLightPosX();
    void changeLightPosY();
    void changeLightPosZ();
    void setBoolGhost(bool value);
    void setBoolBb(bool value);
    void setBoolFloor(bool value);
    void setBoolTiles(bool value);
    void setBoolWalls(bool value);
    void setBoolSmooth(bool value);
    void setBoolShadows(bool value);
    void setBoolFilaire(bool value);
    void run();
    void stop();
    void reset();
    void ResetGraph();
    void drawAllWinActive();
    void showTraj();
    void restoreView();

private:
    Ui::MainWindow *ui;
    KCDpropertiesWindow*    mKCDpropertiesWindow;

    void connectCheckBoxes();
    void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
};

#endif // MAINWINDOW_H
