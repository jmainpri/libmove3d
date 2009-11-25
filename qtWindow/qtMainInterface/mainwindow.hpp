#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "p3d_sys.h"
#include "kcdpropertieswindow.hpp"
#include "../../p3d/env.hpp"

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void open();
    void isPlanning();
    void planningFinished();

protected:
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);

private slots:
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
