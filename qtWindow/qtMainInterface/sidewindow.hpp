#ifndef SIDEWINDOW_HPP
#define SIDEWINDOW_HPP

#include "p3d_sys.h"

#include "mainwindow.hpp"
#include "plotwindow.hpp"
#include "../../p3d/env.hpp"
#include "../qtBase/qt_widgets.hpp"

#include <vector>

namespace Ui {
    class SideWindow;
}

class SideWindow : public QWidget {
    Q_OBJECT
public:
    SideWindow(QWidget *parent = 0);
    ~SideWindow();

    PlotWindow *plot;

    void setMainWindow(MainWindow* mainWin);

protected:
    void changeEvent(QEvent *e);

private slots:
    void setLineEditWithNumber(Env::intParameter p , int val );
    void setWhichTestSlot(int test);

    void enableHriSpace();
    void showTrajCost();
    void setPlotedVector(std::vector<double> v);
    void computeWorkspacePath();
    void computeHoleMotion();
    void KDistance(double value);
    void KVisibility(double value);

    void greedyPlan();
    void biasPos();
    void setCostCriterium(int choise);

    void computeGridAndExtract();
    void computeGrid();
    void optimizeCost();
    void shortCutCost();
    void removeRedundant();
    void graphSearchTest();
    void extractBestTraj();

    void costTest();
    void collisionsTest();
    void localpathsTest();
    void allTests();

private:
    Ui::SideWindow *m_ui;
    MainWindow *mainWin;

    QPushButton* greedy;

    void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
    LabeledSlider* createSlider(QString s, Env::intParameter p,int lower, int upper);
    LabeledDoubleSlider* createDoubleSlider(QString s,Env::doubleParameter p, double lower, double upper);

    void initDiffusion();
    void initHRI();
    void initCost();
    void initGreedy();
    void initOptim();
    void initTest();

};

#endif // SIDEWINDOW_H
