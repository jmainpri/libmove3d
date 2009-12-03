#ifndef SIDEWINDOW_HPP
#define SIDEWINDOW_HPP

#include "p3d_sys.h"

#ifdef QWT
#include "plotwindow.hpp"
#endif

#include "mainwindow.hpp"
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

#ifdef QWT
    PlotWindow *plot;
#endif

    void setMainWindow(MainWindow* mainWin);

protected:
    void changeEvent(QEvent *e);

private slots:
    void setLineEditWithNumber(Env::intParameter p , int val );
    void setWhichTestSlot(int test);

    void enableHriSpace();
    void showTrajCost();
    void setPlotedVector(std::vector<double> v);
    void putGridInGraph();

    void computeWorkspacePath();
    void computeHoleMotion();
    void KDistance(double value);
    void KVisibility(double value);
    void make3DHriGrid();
    void delete3DHriGrid();
    void computeGridCost();
    void resetGridCost();
    void AStarIn3DGrid();
    void drawAllWinActive();

    void greedyPlan();
    void biasPos();
    void setCostCriterium(int choise);

    void computeAStar();
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
    void setAttMatrix();

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
