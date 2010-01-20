#ifndef SIDEWINDOW_HPP
#define SIDEWINDOW_HPP

#ifdef QWT
#include "../qtPlot/BasicPlotWindow.hpp"
#endif

#include "mainwindow.hpp"
#include "../../p3d/env.hpp"
#include "../qtBase/qt_widgets.hpp"

#include <vector>

/**
 * @ingroup qtMainWindow
 */

namespace Ui {
    class SideWindow;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Side window
 */
class SideWindow : public QWidget {
    Q_OBJECT
public:
    SideWindow(QWidget *parent = 0);
    ~SideWindow();

#ifdef QWT
    BasicPlotWindow *plot;
#endif

    void setMainWindow(MainWindow* mainWin);

protected:
    void changeEvent(QEvent *e);

private slots:
    // Global
    void setLineEditWithNumber(Env::intParameter p , int val );
    void setWhichTestSlot(int test);

    // HRI
    void enableHriSpace();
    void showTrajCost();
    void showTemperature();
    void setPlotedVector(std::vector<double> v);
    void putGridInGraph();

    void GrabObject();
    void ReleaseObject();
    void computeWorkspacePath();
    void computeHoleMotion();
    void KDistance(double value);
    void KVisibility(double value);
    void make3DHriGrid();
    void delete3DHriGrid();
    void computeGridCost();
    void resetGridCost();
    void AStarIn3DGrid();
    void HRICSRRT();
    void zoneSizeChanged();
    void drawAllWinActive();
    void resetRandomPoints();

    // Human Like

    // Various
    void greedyPlan();
    void biasPos();
    void setCostCriterium(int choise);

    // Cost
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
    void initHumanLike();
    void initCost();
    void initGreedy();
    void initOptim();
    void initTest();

};

#endif // SIDEWINDOW_H
