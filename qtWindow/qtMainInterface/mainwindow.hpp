#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "p3d_sys.h"
#include "kcdpropertieswindow.hpp"
#include "../../p3d/env.hpp"

#ifdef QWT
#include "../qtPlot/basicPlotWindow.hpp"
#include "../qtPlot/histoWin.hpp"
#endif

#include "../qtBase/qt_widgets.hpp"

#include <vector>

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
  * Tow Widget are derived from other classes The GLWidget widget and the MoveRobot Widget
  \image html Designer.png
  */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void openScenario();
	void saveScenario();
    void isPlanning();
    void planningFinished();

protected:
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
	
private slots:
    //******************************************************************
    // Main
    void changeLightPosX();
    void changeLightPosY();
    void changeLightPosZ();
	void drawAllWinActive();
	void addTrajToDraw();
	void clearTrajToDraw();
	void colorTrajChange(int color);
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
    void showTraj();
    void restoreView();

    //******************************************************************
    // SideWindow

    // Global
//    void setLineEditWithNumber(Env::intParameter p , int val );
    void setWhichTestSlot(int test);
    void changeEvent(QEvent *e);

    // HRI ----------------------------------
    // CSpace
    void newHRIConfigSpace();
    void deleteHRIConfigSpace();
    void makeGridHRIConfigSpace();
    void makePlanHRIConfigSpace();
    void AStarInPlanHRIConfigSpace();
    void writeToOBPlane();
    void hriPlanRRT();
    // Workspace
    void make3DHriGrid();
    void delete3DHriGrid();
    void computeGridCost();
    void resetGridCost();
    void AStarIn3DGrid();
    void HRICSRRT();
    void zoneSizeChanged();
    void resetRandomPoints();

    // Taskspace
    void computeWorkspacePath();
    void computeHoleMotion();
    void KDistance(double value);
    void KVisibility(double value);
    void make2DGrid();

    void enableHriSpace();
    // Human Like

    // Cost
    void showTrajCost();
    void showHRITrajCost();
    void showTemperature();
    void setPlotedVector(std::vector<double> v);
    void putGridInGraph();
    void computeAStar();

    // Optim
    void computeGridAndExtract();
    void computeGrid();
    void runMultiSmooth();
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

    // Multi-Run
    void saveContext();
    void printContext();
    void printAllContext();
    void resetContext();
    void setToSelected();
    void runAllRRT();
    void runAllGreedy();
    void showHistoWindow();

    //Util
    //Greedy
    void greedyPlan();
    void biasPos();
    void setCostCriterium(int choise);
    //Grab Object
    void GrabObject();
    void ReleaseObject();
    void currentObjectChange(int i);
    void SetObjectToCarry();

private:
    //******************************************************************
    // Main
    Ui::MainWindow *m_ui;
    KCDpropertiesWindow*    mKCDpropertiesWindow;

    void connectCheckBoxes();

    void initRunButtons();
    void initViewerButtons();
    void initLightSource();

    //******************************************************************
    // SideWindow
    void connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p);
    LabeledSlider* createSlider(QString s, Env::intParameter p,int lower, int upper);
    LabeledDoubleSlider* createDoubleSlider(QString s,Env::doubleParameter p, double lower, double upper);

    QPushButton* greedy;

#ifdef QWT
    BasicPlotWindow *plot;
    HistoWindow* histoWin;
#endif

    QListWidget* contextList;
    std::vector<QListWidgetItem*> itemList;

    std::vector<QString> mFreeFlyers;

    void initDiffusion();
    void initPRM();
    void initHRI();
    void initHumanLike();
    void initCost();
    void initUtil();
    void initOptim();
    void initModel();
    void initMultiRun();
};

/**
 * @ingroup qtWindow
 * @brief Planner thread class 
 */
class Plannerthread: public QThread
{
	Q_OBJECT
	
public:
	Plannerthread(QObject* parent = 0);
	
protected:
	void run();
	
};

/**
 * @ingroup qtWindow
 * @brief Showtraj thread class 
 */
class Showtrajthread: public QThread
{
	Q_OBJECT
	
public:
	Showtrajthread(QObject* parent = 0);
	
protected:
	void run();
	
};

/**
 * @ingroup qtWindow
 * @brief Smoothing thread class 
 */
class SmoothThread: public QThread
{
	Q_OBJECT
	
public:
	SmoothThread(QObject* parent = 0);
	
protected:
	void run();
	
};


#endif // MAINWINDOW_H
