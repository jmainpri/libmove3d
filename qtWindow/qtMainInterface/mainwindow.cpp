#include "mainwindow.hpp"
#include "ui_mainwindow.h"


#include "../cppToQt.hpp"
#include "../qtOpenGL/glwidget.hpp"
#include "../qtBase/SpinBoxSliderConnector_p.hpp"

#include "P3d-pkg.h"
#include "Util-pkg.h"

#include <iostream>
#include <tr1/memory>
#include <vector>

#ifdef CXX_PLANNER
#include "../../util/CppApi/testModel.hpp"
#include "../../util/CppApi/SaveContext.hpp"
#include "../../planner_cxx/API/planningAPI.hpp"
#include "../../planner_cxx/API/Trajectory/CostOptimization.hpp"
#include "../../planner_cxx/API/Trajectory/Smoothing.hpp"
#include "../../planner_cxx/API/Grids/GridToGraph/gridtograph.h"
#include "../../planner_cxx/API/Search/GraphState.h"
#include "../../planner_cxx/API/Grids/ThreeDPoints.h"
#include "../../planner_cxx/API/Grids/BaseGrid.hpp"
#include "../../planner_cxx/API/Grids/TwoDGrid.hpp"
#endif

#ifdef HRI_COSTSPACE
#include "../../planner_cxx/HRI_CostSpace/HRICS_CSpace.h"
#include "../../planner_cxx/HRI_CostSpace/HRICS_old.h"
#include "../../planner_cxx/HRI_CostSpace/Grid/HRICS_Grid.h"
#include "../../planner_cxx/HRI_CostSpace/Grid/HRICS_GridState.h"
#include "../../planner_cxx/HRI_CostSpace/HRICS_Planner.h"
#ifdef HRI_PLANNER
#include "../../planner_cxx/HRI_CostSpace/HRICS_HAMP.h"
#endif
#endif

Move3D2OpenGl* pipe2openGl;


using namespace std;
using namespace tr1;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);
	
	m_ui->tabMotionPlanner->setMainWindow(this);
	m_ui->tabCost->setMainWindow(this);
	m_ui->tabRobot->setMainWindow(this);
	
	m_ui->tabCost->setMotionWidget(this->m_ui->tabMotionPlanner);
	m_ui->tabCost->initCost();
	m_ui->tabCost->initHRI();
	m_ui->tabRobot->initRobot();

    mKCDpropertiesWindow = new KCDpropertiesWindow();

    //    m_ui->OpenGL->setWinSize(G3D_WIN->size);
    m_ui->OpenGL->setWinSize(600);
    pipe2openGl = new Move3D2OpenGl(m_ui->OpenGL);

    //    m_ui->sidePannel->setMainWindow(this);

    //UITHINGQPalette pal(m_ui->centralWidget->palette()); // copy widget's palette to non const QPalette
    //UITHINGQColor myColor(Qt::darkGray);
    //UITHINGpal.setColor(QPalette::Window,myColor);
    //UITHINGm_ui->centralWidget->setPalette( pal );        // set the widget's palette

    cout << "pipe2openGl = new Move3D2OpenGl(m_ui->OpenGL)" << endl;

    //    connect(m_ui->actionOpen,SIGNAL(triggered()),this,SLOT(open()));
    connect(m_ui->actionOpenScenario,SIGNAL(triggered()),this,SLOT(openScenario()));
	connect(m_ui->actionSaveScenario,SIGNAL(triggered()),this,SLOT(saveScenario()));
    connect(m_ui->actionKCDPropietes,SIGNAL(triggered()),mKCDpropertiesWindow,SLOT(show()));

    //    connect(m_ui->pagesOfStakedWidget, SIGNAL(activated(int)),m_ui->stackedWidget, SLOT(setCurrentIndex(int)));

    connectCheckBoxes();

    // MainWindow
    initRunButtons();
    initViewerButtons();
    initLightSource();
	
	initUtil();

    // Side Window

//    initDiffusion();
//    initPRM();
//    initOptim();
//    initMultiRun();	
	
//    initCost();
//    initHRI();
//    initHumanLike();
	
//    initModel();
}

MainWindow::~MainWindow()
{
    delete mKCDpropertiesWindow;
    delete m_ui;
}


const char *qt_fileName = NULL;

void MainWindow::openScenario()
{
    QString fileName = QFileDialog::getOpenFileName(this);

    if (!fileName.isEmpty())
    {
        qt_fileName = fileName.toStdString().c_str();

#ifdef WITH_XFORMS
        std::string str = "readP3DScenarion";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
		cout << "Open scenario " << fileName.toStdString() << endl;
#else
		qt_readScenario();
		//m_ui->formRobot->updateAllRobotInitPos();
		this->drawAllWinActive();
#endif
        
    }
}

void MainWindow::saveScenario()
{
    QString fileName = QFileDialog::getSaveFileName(this);
	
    if (!fileName.isEmpty())
    {
        qt_fileName = fileName.toStdString().c_str();
		
#ifdef WITH_XFORMS
        std::string str = "readP3DScenarion";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
		cout << "Open scenario " << fileName.toStdString() << endl;
#else
		qt_saveScenario();
		//m_ui->formRobot->updateAllRobotInitPos();
		this->drawAllWinActive();
#endif
    }
}

void MainWindow::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
    connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
}

GLWidget* MainWindow::getOpenGL()
{ 
	return m_ui->OpenGL; 
}

// Light sliders ------------------------------------------------
// --------------------------------------------------------------
void MainWindow::initLightSource()
{
    connectCheckBoxToEnv(m_ui->checkBoxDrawLightSource,      Env::drawLightSource);

    vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;

    m_ui->doubleSpinBoxLightX->setMinimum(2*envSize[0]);
    m_ui->doubleSpinBoxLightX->setMaximum(2*envSize[1]);
    m_ui->doubleSpinBoxLightY->setMinimum(2*envSize[2]);
    m_ui->doubleSpinBoxLightY->setMaximum(2*envSize[3]);
    m_ui->doubleSpinBoxLightZ->setMinimum(2*envSize[4]);
    m_ui->doubleSpinBoxLightZ->setMaximum(2*envSize[5]);

    QtShiva::SpinBoxSliderConnector *connectorLightX = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxLightX, m_ui->horizontalSliderLightX);
    QtShiva::SpinBoxSliderConnector *connectorLightY = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxLightY, m_ui->horizontalSliderLightY);
    QtShiva::SpinBoxSliderConnector *connectorLightZ = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxLightZ, m_ui->horizontalSliderLightZ);

    connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosX()));
    connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosY()));
    connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosZ()));

    connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
    connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
    connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));

    m_ui->doubleSpinBoxLightX->setValue(G3D_WIN->vs.lightPosition[0]);
    m_ui->doubleSpinBoxLightY->setValue(G3D_WIN->vs.lightPosition[1]);
    m_ui->doubleSpinBoxLightZ->setValue(G3D_WIN->vs.lightPosition[2]);
}

void MainWindow::changeLightPosX()
{
    float* lightPosition = G3D_WIN->vs.lightPosition;
    lightPosition[0] = m_ui->doubleSpinBoxLightX->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN->vs);
    //    cout << "Change X value" << endl;
#ifndef WITH_XFORMS
    this->drawAllWinActive();
#endif
}

void MainWindow::changeLightPosY()
{
    float* lightPosition = G3D_WIN->vs.lightPosition;
    lightPosition[1] = m_ui->doubleSpinBoxLightY->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN->vs);
    //    cout << "Change Y value" << endl;
#ifndef WITH_XFORMS
    this->drawAllWinActive();
#endif
}

void MainWindow::changeLightPosZ()
{
    float* lightPosition = G3D_WIN->vs.lightPosition;
    lightPosition[2] = m_ui->doubleSpinBoxLightZ->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN->vs);
    //    cout << "Change Z value" << endl;
#ifndef WITH_XFORMS
    this->drawAllWinActive();
#endif
}

// Viewer Buttons -----------------------------------------------
// --------------------------------------------------------------
void MainWindow::initViewerButtons()
{
    connect(m_ui->checkBoxDrawGraph,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()));
    connect(m_ui->checkBoxDrawTraj,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()));

    connectCheckBoxToEnv(m_ui->checkBoxDisableDraw,Env::drawDisabled);
    connectCheckBoxToEnv(m_ui->checkBoxDrawGraph,Env::drawGraph);
    connectCheckBoxToEnv(m_ui->checkBoxDrawTraj,Env::drawTraj);
	connectCheckBoxToEnv(m_ui->checkBoxDrawTrajVector,Env::drawTrajVector);
	connectCheckBoxToEnv(m_ui->checkBoxDrawDebug,Env::debugCostOptim);
    m_ui->checkBoxDrawGraph->setCheckState(Qt::Checked);

    connect(m_ui->pushButtonShowTraj,SIGNAL(clicked(bool)),this,SLOT(showTraj()),Qt::DirectConnection);
	new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxTrajSpeed, m_ui->horizontalSliderTrajSpeed , Env::showTrajFPS );

    connect(m_ui->pushButtonRestoreView,SIGNAL(clicked(bool)),this,SLOT(restoreView()),Qt::DirectConnection);
    connect(m_ui->pushButtonResetGraph,SIGNAL(clicked()),this,SLOT(ResetGraph()));
	
	connect(m_ui->pushButtonAddTraj,SIGNAL(clicked()),this,SLOT(addTrajToDraw()));
	connect(m_ui->pushButtonClearTraj,SIGNAL(clicked()),this,SLOT(clearTrajToDraw()));
	
	connect(m_ui->comboBoxColorTraj, SIGNAL(currentIndexChanged(int)),this,SLOT(colorTrajChange(int)));
}

// Show Trajectory ----------------------------------------------
// --------------------------------------------------------------

GLWidget* ptrOpenGL;

static int default_drawtraj_fct_qt_pipe(p3d_rob* robot, p3d_localpath* curLp)
{
#ifdef WITH_XFORMS
	std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	g3d_draw_allwin_active();
#endif
    usleep(20000);
	//cout << "ENV.getDouble(Env::showTrajFPS) = " << ENV.getDouble(Env::showTrajFPS) << endl;
    return TRUE;
}

void MainWindow::showTraj()
{
	cout << "Show Traj....."<< endl;
	//m_ui->pushButtonShowTraj->disable(true);
    ptrOpenGL = m_ui->OpenGL;
	this->isPlanning();
	Showtrajthread* showTrajThread = new Showtrajthread;
	showTrajThread->start();
}

Showtrajthread::Showtrajthread(QObject* parent) :
QThread(parent)
{
	
}

void Showtrajthread::run()
{	
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    g3d_show_tcur_rob(robotPt,default_drawtraj_fct_qt_pipe);
	ENV.setBool(Env::isRunning,false);
}

// --------------------------------------------------------------
void MainWindow::connectCheckBoxes()
{
    connect(m_ui->checkBoxGhosts, SIGNAL(toggled(bool)), this , SLOT(setBoolGhost(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxGhosts, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
    connect(m_ui->checkBoxBB, SIGNAL(toggled(bool)), this , SLOT(setBoolBb(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxBB, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
    connect(m_ui->checkBoxFloor, SIGNAL(toggled(bool)), this , SLOT(setBoolFloor(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxFloor, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
    m_ui->checkBoxFloor->setCheckState(Qt::Checked);
	
    connect(m_ui->checkBoxTiles, SIGNAL(toggled(bool)), this , SLOT(setBoolTiles(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxTiles, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
    m_ui->checkBoxTiles->setCheckState(Qt::Checked);
	
    connect(m_ui->checkBoxWalls, SIGNAL(toggled(bool)), this , SLOT(setBoolWalls(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxWalls, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
    connect(m_ui->checkBoxShadows, SIGNAL(toggled(bool)), this , SLOT(setBoolShadows(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxShadows, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
    connect(m_ui->checkBoxSmooth, SIGNAL(toggled(bool)), this , SLOT(setBoolSmooth(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxSmooth, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
    connect(m_ui->checkBoxFilaire, SIGNAL(toggled(bool)), this , SLOT(setBoolFilaire(bool)), Qt::DirectConnection);
    connect(m_ui->checkBoxFilaire, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
    connectCheckBoxToEnv(m_ui->checkBoxAxis, Env::drawFrame);
    connect(m_ui->checkBoxAxis, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
}

void MainWindow::setBoolGhost(bool value)
{
    G3D_WIN->vs.GHOST = value;
}

void MainWindow::setBoolBb(bool value)
{
    G3D_WIN->vs.BB = value;
}


void MainWindow::setBoolFloor(bool value)
{
    G3D_WIN->vs.displayFloor = value;
}


void MainWindow::setBoolTiles(bool value)
{
    G3D_WIN->vs.displayTiles = value;
}


void MainWindow::setBoolWalls(bool value)
{
    G3D_WIN->vs.displayWalls = value;
}

void MainWindow::setBoolShadows(bool value)
{
    G3D_WIN->vs.displayShadows = value;
}

void MainWindow::setBoolSmooth(bool value)
{
    G3D_WIN->vs.GOURAUD = value;
}

void MainWindow::setBoolFilaire(bool value)
{
    G3D_WIN->vs.FILAIRE = value;
}

void MainWindow::restoreView()
{
    g3d_restore_win_camera(G3D_WIN->vs);
    drawAllWinActive();
}

void MainWindow::addTrajToDraw()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_traj* CurrentTrajPt = robotPt->tcur;
#ifdef CXX_PLANNER
	Trajectory traj(new Robot(robotPt),CurrentTrajPt);
	trajToDraw.push_back(traj);
#endif
}

void MainWindow::clearTrajToDraw()
{
#ifdef CXX_PLANNER
	trajToDraw.clear();
#endif
}

void MainWindow::colorTrajChange(int color)
{
	cout << "Change traj color" << endl;
#ifdef CXX_PLANNNER
	for( unsigned int i=0; i<trajToDraw.size(); i++ ) 
	{
		cout << " Change traj " << i << " to : " << color << endl;
		trajToDraw[i].setColor(color);
	}
#endif
	this->drawAllWinActive();
}


// Run Buttons -----------------------------------------------
// -----------------------------------------------------------

void MainWindow::initRunButtons()
{
    connect(m_ui->pushButtonRun,SIGNAL(clicked(bool)),this,SLOT(run()),Qt::DirectConnection);

    connect(m_ui->pushButtonReset,SIGNAL(clicked(bool)),this,SLOT(reset()),Qt::DirectConnection);
    connect(m_ui->pushButtonStop,SIGNAL(clicked(bool)),this,SLOT(stop()),Qt::DirectConnection);

    m_ui->pushButtonRun->setDisabled(false);
    m_ui->pushButtonStop->setDisabled(true);
    m_ui->pushButtonReset->setDisabled(true);

    connect(ENV.getObject(Env::isPRMvsDiffusion), SIGNAL(valueChanged(bool)), m_ui->radioButtonPRM, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(m_ui->radioButtonPRM, SIGNAL(toggled(bool)), ENV.getObject(Env::isPRMvsDiffusion), SLOT(set(bool)), Qt::DirectConnection);
    m_ui->radioButtonDiff->setChecked(!ENV.getBool(Env::isPRMvsDiffusion));

    connectCheckBoxToEnv(m_ui->checkBoxWithSmoothing,      Env::withSmoothing);

    connect( ENV.getObject(Env::isRunning), SIGNAL(valueChanged(bool)), this, SLOT(planningFinished(void)) , Qt::DirectConnection );
}

/**
 * @ingroup qtWindow
 * @brief Planner thread class
 */
//-----------------------------------------------
Plannerthread::Plannerthread(QObject* parent) :
QThread(parent)
{
	
}

void Plannerthread::run()
{	
	if(!ENV.getBool(Env::isPRMvsDiffusion))
    {
		qt_runDiffusion();
    }
    else
    {
        qt_runPRM();
    }
	
//	exec();
    cout << "Ends Planner Thread" << endl;
}

//-----------------------------------------------

void MainWindow::run()
{
	cout << "MainWindow::run" << endl;
	
#ifdef WITH_XFORMS
    if(!ENV.getBool(Env::isPRMvsDiffusion))
    {
        std::string str = "RunDiffusion";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    }
    else
    {
        std::string str = "RunPRM";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    }
	ENV.setBool(Env::isRunning,true);
#else
	this->isPlanning();
    Plannerthread* ptrPlan = new Plannerthread;
	cout << "Start Planning Thread" << endl;
    ptrPlan->start();
#endif
}

void MainWindow::stop()
{
#ifdef P3D_PLANNER
    p3d_SetStopValue(true);
#endif
    ENV.setBool(Env::isRunning,false);
}

void MainWindow::reset()
{
#ifdef WITH_XFORMS
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	qt_resetGraph();
#endif
    m_ui->pushButtonRun->setDisabled(false);
    m_ui->pushButtonStop->setDisabled(true);
    m_ui->pushButtonReset->setDisabled(true);
#ifdef P3D_PLANNER
    p3d_SetStopValue(false);
#endif
	this->drawAllWinActive();
}

void MainWindow::ResetGraph()
{
#ifdef WITH_XFORMS
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	qt_resetGraph();
#endif
	this->drawAllWinActive();
}

void MainWindow::isPlanning()
{
    m_ui->pushButtonRun->setDisabled(true);
    m_ui->pushButtonReset->setDisabled(true);
    m_ui->pushButtonStop->setDisabled(false);

    ENV.setBool(Env::isRunning,true);

    QPalette pal(Qt::red);
    m_ui->labelRunning->setPalette( pal );
    m_ui->labelRunning->setText("RUNNING" );

    //UITHINGQPalette pal(Qt::lightGray); // copy widget's palette to non const QPalette
    //UITHINGm_ui->toolBox->setPalette( pal );        // set the widget's palette

    m_ui->labelRunning->setText(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
    "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
    "p, li { white-space: pre-wrap; }\n"
    "</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
    "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#FF0000;\">Running</span></p></body></html>",
    0, QApplication::UnicodeUTF8));

}

void MainWindow::planningFinished()
{
    if( ENV.getBool(Env::isRunning) == false )
    {
        m_ui->pushButtonStop->setDisabled(true);
        m_ui->pushButtonReset->setDisabled(false);

//        m_ui->labelRunning->setText("Not Running" );

        m_ui->labelRunning->setText(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
        "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
        "p, li { white-space: pre-wrap; }\n"
        "</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#008d00;\">Not Running</span></p></body></html>",
        0, QApplication::UnicodeUTF8));

                // set the widget's palette
    }
    else
    {
        this->isPlanning();
    }
}

void MainWindow::drawAllWinActive()
{
#ifdef WITH_XFORMS
	std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	if(!ENV.getBool(Env::isRunning))
	{
		m_ui->OpenGL->updateGL();
	}
#endif
}



// Key events ---------------------------------------------------
// --------------------------------------------------------------

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    //    cout << "Key pressed" << endl;
    switch(event->key())
    {
    case Qt::Key_X:
        mouse_mode = 1;
        cout << "Switch to second" << endl;
        break;

    case Qt::Key_C:
        mouse_mode = 2;
        cout << "Switch to third" << endl;
        break;

        //            if(mouse_mode == 2)
        //            {
        //                mouse_mode = 0;
        //                cout << "Switch to normal" << endl;
        //                return;
        //            }
        //                break;
    }
}

void MainWindow::keyReleaseEvent(QKeyEvent *e)
{
    mouse_mode = 0;
}


// Basic function -----------------------------------------------
// --------------------------------------------------------------

LabeledSlider* MainWindow::createSlider(QString s, Env::intParameter p,
                                        int lower, int upper)
{
    LabeledSlider* slider = new LabeledSlider(lower, upper, lower, s);
    connect(ENV.getObject(p), SIGNAL(valueChanged(int)), slider,
            SLOT(setValue(int)), Qt::DirectConnection);
    connect(slider, SIGNAL(valueChanged(int)), ENV.getObject(p),
            SLOT(set(int)), Qt::DirectConnection);
    slider->setValue(ENV.getInt(p));
    return (slider);
}

LabeledDoubleSlider* MainWindow::createDoubleSlider(QString s,
                                                    Env::doubleParameter p, double lower, double upper)
{
    LabeledDoubleSlider* slider = new LabeledDoubleSlider(lower, upper, lower,s);
    connect(ENV.getObject(p), SIGNAL(valueChanged(double)), slider,
            SLOT(setValue(double)), Qt::DirectConnection);
    connect(slider, SIGNAL(valueChanged(double)), ENV.getObject(p),
            SLOT(set(double)), Qt::DirectConnection);
    slider->setValue(ENV.getDouble(p));
    return (slider);
}

void MainWindow::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}



//---------------------------------------------------------------------
// GREEDY
//---------------------------------------------------------------------
void MainWindow::initUtil()
{
    // Greedy
    greedy = new QPushButton("Greedy Planner");
    connect(greedy, SIGNAL(clicked()),this, SLOT(greedyPlan()),Qt::DirectConnection);

    connectCheckBoxToEnv(m_ui->checkBoxDebug,               Env::debugCostOptim);
    connectCheckBoxToEnv(m_ui->checkBoxRecomputeTrajCost,   Env::trajCostRecompute);
//    connectCheckBoxToEnv(m_ui->checkBoxWithShortCut,        Env::withShortCut);
    connectCheckBoxToEnv(m_ui->checkBoxUseTRRT,             Env::useTRRT);

    LabeledSlider* nbGreedyTraj = createSlider(tr("Number of trajectories"), Env::nbGreedyTraj, 1, 10 );
    LabeledSlider* hight = createSlider(tr("Height Factor"), Env::heightFactor, 1, 10 );
    LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 500 );
    LabeledDoubleSlider* maxFactor = createDoubleSlider(tr("Start Factor"), Env::MaxFactor, 0, 2000 );
    LabeledDoubleSlider* minStep = createDoubleSlider(tr("Min step"), Env::MinStep, 0, 1000 );
    LabeledDoubleSlider* costStep = createDoubleSlider(tr("Cost Step"), Env::costStep, 0.01, 10 );

    //	double dmax=0;
    //	p3d_col_get_dmax(&dmax);
    //
    //	minStep->setValue(dmax);
    numberIterations->setValue(ENV.getInt(Env::nbCostOptimize));

//    QPushButton* biasPos = new QPushButton("Biased Pos");
//    connect(biasPos, SIGNAL(clicked()),this, SLOT(biasPos()),Qt::DirectConnection);

//    QComboBox* costCriterium = new QComboBox();
//    costCriterium->insertItem(INTEGRAL, "Integral");
//    costCriterium->insertItem(MECHANICAL_WORK, "Mechanical Work");
//    costCriterium->insertItem(VISIBILITY, "Visibility");
//    costCriterium->setCurrentIndex((int)(INTEGRAL));
//
//    connect(costCriterium, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)), Qt::DirectConnection);

    m_ui->greedyLayout->addWidget(greedy);
    m_ui->greedyLayout->addWidget(nbGreedyTraj);
    m_ui->greedyLayout->addWidget(numberIterations);
    m_ui->greedyLayout->addWidget(hight);
    m_ui->greedyLayout->addWidget(maxFactor);
    m_ui->greedyLayout->addWidget(minStep);
    m_ui->greedyLayout->addWidget(costStep);
//    m_ui->greedyLayout->addWidget(costCriterium);
//    m_ui->greedyLayout->addWidget(biasPos);

}

void MainWindow::biasPos() {
//    Robot* R = new Robot(XYZ_ROBOT);
//    CostOptimization* costOptim = new CostOptimization(R,R->getTrajStruct());
//    tr1::shared_ptr<Configuration> q = costOptim->cheat();
//    costOptim->getRobot()->setAndUpdate(*q);
//    this->drawAllWinActive();
}

void MainWindow::greedyPlan() {
    //	  runButton->setDisabled(true);
    //	  resetButton->setDisabled(true);
    std::string str = "p3d_RunGreedy";

    isPlanning();

    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}


