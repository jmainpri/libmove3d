#include "mainwindow.hpp"
#include "ui_mainwindow.hpp"


#include "../cppToQt.hpp"
//#include "qdebugstream.hpp"
#include "../qtOpenGL/glwidget.hpp"

#include "Graphic-pkg.h"

#include "../qtBase/SpinBoxSliderConnector_p.hpp"

#include "../../util/CppApi/testModel.hpp"

#include "../../planner_cxx/API/planningAPI.hpp"
#include "../../planner_cxx/API/Trajectory/CostOptimization.hpp"
#include "../../planner_cxx/API/Trajectory/BaseOptimization.hpp"

#include "../../planner_cxx/HRI_CostSpace/HRICS_HAMP.h"
#include "../../planner_cxx/HRI_CostSpace/HRICS_old.h"
#include "../../planner_cxx/HRI_CostSpace/Grid/HRICS_Grid.h"
#include "../../planner_cxx/HRI_CostSpace/Grid/HRICS_GridState.h"
#include "../../planner_cxx/HRI_CostSpace/HRICS_Planner.h"

#include "../../planner_cxx/API/3DGrid/GridToGraph/gridtograph.h"
#include "../../planner_cxx/API/Search/GraphState.h"

#include "../../planner_cxx/API/3DGrid/points.h"

#ifdef QWT
#include "../qtPlot/basicPlot.hpp"
#include "../qtPlot/tempWin.hpp"
#endif

#include "../util/CppApi/SaveContext.hpp"

Move3D2OpenGl* pipe2openGl;

using namespace std;
using namespace tr1;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);

    mKCDpropertiesWindow = new KCDpropertiesWindow();

    m_ui->OpenGL->setWinSize(G3D_WIN->size);
    pipe2openGl = new Move3D2OpenGl(m_ui->OpenGL);

    //    m_ui->sidePannel->setMainWindow(this);

    //UITHINGQPalette pal(m_ui->centralWidget->palette()); // copy widget's palette to non const QPalette
    //UITHINGQColor myColor(Qt::darkGray);
    //UITHINGpal.setColor(QPalette::Window,myColor);
    //UITHINGm_ui->centralWidget->setPalette( pal );        // set the widget's palette

    cout << "pipe2openGl = new Move3D2OpenGl(m_ui->OpenGL)" << endl;

    //    connect(m_ui->actionOpen,SIGNAL(triggered()),this,SLOT(open()));
    connect(m_ui->actionOpenScenario,SIGNAL(triggered()),this,SLOT(openScenario()));
    connect(m_ui->actionKCDPropietes,SIGNAL(triggered()),mKCDpropertiesWindow,SLOT(show()));

    //    connect(m_ui->pagesOfStakedWidget, SIGNAL(activated(int)),m_ui->stackedWidget, SLOT(setCurrentIndex(int)));

    connectCheckBoxes();

    // MainWindow
    initRunButtons();
    initViewerButtons();
    initLightSource();

    // Side Window
    initDiffusion();
    initHRI();
    initHumanLike();
    initCost();
    initGreedy();
    initOptim();
    initTest();
    initMultiRun();
}

MainWindow::~MainWindow()
{
    delete mKCDpropertiesWindow;
#ifdef QWT
    delete this->plot;
#endif
    delete m_ui;
}

const char *qt_fileName = NULL;

void MainWindow::openScenario()
{
    QString fileName = QFileDialog::getOpenFileName(this);

    if (!fileName.isEmpty())
    {
        qt_fileName = fileName.toStdString().c_str();

        std::string str = "readP3DScenarion";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);

        cout << "Open scenarion " << fileName.toStdString() << endl;
    }
}

void MainWindow::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
    connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
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

    m_ui->doubleSpinBoxLightX->setValue(G3D_WIN->lightPosition[0]);
    m_ui->doubleSpinBoxLightY->setValue(G3D_WIN->lightPosition[1]);
    m_ui->doubleSpinBoxLightZ->setValue(G3D_WIN->lightPosition[2]);
}

void MainWindow::changeLightPosX()
{
    float* lightPosition = G3D_WIN->lightPosition;
    lightPosition[0] = m_ui->doubleSpinBoxLightX->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN);
    //    cout << "Change X value" << endl;
}

void MainWindow::changeLightPosY()
{
    float* lightPosition = G3D_WIN->lightPosition;
    lightPosition[1] = m_ui->doubleSpinBoxLightY->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN);
    //    cout << "Change Y value" << endl;
}

void MainWindow::changeLightPosZ()
{
    float* lightPosition = G3D_WIN->lightPosition;
    lightPosition[2] = m_ui->doubleSpinBoxLightZ->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN);
    //    cout << "Change Z value" << endl;
}

// Viewer Buttons -----------------------------------------------
// --------------------------------------------------------------
void MainWindow::initViewerButtons()
{
    connect(m_ui->checkBoxDrawGraph,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()),Qt::DirectConnection);
    connect(m_ui->checkBoxDrawTraj,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()),Qt::DirectConnection);

    connectCheckBoxToEnv(m_ui->checkBoxDrawGraph,Env::drawGraph);
    connectCheckBoxToEnv(m_ui->checkBoxDrawTraj,Env::drawTraj);
    m_ui->checkBoxDrawGraph->setCheckState(Qt::Checked);

    connect(m_ui->pushButtonShowTraj,SIGNAL(clicked(bool)),this,SLOT(showTraj()),Qt::DirectConnection);

    //    m_ui->consoleOutput->setTextFormat(Qt::LogText);
    //    QDebugStream qout(cout, m_ui->consoleOutput);

    //UITHINGm_ui->progressBar->setValue(0);
    //    cout << "Max progress bar" << m_ui->progressBar->maximum() << endl;
    //UITHINGconnect(ENV.getObject(Env::progress),SIGNAL(valueChanged(int)),m_ui->progressBar,SLOT(setValue(int)));

    connect(m_ui->pushButtonRestoreView,SIGNAL(clicked(bool)),this,SLOT(restoreView()),Qt::DirectConnection);
    connect(m_ui->pushButtonResetGraph,SIGNAL(clicked()),this,SLOT(ResetGraph()));
}

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
    G3D_WIN->GHOST = value;
}

void MainWindow::setBoolBb(bool value)
{
    G3D_WIN->BB = value;
}


void MainWindow::setBoolFloor(bool value)
{
    G3D_WIN->displayFloor = value;
}


void MainWindow::setBoolTiles(bool value)
{
    G3D_WIN->displayTiles = value;
}


void MainWindow::setBoolWalls(bool value)
{
    G3D_WIN->displayWalls = value;
}

void MainWindow::setBoolShadows(bool value)
{
    G3D_WIN->displayShadows = value;
}

void MainWindow::setBoolSmooth(bool value)
{
    G3D_WIN->GOURAUD = value;
}

void MainWindow::setBoolFilaire(bool value)
{
    G3D_WIN->FILAIRE = value;
}

void MainWindow::restoreView()
{
    g3d_restore_win_camera(G3D_WIN);
    drawAllWinActive();
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

    connectCheckBoxToEnv(m_ui->checkBoxWithShortCut,      Env::withShortCut);

    connect( ENV.getObject(Env::isRunning), SIGNAL(valueChanged(bool)), this, SLOT(planningFinished(void)), Qt::DirectConnection);
}

void MainWindow::run()
{
    cout << "Run Motion Planning" << endl;

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
}

void MainWindow::stop()
{
    p3d_SetStopValue(true);
    ENV.setBool(Env::isRunning,false);
}

void MainWindow::reset()
{
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    m_ui->pushButtonRun->setDisabled(false);
    m_ui->pushButtonStop->setDisabled(true);
    m_ui->pushButtonReset->setDisabled(true);
    p3d_SetStopValue(false);
}

void MainWindow::isPlanning()
{
    m_ui->pushButtonRun->setDisabled(true);
    m_ui->pushButtonReset->setDisabled(true);
    m_ui->pushButtonStop->setDisabled(false);

    ENV.setBool(Env::isRunning,true);

    //UITHINGQPalette pal(Qt::lightGray); // copy widget's palette to non const QPalette
    //UITHINGm_ui->toolBox->setPalette( pal );        // set the widget's palette
}

void MainWindow::planningFinished()
{
    if( ENV.getBool(Env::isRunning) == false )
    {
        m_ui->pushButtonStop->setDisabled(true);
        m_ui->pushButtonReset->setDisabled(false);

        //UITHINGQPalette pal(Qt::white); // copy widget's palette to non const QPalette
        //UITHINGm_ui->toolBox->setPalette( pal );        // set the widget's palette
    }
    else
    {
        this->isPlanning();
    }
}
void MainWindow::drawAllWinActive()
{
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::ResetGraph()
{
    std::string str = "ResetGraph";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

// Show Traj functions ------------------------------------------
// --------------------------------------------------------------
static int traj_play = TRUE;
GLWidget* ptrOpenGL;

static int default_drawtraj_fct_qt_pipe(p3d_rob* robot, p3d_localpath* curLp)
{
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
    //    ptrOpenGL->updateGL();

    return(traj_play);
}

void MainWindow::showTraj()
{
    cout << "Show Traj....."<< endl;
    ptrOpenGL = m_ui->OpenGL;
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    g3d_show_tcur_rob(robotPt,default_drawtraj_fct_qt_pipe);
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

//***********************************************************************************
// Side Window
//***********************************************************************************
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
// DIFFUSION
//---------------------------------------------------------------------
void MainWindow::initDiffusion()
{
    connectCheckBoxToEnv(m_ui->isCostSpace,         Env::isCostSpace);
    connectCheckBoxToEnv(m_ui->isWithGoal,          Env::expandToGoal);
    connectCheckBoxToEnv(m_ui->isManhattan,         Env::isManhattan);
    connectCheckBoxToEnv(m_ui->isEST,               Env::treePlannerIsEST);
    connectCheckBoxToEnv(m_ui->isBidir,             Env::biDir);
    connectCheckBoxToEnv(m_ui->isBalanced,          Env::expandBalanced);
    connectCheckBoxToEnv(m_ui->isExpandControl,     Env::expandControl);
    connectCheckBoxToEnv(m_ui->isDiscardingNodes,   Env::discardNodes);
    connectCheckBoxToEnv(m_ui->checkBoxIsGoalBias,  Env::isGoalBiased);

    m_ui->expansionMethod->setCurrentIndex((int)ENV.getExpansionMethod());
    connect(m_ui->expansionMethod, SIGNAL(currentIndexChanged(int)),&ENV, SLOT(setExpansionMethodSlot(int)), Qt::DirectConnection);
    connect(&ENV, SIGNAL(expansionMethodChanged(int)),m_ui->expansionMethod, SLOT(setCurrentIndex(int)));

    m_ui->lineEditMaxNodes->setText( QString::number(ENV.getInt(Env::maxNodeCompco)));
    //    connect(ENV.getObject(Env::maxNodeCompco),SIGNAL(valueChanged(int)),this,SLOT(setLineEditWithNumber(Env::maxNodeCompco,int)));
    //    connect(m_ui->lineEditMaxNodes,SIGNAL(getText(QString::number(int))),ENV.getObject(Env::maxNodeCompco),SLOT(setInt(int)));
    //    cout << "ENV.getBool(Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;

    //    connect(m_ui->lineEditExtentionStep,SIGNAL(textEdited(QString)),this,SLOT(lineEditChangedStep()));
    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxExtentionStep, m_ui->horizontalSliderExtentionStep , Env::extensionStep );

    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxBias, m_ui->horizontalSliderBias , Env::Bias );
}

void MainWindow::setLineEditWithNumber(Env::intParameter p,int num)
{
    if(p == Env::maxNodeCompco)
    {
        m_ui->lineEditMaxNodes->setText(QString::number(num));
    }
}

//---------------------------------------------------------------------
// HRI
//---------------------------------------------------------------------
void MainWindow::initHRI()
{
    connectCheckBoxToEnv(m_ui->enableHri_2,                   Env::enableHri);
    connectCheckBoxToEnv(m_ui->enableHriTS,                 Env::isHriTS);
    connectCheckBoxToEnv(m_ui->checkBoxDrawGrid,            Env::drawGrid);
    connectCheckBoxToEnv(m_ui->checkBoxDrawDistance,        Env::drawDistance);
    connectCheckBoxToEnv(m_ui->checkBoxDrawRandPoints,      Env::drawPoints);
    connectCheckBoxToEnv(m_ui->checkBoxHRICS_MOPL,          Env::hriCsMoPlanner);
    connectCheckBoxToEnv(m_ui->checkBoxBBDist,              Env::bbDist);
    connectCheckBoxToEnv(m_ui->checkBoxHRIGoalBiased,       Env::isGoalBiased);
    connectCheckBoxToEnv(m_ui->checkBoxInverseKinematics,   Env::isInverseKinematics);


    connect(m_ui->checkBoxDrawGrid,SIGNAL(clicked()),this,SLOT(drawAllWinActive()));
    connect(m_ui->pushButtonHRITS,SIGNAL(clicked()),this,SLOT(enableHriSpace()));

    connect(m_ui->whichTestBox, SIGNAL(currentIndexChanged(int)),ENV.getObject(Env::hriCostType), SLOT(set(int)), Qt::DirectConnection);
    connect(ENV.getObject(Env::hriCostType), SIGNAL(valueChanged(int)),this, SLOT(setWhichTestSlot(int)), Qt::DirectConnection);
    m_ui->whichTestBox->setCurrentIndex(ENV.getInt(Env::hriCostType));

    connect(m_ui->spinBoxJoint, SIGNAL(valueChanged(int)),ENV.getObject(Env::akinJntId), SLOT(set(int)), Qt::DirectConnection);
    connect(ENV.getObject(Env::akinJntId), SIGNAL(valueChanged(int)),m_ui->spinBoxJoint, SLOT(setValue(int)), Qt::DirectConnection);
    m_ui->spinBoxJoint->setValue(ENV.getInt(Env::akinJntId));

    connect(m_ui->pushButtonWorkspacePath, SIGNAL(clicked()),this, SLOT(computeWorkspacePath()), Qt::DirectConnection);
    connect(m_ui->pushButtonHoleMotion, SIGNAL(clicked()),this, SLOT(computeHoleMotion()), Qt::DirectConnection);
    m_ui->HRITaskSpace->setDisabled(true);

    QtShiva::SpinBoxSliderConnector *connectorD = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxDistance, m_ui->horizontalSliderDistance, Env::Kdistance);
    QtShiva::SpinBoxSliderConnector *connectorV = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxVisibility, m_ui->horizontalSliderVisibility, Env::Kvisibility );

    connect(connectorD,SIGNAL(valueChanged(double)),this,SLOT(KDistance(double)));
    connect(connectorV,SIGNAL(valueChanged(double)),this,SLOT(KVisibility(double)));

    connect(m_ui->pushButtonMakeGrid,SIGNAL(clicked()),this,SLOT(make3DHriGrid()));
    connect(m_ui->pushButtonDeleteGrid,SIGNAL(clicked()),this,SLOT(delete3DHriGrid()));

    connect(m_ui->pushButtonComputeCost,SIGNAL(clicked()),this,SLOT(computeGridCost()));
    connect(m_ui->pushButtonResetCost,SIGNAL(clicked()),this,SLOT(resetGridCost()));

    connect(m_ui->pushButtonAStaIn3DGrid,SIGNAL(clicked()),this,SLOT(AStarIn3DGrid()));
    connect(m_ui->pushButtonHRICSRRT,SIGNAL(clicked()),this,SLOT(HRICSRRT()));

    QtShiva::SpinBoxSliderConnector *connectorCell = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxCellSize, m_ui->horizontalSliderCellSize ,Env::CellSize );

    QtShiva::SpinBoxSliderConnector *connectorZoneSize = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxZoneSize, m_ui->horizontalSliderZoneSize ,Env::zone_size );

    connect(connectorZoneSize,SIGNAL(valueChanged(double)),this,SLOT(zoneSizeChanged()),Qt::DirectConnection);

    m_ui->HRICSPlanner->setDisabled(true);

    connect(m_ui->pushButtonResetRandPoints,SIGNAL(clicked()),this,SLOT(resetRandomPoints()));

    connect(m_ui->pushButtonGrabObject,SIGNAL(clicked()),this,SLOT(GrabObject()));
    connect(m_ui->pushButtonReleaseObject,SIGNAL(clicked()),this,SLOT(ReleaseObject()));
}

void MainWindow::GrabObject()
{
#ifdef HRI_COSTSPACE
    AStarIn3DGrid();
    p3d_rob* robotPt = HRICS_MOPL->getActivRobot()->getRobotStruct();
    p3d_set_object_to_carry(robotPt,"Horse");
    p3d_grab_object(robotPt);
#endif
}

void MainWindow::ReleaseObject()
{
#ifdef HRI_COSTSPACE
    p3d_rob* robotPt = HRICS_MOPL->getActivRobot()->getRobotStruct();
    p3d_release_object(robotPt);
#endif
}

void MainWindow::setWhichTestSlot(int test)
{
#ifdef HRI_COSTSPACE
    hriSpace->changeTest(test);
    cout << "Change test to :" << test << endl;
#else
    cout << "HRI Planner not compiled nor linked" << endl;
#endif
}

void MainWindow::enableHriSpace()
{
#ifdef HRI_COSTSPACE
    //    if(hriSpace)
    //    {
    //        delete hriSpace;
    //    }
    //    hriSpace = new HriSpaceCost(XYZ_ROBOT,ENV.getInt(Env::akinJntId));
#else
    cout << "HRI Planner not compiled nor linked" << endl;
#endif

#ifdef HRI_COSTSPACE
    ENV.setBool(Env::isCostSpace,true);
    ENV.setBool(Env::enableHri,true);
    ENV.setBool(Env::isHriTS,true);
    cout << "Env::enableHri is set to true, joint number is :"<< ENV.getInt(Env::akinJntId) << endl;
    cout << "Robot is :" << XYZ_ROBOT->name << endl;
    m_ui->HRITaskSpace->setDisabled(false);
#endif
}

void MainWindow::make3DHriGrid()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL = new HRICS::MainPlanner;
    HRICS_MOPL->initGrid();
    HRICS_MOPL->initDistance();
    m_ui->HRICSPlanner->setDisabled(false);
    ENV.setBool(Env::hriCsMoPlanner,true);
    //    ENV.setBool(Env::biDir,false);
    ENV.setDouble(Env::zone_size,0.7);
    enableHriSpace();
#endif
}

void MainWindow::delete3DHriGrid()
{
#ifdef HRI_COSTSPACE
    ENV.setBool(Env::drawGrid,false);
    ENV.setBool(Env::hriCsMoPlanner,false);

    delete HRICS_MOPL;
    m_ui->HRICSPlanner->setDisabled(true);

    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#endif

}

void MainWindow::zoneSizeChanged()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getDistance()->parseHumans();
    drawAllWinActive();
    cout << "Zone Size Changed" << endl;
#endif
}

void MainWindow::resetRandomPoints()
{
#ifdef HRI_COSTSPACE
    ENV.setBool(Env::drawPoints,false);
    if(PointsToDraw != NULL)
    {
        delete PointsToDraw;
    }
#endif
}

void MainWindow::computeGridCost()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getGrid()->computeAllCellCost();
#endif
}

void MainWindow::resetGridCost()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getGrid()->resetCellCost();
#endif
}

void MainWindow::AStarIn3DGrid()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->computeAStarIn3DGrid();
    ENV.setBool(Env::drawTraj,true);
    this->drawAllWinActive();
#endif
}

void MainWindow::HRICSRRT()
{
    std::string str = "runHRICSRRT";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::computeWorkspacePath()
{
    std::string str = "computeWorkspacePath";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::computeHoleMotion()
{
    std::string str = "computeHoleManipulationPath";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::KDistance(double value)
{
#ifdef HRI_COSTSPACE
    //    cout << "HRI_WEIGHTS[0] = " <<  ENV.getDouble(Env::Kdistance) << endl;
    HRI_WEIGHTS[0] = ENV.getDouble(Env::Kdistance);
#endif
}

void MainWindow::KVisibility(double value)
{
#ifdef HRI_COSTSPACE
    //    cout << "HRI_WEIGHTS[1] = " <<  ENV.getDouble(Env::Kvisibility) << endl;
    HRI_WEIGHTS[1] = ENV.getDouble(Env::Kvisibility);
#endif
}

//---------------------------------------------------------------------
// Human Like
//---------------------------------------------------------------------
void MainWindow::initHumanLike()
{
    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxNatural, m_ui->horizontalSliderNatural , Env::coeffNat );

    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxJointLimit, m_ui->horizontalSliderJointLimit , Env::coeffLim );
    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxTaskDist, m_ui->horizontalSliderTaskDist , Env::coeffTas );
    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxHeight, m_ui->horizontalSliderHeight , Env::coeffHei );
}

//---------------------------------------------------------------------
// COST
//---------------------------------------------------------------------
void MainWindow::initCost()
{
    connectCheckBoxToEnv(m_ui->isCostSpaceCopy,         Env::isCostSpace);
    connectCheckBoxToEnv(m_ui->checkBoxCostBefore,      Env::CostBeforeColl);

    QtShiva::SpinBoxSliderConnector *connectorInitTemp = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxInitTemp, m_ui->horizontalSliderInitTemp ,Env::initialTemperature );

    QtShiva::SpinBoxSliderConnector *connectorNFailMax = new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxNFailMax, m_ui->horizontalSliderNFailMax ,Env::temperatureRate );

#ifdef QWT
    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this,SLOT(showTrajCost()));
    connect(m_ui->pushButtonShowTemp,SIGNAL(clicked()),this,SLOT(showTemperature()));
    connectCheckBoxToEnv(m_ui->checkBoxRescale, Env::initPlot);

    this->plot = new BasicPlotWindow();

#endif

    qRegisterMetaType< std::vector<double> > ("std::vector<double>");
    connect(ENV.getObject(Env::costAlongTraj), SIGNAL(valueChanged(std::vector<double>)), this, SLOT(setPlotedVector(std::vector<double>)));
    //    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this->plot,SLOT(show()));
    connect(m_ui->pushButtonGridInGraph,SIGNAL(clicked()),this,SLOT(putGridInGraph()));
    //    connect(m_ui->pushButtonAStar,SIGNAL(clicked()),this,SLOT(computeAStar()));
}

void MainWindow::showTrajCost()
{
#ifdef QWT
    cout << "showTrajCost" << endl;
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_traj* CurrentTrajPt = robotPt->tcur;

    BasicPlot* myPlot = new BasicPlot(this->plot);
    myPlot->setGeometry(this->plot->getPlot()->geometry());
    int nbSample = myPlot->getPlotSize();

    Trajectory traj(new Robot(robotPt),CurrentTrajPt);

    double step = traj.getRangeMax() / (double) nbSample;

    vector<double> cost;

    //    cout << "Traj param max = " << traj.getRangeMax() << endl;
    //    cout << "Traj step = " << step << endl;

    for( double param=0; param<traj.getRangeMax(); param = param + step)
    {
        shared_ptr<Configuration> ptr = traj.configAtParam(param);
        cost.push_back(ptr->cost());
        //        cout << cost.back() << endl;
    }

    myPlot->setData(cost);
    delete this->plot->getPlot();
    this->plot->setPlot(myPlot);
    this->plot->show();
#endif
}

void MainWindow::setPlotedVector(vector<double> v)
{
    cout << "PLOTTING ------------------------------------------" << endl;
#ifdef QWT
    BasicPlot* myPlot = dynamic_cast<BasicPlot*>(this->plot->getPlot());
    vector<double> cost = ENV.getVector(Env::costAlongTraj);
    cost.resize(myPlot->getPlotSize());
    myPlot->setData(cost);
    this->plot->show();
#endif
}

void MainWindow::showTemperature()
{
#ifdef QWT
    TempWin* window = new TempWin();
    window->show();
#endif
}

void MainWindow::computeAStar()
{
    if(!ENV.getBool(Env::isCostSpace))
    {
        return;
    }

    if(!(XYZ_GRAPH->start_nodePt))
    {

        XYZ_GRAPH->search_start = XYZ_GRAPH->nodes->N;
        XYZ_GRAPH->search_goal = XYZ_GRAPH->last_node->N;
        cout << "p3d_initSearch" << endl;
        p3d_initSearch(XYZ_GRAPH);

        cout << "Number Of Graph nodes = " << XYZ_GRAPH->nnode << endl;
        GraphState* InitialState = new GraphState(XYZ_GRAPH->nodes->N);

        //        N = new Node(ptrGraph,rob->getGoTo());
        //        ptrGraph->insertNode(N);
        //        ptrGraph->linkNode(N);

        AStar search;
        vector<API::State*> path = search.solve(InitialState);

        if(path.size() == 0 )
        {
            return;
        }

        Trajectory* traj = new Trajectory(new Robot(XYZ_ROBOT));

        for (int i=0;i<path.size();i++)
        {
            configPt conf = dynamic_cast<GraphState*>(path[i])->getGraphNode()->q;
            shared_ptr<Configuration> q(new Configuration(new Robot(XYZ_ROBOT),conf));
            traj->push_back(q);
        }

        traj->replaceP3dTraj();
        std::string str = "g3d_draw_allwin_active";
        write(qt_fl_pipe[1],str.c_str(),str.length()+1);

        cout << "solution : End Search" << endl;
    }
    else
    {
        cout << "No start node" << endl;
    }
}

void MainWindow::putGridInGraph()
{
    cout << "Computing Grid" << endl;

    Vector3i     gridSize;

    gridSize[0] = 10;
    gridSize[1] = 10;
    gridSize[2] = 10;

    //    vector<double>  envSize(6);
    //    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    //    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    //    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;

    //    Grid myGrid(gridSize,envSize);
    //    myGrid.createAllCells();
    //
    //    for(int i=0;i<myGrid.getNumberOfCells();i++)
    //    {
    //        vector<double> center = myGrid.getCell(i)->getCenter();
    //        cout << i << " =  ("<< center[0] << "," << center[1] << "," << center[2] << ")" << endl;
    //    }
    //---------------
    GridToGraph theGrid(gridSize);
    theGrid.putGridInGraph();

    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

//---------------------------------------------------------------------
// GREEDY
//---------------------------------------------------------------------
void MainWindow::initGreedy()
{
    greedy = new QPushButton("Greedy Planner");
    connect(greedy, SIGNAL(clicked()),this, SLOT(greedyPlan()),Qt::DirectConnection);

    connectCheckBoxToEnv(m_ui->checkBoxDebug,               Env::debugCostOptim);
    connectCheckBoxToEnv(m_ui->checkBoxRecomputeTrajCost,   Env::trajCostRecompute);
    connectCheckBoxToEnv(m_ui->checkBoxWithShortCut,        Env::withShortCut);
    connectCheckBoxToEnv(m_ui->checkBoxUseTRRT,             Env::useTRRT);

    LabeledSlider* nbGreedyTraj = createSlider(tr("Number of trajectories"), Env::nbGreedyTraj, 1, 10 );
    LabeledSlider* hight = createSlider(tr("Height Factor"), Env::heightFactor, 1, 10 );
    LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 500 );
    LabeledDoubleSlider* maxFactor = createDoubleSlider(tr("Start Factor"), Env::MaxFactor, 0, 2000 );
    LabeledDoubleSlider* minStep = createDoubleSlider(tr("Min step"), Env::MinStep, 0, 1000 );
    LabeledDoubleSlider* costStep = createDoubleSlider(tr("Cost Step"), Env::CostStep, 0.01, 10 );

    //	double dmax=0;
    //	p3d_col_get_dmax(&dmax);
    //
    //	minStep->setValue(dmax);
    numberIterations->setValue(ENV.getInt(Env::nbCostOptimize));

    QPushButton* biasPos = new QPushButton("Biased Pos");
    connect(biasPos, SIGNAL(clicked()),this, SLOT(biasPos()),Qt::DirectConnection);

    QComboBox* costCriterium = new QComboBox();
    costCriterium->insertItem(INTEGRAL, "Integral");
    costCriterium->insertItem(MECHANICAL_WORK, "Mechanical Work");
    costCriterium->insertItem(VISIBILITY, "Visibility");
    costCriterium->setCurrentIndex((int)(INTEGRAL));

    connect(costCriterium, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)), Qt::DirectConnection);

    m_ui->greedyLayout->addWidget(greedy);
    m_ui->greedyLayout->addWidget(nbGreedyTraj);
    m_ui->greedyLayout->addWidget(numberIterations);
    m_ui->greedyLayout->addWidget(hight);
    m_ui->greedyLayout->addWidget(maxFactor);
    m_ui->greedyLayout->addWidget(minStep);
    m_ui->greedyLayout->addWidget(costStep);
    m_ui->greedyLayout->addWidget(costCriterium);
    m_ui->greedyLayout->addWidget(biasPos);
}

void MainWindow::biasPos() {
    Robot* R = new Robot(XYZ_ROBOT);
    CostOptimization* costOptim = new CostOptimization(R,R->getTrajStruct());
    tr1::shared_ptr<Configuration> q = costOptim->cheat();
    costOptim->getRobot()->setAndUpdate(*q);
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::greedyPlan() {
    //	  runButton->setDisabled(true);
    //	  resetButton->setDisabled(true);
    std::string str = "p3d_RunGreedy";

    isPlanning();

    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::setCostCriterium(int choise) {
    cout << "Set Delta Step Choise to " << choise << endl;
    p3d_SetDeltaCostChoice(choise);
}

//---------------------------------------------------------------------
// OPTIM
//---------------------------------------------------------------------
void MainWindow::initOptim()
{
    QPushButton* computeGridAndExtract = new QPushButton("2D Best Path");
    connect(computeGridAndExtract, SIGNAL(clicked()),this, SLOT(computeGridAndExtract()));

    LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 5000 );

    QPushButton* optimize = new QPushButton("Cost Optimize");
    connect(optimize, SIGNAL(clicked()),this, SLOT(optimizeCost()),Qt::DirectConnection);

    QPushButton* shortCut = new QPushButton("Cost ShortCut");
    connect(shortCut, SIGNAL(clicked()),this, SLOT(shortCutCost()),Qt::DirectConnection);

    QPushButton* removeNodes = new QPushButton("Remove Redundant Nodes");
    connect(removeNodes, SIGNAL(clicked()),this, SLOT(removeRedundant()),Qt::DirectConnection);

    QPushButton* testGraphSearch = new QPushButton("Dijkstra");
    connect(testGraphSearch, SIGNAL(clicked()),this, SLOT(graphSearchTest()),Qt::DirectConnection);

    QComboBox* costCriterium = new QComboBox();
    costCriterium->insertItem(INTEGRAL, "Integral");
    costCriterium->insertItem(MECHANICAL_WORK, "Mechanical Work");
    costCriterium->setCurrentIndex((int)(INTEGRAL));

    connect(costCriterium, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)), Qt::DirectConnection);

    LabeledDoubleSlider* step = createDoubleSlider(tr("Step"), Env::MinStep, 0, 100 );

    m_ui->optimizeLayout->addWidget(computeGridAndExtract);
    m_ui->optimizeLayout->addWidget(numberIterations);
    m_ui->optimizeLayout->addWidget(step);
    m_ui->optimizeLayout->addWidget(optimize);
    m_ui->optimizeLayout->addWidget(shortCut);
    m_ui->optimizeLayout->addWidget(removeNodes);
    m_ui->optimizeLayout->addWidget(testGraphSearch);
    m_ui->optimizeLayout->addWidget(costCriterium);
}

void MainWindow::computeGridAndExtract()
{
    cout << "Extracting Grid" << endl;

    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

    if(XYZ_GRAPH)
    {
        p3d_del_graph(XYZ_GRAPH);
    }

    cout << "Creating Dense Roadmap" << endl;
    p3d_CreateDenseRoadmap(robotPt);

    Graph* ptrGraph = new Graph(XYZ_GRAPH);

    shared_ptr<Configuration> Init = ptrGraph->getRobot()->getInitialPosition();
    shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoTo();

    cout << "Dijkstra graph search on graph" << endl;
    Dijkstra graphSearch(ptrGraph);

    cout << "graphSearch.extractTrajectory" << endl;
    Trajectory* traj = graphSearch.extractTrajectory(Init,Goal);

    cout << "-------------------------------" << endl;
    cout << "Trajectory Cost = "<< traj->cost() << endl;
    cout << "   nl = "<< traj->getNbPaths() << endl;
    cout << "   length = "<< traj->getRangeMax() << endl;

    ENV.setBool(Env::drawGraph,false);
    ENV.setBool(Env::drawTraj,true);

    traj->replaceP3dTraj();

    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);

    delete traj;

}

void MainWindow::computeGrid()
{
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_CreateDenseRoadmap(robotPt);
}

void MainWindow::optimizeCost()
{
    std::string str = "optimize";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::shortCutCost()
{
    std::string str = "shortCut";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::removeRedundant()
{
    std::string str = "removeRedunantNodes";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::graphSearchTest()
{
    cout << "graphSearchTest" << endl;
    std::string str = "graphSearchTest";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}


void MainWindow::extractBestTraj()
{
    p3d_ExtractBestTraj(XYZ_GRAPH);
}

//---------------------------------------------------------------------
// TEST MODEL
//---------------------------------------------------------------------
void MainWindow::initTest()
{
    connect(m_ui->pushButtonCollision,SIGNAL(clicked()),this,SLOT(collisionsTest()));
    connect(m_ui->pushButtonLocalPath,SIGNAL(clicked()),this,SLOT(localpathsTest()));
    connect(m_ui->pushButtonCost,SIGNAL(clicked()),this,SLOT(costTest()));
    connect(m_ui->pushButtonTestAll,SIGNAL(clicked()),this,SLOT(allTests()));

    connect(ENV.getObject(Env::numberOfCollisionPerSec),SIGNAL(valueChanged(QString)),m_ui->labelCollision,SLOT(setText(QString)));
    connect(ENV.getObject(Env::numberOfLocalPathPerSec),SIGNAL(valueChanged(QString)),m_ui->labelLocalPath,SLOT(setText(QString)));
    connect(ENV.getObject(Env::numberOfCostPerSec),SIGNAL(valueChanged(QString)),m_ui->labelTimeCost,SLOT(setText(QString)));

    connect(m_ui->pushButtonAttMat,SIGNAL(clicked()),this,SLOT(setAttMatrix()));
}

void MainWindow::costTest()
{
    if(ENV.getBool(Env::isCostSpace))
    {
        TestModel tests;
        tests.nbOfCostPerSeconds();
    }
}

void MainWindow::collisionsTest()
{
    TestModel tests;
    tests.nbOfColisionsPerSeconds();
}

void MainWindow::localpathsTest()
{
    TestModel tests;
    tests.nbOfLocalPathsPerSeconds();
}

void MainWindow::allTests()
{
    TestModel tests;
    tests.runAllTests();
}

void MainWindow::setAttMatrix()
{
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    //  p3d_compute_attached_matrix_from_virt_obj(robotPt->ccCntrts[0]);
}

//---------------------------------------------------------------------
// Multiple Runs
//---------------------------------------------------------------------
void MainWindow::initMultiRun()
{
    connect(m_ui->pushButtonSaveContext, SIGNAL(clicked()),this,SLOT(saveContext()));
    connect(m_ui->pushButtonPrintSelected, SIGNAL(clicked()),this,SLOT(printContext()));
    connect(m_ui->pushButtonPrintAllContext, SIGNAL(clicked()),this,SLOT(printAllContext()));
    connect(m_ui->pushButtonResetContext, SIGNAL(clicked()),this,SLOT(resetContext()));
    connect(m_ui->pushButtonSetSelected, SIGNAL(clicked()),this,SLOT(setToSelected()));

    contextList = new QListWidget;
    m_ui->multiRunLayout->addWidget(contextList);

    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxNbRounds, m_ui->horizontalSliderNbRounds , Env::nbRound );

    connect(m_ui->pushButtonRunAllRRT, SIGNAL(clicked()),this,SLOT(runAllRRT()));
    connect(m_ui->pushButtonRunAllGreedy, SIGNAL(clicked()),this,SLOT(runAllGreedy()));
    connect(m_ui->pushButtonShowHisto, SIGNAL(clicked()),this, SLOT(showHistoWindow()));
}

void MainWindow::saveContext()
{
    ENV.setString(Env::nameOfFile,m_ui->lineEditContext->text());

    QListWidgetItem* item= new QListWidgetItem(contextList);
    itemList.push_back(item);
    itemList.back()->setText(m_ui->lineEditContext->text());

    storedContext.saveCurrentEnvToStack();
}

void MainWindow::printAllContext()
{
    if( storedContext.getNumberStored()>0){

        for(uint i=0;i<storedContext.getNumberStored();i++){
            std::cout << "------------ Context Number " << i << " ------------" << std::endl;
            storedContext.printData(i);
        }
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << " Total number of contexts in stack =  " << storedContext.getNumberStored() << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
    }
    else{
        std::cout << "Warning: no context in stack" << std::endl;
    }
}

void MainWindow::printContext()
{
    if( storedContext.getNumberStored() > 0 )
    {
        int i =  contextList->currentRow();
        std::cout << "------------ Context Number " << i << " ------------" << std::endl;
        storedContext.printData(i);
    }
    else
    {
        std::cout << "Warning: no context in stack" << std::endl;
    }
}

void MainWindow::setToSelected()
{
    if( storedContext.getNumberStored()>0)
    {
        int i =  contextList->currentRow();
        storedContext.switchCurrentEnvTo(i);
    }
    else{
        std::cout << "Warning: no context in stack" << std::endl;
    }
}

void MainWindow::resetContext()
{
    storedContext.clear();
    //	setContextUserApp(context);
    for(uint i=0;i<itemList.size();i++)
    {
        delete itemList.at(i);
    }
    itemList.clear();
}

void MainWindow::runAllRRT()
{
    //	runAllRounds->setDisabled(true);
//    std::string str = "MultiRRT";
//    write(qt_fl_pipe[1],str.c_str(),str.length()+1);

}

void MainWindow::runAllGreedy()
{
    //	runAllRounds->setDisabled(true);
    std::string str = "MultiGreedy";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MainWindow::showHistoWindow()
{
#ifdef QWT
    histoWin = new HistoWindow();
    histoWin->startWindow();
#endif
}

#include "moc_mainwindow.cpp"
