#include "sidewindow.hpp"
#include "ui_sidewindow.hpp"

#include "../qtBase/SpinBoxSliderConnector_p.hpp"
#include "../cppToQt.hpp"

#include "../../userappli/CppApi/testModel.hpp"

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

using namespace std;
using namespace tr1;

SideWindow::SideWindow(QWidget *parent) :
        QWidget(parent),
        m_ui(new Ui::SideWindow)
{
    m_ui->setupUi(this);

    initDiffusion();
    initHRI();
    initHumanLike();
    initCost();
    initGreedy();
    initOptim();
    initTest();
}

SideWindow::~SideWindow()
{
    delete m_ui;
}

void SideWindow::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
    connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
}


LabeledSlider* SideWindow::createSlider(QString s, Env::intParameter p,
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

LabeledDoubleSlider* SideWindow::createDoubleSlider(QString s,
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

void SideWindow::changeEvent(QEvent *e)
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

void SideWindow::setMainWindow(MainWindow* mainWin)
{
    this->mainWin = mainWin;
    connect(greedy, SIGNAL(clicked()),this->mainWin, SLOT(isPlanning()));
}

//---------------------------------------------------------------------
// DIFFUSION
//---------------------------------------------------------------------
void SideWindow::initDiffusion()
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

void SideWindow::setLineEditWithNumber(Env::intParameter p,int num)
{
    if(p == Env::maxNodeCompco)
    {
        m_ui->lineEditMaxNodes->setText(QString::number(num));
    }
}

//---------------------------------------------------------------------
// HRI
//---------------------------------------------------------------------
void SideWindow::initHRI()
{
    connectCheckBoxToEnv(m_ui->enableHri,               Env::enableHri);
    connectCheckBoxToEnv(m_ui->enableHriTS,             Env::isHriTS);
    connectCheckBoxToEnv(m_ui->checkBoxDrawGrid,        Env::drawGrid);
    connectCheckBoxToEnv(m_ui->checkBoxDrawDistance,    Env::drawDistance);
    connectCheckBoxToEnv(m_ui->checkBoxDrawRandPoints,  Env::drawPoints);
    connectCheckBoxToEnv(m_ui->checkBoxHRICS_MOPL,      Env::hriCsMoPlanner);
    connectCheckBoxToEnv(m_ui->checkBoxBBDist,          Env::bbDist);
    connectCheckBoxToEnv(m_ui->checkBoxHRIGoalBiased,   Env::isGoalBiased);
    connectCheckBoxToEnv(m_ui->checkBoxInverseKinematics,  Env::isInverseKinematics);


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

void SideWindow::GrabObject()
{
#ifdef HRI_COSTSPACE
    AStarIn3DGrid();
    p3d_rob* robotPt = HRICS_MOPL->getActivRobot()->getRobotStruct();
    p3d_set_object_to_carry(robotPt,"Horse");
    p3d_grab_object(robotPt);
#endif
}

void SideWindow::ReleaseObject()
{
#ifdef HRI_COSTSPACE
   p3d_rob* robotPt = HRICS_MOPL->getActivRobot()->getRobotStruct();
   p3d_release_object(robotPt);
#endif
}

void SideWindow::setWhichTestSlot(int test)
{
#ifdef HRI_COSTSPACE
    hriSpace->changeTest(test);
    cout << "Change test to :" << test << endl;
#else
    cout << "HRI Planner not compiled nor linked" << endl;
#endif
}

void SideWindow::enableHriSpace()
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

void SideWindow::make3DHriGrid()
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

void SideWindow::delete3DHriGrid()
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

void SideWindow::zoneSizeChanged()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getDistance()->parseHumans();
    drawAllWinActive();
    cout << "Zone Size Changed" << endl;
#endif
}

void SideWindow::resetRandomPoints()
{
#ifdef HRI_COSTSPACE
    ENV.setBool(Env::drawPoints,false);
    if(PointsToDraw != NULL)
    {
        delete PointsToDraw;
    }
#endif
}

void SideWindow::drawAllWinActive()
{
#ifdef HRI_COSTSPACE
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#endif
}

void SideWindow::computeGridCost()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getGrid()->computeAllCellCost();
#endif
}

void SideWindow::resetGridCost()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getGrid()->resetCellCost();
#endif
}

void SideWindow::AStarIn3DGrid()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->computeAStarIn3DGrid();
    ENV.setBool(Env::drawTraj,true);
    this->drawAllWinActive();
#endif
}

void SideWindow::HRICSRRT()
{
    std::string str = "runHRICSRRT";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::computeWorkspacePath()
{
    std::string str = "computeWorkspacePath";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::computeHoleMotion()
{
    std::string str = "computeHoleManipulationPath";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::KDistance(double value)
{
#ifdef HRI_COSTSPACE
    //    cout << "HRI_WEIGHTS[0] = " <<  ENV.getDouble(Env::Kdistance) << endl;
    HRI_WEIGHTS[0] = ENV.getDouble(Env::Kdistance);
#endif
}

void SideWindow::KVisibility(double value)
{
#ifdef HRI_COSTSPACE
    //    cout << "HRI_WEIGHTS[1] = " <<  ENV.getDouble(Env::Kvisibility) << endl;
    HRI_WEIGHTS[1] = ENV.getDouble(Env::Kvisibility);
#endif
}

//---------------------------------------------------------------------
// Human Like
//---------------------------------------------------------------------
void SideWindow::initHumanLike()
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
void SideWindow::initCost()
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

void SideWindow::showTrajCost()
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

void SideWindow::setPlotedVector(vector<double> v)
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

void SideWindow::showTemperature()
{
#ifdef QWT
    TempWin* window = new TempWin();
    window->show();
#endif
}

void SideWindow::computeAStar()
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

void SideWindow::putGridInGraph()
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
void SideWindow::initGreedy()
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

void SideWindow::biasPos() {
    Robot* R = new Robot(XYZ_ROBOT);
    CostOptimization* costOptim = new CostOptimization(R,R->getTrajStruct());
    tr1::shared_ptr<Configuration> q = costOptim->cheat();
    costOptim->getRobot()->setAndUpdate(*q);
    std::string str = "g3d_draw_allwin_active";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::greedyPlan() {
    //	  runButton->setDisabled(true);
    //	  resetButton->setDisabled(true);
    std::string str = "p3d_RunGreedy";

    mainWin->isPlanning();

    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::setCostCriterium(int choise) {
    cout << "Set Delta Step Choise to " << choise << endl;
    p3d_SetDeltaCostChoice(choise);
}

//---------------------------------------------------------------------
// OPTIM
//---------------------------------------------------------------------
void SideWindow::initOptim()
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

void SideWindow::computeGridAndExtract()
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

void SideWindow::computeGrid()
{
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_CreateDenseRoadmap(robotPt);
}

void SideWindow::optimizeCost()
{
    std::string str = "optimize";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::shortCutCost()
{
    std::string str = "shortCut";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::removeRedundant()
{
    std::string str = "removeRedunantNodes";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void SideWindow::graphSearchTest()
{
    cout << "graphSearchTest" << endl;
    std::string str = "graphSearchTest";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}


void SideWindow::extractBestTraj()
{
    p3d_ExtractBestTraj(XYZ_GRAPH);
}

//---------------------------------------------------------------------
// TEST MODEL
//---------------------------------------------------------------------
void SideWindow::initTest()
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

void SideWindow::costTest()
{
    if(ENV.getBool(Env::isCostSpace))
    {
        TestModel tests;
        tests.nbOfCostPerSeconds();
    }
}

void SideWindow::collisionsTest()
{
    TestModel tests;
    tests.nbOfColisionsPerSeconds();
}

void SideWindow::localpathsTest()
{
    TestModel tests;
    tests.nbOfLocalPathsPerSeconds();
}

void SideWindow::allTests()
{
    TestModel tests;
    tests.runAllTests();
}

void SideWindow::setAttMatrix()
{
    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
    //  p3d_compute_attached_matrix_from_virt_obj(robotPt->ccCntrts[0]);
}
