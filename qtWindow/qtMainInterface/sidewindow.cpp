#include "sidewindow.hpp"
#include "ui_sidewindow.hpp"

#include "../../planning_api/planningAPI.hpp"
#include "../../planning_api/Trajectory/CostOptimization.hpp"
#include "../../planning_api/Trajectory/BaseOptimization.hpp"
#include "../../planner_cxx/HRICost/HriTaskSpaceCost.hpp"
#include "../../planner_cxx/HRICost/HriCost.hpp"
#include "../../userappli/CppApi/testModel.hpp"
#include "../cppToQt.hpp"
#include "../planning_api/Roadmap/search/dijkstra.hpp"
#include "../qtBase/SpinBoxSliderConnector_p.hpp"
#include "Hri_planner-pkg.h"

#ifdef QWT
#include "../qtPlot/basicPlot.hpp"
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

    m_ui->expansionMethod->setCurrentIndex((int)ENV.getExpansionMethod());
    connect(m_ui->expansionMethod, SIGNAL(currentIndexChanged(int)),&ENV, SLOT(setExpansionMethodSlot(int)), Qt::DirectConnection);
    connect(&ENV, SIGNAL(expansionMethodChanged(int)),m_ui->expansionMethod, SLOT(setCurrentIndex(int)));

    m_ui->lineEditMaxNodes->setText( QString::number(ENV.getInt(Env::maxNodeCompco)));
    //    connect(ENV.getObject(Env::maxNodeCompco),SIGNAL(valueChanged(int)),this,SLOT(setLineEditWithNumber(Env::maxNodeCompco,int)));
    //    connect(m_ui->lineEditMaxNodes,SIGNAL(getText(QString::number(int))),ENV.getObject(Env::maxNodeCompco),SLOT(setInt(int)));
    //    cout << "ENV.getBool(Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;

    //    connect(m_ui->lineEditExtentionStep,SIGNAL(textEdited(QString)),this,SLOT(lineEditChangedStep()));
    new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxExtentionStep, m_ui->horizontalSliderExtentionStep , Env::extensionStep );

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
    connectCheckBoxToEnv(m_ui->enableHri,           Env::enableHri);
    connectCheckBoxToEnv(m_ui->enableHriTS,         Env::isHriTS);
    connect(m_ui->pushButtonHRITS,SIGNAL(clicked()),this,SLOT(enableHriSpace()));
    connect(m_ui->whichTestBox, SIGNAL(currentIndexChanged(int)),this, SLOT(setWhichTestSlot(int)), Qt::DirectConnection);
    connect(m_ui->spinBoxJoint, SIGNAL(valueChanged(int)),ENV.getObject(Env::akinJntId), SLOT(set(int)), Qt::DirectConnection);
    connect(m_ui->pushButtonWorkspacePath, SIGNAL(clicked()),this, SLOT(computeWorkspacePath()), Qt::DirectConnection);
    connect(m_ui->pushButtonHoleMotion, SIGNAL(clicked()),this, SLOT(computeHoleMotion()), Qt::DirectConnection);
    m_ui->HRITaskSpace->setDisabled(true);

    QtShiva::SpinBoxSliderConnector *connectorD = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxDistance, m_ui->horizontalSliderDistance ,Env::Kdistance);
    QtShiva::SpinBoxSliderConnector *connectorV = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxVisibility, m_ui->horizontalSliderVisibility ,Env::Kvisibility );
    connect(connectorD,SIGNAL(valueChanged(double)),this,SLOT(KDistance(double)));
    connect(connectorV,SIGNAL(valueChanged(double)),this,SLOT(KVisibility(double)));
}

void SideWindow::setWhichTestSlot(int test)
{
#ifdef HRI_PLANNER
    hriSpace->changeTest(test);
    cout << "Change test to :" << test << endl;
#else
    cout << "HRI Planner not compiled nor linked" << endl;
#endif
}

void SideWindow::enableHriSpace()
{
#ifdef HRI_PLANNER
    if(hriSpace)
    {
        delete hriSpace;
    }
    hriSpace = new HriSpaceCost(XYZ_ROBOT,ENV.getInt(Env::akinJntId));
#else
    cout << "HRI Planner not compiled nor linked" << endl;
#endif

    ENV.setBool(Env::isCostSpace,true);
    ENV.setBool(Env::enableHri,true);
    ENV.setBool(Env::isHriTS,true);
    cout << "Env::enableHri is set to true, joint number is :"<< ENV.getInt(Env::akinJntId) << endl;
    cout << "Robot is :" << XYZ_ROBOT->name << endl;
    m_ui->HRITaskSpace->setDisabled(false);
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
    cout << "HRI_WEIGHTS[0] = " <<  ENV.getDouble(Env::Kdistance) << endl;
    HRI_WEIGHTS[0] = ENV.getDouble(Env::Kdistance);
}

void SideWindow::KVisibility(double value)
{
    cout << "HRI_WEIGHTS[1] = " <<  ENV.getDouble(Env::Kvisibility) << endl;
    HRI_WEIGHTS[1] = ENV.getDouble(Env::Kvisibility);
}


//---------------------------------------------------------------------
// COST
//---------------------------------------------------------------------
void SideWindow::initCost()
{
#ifdef QWT
    this->plot = new PlotWindow();
    connectCheckBoxToEnv(m_ui->checkBoxCostBefore,         Env::CostBeforeColl);
#endif
    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this,SLOT(showTrajCost()));
    qRegisterMetaType< std::vector<double> > ("std::vector<double>");
    connect(ENV.getObject(Env::costAlongTraj), SIGNAL(valueChanged(std::vector<double>)), this, SLOT(setPlotedVector(std::vector<double>)));
    //    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this->plot,SLOT(show()));
    connectCheckBoxToEnv(m_ui->checkBoxRescale,           Env::initPlot);
}

void SideWindow::setPlotedVector(vector<double> v)
{
    cout << "PLOTTING ------------------------------------------" << endl;
#ifdef QWT
    BasicPlot* myPlot = this->plot->getPlot();
    vector<double> cost = ENV.getVector(Env::costAlongTraj);
    cost.resize(myPlot->getPlotSize());
    myPlot->setData(cost);
    this->plot->show();
#endif
}

void SideWindow::showTrajCost()
{
    cout << "showTrajCost" << endl;
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_traj* CurrentTrajPt = robotPt->tcur;
#ifdef QWT
    BasicPlot* myPlot = this->plot->getPlot();
    int nbSample = myPlot->getPlotSize();

    Trajectory traj(new Robot(robotPt, new Graph(XYZ_GRAPH)),CurrentTrajPt);

    double step = traj.getRangeMax() / (double) nbSample;

    vector<double> cost;

    //    cout << "Traj param max = " << traj.getRangeMax() << endl;
    //    cout << "Traj step = " << step << endl;

    for( double param=0; param<traj.getRangeMax(); param = param + step)
    {
        tr1::shared_ptr<Configuration> ptr = traj.configAtParam(param);
        cost.push_back(ptr->cost());
        //        cout << cost.back() << endl;
    }

    myPlot->setData(cost);
    this->plot->show();
#endif
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
    Robot* R = new Robot(XYZ_ROBOT,new Graph(XYZ_GRAPH));
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
// GREEDY
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
  p3d_compute_attached_matrix_from_virt_obj(robotPt->ccCntrts[0]);
}

#include "moc_sidewindow.cpp"
