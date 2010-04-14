/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "qtCost.hpp"
#include "ui_qtCost.h"

#include <iostream>
#include <tr1/memory>

#include "../qtBase/SpinBoxSliderConnector_p.hpp"

#include "cppToQt.hpp"

#ifdef HRI_COSTSPACE
#include "../planner_cxx/HRI_CostSpace/HRICS_CSpace.h"
#endif

#ifdef QWT
#include "../qtPlot/basicPlot.hpp"
#include "../qtPlot/DoublePlot.hpp"
#include "../qtPlot/tempWin.hpp"
#endif

#include "mainwindow.hpp"
#include "qtMotionPlanner.hpp"

#include "Planner-pkg.h"


using namespace std;
using namespace tr1;

CostWidget::CostWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::CostWidget)
{
    m_ui->setupUi(this);
	
	//initCost();
	//initHRI();
	initHumanLike();
}

CostWidget::~CostWidget()
{
    delete m_ui;
//#ifdef QWT
//    delete this->plot;
//#endif
}

//---------------------------------------------------------------------
// HRI
//---------------------------------------------------------------------
void CostWidget::initHRI()
{
    m_mainWindow->connectCheckBoxToEnv(m_ui->enableHri_2,                   Env::enableHri);
    m_mainWindow->connectCheckBoxToEnv(m_ui->enableHriTS,                 Env::HRIPlannerTS);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawGrid,            Env::drawGrid);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawDistance,        Env::drawDistance);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawRandPoints,      Env::drawPoints);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRICS_MOPL,          Env::HRIPlannerWS);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBBDist,              Env::useBoxDist);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBallDist,            Env::useBallDist);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRIGoalBiased,       Env::isGoalBiased);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxInverseKinematics,   Env::isInverseKinematics);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxEnableHRIConfigSpace,   Env::HRIPlannerCS);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPlannerTRRT,      Env::HRIPlannerTRRT);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPathDistance,      Env::HRIPathDistance);
	
    connect(m_ui->checkBoxDrawGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
	
    connect(m_ui->pushButtonHRITS,SIGNAL(clicked()),this,SLOT(enableHriSpace()));
	
    // Wich Test
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
	
	
    connect(m_ui->pushButtonMake2DGrid,SIGNAL(clicked()),this,SLOT(make2DGrid()));
	
    connect(m_ui->pushButtonMakeGrid,SIGNAL(clicked()),this,SLOT(make3DHriGrid()));
    connect(m_ui->pushButtonDeleteGrid,SIGNAL(clicked()),this,SLOT(delete3DHriGrid()));
    m_ui->pushButtonDeleteGrid->setDisabled(true);
	
    connect(m_ui->pushButtonComputeCost,SIGNAL(clicked()),this,SLOT(computeGridCost()));
    connect(m_ui->pushButtonResetCost,SIGNAL(clicked()),this,SLOT(resetGridCost()));
	
    connect(m_ui->pushButtonAStarIn3DGrid,SIGNAL(clicked()),this,SLOT(AStarIn3DGrid()));
    connect(m_ui->pushButtonHRICSRRT,SIGNAL(clicked()),this,SLOT(HRICSRRT()));
	
    QtShiva::SpinBoxSliderConnector* connectorZoneSize  = new QtShiva::SpinBoxSliderConnector(
																							  this, m_ui->doubleSpinBoxZoneSize, m_ui->horizontalSliderZoneSize ,Env::zone_size );
    connect(connectorZoneSize,SIGNAL(valueChanged(double)),this,SLOT(zoneSizeChanged()),Qt::DirectConnection);
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxCellSize, m_ui->horizontalSliderCellSize ,Env::CellSize );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxPlanCellSize, m_ui->horizontalSliderPlanCellSize ,Env::PlanCellSize );
	
    m_ui->HRICSPlanner->setDisabled(true);
	
    connect(m_ui->pushButtonResetRandPoints,SIGNAL(clicked()),this,SLOT(resetRandomPoints()));
	
    m_ui->HRIConfigSpace->setDisabled(true);
	
    connect(m_ui->pushButtonNewHRIConfigSpace,SIGNAL(clicked()),this,SLOT(newHRIConfigSpace()));
    connect(m_ui->pushButtonDeleteHRICSpace,SIGNAL(clicked()),this,SLOT(deleteHRIConfigSpace()));
    m_ui->pushButtonDeleteHRICSpace->setDisabled(true);
	
    connect(m_ui->pushButtonCreateGrid,SIGNAL(clicked()),this,SLOT(makeGridHRIConfigSpace()));
    connect(m_ui->pushButtonCreatePlan,SIGNAL(clicked()),this,SLOT(makePlanHRIConfigSpace()));
    connect(m_ui->pushButtonAStarIn2DGrid,SIGNAL(clicked()),this,SLOT(AStarInPlanHRIConfigSpace()));
	
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRecomputeCost, Env::RecomputeCellCost);
	
    QtShiva::SpinBoxSliderConnector* connectorColor1 = new QtShiva::SpinBoxSliderConnector(
																						   this, m_ui->doubleSpinBoxColor1, m_ui->horizontalSliderColor1 , Env::colorThreshold1 );
    QtShiva::SpinBoxSliderConnector* connectorColor2 = new QtShiva::SpinBoxSliderConnector(
																						   this, m_ui->doubleSpinBoxColor2, m_ui->horizontalSliderColor2 , Env::colorThreshold2 );
	
    connect(connectorColor1,SIGNAL(valueChanged(double)),m_mainWindow,SLOT(drawAllWinActive()),Qt::DirectConnection);
    connect(connectorColor2,SIGNAL(valueChanged(double)),m_mainWindow,SLOT(drawAllWinActive()),Qt::DirectConnection);
	
    connect(m_ui->pushButtonWriteToObPlane,SIGNAL(clicked()),this,SLOT(writeToOBPlane()));
    connect(m_ui->pushButtonHRIPlanRRT,SIGNAL(clicked()),this,SLOT(hriPlanRRT()));
}

void CostWidget::setWhichTestSlot(int test)
{
    cout << "Change test to :" << test << endl;
	
	if(ENV.getBool(Env::HRIPlannerTS))
    {
#ifdef HRI_PLANNER
        hriSpace->changeTest(test);
#else
		cout << "HRI Planner not compiled nor linked" << endl;
#endif
    }
}

///////////////////////////////////////////////////////////////
void CostWidget::newHRIConfigSpace()
{
#ifdef HRI_COSTSPACE
    m_ui->HRIConfigSpace->setDisabled(false);
    HRICS_CSpaceMPL  = new HRICS::CSpace;
    HRICS_activeDist = HRICS_CSpaceMPL->getDistance();
	
    ENV.setBool(Env::HRIPlannerCS,true);
    ENV.setBool(Env::enableHri,true);
    ENV.setBool(Env::isCostSpace,true);
	
    ENV.setBool(Env::useBallDist,false);
    ENV.setBool(Env::useBoxDist,true);
	
    m_ui->pushButtonNewHRIConfigSpace->setDisabled(true);
    m_ui->pushButtonDeleteHRICSpace->setDisabled(false);
	
    m_mainWindow->drawAllWinActive();
#endif
}

void CostWidget::deleteHRIConfigSpace()
{
#ifdef HRI_COSTSPACE
    ENV.setBool(Env::drawGrid,false);
    ENV.setBool(Env::HRIPlannerCS,false);
	
    delete HRICS_CSpaceMPL;
	
    m_ui->pushButtonDeleteHRICSpace->setDisabled(true);
    m_ui->pushButtonNewHRIConfigSpace->setDisabled(false);
    m_ui->HRIConfigSpace->setDisabled(true);
    m_mainWindow->drawAllWinActive();
#endif
}

void CostWidget::makeGridHRIConfigSpace()
{
#ifdef HRI_COSTSPACE
    if(ENV.getBool(Env::HRIPlannerCS))
    {
        if(ENV.getInt(Env::hriCostType)==0)
        {
            cout << " Compute Distance Grid"  << endl;
            HRICS_CSpaceMPL->computeDistanceGrid();
            API_activeGrid = HRICS_CSpaceMPL->getGrid();
            ENV.setBool(Env::drawGrid,true);
            m_mainWindow->drawAllWinActive();
        }
        if(ENV.getInt(Env::hriCostType)==1)
        {
            cout << " Compute Visibility Grid"  << endl;
            HRICS_CSpaceMPL->computeVisibilityGrid();
            API_activeGrid = HRICS_CSpaceMPL->getGrid();
            ENV.setBool(Env::drawGrid,true);
            m_mainWindow->drawAllWinActive();
        }
        else
        {
            cout << "Nor Distance, Nor Visib : No grid made" << endl;
        }
    }
#endif
}

void CostWidget::makePlanHRIConfigSpace()
{
#ifdef HRI_COSTSPACE
    if(ENV.getBool(Env::HRIPlannerCS))
    {
        API_activeGrid = HRICS_CSpaceMPL->getPlanGrid();
        ENV.setBool(Env::drawGrid,true);
        m_mainWindow->setBoolFloor(false);
        m_mainWindow->drawAllWinActive();
    }
#endif
}

void CostWidget::AStarInPlanHRIConfigSpace()
{
#ifdef HRI_COSTSPACE
    cout << "Computing 2D HRI A*" << endl;
    if(ENV.getBool(Env::HRIPlannerCS))
    {
        HRICS_CSpaceMPL->computeAStarIn2DGrid();
        ENV.setBool(Env::drawTraj,true);
    }
#endif
}

void CostWidget::writeToOBPlane()
{
#ifdef HRI_COSTSPACE
    if( ENV.getBool(Env::HRIPlannerCS) && HRICS_CSpaceMPL->getPlanGrid() )
    {
        HRICS_CSpaceMPL->getPlanGrid()->writeToOBPlane();
    }
#endif
}

void CostWidget::hriPlanRRT()
{
#ifdef HRI_COSTSPACE
    HRICS_CSpaceMPL->initHriRRT();
    m_mainWindow->drawAllWinActive();
#endif
}

///////////////////////////////////////////////////////////////
void CostWidget::make3DHriGrid()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL = new HRICS::MainPlanner;
    HRICS_MOPL->initGrid();
    HRICS_MOPL->initDistance();
    m_ui->HRICSPlanner->setDisabled(false);
    ENV.setBool(Env::HRIPlannerWS,true);
    //    ENV.setBool(Env::biDir,false);
    ENV.setDouble(Env::zone_size,0.5);
    HRICS_activeDist = HRICS_MOPL->getDistance();
	//    enableHriSpace();
#endif
    m_ui->pushButtonMakeGrid->setDisabled(true);
    m_ui->pushButtonDeleteGrid->setDisabled(false);
	
    ENV.setBool(Env::enableHri,true);
    ENV.setBool(Env::isCostSpace,true);
	
    m_mainWindow->drawAllWinActive();
}

void CostWidget::delete3DHriGrid()
{
#ifdef HRI_COSTSPACE
    ENV.setBool(Env::drawGrid,false);
    ENV.setBool(Env::HRIPlannerWS,false);
	
    delete HRICS_MOPL;
    m_ui->HRICSPlanner->setDisabled(true);
	
    m_mainWindow->drawAllWinActive();
#endif
    m_ui->pushButtonMakeGrid->setDisabled(false);
    m_ui->pushButtonDeleteGrid->setDisabled(true);
}

void CostWidget::zoneSizeChanged()
{
#ifdef HRI_COSTSPACE
    if(ENV.getBool(Env::HRIPlannerWS))
    {
        HRICS_activeDist = HRICS_MOPL->getDistance();
        HRICS_activeDist->parseHumans();
    }
    else if(ENV.getBool(Env::HRIPlannerCS))
    {
        HRICS_activeDist = HRICS_CSpaceMPL->getDistance();
        HRICS_activeDist->parseHumans();
    }
	
    m_mainWindow->drawAllWinActive();
    cout << "Zone Size Changed" << endl;
#endif
}

void CostWidget::resetRandomPoints()
{
#ifdef HRI_COSTSPACE
    ENV.setBool(Env::drawPoints,false);
    if(PointsToDraw != NULL)
    {
        delete PointsToDraw;
    }
#endif
}

void CostWidget::computeGridCost()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getGrid()->computeAllCellCost();
    API_activeGrid = HRICS_MOPL->getGrid();
#endif
}

void CostWidget::resetGridCost()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->getGrid()->resetCellCost();
#endif
}

void CostWidget::AStarIn3DGrid()
{
#ifdef HRI_COSTSPACE
    HRICS_MOPL->computeAStarIn3DGrid();
    ENV.setBool(Env::drawTraj,true);
    m_mainWindow->drawAllWinActive();
#endif
}

void CostWidget::HRICSRRT()
{
	//    std::string str = "runHRICSRRT";
	//    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#ifdef HRI_COSTSPACE
    HRICS_MOPL->initHriRRT();
    ENV.setBool(Env::drawTraj,true);
    m_mainWindow->drawAllWinActive();
#endif
}

///////////////////////////////////////////////////////////////

void CostWidget::computeWorkspacePath()
{
    std::string str = "computeWorkspacePath";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void CostWidget::computeHoleMotion()
{
    std::string str = "computeHoleManipulationPath";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void CostWidget::make2DGrid()
{
    vector<double>  envSize(4);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
	
#ifdef HRI_COSTSPACE
    ENV.setBool(Env::drawGrid,false);
    API::TwoDGrid* grid = new API::TwoDGrid(ENV.getDouble(Env::CellSize),envSize);
    grid->createAllCells();
    ENV.setBool(Env::drawGrid,true);
    API_activeGrid = grid;
#endif
}

void CostWidget::KDistance(double value)
{
#ifdef HRI_PLANNER
    //    cout << "HRI_WEIGHTS[0] = " <<  ENV.getDouble(Env::Kdistance) << endl;
    HRI_WEIGHTS[0] = ENV.getDouble(Env::Kdistance);
#endif
}

void CostWidget::KVisibility(double value)
{
#ifdef HRI_PLANNER
    //    cout << "HRI_WEIGHTS[1] = " <<  ENV.getDouble(Env::Kvisibility) << endl;
    HRI_WEIGHTS[1] = ENV.getDouble(Env::Kvisibility);
#endif
}

///////////////////////////////////////////////////////////////
void CostWidget::enableHriSpace()
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
    ENV.setBool(Env::HRIPlannerTS,true);
    cout << "Env::enableHri is set to true, joint number is :"<< ENV.getInt(Env::akinJntId) << endl;
    cout << "Robot is :" << XYZ_ROBOT->name << endl;
    m_ui->HRITaskSpace->setDisabled(false);
#endif
}

//---------------------------------------------------------------------
// Human Like
//---------------------------------------------------------------------
void CostWidget::initHumanLike()
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
void CostWidget::initCost()
{
    m_mainWindow->connectCheckBoxToEnv(m_ui->isCostSpaceCopy,         Env::isCostSpace);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostBefore,      Env::costBeforeColl);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostExpandToGoal, Env::costExpandToGoal);
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxInitTemp, m_ui->horizontalSliderInitTemp , Env::initialTemperature );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxNFailMax, m_ui->horizontalSliderNFailMax , Env::temperatureRate );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxLengthWeight, m_ui->horizontalSliderLengthWeight , Env::KlengthWeight );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxMinConnectGap, m_ui->horizontalSliderMinConnectGap , Env::minimalFinalExpansionGap );
	
#ifdef QWT
    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this,SLOT(showTrajCost()));
    connect(m_ui->pushButtonShowHRITrajCost,SIGNAL(clicked()),this,SLOT(showHRITrajCost()));
    connect(m_ui->pushButtonShowTemp,SIGNAL(clicked()),this,SLOT(showTemperature()));
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRescale, Env::initPlot);
	
    this->plot = new BasicPlotWindow();
#endif
    qRegisterMetaType< std::vector<double> > ("std::vector<double>");
    connect(ENV.getObject(Env::costAlongTraj), SIGNAL(valueChanged(std::vector<double>)), this, SLOT(setPlotedVector(std::vector<double>)));
    //    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this->plot,SLOT(show()));
    connect(m_ui->pushButtonGridInGraph,SIGNAL(clicked()),this,SLOT(putGridInGraph()));
    //    connect(m_ui->pushButtonAStar,SIGNAL(clicked()),this,SLOT(computeAStar()));
	
    // costCriterium 2
    connect(m_ui->comboBoxTrajCostExtimation_2, SIGNAL(currentIndexChanged(int)),
			m_motionWidget, SLOT(setCostCriterium(int)));
	
    connect(ENV.getObject(Env::costDeltaMethod), SIGNAL(valueChanged(int)),m_ui->comboBoxTrajCostExtimation_2, SLOT(setCurrentIndex(int)));
    m_ui->comboBoxTrajCostExtimation_2->setCurrentIndex( MECHANICAL_WORK /*INTEGRAL*/ );
	
	connect(m_ui->pushButton2DAstar,SIGNAL(clicked()),this,SLOT(computeGridAndExtract()));
    connect(m_ui->pushButton2DDijkstra,SIGNAL(clicked()),this,SLOT(graphSearchTest()));
	
}

void CostWidget::computeGridAndExtract()
{
	
	
}

void CostWidget::graphSearchTest()
{
    cout << "Extracting Grid" << endl;
	
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	
#ifdef P3D_PLANNER
    if(XYZ_GRAPH)
    {
        p3d_del_graph(XYZ_GRAPH);
    }
	
    cout << "Creating Dense Roadmap" << endl;
    p3d_CreateDenseRoadmap(robotPt);
#endif
	
#ifdef CXX_PLANNER
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
	
    m_mainWindow->drawAllWinActive();
	
    delete traj;
#endif
}


void CostWidget::showTrajCost()
{
#if defined(QWT) && defined(CXX_PLANNER)
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
	
    cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
	
	
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

void CostWidget::showHRITrajCost()
{
#if defined(QWT) && defined(CXX_PLANNER)
    cout << "--------------------------------" << endl;
	
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_traj* CurrentTrajPt = robotPt->tcur;
	
    DoublePlot* myPlot = new DoublePlot(this->plot);
    myPlot->setGeometry(this->plot->getPlot()->geometry());
    int nbSample = myPlot->getPlotSize();
	
    Robot* thisRob = new Robot(robotPt);
    Trajectory traj(thisRob,CurrentTrajPt);
	
    cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
	
    double kDistanceTmp = ENV.getDouble(Env::Kdistance);
    double kVisibiliTmp = ENV.getDouble(Env::Kvisibility);
	
    ENV.setDouble(Env::Kvisibility,0.0);
    cout << "Distance Cost = " << traj.costDeltaAlongTraj() << endl;
	
    ENV.setDouble(Env::Kdistance,0.0);
    ENV.setDouble(Env::Kvisibility,kVisibiliTmp);
	
    cout << "Visibility Cost = " << traj.costDeltaAlongTraj() << endl;
    ENV.setDouble(Env::Kdistance,kDistanceTmp);
	
    cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
	
    double step = traj.getRangeMax() / (double) nbSample;
	
    vector<double> costDistance;
    vector<double> costVisibili;
	
    //    cout << "Traj param max = " << traj.getRangeMax() << endl;
    //    cout << "Traj step = " << step << endl;
	
    std::ostringstream oss;
    oss << "statFiles/CostAlongTraj.csv";
	
    const char *res = oss.str().c_str();
	
    std::ofstream s;
    s.open(res);
	
    cout << "Opening save file : " << res << endl;
	
    s << "Distance"  << ";";
    s << "Visib"  << ";";
	
    s << endl;
	
    for( double param=0; param<traj.getRangeMax(); param = param + step)
    {
        shared_ptr<Configuration> ptr = traj.configAtParam(param);
		
#ifdef HRI_COSTSPACE
        if(ENV.getBool(Env::HRIPlannerCS))
        {
            ptr->cost();
			
            double dCost = HRICS_CSpaceMPL->getLastDistanceCost();
            double vCost = HRICS_CSpaceMPL->getLastVisibiliCost();
            costDistance.push_back(dCost);
            costVisibili.push_back(vCost);
        }
        if(ENV.getBool(Env::HRIPlannerWS))
        {
            thisRob->setAndUpdate(*ptr);
			
            double dCost = HRICS_MOPL->getDistance()->getDistToZones()[0];
			
            int object = HRICS_MOPL->getIndexObjectDof();
			
            Vector3d cellCenter;
			
            cellCenter[0] = ptr->at(object+0);
            cellCenter[1] = ptr->at(object+1);
            cellCenter[2] = ptr->at(object+2);
			
            double vCost = HRICS_MOPL->getVisibilityCost(cellCenter);
			
            costDistance.push_back(ENV.getDouble(Env::Kdistance)*dCost);
            costVisibili.push_back(ENV.getDouble(Env::Kvisibility)*vCost);
			
            s << ENV.getDouble(Env::Kdistance)*dCost << ";";
            s << ENV.getDouble(Env::Kvisibility)*vCost << ";";
            s << endl;
        }
        //        cout << cost.back() << endl;
#endif
    }
	
    myPlot->setData(costDistance,costVisibili);
    delete this->plot->getPlot();
    this->plot->setPlot(myPlot);
    this->plot->show();
	
    s.close();
#endif
	
    cout << "Closing save file" << endl;
	
}

void CostWidget::setPlotedVector(vector<double> v)
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

void CostWidget::showTemperature()
{
#ifdef QWT
    TempWin* window = new TempWin();
    window->show();
#endif
}

void CostWidget::computeAStar()
{
    if(!ENV.getBool(Env::isCostSpace))
    {
        return;
    }
#ifdef P3D_PLANNER
    if(!(XYZ_GRAPH->start_nodePt))
    {
		
        XYZ_GRAPH->search_start = XYZ_GRAPH->nodes->N;
        XYZ_GRAPH->search_goal = XYZ_GRAPH->last_node->N;
        cout << "p3d_initSearch" << endl;
        p3d_initSearch(XYZ_GRAPH);
		
        cout << "Number Of Graph nodes = " << XYZ_GRAPH->nnode << endl;
#ifdef CXX_PLANNNER
        GraphState* InitialState = new GraphState(XYZ_GRAPH->nodes->N);
		
        //        N = new Node(ptrGraph,rob->getGoTo());
        //        ptrGraph->insertNode(N);
        //        ptrGraph->linkNode(N);
		
        API::AStar search;
        vector<API::State*> path = search.solve(InitialState);
		
        if(path.size() == 0 )
        {
            return;
        }
		
        Trajectory* traj = new Trajectory(new Robot(XYZ_ROBOT));
		
        for (unsigned int i=0;i<path.size();i++)
        {
            configPt conf = dynamic_cast<GraphState*>(path[i])->getGraphNode()->q;
            shared_ptr<Configuration> q(new Configuration(new Robot(XYZ_ROBOT),conf));
            traj->push_back(q);
        }
		
        traj->replaceP3dTraj();
#endif
        m_mainWindow->drawAllWinActive();
		
        cout << "solution : End Search" << endl;
    }
    else
    {
        cout << "No start node" << endl;
    }
#endif
}

void CostWidget::putGridInGraph()
{
    cout << "Computing Grid" << endl;
	
#ifdef CXX_PLANNNER
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
#endif
	
    m_mainWindow->drawAllWinActive();
}
