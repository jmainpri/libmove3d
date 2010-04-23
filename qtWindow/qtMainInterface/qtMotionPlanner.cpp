/*
 *  qtMotionPlanner.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "qtMotionPlanner.hpp"
#include "ui_qtMotionPlanner.h"

#include <iostream>
#include <tr1/memory>

#include "../qtBase/SpinBoxSliderConnector_p.hpp"

#include "P3d-pkg.h"
#include "Planner-pkg.h"

#include "cppToQt.hpp"

using namespace std;
using namespace tr1;

MotionPlanner::MotionPlanner(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::MotionPlanner)
{
    m_ui->setupUi(this);
	
	initDiffusion();
	initPRM();
	initOptim();
	initMultiRun();
}

MotionPlanner::~MotionPlanner()
{
    delete m_ui;
}


//---------------------------------------------------------------------
// DIFFUSION
//---------------------------------------------------------------------
void MotionPlanner::initDiffusion()
{
	//    m_mainWindow->connectCheckBoxToEnv(m_ui->isCostSpace,         Env::isCostSpace);
    m_mainWindow->connectCheckBoxToEnv(m_ui->isWithGoal,          Env::expandToGoal);
    m_mainWindow->connectCheckBoxToEnv(m_ui->isManhattan,         Env::isManhattan);
    m_mainWindow->connectCheckBoxToEnv(m_ui->isEST,               Env::treePlannerIsEST);
    m_mainWindow->connectCheckBoxToEnv(m_ui->isBidir,             Env::biDir);
    m_mainWindow->connectCheckBoxToEnv(m_ui->isBalanced,          Env::expandBalanced);
    m_mainWindow->connectCheckBoxToEnv(m_ui->isExpandControl,     Env::expandControl);
    m_mainWindow->connectCheckBoxToEnv(m_ui->isDiscardingNodes,   Env::discardNodes);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsGoalBias,  Env::isGoalBiased);
	//    m_mainWindow->connectCheckBoxToEnv(m_ui->isCostTransition,    Env::costBeforeColl);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRandomInCompCo, Env::randomConnectionToGoal);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxClosestInCompCo, Env::tryClosest);
    m_mainWindow->connectCheckBoxToEnv(m_ui->ligandAccessStop, Env::MLTRRTDistanceStop);
	
    m_ui->expansionMethod->setCurrentIndex((int)ENV.getExpansionMethod());
    connect(m_ui->expansionMethod, SIGNAL(currentIndexChanged(int)),&ENV, SLOT(setExpansionMethodSlot(int)), Qt::DirectConnection);
    connect(&ENV, SIGNAL(expansionMethodChanged(int)),m_ui->expansionMethod, SLOT(setCurrentIndex(int)));
	
    //    m_ui->lineEditMaxNodes->setText( QString::number(ENV.getInt(Env::maxNodeCompco)));
    m_ui->spinBoxMaxNodes->setValue(ENV.getInt(Env::maxNodeCompco));
    connect(m_ui->spinBoxMaxNodes, SIGNAL(valueChanged( int )), ENV.getObject(Env::maxNodeCompco), SLOT(set(int)));
    connect(ENV.getObject(Env::maxNodeCompco), SIGNAL(valueChanged( int )), m_ui->spinBoxMaxNodes, SLOT(setValue(int)));
	
    m_ui->spinBoxNbTry->setValue(ENV.getInt(Env::NbTry));
    connect(m_ui->spinBoxNbTry, SIGNAL(valueChanged( int )), ENV.getObject(Env::NbTry), SLOT(set(int)));
    connect(ENV.getObject(Env::NbTry), SIGNAL(valueChanged( int )), m_ui->spinBoxNbTry, SLOT(setValue(int)));
	
    //    connect(ENV.getObject(Env::maxNodeCompco),SIGNAL(valueChanged(int)),this,SLOT(setLineEditWithNumber(Env::maxNodeCompco,int)));
    //    connect(m_ui->lineEditMaxNodes,SIGNAL(getText(QString::number(int))),ENV.getObject(Env::maxNodeCompco),SLOT(setInt(int)));
    //    cout << "ENV.getBool(Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
	
    //    connect(m_ui->lineEditExtentionStep,SIGNAL(textEdited(QString)),this,SLOT(lineEditChangedStep()));
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxExtentionStep, m_ui->horizontalSliderExtentionStep , Env::extensionStep );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxBias, m_ui->horizontalSliderBias , Env::Bias );
}

//void MainWindow::setLineEditWithNumber(Env::intParameter p,int num)
//{
//    if(p == Env::maxNodeCompco)
//    {
//        m_ui->lineEditMaxNodes->setText(QString::number(num));
//    }
//}

//---------------------------------------------------------------------
// PRM
//---------------------------------------------------------------------
void MotionPlanner::initPRM()
{
    // PRMType
    connect(m_ui->comboBoxPRMType, SIGNAL(currentIndexChanged(int)),ENV.getObject(Env::PRMType),SLOT(set(int)));
    connect( ENV.getObject(Env::PRMType), SIGNAL(valueChanged(int)),m_ui->comboBoxPRMType, SLOT(setCurrentIndex(int)));
    m_ui->comboBoxPRMType->setCurrentIndex( 0 /*INTEGRAL*/ );
    // 0 => PRM
    // 1 => Visib
    // 2 => ACR
	
	//    connect(ENV.getObject(Env::PRMType), SIGNAL(valueChanged(int)),m_ui->comboBoxPRMType, SLOT(setCurrentIndex(int)));
	
	m_ui->spinBoxMaxConnect->setValue(ENV.getInt(Env::maxConnect));
    connect(m_ui->spinBoxMaxConnect, SIGNAL(valueChanged( int )), ENV.getObject(Env::maxConnect), SLOT(set(int)));
    connect(ENV.getObject(Env::maxConnect), SIGNAL(valueChanged( int )), m_ui->spinBoxMaxConnect, SLOT(setValue(int)));
	
}

//---------------------------------------------------------------------
// OPTIM
//---------------------------------------------------------------------
void MotionPlanner::initOptim()
{
	//    QPushButton* computeGridAndExtract = new QPushButton("2D Best Path");
	//    connect(computeGridAndExtract, SIGNAL(clicked()),this, SLOT(computeGridAndExtract()));
	
	//    LabeledSlider* numberIterations = createSlider(tr("Number of iteration"), Env::nbCostOptimize, 0, 5000 );
	
	//    QPushButton* optimize = new QPushButton("Cost Optimize");
	//    connect(optimize, SIGNAL(clicked()),this, SLOT(optimizeCost()),Qt::DirectConnection);
	
	//    QPushButton* shortCut = new QPushButton("Cost ShortCut");
	//    connect(shortCut, SIGNAL(clicked()),this, SLOT(shortCutCost()),Qt::DirectConnection);
	//
	//    QPushButton* removeNodes = new QPushButton("Remove Redundant Nodes");
	//    connect(removeNodes, SIGNAL(clicked()),this, SLOT(removeRedundant()),Qt::DirectConnection);
	
	//    QPushButton* testGraphSearch = new QPushButton("Dijkstra");
	//    connect(testGraphSearch, SIGNAL(clicked()),this, SLOT(graphSearchTest()),Qt::DirectConnection);
	
	//    QComboBox* costCriterium = new QComboBox();
	//    costCriterium->insertItem(INTEGRAL, "Integral");
	//    costCriterium->insertItem(MECHANICAL_WORK, "Mechanical Work");
	//    costCriterium->setCurrentIndex((int)(INTEGRAL));
	//    connect(costCriterium, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)), Qt::DirectConnection);
	
	//    LabeledDoubleSlider* step = createDoubleSlider(tr("Step"), Env::MinStep, 0, 100 );
	
    //    m_ui->optimizeLayout->addWidget(computeGridAndExtract);
    //    m_ui->optimizeLayout->addWidget(numberIterations);
    //    m_ui->optimizeLayout->addWidget(step);
    //    m_ui->optimizeLayout->addWidget(optimize);
    //    m_ui->optimizeLayout->addWidget(shortCut);
    //    m_ui->optimizeLayout->addWidget(removeNodes);
    //    m_ui->optimizeLayout->addWidget(testGraphSearch);
    //    m_ui->optimizeLayout->addWidget(costCriterium);
	
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostSpace2,Env::isCostSpace);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDebug2,Env::debugCostOptim);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxSaveTrajCost,Env::saveTrajCost);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxWithDeform,Env::withDeformation);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxWithShortCut,Env::withShortCut);
	
    connect(m_ui->pushButtonRandomShortCut,SIGNAL(clicked()),this,SLOT(shortCutCost()));
    connect(m_ui->pushButtonRemoveRedundantNodes,SIGNAL(clicked()),this,SLOT(removeRedundant()));
	
    connect(m_ui->pushButtonTriangleDeformation,SIGNAL(clicked()),this,SLOT(optimizeCost()));
	
    // costCriterium
    connect(m_ui->comboBoxTrajCostExtimation, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostCriterium(int)));   
    connect(ENV.getObject(Env::costDeltaMethod), SIGNAL(valueChanged(int)),m_ui->comboBoxTrajCostExtimation, SLOT(setCurrentIndex(int)));
    m_ui->comboBoxTrajCostExtimation->setCurrentIndex( MECHANICAL_WORK /*INTEGRAL*/ );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxNbRounds, m_ui->horizontalSliderNbRounds_2 , Env::nbCostOptimize );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxNbMultiSmooth, m_ui->horizontalSliderNbMultiSmooth , Env::nbMultiSmooth );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxMinDeformStep, m_ui->horizontalSliderMinDeformStep ,Env::MinStep );
	
    new QtShiva::SpinBoxSliderConnector(
										this, m_ui->doubleSpinBoxMaxDeformStep, m_ui->horizontalSliderMaxDeformStep ,Env::MaxFactor );
	
    connect(m_ui->pushButtonRunMultiSmooth,SIGNAL(clicked()),this,SLOT(runMultiSmooth()));
}

void MotionPlanner::setCostCriterium(int choice) {
    cout << "Set Delta Step Choise to " << choice << endl;
#ifdef P3D_PLANNER
    p3d_SetDeltaCostChoice(choice);
#endif
    ENV.setInt(Env::costDeltaMethod,choice);
}

void MotionPlanner::computeGrid()
{
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
#ifdef P3D_PLANNER
    p3d_CreateDenseRoadmap(robotPt);
#endif
}

void MotionPlanner::runMultiSmooth()
{
#ifdef WITH_XFORMS
    std::string str = "MultiSmooth";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	cout << "Not implemented" << endl;
#endif
	
}

/**
 * @ingroup qtWindow
 * @brief Planner thread class
 */
//-----------------------------------------------
SmoothThread::SmoothThread(QObject* parent) :
QThread(parent)
{
	
}

void SmoothThread::run()
{	
	qt_shortCut();
    cout << "Ends Smooth Thread" << endl;
}
//-----------------------------------------------

void MotionPlanner::optimizeCost()
{
#ifdef WITH_XFORMS
    std::string str = "optimize";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	cout << "Not implemented" << endl;
#endif
	
}

void MotionPlanner::shortCutCost()
{
#ifdef WITH_XFORMS
    std::string str = "shortCut";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	m_mainWindow->isPlanning();
    SmoothThread* ptrSmooth = new SmoothThread;
	cout << "Start Smooth Thread" << endl;
    ptrSmooth->start();
#endif
	
}

void MotionPlanner::removeRedundant()
{
#ifdef WITH_XFORMS
    std::string str = "removeRedunantNodes";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
#else
	cout << "Not implemented" << endl;
#endif
}



void MotionPlanner::extractBestTraj()
{
#ifdef P3D_PLANNER
    p3d_ExtractBestTraj(XYZ_GRAPH);
#endif
}

//---------------------------------------------------------------------
// Multiple Runs
//---------------------------------------------------------------------
void MotionPlanner::initMultiRun()
{
    connect(m_ui->pushButtonSaveContext, SIGNAL(clicked()),this,SLOT(saveContext()));
    connect(m_ui->pushButtonPrintSelected, SIGNAL(clicked()),this,SLOT(printContext()));
    connect(m_ui->pushButtonPrintAllContext, SIGNAL(clicked()),this,SLOT(printAllContext()));
    connect(m_ui->pushButtonResetContext, SIGNAL(clicked()),this,SLOT(resetContext()));
    connect(m_ui->pushButtonSetSelected, SIGNAL(clicked()),this,SLOT(setToSelected()));
	
    contextList = new QListWidget;
    m_ui->multiRunLayout->addWidget(contextList);
	
	//    new QtShiva::SpinBoxSliderConnector(
	//            this, m_ui->doubleSpinBoxNbRounds, m_ui->horizontalSliderNbRounds , Env::nbMultiRun );
	
	
    connect(m_ui->horizontalSliderNbMutliRun, SIGNAL(valueChanged(int)), m_ui->spinBoxNbMutliRun, SLOT(setValue(int)) );
    connect(m_ui->spinBoxNbMutliRun, SIGNAL(valueChanged(int)),ENV.getObject(Env::nbMultiRun), SLOT(set(int)) );
    connect(m_ui->spinBoxNbMutliRun, SIGNAL(valueChanged(int)),m_ui->horizontalSliderNbMutliRun, SLOT(setValue(int)) );
    connect(ENV.getObject(Env::nbMultiRun), SIGNAL(valueChanged(int)),m_ui->spinBoxNbMutliRun, SLOT(setValue(int)) );
    m_ui->spinBoxNbMutliRun->setValue(ENV.getInt(Env::nbMultiRun));
	
	
    connect(m_ui->pushButtonRunAllRRT, SIGNAL(clicked()),this,SLOT(runAllRRT()));
    connect(m_ui->pushButtonRunAllGreedy, SIGNAL(clicked()),this,SLOT(runAllGreedy()));
    connect(m_ui->pushButtonShowHisto, SIGNAL(clicked()),this, SLOT(showHistoWindow()));
	
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxStopMultiRun,Env::StopMultiRun);
}

void MotionPlanner::saveContext()
{
    ENV.setString(Env::nameOfFile,m_ui->lineEditContext->text());
	
    QListWidgetItem* item= new QListWidgetItem(contextList);
    itemList.push_back(item);
    itemList.back()->setText(m_ui->lineEditContext->text());
	
#ifdef CXX_PLANNNER
    storedContext.saveCurrentEnvToStack();
#endif
}

void MotionPlanner::printAllContext()
{
#ifdef CXX_PLANNNER
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
#endif
}

void MotionPlanner::printContext()
{
#ifdef CXX_PLANNNER
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
#endif
}

void MotionPlanner::setToSelected()
{
#ifdef CXX_PLANNNER
    if( storedContext.getNumberStored()>0)
    {
        int i =  contextList->currentRow();
        storedContext.switchCurrentEnvTo(i);
    }
    else{
        std::cout << "Warning: no context in stack" << std::endl;
    }
#endif
}

void MotionPlanner::resetContext()
{
#ifdef CXX_PLANNNER
    storedContext.clear();
    //	setContextUserApp(context);
    for(uint i=0;i<itemList.size();i++)
    {
        delete itemList.at(i);
    }
    itemList.clear();
#endif
}

void MotionPlanner::runAllRRT()
{
    //	runAllRounds->setDisabled(true);
    std::string str = "MultiRRT";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
	
}

void MotionPlanner::runAllGreedy()
{
    //	runAllRounds->setDisabled(true);
    std::string str = "MultiGreedy";
    write(qt_fl_pipe[1],str.c_str(),str.length()+1);
}

void MotionPlanner::showHistoWindow()
{
#ifdef QWT
    histoWin = new HistoWindow();
    histoWin->startWindow();
#endif
}
