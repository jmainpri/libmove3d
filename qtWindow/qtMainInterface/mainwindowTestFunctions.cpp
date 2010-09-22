/*
 *  mainwindowTestFunctions.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/08/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "mainwindowTestFunctions.hpp"
#include "ui_mainwindow.h"

#include <iostream>
#include <tr1/memory>
#include <vector>

#ifdef CXX_PLANNER
#include "util/CppApi/testModel.hpp"
#include "util/CppApi/SaveContext.hpp"
#include "API/Device/robot.hpp"
#include "API/ConfigSpace/configuration.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "API/Trajectory/smoothing.hpp"
//Warning contains boos function that conlicts with Qt
//#include "API/Trajectory/RoboptimTrajectory.h"
#include "API/Trajectory/CostOptimization.hpp"
#include "API/Grids/GridToGraph/gridtograph.h"
#include "API/Search/GraphState.h"
#include "API/Grids/ThreeDPoints.h"
#include "API/Grids/BaseGrid.hpp"
#include "API/Grids/TwoDGrid.hpp"
//#include "API/Roadmap2/BoostGraphTest.h"
#include "cost_space.hpp"
#endif

#ifdef HRI_COSTSPACE
#include "ui_qtHrics.h"
#include "hri_costspace/HRICS_costspace.h"
#endif

MainWindowTestFunctions::MainWindowTestFunctions(MainWindow* MainWinPt) : m_mainWindow(MainWinPt)
{
	connect(m_mainWindow->Ui()->pushButtonTest1,SIGNAL(clicked()),this,SLOT(test1()));
	connect(m_mainWindow->Ui()->pushButtonTest2,SIGNAL(clicked()),this,SLOT(test2()));
	connect(m_mainWindow->Ui()->pushButtonTest3,SIGNAL(clicked()),this,SLOT(test3()));	
}

void MainWindowTestFunctions::test1()
{
	ENV.setBool(Env::tRrtComputeGradient,true);
	ENV.setDouble(Env::extensionStep,3.0);
	global_costSpace->setCost("costMap2D");
}

void MainWindowTestFunctions::test2()
{
	using namespace std;
	
	cout << "------------------- test2 -------------------" << endl;
	cout << "---------------------------------------------" << endl;
	
#ifdef HRI_COSTSPACE
	ENV.setBool(Env::drawGraph,false);
	ENV.setBool(Env::drawTraj,true);
	ENV.setBool(Env::biDir,true);
	ENV.setBool(Env::costStarRRT,false);
	ENV.setBool(Env::useBoxDist,true);
	ENV.setBool(Env::useBallDist,false);
	ENV.setBool(Env::costStarRRT,false);
	ENV.setDouble(Env::extensionStep,5.0);
	
	ENV.setDouble(Env::Knatural,0.0);
	ENV.setDouble(Env::Kreachable,0.0);
	ENV.setDouble(Env::CellSize,0.2);
	
	//#ifdef LIGHT_PLANNER
	//	Robot* rob = global_Project->getActiveScene()->getRobotByNameContaining("ROBOT");
	//	if(rob)
	//	{
	//		rob->activateCcConstraint();
	//	}
	//#endif
	
	if (HRICS_MotionPL) 
	{
		delete HRICS_MotionPL;
		HRICS_MotionPL= NULL;
	}
	
	HRICS_MotionPL = new HRICS::Workspace;
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initGrid();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initDistance();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initVisibility();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initNatural();
	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initReachable();
	
	HRICS_activeDist = HRICS_MotionPL->getDistance();
	API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
	
	PointsToDraw = new ThreeDPoints;
	
	m_mainWindow->Ui()->tabCost->getHriWidget()->Ui()->HRICSPlanner->setDisabled(false);
	m_mainWindow->Ui()->tabCost->getHriWidget()->Ui()->pushButtonMakeGrid->setDisabled(true);
	m_mainWindow->Ui()->tabCost->getHriWidget()->Ui()->pushButtonDeleteGrid->setDisabled(false);
	m_mainWindow->Ui()->tabCost->setCostFunction("costHRI");
	
	ENV.setBool(Env::HRIPlannerWS,true);
	ENV.setBool(Env::enableHri,true);
	ENV.setBool(Env::isCostSpace,true);

	ENV.setDouble(Env::zone_size,1.0);
	
	ENV.setBool(Env::drawDistance,false);
	//ENV.setBool(Env::drawGrid,true);
	m_mainWindow->drawAllWinActive();
	
	//	dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid()->computeVectorField();
	//	ENV.setBool(Env::drawGrid,true);
	//	ENV.setBool(Env::drawVectorField,true);
	//	drawAllWinActive();
#endif
}

void MainWindowTestFunctions::test3()
{
	/*roboptim::RoboptimTrajectory OptimTraj;
	 OptimTraj.run_CostMap();*/
	
	//boost_test_1();
	//boost_test_2();
	//boost_test_3();
	///boost_test_4();
	
	//	Graph* G = new Graph(XYZ_GRAPH);
}