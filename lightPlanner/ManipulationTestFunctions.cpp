/*
 *  ManipulationTestFunctions.cpp
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 05/11/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "proto/ManipulationTestFunctions.hpp"
#include "proto/ManipulationPlanner.hpp"

#include "Planner-pkg.h"

using namespace std;

//! Constructor
ManipulationTestFunctions::ManipulationTestFunctions()
{
	m_Robot = p3d_get_robot_by_name_containing("ROBOT");
	
	cout << "Manipulation planner robot is : " << m_Robot->name << endl;
	
	m_qInit = NULL;
	m_qGoal = NULL;
	
	m_manipulation = NULL;
}

//! Constructor
ManipulationTestFunctions::ManipulationTestFunctions(std::string RobotNameContains)
{
	m_Robot = p3d_get_robot_by_name_containing(RobotNameContains.c_str());
	
	m_qInit = NULL;
	m_qGoal = NULL;
	
	m_manipulation = NULL;
}

//! Destructor
ManipulationTestFunctions::~ManipulationTestFunctions()
{
	
}

//! Initializes the manipulation
//! 
void ManipulationTestFunctions::initManipulationGenom() 
{
	if (m_manipulation != NULL) 
	{
		delete m_manipulation;
		m_manipulation = NULL;
	}
	
  if (m_manipulation == NULL) 
	{
		cout << "ManipulationTestFunctions::newManipulationPlanner" << endl;
		
		m_manipulation= new ManipulationPlanner(m_Robot);
		//         manipulation->setArmType(GP_LWR); // set the arm type
		
		p3d_destroy_config(m_Robot,m_qInit);
		p3d_destroy_config(m_Robot,m_qGoal);
		
//		print_config(m_Robot, m_Robot->ROBOT_POS);
//		print_config(m_Robot, m_Robot->ROBOT_GOTO);
		
		m_qInit = p3d_copy_config(m_Robot,m_Robot->ROBOT_POS);
		//m_qInit = p3d_get_robot_config(m_Robot);
		m_qGoal = p3d_copy_config(m_Robot,m_Robot->ROBOT_GOTO);
  }
	
  return;
}

bool ManipulationTestFunctions::testArmFree()
{
	bool succeed = false;
	
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs; 
	
	if (p3d_equal_config(m_Robot, m_qInit, m_qGoal)) 
	{
		cout << "Error : ManipulationTestFunctions::p3d_equal_config(m_Robot, m_qInit, m_qGoal)" << endl;
		return succeed;
	}
	
	switch ( m_manipulation->robot()->lpl_type ) 
	{
		case P3D_LINEAR_PLANNER :
		{
			MANIPULATION_TASK_MESSAGE status = m_manipulation->armPlanTask(ARM_FREE,0,
																																	 m_qInit, 
																																	 m_qGoal, 
																																	 m_OBJECT_NAME.c_str(), "", trajs);
			if(status == MANIPULATION_TASK_OK )
			{
				m_manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
				
				for(unsigned int i = 1; i < trajs.size(); i++){
					p3d_concat_traj(m_manipulation->robot()->tcur, trajs[i]);
				}
			}
		}
			break;
			
		case P3D_MULTILOCALPATH_PLANNER :
			
			m_manipulation->armPlanTask(ARM_FREE,0,
																m_qInit, 
																m_qGoal, 
																m_OBJECT_NAME.c_str(), "", confs, smTrajs);
			break;
			
		case P3D_SOFT_MOTION_PLANNER:
			cout << "Manipulation : localpath softmotion should not be called" << endl;
			break;
	}
	
	return succeed;
}

bool ManipulationTestFunctions::testArmPickGoto()
{
	bool succeed = false;
	
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs; 
	
	switch ( m_manipulation->robot()->lpl_type ) 
	{
		case P3D_LINEAR_PLANNER :
		{
			MANIPULATION_TASK_MESSAGE status = m_manipulation->armPlanTask(ARM_PICK_GOTO,0,
																																	 m_qInit, 
																																	 m_qGoal, 
																																	 m_OBJECT_NAME.c_str(), "", trajs);
			if(status == MANIPULATION_TASK_OK )
			{
				m_manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
				
				for(unsigned int i = 1; i < trajs.size(); i++){
					p3d_concat_traj(m_manipulation->robot()->tcur, trajs[i]);
				}
			}
		}
			break;
			
		case P3D_MULTILOCALPATH_PLANNER :
			
			m_manipulation->armPlanTask(ARM_PICK_GOTO,0,
																m_qInit, 
																m_qGoal, 
																m_OBJECT_NAME.c_str(), "", confs, smTrajs);
			break;
			
		case P3D_SOFT_MOTION_PLANNER:
			cout << "Manipulation : localpath softmotion should not be called" << endl;
			break;
			
		default:
			break;
	}
	
	return succeed;
}

void ManipulationTestFunctions::runTest(int id)
{
	m_OBJECT_NAME = "GREY_TAPE";
	
	if (id == 1) 
	{
		initManipulationGenom();
		testArmFree();
		return;
	}
	
	if (id == 2) 
	{
		initManipulationGenom();
		testArmPickGoto();
		return;
	}
}

// ------------------------------------------------------------------------------
// Arm manipulations
// ------------------------------------------------------------------------------

//#ifdef MULTILOCALPATH
//static ManipulationPlanner *manipulation= NULL;
//
//namespace Manip
//{
//	typedef enum ManipulationType
//	{
//		armFree,
//		pickGoto,
//		takeToFree,
//		pickGotoAndTakeToFree,
//	} 
//	ManipulationType;
//};
//
//Manip::ManipulationType ManipPhase;
//
//shared_ptr<Configuration> qInit;
//shared_ptr<Configuration> qGoal;
//
//
//#endif
//
///**
// * @ingroup qtWindow
// * @brief Planner thread class
// */
////-----------------------------------------------
//Manipulationthread::Manipulationthread(QObject* parent) :
//QThread(parent)
//{
//	
//}
//
//void Manipulationthread::run()
//{
//#ifdef MULTILOCALPATH	
//	//         double x, y, theta;
//	if (manipulation== NULL) 
//	{
//		initManipulationGenom();
//	}
//	
//	
//	
//	cout << "Selected object is : " << ENV.getString(Env::ObjectToCarry).toStdString() << endl;
//	
//	if ( ENV.getString(Env::ObjectToCarry).toStdString().compare("No Object") == 0) 
//	{
//		cout << "Warning : No object selected" << endl;
//		ENV.setBool(Env::isRunning,false);
//		return;
//	}
//	
//	string str = ENV.getString(Env::ObjectToCarry).toStdString();
//	const char* OBJECT_NAME = str.c_str();
//	
//	switch (ManipPhase) 
//	{
//		case Manip::armFree:
//		{
//			
//		}
//			
//			break;
//			
//		case Manip::pickGoto:
//		{
//			//         manipulation->setObjectToManipulate((char*)OBJECT_NAME);
//			//         manipulation->setSupport((char*)SUPPORT_NAME);
//			//         manipulation->setCameraJnt((char*)CAMERA_JNT_NAME);
//			//         manipulation->setCameraFOV(CAMERA_FOV);
//			//         manipulation->setCameraImageSize(200, 200);
//			
//			confs.clear();
//			smTrajs.clear();
//			trajs.clear();
//			
//			switch ( manipulation->robot()->lpl_type ) 
//			{
//				case P3D_LINEAR_PLANNER :
//				{
//					MANIPULATION_TASK_MESSAGE status = manipulation->armPlanTask(ARM_PICK_GOTO,0,
//																																			 qInit->getConfigStruct(), 
//																																			 qGoal->getConfigStruct(), 
//																																			 OBJECT_NAME, "", trajs);
//					if(status == MANIPULATION_TASK_OK )
//					{
//						manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
//						
//						for(unsigned int i = 1; i < trajs.size(); i++){
//							p3d_concat_traj(manipulation->robot()->tcur, trajs[i]);
//						}
//					}
//				}
//					break;
//					
//				case P3D_MULTILOCALPATH_PLANNER :
//					
//					manipulation->armPlanTask(ARM_PICK_GOTO,0,
//																		qInit->getConfigStruct(), 
//																		qGoal->getConfigStruct(), 
//																		OBJECT_NAME, "", confs, smTrajs);
//					break;
//					
//				case P3D_SOFT_MOTION_PLANNER:
//					cout << "Manipulation : localpath softmotion should not be called" << endl;
//					break;
//					
//				default:
//					break;
//			}
//			
//			break;
//		}
//			
//		case Manip::takeToFree:
//		{
//			confs.clear();
//			smTrajs.clear();
//			trajs.clear();
//			
//			switch ( manipulation->robot()->lpl_type ) 
//			{
//				case P3D_LINEAR_PLANNER :
//				{
//					MANIPULATION_TASK_MESSAGE status = manipulation->armPlanTask(ARM_PICK_TAKE_TO_FREE,0,qInit->getConfigStruct(), 
//																																			 qGoal->getConfigStruct(), 
//																																			 OBJECT_NAME, (char*)"", trajs);
//					
//					if( status == MANIPULATION_TASK_OK)
//					{
//						manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
//						
//						for(unsigned int i = 1; i < trajs.size(); i++){
//							p3d_concat_traj(manipulation->robot()->tcur, trajs[i]);
//						}
//					}
//				}
//					break;
//					
//				case P3D_MULTILOCALPATH_PLANNER :
//					
//					manipulation->armPlanTask(ARM_PICK_TAKE_TO_FREE,0,
//																		qInit->getConfigStruct(), 
//																		qGoal->getConfigStruct(),  
//																		OBJECT_NAME, "", confs, smTrajs);
//					break;
//					
//				case P3D_SOFT_MOTION_PLANNER:
//					cout << "Manipulation : localpath softmotion should not be called" << endl;
//					break;
//					
//				default:
//					break;
//			}
//			break;
//		}
//			
//		case Manip::pickGotoAndTakeToFree :
//		{
//			confs.clear();
//			smTrajs.clear();
//			trajs.clear();
//			
//			switch ( manipulation->robot()->lpl_type ) 
//			{
//				case P3D_LINEAR_PLANNER :
//				{
//					MANIPULATION_TASK_MESSAGE status = manipulation->armPlanTask(ARM_PICK_GOTO_AND_TAKE_TO_FREE,0,
//																																			 qInit->getConfigStruct(), 
//																																			 qGoal->getConfigStruct(), 
//																																			 OBJECT_NAME, (char*)"", trajs);
//					
//					if( status == MANIPULATION_TASK_OK)
//					{
//						manipulation->robot()->tcur = p3d_create_traj_by_copy(trajs[0]);
//						
//						for(unsigned int i = 1; i < trajs.size(); i++){
//							p3d_concat_traj(manipulation->robot()->tcur, trajs[i]);
//						}
//					}
//				}
//					break;
//					
//				case P3D_MULTILOCALPATH_PLANNER :
//					
//					manipulation->armPlanTask(ARM_PICK_GOTO_AND_TAKE_TO_FREE,0,
//																		qInit->getConfigStruct(), 
//																		qGoal->getConfigStruct(),  
//																		OBJECT_NAME, "", confs, smTrajs);
//					
//					//manipulation->robot()
//					
//					break;
//					
//				case P3D_SOFT_MOTION_PLANNER:
//					cout << "Manipulation : localpath softmotion should not be called" << endl;
//					break;
//					
//				default:
//					break;
//			}
//			break;
//		}
//		default:
//			cout << "Manipulation : request does not exist" << endl;
//			break;
//	}
//	
//	
//	//	g3d_win *win= NULL;
//	//	win= g3d_get_cur_win();
//	//	win->fct_draw2= &(genomDraw);
//	//	win->fct_key1= &(genomKey);
//	g3d_draw_allwin_active();
//	ENV.setBool(Env::isRunning,false);
//	cout << "Ends Manipulation Thread" << endl;
//#endif
//}
//
//void RobotWidget::armFree()
//{
//	cout << "Manipulation : free" << endl;
//	
//#ifdef MULTILOCALPATH	
//	ManipPhase = Manip::armFree;
//	Manipulationthread* manip = new Manipulationthread(this);
//	m_mainWindow->isPlanning();
//	manip->start();
//#else
//	cout << "Error : use MultiLocalPath" << endl;
//#endif
//}
//
//void RobotWidget::armPickGoto()
//{
//	cout << "Manipulation : pick goto" << endl;
//	
//#ifdef MULTILOCALPATH	
//	ManipPhase = Manip::pickGoto;
//	Manipulationthread* manip = new Manipulationthread(this);
//	m_mainWindow->isPlanning();
//	manip->start();
//#else
//	cout << "Error : use MultiLocalPath" << endl;
//#endif
//}
//
//void RobotWidget::armPickTakeToFree()
//{
//	cout << "Manipulation : take to free" << endl;
//	
//#ifdef MULTILOCALPATH	
//	ManipPhase = Manip::takeToFree;
//	Manipulationthread* manip = new Manipulationthread(this);
//	m_mainWindow->isPlanning();
//	manip->start();
//#else
//	cout << "Error : use MultiLocalPath" << endl;
//#endif
//}
//
//void RobotWidget::armPickGotoAndTakeToFree()
//{
//	cout << "Manipulation : pick goto and take to free" << endl;
//	
//#ifdef MULTILOCALPATH	
//	ManipPhase = Manip::pickGotoAndTakeToFree;
//	Manipulationthread* manip = new Manipulationthread(this);
//	m_mainWindow->isPlanning();
//	manip->start();
//#else
//	cout << "Error : use MultiLocalPath" << endl;
//#endif
//}
