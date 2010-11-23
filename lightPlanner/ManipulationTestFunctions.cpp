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
//! A new manipulation planner is created 
//! and the initial and goal configuration
//! are created
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

bool ManipulationTestFunctions::manipTest(MANIPULATION_TASK_TYPE_STR type)
{
	bool succeed = false;
	
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs;
  std::vector<double> objStart, objGoto;
	
	if (p3d_equal_config(m_Robot, m_qInit, m_qGoal)) 
	{
		cout << "ManipulationTestFunctions::p3d_equal_config(m_Robot, m_qInit, m_qGoal)" << endl;
		return succeed;
	}
	
	MANIPULATION_TASK_MESSAGE status;
	
	switch ( (unsigned int) m_manipulation->robot()->lpl_type ) 
	{
		case P3D_LINEAR_PLANNER :
		{
			status = m_manipulation->armPlanTask(type,0,m_qInit,m_qGoal, objStart, objGoto, m_OBJECT_NAME.c_str(), "", trajs);
			
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
			
			status = m_manipulation->armPlanTask(type,0,m_qInit,m_qGoal, objStart, objGoto, m_OBJECT_NAME.c_str(), "", confs, smTrajs);
			break;
			
		case P3D_SOFT_MOTION_PLANNER:
			cout << "Manipulation : localpath softmotion should not be called" << endl;
			succeed = false;
			break;
	}
	
	
	if (status != MANIPULATION_TASK_OK ) 
	{
		succeed = false;
	}
	else 
	{
		succeed = true;
	}

	return succeed;
}


//! Main function that 
//!
bool ManipulationTestFunctions::runTest(int id)
{
	m_OBJECT_NAME = "GREY_TAPE";
	
	initManipulationGenom();
	
	if (id == 1) 
	{
		return manipTest(ARM_FREE);
	}
	
	if (id == 2) 
	{
		return manipTest(ARM_PICK_GOTO);
	}
	
	if (id == 3) 
	{
		return manipTest(ARM_PICK_GOTO_AND_TAKE_TO_FREE);
	}
	
	return false;
}

