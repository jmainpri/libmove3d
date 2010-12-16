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
//#include "P3d-pkg.h"
#include "../p3d/proto/p3d_setpos_proto.h"
#include "../p3d/proto/p3d_get_proto.h"

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
	p3d_destroy_config(m_Robot,m_qInit);
  p3d_destroy_config(m_Robot,m_qGoal);
}

void ManipulationTestFunctions::setDebugMode(bool value)
{
  if( m_manipulation )
  {
    m_manipulation->setDebugMode(value);
  }
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
		
		m_qInit = p3d_copy_config(m_Robot,m_Robot->ROBOT_POS);
		//m_qInit = p3d_get_robot_config(m_Robot);
		m_qGoal = p3d_copy_config(m_Robot,m_Robot->ROBOT_GOTO);

    // Warning SCENARIO dependant part, gsJidoKukaSAHand.p3d
    m_OBJECT_NAME = "GREY_TAPE";
    
    m_objGoto.resize(3);
    
//    m_objGoto[0] = 3.90;  // X
//    m_objGoto[1] = -2.85; // Y
//    m_objGoto[2] = 1.30;  // Z
    
    m_objGoto[0] = 4.23;
    m_objGoto[1] = -2.22;
    m_objGoto[2] = 1.00;

  }
	
  return;
}

bool ManipulationTestFunctions::manipTest(MANIPULATION_TASK_TYPE_STR type)
{
	bool succeed = false;
	
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
	std::vector <p3d_traj*> trajs;
	
//	if (p3d_equal_config(m_Robot, m_qInit, m_qGoal)) 
//	{
//		cout << "ManipulationTestFunctions::p3d_equal_config(m_Robot, m_qInit, m_qGoal)" << endl;
//		return succeed;
//	}
  
  cout << "Manipulation planning for " << m_OBJECT_NAME << endl;
	
	MANIPULATION_TASK_MESSAGE status;
	
	switch ( (unsigned int) m_manipulation->robot()->lpl_type ) 
	{
		case P3D_LINEAR_PLANNER :
		{
			status = m_manipulation->armPlanTask(type,0,m_qInit,m_qGoal, m_objStart, m_objGoto, m_OBJECT_NAME.c_str(), "", trajs);
			
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
			
			status = m_manipulation->armPlanTask(type,0,m_qInit,m_qGoal, m_objStart, m_objGoto, m_OBJECT_NAME.c_str(), "", confs, smTrajs);
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


//! Tests the ARM_PICK_GOTO task for different orientations of the object (but keeping its current position).
//! \param rotate_only_around_z if true the orientations are obtained from the object's current orientation by random rotations around the vertical axis,
//! if false the rotations are fully randomly generated.
bool ManipulationTestFunctions::manipTestGraspingWithDifferentObjectOrientations(bool rotate_only_around_z)
{
  MANIPULATION_TASK_MESSAGE status;
  
  bool result =false;
  int n, nbOrientations;
  p3d_rob *object;
  double x, y, z, rx, ry, rz;

  object= (p3d_rob*) p3d_get_robot_by_name((char*) m_OBJECT_NAME.c_str());
  if(object==NULL)
  { 
    printf("%s: %d: there is no robot named \"%s\".\n",__FILE__,__LINE__,m_OBJECT_NAME.c_str());
    return false;
  }
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);
  
  m_manipulation->setDebugMode(false);
  
  n= 0;
  nbOrientations= 100;
  for(int i=1; i<=nbOrientations; ++i)
  {
    printf("****************test %d/%d************************\n",i,nbOrientations);
    p3d_set_and_update_this_robot_conf(m_manipulation->robot(), m_qInit);

    if(rotate_only_around_z)
    {  p3d_set_freeflyer_pose2(object, x, y, z, 0, 0, p3d_random(-M_PI,M_PI));  }
    else
    {  p3d_set_freeflyer_pose2(object, x, y, z, p3d_random(-M_PI,M_PI), p3d_random(-M_PI,M_PI), p3d_random(-M_PI,M_PI));  } 
 
    //result = manipTest(ARM_PICK_GOTO);
    //if( result )
    //{  n++; }   
    p3d_multiLocalPath_disable_all_groupToPlan(m_manipulation->robot());
    p3d_multiLocalPath_set_groupToPlan(m_manipulation->robot(), m_manipulation->getUpBodyMLP() , 1);
    
    m_manipulation->checkConfigForCartesianMode(m_qInit, object);
    m_manipulation->fixAllHands(m_qInit, false);
    
    ManipulationData data(m_Robot);
    if ( m_manipulation->findArmGraspsConfigs(0, object, data) == MANIPULATION_TASK_OK) 
    {
      n++;
      cout << "!!! OK =-) !!!" << endl;
    }
    else {
      p3d_set_and_update_this_robot_conf(m_manipulation->robot(), m_qInit);
    }
    p3d_set_and_update_this_robot_conf( m_manipulation->robot(), data.getApproachFreeConfig() );
    g3d_draw_allwin_active();
  }  
  
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("%s: %d: ARM_PICK_GOTO succeeded for %d/%d different orientations of the object.\n",__FILE__,__LINE__,n,nbOrientations);
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  
  return (n!=0 ? true : false);
}

//! Main function that 
//!
bool ManipulationTestFunctions::runTest(int id)
{
  initManipulationGenom();
  
  if (id == 1) 
  {
    return manipTest(ARM_FREE);
  }
  
  if (id == 2) 
  {
    return manipTest(ARM_PICK_GOTO);
  }
  
  
  if (id == 4) 
  {
    manipTest(ARM_PICK_GOTO);
    return manipTest(ARM_TAKE_TO_FREE);
  }

  if (id == 6) 
  {
    return this->manipTestGraspingWithDifferentObjectOrientations(false);
  }

  if (id == 7) 
  {
    return this->manipTestGraspingWithDifferentObjectOrientations(true);
  }
  if (id == 7) 
  {
    return this->manipTestGraspingWithDifferentObjectOrientations(true);
  }
  else {
    cout << "Test : " << id << " not defined " << endl;
  }

	
  return false;
}

