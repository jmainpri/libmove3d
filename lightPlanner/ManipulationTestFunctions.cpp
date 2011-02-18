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

#include <fstream>
#include <sstream>

using namespace std;

ManipulationTestFunctions* global_manipPlanTest = NULL;

//! Constructor
ManipulationTestFunctions::ManipulationTestFunctions()
{
	m_Robot = p3d_get_robot_by_name_containing("JIDOKUKA_ROBOT");
	
	cout << "Manipulation planner robot is : " << m_Robot->name << endl;
	
	m_qInit = NULL;
	m_qGoal = NULL;
	
	m_manipulation = NULL;
  
  m_nbOrientations = 100;
}

//! Constructor
ManipulationTestFunctions::ManipulationTestFunctions(std::string RobotNameContains)
{
	m_Robot = p3d_get_robot_by_name_containing(RobotNameContains.c_str());
	
	m_qInit = NULL;
	m_qGoal = NULL;
	
	m_manipulation = NULL;
  
  m_nbOrientations = 100;
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
    
    m_objGoto.resize(6);
    
//    m_objGoto[0] = 4.23;
//    m_objGoto[1] = -2.22;
//    m_objGoto[2] = 1.00;
    
    m_objGoto[0] = P3D_HUGE;
    m_objGoto[1] = P3D_HUGE;
    m_objGoto[2] = P3D_HUGE;
    
    m_objGoto[3] = P3D_HUGE;
    m_objGoto[4] = P3D_HUGE;
    m_objGoto[5] = P3D_HUGE;
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
//! returns a successRate between 0 and 1
bool ManipulationTestFunctions::manipTestGraspingWithDifferentObjectOrientations(bool rotate_only_around_z,double& successRate)
{
  MANIPULATION_TASK_MESSAGE status;
  
  bool result =false;
  int n;
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
  for(int i=1; i<=m_nbOrientations; ++i)
  {
    printf("****************test %d/%d************************\n",i,m_nbOrientations);
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
    p3d_set_collision_tolerance_inhibition(object, TRUE);
    ManipulationData data(m_Robot);
    if ( m_manipulation->findArmGraspsConfigs(0, object, data) == MANIPULATION_TASK_OK) 
    {
      n++;
      cout << "!!! OK =-) !!!" << endl;
      p3d_set_and_update_this_robot_conf( m_manipulation->robot(), data.getApproachFreeConfig() );
    }
    else {
      p3d_set_and_update_this_robot_conf(m_manipulation->robot(), m_qInit);
    }
    g3d_draw_allwin_active();
    p3d_set_collision_tolerance_inhibition(object, FALSE);
  }  
  
  successRate = ((double)n)/m_nbOrientations;
  
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("%s: %d: ARM_PICK_GOTO succeeded for %d/%d different orientations of the object.\n",__FILE__,__LINE__,n,m_nbOrientations);
  printf(" equivalent to a success rate of %f\n",successRate);
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  printf("---------------------------------------------------\n");
  
  return (n!=0 ? true : false);
}

//! Stores the success rate 
//! of the workspace point
void ManipulationTestFunctions::saveToFileEvalutedWorkspace()
{
  std::ostringstream oss;
	oss << "workspace" << ".csv";
  
  std::ofstream s;
  const char *res = oss.str().c_str();
	s.open(res);
  
  s << "success" << ",";
  s << "X" << ",";
	s << "Y" << ",";
	s << "Z";
  s << endl;
  
  for ( unsigned int i=0;i<m_workspacePoints.size();i++ ) 
  {
    s << m_workspacePoints[i].first << "," ;
    s << m_workspacePoints[i].second[0] << "," ;
    s << m_workspacePoints[i].second[1] << "," ;
    s << m_workspacePoints[i].second[2];
    s << endl;
  }
  
  s.close();
}

bool ManipulationTestFunctions::readWorkspaceFromFile(std::string fileName)
{
  std::ifstream file;
  std::string line, tmp;

  m_workspacePoints.clear();
  
  file.open (fileName.c_str());
  
  if (file.is_open())
  {
    while ( file.good())
    {
      std::getline (file, line);
      std::stringstream iss(line);
      std::istringstream sstmp;
      std::string tmp;
      
      double successRate;
      vector<double> pos(3);
      
      std::getline(iss, tmp, ',');
      sstmp.str(tmp);
      successRate = atof(tmp.c_str());
      
      std::getline(iss, tmp, ',');
      pos[0] = atof(tmp.c_str());
      std::getline(iss, tmp, ',');
      pos[1] = atof(tmp.c_str());
      std::getline(iss, tmp, ',');
      pos[2] = atof(tmp.c_str());
      
      m_workspacePoints.push_back( make_pair( successRate , pos ) );
    }
  }
  else
  {
    std::cout << "Unable to open file " << fileName << std::endl;
    return false;
  }
  
  file.close();
  return true;
}

//! Stores the success rate 
//! of the workspace point
void ManipulationTestFunctions::drawEvalutedWorkspace()
{
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
  double colorvector[4];
	
  colorvector[0] = 0.0;       //red
  colorvector[1] = 0.0;       //green
  colorvector[2] = 0.0;       //blue
  colorvector[3] = 0.01;      //transparency
  
  for ( unsigned int i=0;i<m_workspacePoints.size();i++ ) 
  {
    double x,y,z;
    
    double successRate = m_workspacePoints[i].first;
    
    GroundColorMixGreenToRed(colorvector,1-successRate);
    
    x = m_workspacePoints[i].second[0];
    y = m_workspacePoints[i].second[1];
    z = m_workspacePoints[i].second[2];
    
    //if(successRate == 0.0)
    {
      colorvector[3] = successRate+0.2;
    }
       
    glColor4dv(colorvector);
    //g3d_set_color(Any,colorvector);
    
    g3d_draw_solid_sphere(x,y,z,0.025,10);
    //g3d_drawSphere(x,y,z,0.025);
  }
  
  glDisable(GL_BLEND);
}

//! Stores the success rate of the workspace point
bool ManipulationTestFunctions::evaluateWorkspace()
{
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*) m_OBJECT_NAME.c_str());
  p3d_rob* plate= (p3d_rob*) p3d_get_robot_by_name((char*) "PLATE");
  if(object==NULL)
  { 
    printf("%s: %d: there is no robot named \"%s\".\n",__FILE__,__LINE__,m_OBJECT_NAME.c_str());
    return false;
  }
  
  m_workspacePoints.clear();
  
  double successRate;
  double x, y, z, rx, ry, rz;
  
  p3d_get_freeflyer_pose2(object, &x, &y, &z, &rx, &ry, &rz);
  
  double xref = x;
  double yref = y;
  double zref = z;
  
  m_nbOrientations = 5;
  
  const double SizeInXPos = 1.4; // Taille du decallage selon X
  const double SizeInYPos = 0.9; // Taille du decallage selon Y
  const double SizeInZPos = 1.5; // Taille du decallage selon Z
  const double SizeInXNeg = -0.9; // Taille du decallage selon X
  const double SizeInYNeg = -1.4 ; // Taille du decallage selon Y
  const double SizeInZNeg = 0; // Taille du decallage selon Z

  for (double dz=SizeInXNeg; dz<=SizeInXPos; dz=dz+0.1){
    for (double dx=SizeInXNeg; dx<=SizeInXPos; dx=dx+0.1)
    {
      for (double dy=SizeInYNeg; dy<=SizeInYPos; dy=dy+0.1)
      {
        vector<double> pos(3);

        x = xref + dx;
        y = yref + dy;
        z = zref + dz;
        
        p3d_set_freeflyer_pose2(object, x, y, z, rx, ry, rz);
        p3d_set_freeflyer_pose2(plate, x, y, z, 0, 0, 0);
        manipTestGraspingWithDifferentObjectOrientations(true,successRate);

        pos[0] = x;
        pos[1] = y;
        pos[2] = z;

        m_workspacePoints.push_back( make_pair( successRate , pos ) );
        saveToFileEvalutedWorkspace();
      }
    }
  }
  return true;
}

static int 
drawtraj_fct(p3d_rob* robot, p3d_localpath* curLp)
{
  g3d_draw_allwin_active();
  return true;
}

bool ManipulationTestFunctions::computeTrajectories()
{
  m_nbOrientations = 4;
  bool const rotate_only_around_z = true;
  
  p3d_rob* object= (p3d_rob*) p3d_get_robot_by_name((char*) m_OBJECT_NAME.c_str());
  p3d_rob* plate= (p3d_rob*) p3d_get_robot_by_name((char*) "IKEA_SHELF");
  if(object==NULL || plate == NULL)
  { 
    printf("%s: %d: there is no robot named \"%s\".\n",__FILE__,__LINE__,m_OBJECT_NAME.c_str());
    return false;
  }
  
  p3d_rob* shelf= (p3d_rob*) p3d_get_robot_by_name((char*) "SHELF");
  p3d_set_freeflyer_pose2(shelf, 0, 0, 0, 0, 0, 0);
  
  initManipulationGenom() ;
  
  vector< pair< vector<double> , vector<double> > > objectAndPlate;
  objectAndPlate.clear();
  
  vector<double> posOject(3);
  vector<double> posPlate(3);
  
  posPlate[0] = 4.35; posPlate[1] = -2.80; posPlate[2] = 0.00; 
  posOject[0] = 4.35; posOject[1] = -2.80; posOject[2] = 0.80;
  
  objectAndPlate.push_back( make_pair(posOject,posPlate) );
  
  posPlate[0] = 4.50; posPlate[1] = -2.80; posPlate[2] = 0.00; 
  posOject[0] = 4.50; posOject[1] = -2.80; posOject[2] = 0.80;
  
  objectAndPlate.push_back( make_pair(posOject,posPlate) );
  
  posPlate[0] = 4.35; posPlate[1] = -2.00; posPlate[2] = 0.00; 
  posOject[0] = 4.35; posOject[1] = -2.00; posOject[2] = 0.80;
  
  objectAndPlate.push_back( make_pair(posOject,posPlate) );
  
  posPlate[0] = 4.50; posPlate[1] = -2.00; posPlate[2] = 0.00; 
  posOject[0] = 4.50; posOject[1] = -2.00; posOject[2] = 0.80;
  
  objectAndPlate.push_back( make_pair(posOject,posPlate) );

  
  for (unsigned int i=0; i<objectAndPlate.size(); i++)
  {
    vector<double> pos;
    pos = objectAndPlate[i].first;
    
    double x = pos[0];
    double y = pos[1];
    double z = pos[2];
    
    pos = objectAndPlate[i].second;
    
    for (unsigned int j=0; j<m_nbOrientations; j++) 
    {
      if(rotate_only_around_z)
      {  p3d_set_freeflyer_pose2(object, x, y, z, 0, 0, p3d_random(-M_PI,M_PI));  }
      else
      {  p3d_set_freeflyer_pose2(object, x, y, z, p3d_random(-M_PI,M_PI), p3d_random(-M_PI,M_PI), p3d_random(-M_PI,M_PI));  } 
    
      
      p3d_set_freeflyer_pose2(plate,  pos[0], pos[1], pos[2], 0, 0, 0);
      p3d_set_and_update_this_robot_conf(m_Robot,m_qInit);
      
      g3d_draw_allwin_active();
      
      if( manipTest(ARM_PICK_GOTO) )
      {
        g3d_show_tcur_rob(m_Robot,drawtraj_fct); 
      }
    }
  }
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
    double succesRate=0;
    return this->manipTestGraspingWithDifferentObjectOrientations(false,succesRate);
  }
  
  if (id == 7) 
  {
    double succesRate=0;
    return this->manipTestGraspingWithDifferentObjectOrientations(true,succesRate);
  }
  
  if (id == 8) 
  {
//     global_manipPlanTest = this;
    return this->evaluateWorkspace();
  }
  if (id == 9) 
  {
    //     global_manipPlanTest = this;
    return this->computeTrajectories();
  }
  else {
    cout << "Test : " << id << " not defined " << endl;
  }
  
	
  return false;
}
