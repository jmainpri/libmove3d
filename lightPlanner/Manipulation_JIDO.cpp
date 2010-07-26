#include "Manipulation.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "GraspPlanning-pkg.h"
#include "gbM/Proto_gbModeles.h"
#include "lightPlanner.h"
#include "lightPlannerApi.h"
#include <gp_grasp_generation_proto.h>
#include <gp_grasp_generation_proto.h>
#ifdef DPG
#include "p3d_chanEnv_proto.h"
#endif
#include "planner_cxx/plannerFunctions.hpp"
#include <list>
#include <string>
#include <iostream>


#define p3d_polyhedre poly_polyhedre
#include <graspPlanning.h>

using namespace std;

void undefinedRobotMessage() {
  printf("The robot has not been defined in Manipulation_JIDO. Recreate an instance of Manipulation_JIDO.\n");
}

void undefinedObjectMessage() {
  printf("The object has not been defined in Manipulation_JIDO. Set it with setObjectToManipulate().\n");
}

void undefinedSupportMessage() {
  printf("The support has not been defined in Manipulation_JIDO. Set it with setSupport().\n");
}

void undefinedCameraMessage() {
  printf("The joint of the pan/tilt unit has not been defined in Manipulation_JIDO. Set it with setCameraJnt().\n");
}

Manipulation_JIDO::Manipulation_JIDO(p3d_rob *robotPt, gpHand_type handType)//: // _capture(false)
{
  if(robotPt->nbCcCntrts != 1){
    cout << "Error: the number of arm  != 1" << endl;
    return;
  }
  _hand_robotPt= NULL;
  _handProp.initialize(handType);

  switch (handType) {
    case GP_GRIPPER:
      _hand_robotPt= p3d_get_robot_by_name((char*)GP_GRIPPER_ROBOT_NAME);
      break;
    default:
      
      break;
  }
  
  if(_hand_robotPt==NULL)
  {
    printf("%s: %d: Manipulation_JIDO::Manipulation_JIDO: a robot \"%s\" is required.\n",__FILE__,__LINE__,(char*)GP_GRIPPER_ROBOT_NAME);
    return;
  }

   _robotPt = robotPt;
   _cameraJnt= NULL;
   _cameraFOV= 60;
   _cameraImageWidth= 200;
   _cameraImageHeight= 200;

   _liftUpDistance= 0.15;
   _nbGraspsToTestForPickGoto= 1;
   _placementTranslationStep= 0.2; 
   _placementNbOrientations= 8;
   _placementOffset= -0.005;

  for(int i=0; i<6; i++){
    _QCUR[i]= 0.0;
    _QGOAL[i]= 0.0;
    _XCUR[i]= 0.0;
    _XGOAL[i]= 0.0;
  }

  double deg2rad= (M_PI/180.0);
  _qrest[0]= -64.288472*deg2rad;
  _qrest[1]= 27.829957*deg2rad;
  _qrest[2]= 112.040164*deg2rad;
  _qrest[3]= 183.719232*deg2rad;
  _qrest[4]= -45.732049*deg2rad;
  _qrest[5]= 135.484701*deg2rad;

  _object           = NULL;
  _support          = NULL;
  _human            = NULL;
  _capture          = false;
  _cartesian        = false;
  _objectGrabed     = false;
  displayGrasps     = false;
  displayPlacements = false;
}

Manipulation_JIDO::~Manipulation_JIDO(){
 
}

void Manipulation_JIDO::clear(){
  _robotPt = NULL;
}

//! Pushes a config at the back of _configTraj.
//! The input config is copied.
//! \param config the config to copy and store
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::addConfigTraj(configPt config) {
  if(_robotPt==NULL)  {
    printf("%s: %d: Manipulation_JIDO::addConfigTraj().\n",__FILE__,__LINE__);
    undefinedRobotMessage();
    return 1;
  }
  if(config==NULL)  {
    printf("%s: %d: Manipulation_JIDO::addConfigTraj(): input configuration is NULL\n",__FILE__,__LINE__);
    return 1;
  }

  configPt new_conf= NULL;

  new_conf= p3d_alloc_config(_robotPt);
  p3d_copy_config_into(_robotPt, config, &new_conf);

  _configTraj.push_back(new_conf);

  return 0;
}


//! Clears the _configTraj vector and sets free all the allocated data..
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::clearConfigTraj() {
  if(_robotPt==NULL)  {
    printf("%s: %d: Manipulation_JIDO::clearConfigTraj().\n",__FILE__,__LINE__);
    undefinedRobotMessage();
    return 1;
  }

  unsigned int i;

  for(i=0; i<_configTraj.size(); ++i)  {
     p3d_destroy_config(_robotPt, _configTraj[i]);
  }
  _configTraj.clear();

  return 0;
}

//! Copies the content of configTraj to the content of the robot Xform window..
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::copyConfigTrajToFORM() {
  if(_robotPt==NULL)  {
    printf("%s: %d: Manipulation_JIDO::copyConfigTrajToFORM().\n",__FILE__,__LINE__);
    undefinedRobotMessage();
    return 1;
  }

  unsigned int i;
  char name[128];

  for(i=0; i<_configTraj.size(); ++i) {
     sprintf(name, "configTraj_%i", i);
     p3d_set_new_robot_config(name, _configTraj[i], _robotPt->ikSol, _robotPt->confcur);
     _robotPt->confcur = _robotPt->conf[0];
     FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  }

  return 0;
}

//! Destroys the trajectories currently stored in the robot.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::destroyTrajectories() {
  if(_robotPt==NULL)
  {
    printf("%s: %d: Manipulation_JIDO::destroyTrajectories().\n",__FILE__,__LINE__);
    undefinedRobotMessage();
    return 1;
  }

  while(_robotPt->nt!=0)
  {   p3d_destroy_traj(_robotPt, _robotPt->t[0]);  }

  FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));

  return 0;
}

//! Sets the robot's arm configuration with the given values (in radians).
//! NB: The respect of joint bounds is verified.
//! \param q1 value of joint #1
//! \param q2 value of joint #2
//! \param q3 value of joint #3
//! \param q4 value of joint #4
//! \param q5 value of joint #5
//! \param q6 value of joint #6
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setArmQ(double q1, double q2, double q3, double q4, double q5, double q6){
  if(_robotPt==NULL)
  {
    printf("%s: %d: Manipulation_JIDO::setArmQ().\n",__FILE__,__LINE__);
    undefinedRobotMessage();
    return 1;
  }

  int isValid, verbose= 1; //set verbose to 1/0 to enable/disable printf
  int result;
  double qmin, qmax;
  p3d_jnt *armJoint= NULL;
  configPt q= NULL;

  q= p3d_alloc_config(_robotPt);
  p3d_get_robot_config_into(_robotPt, &q);

  ////////////////////////q1////////////////////////////
  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT1);
  if(armJoint==NULL)
    {
      p3d_destroy_config(_robotPt, q);
      return 1;
    }
  isValid= 1;
  qmin= armJoint->dof_data[0].vmin;
  qmax= armJoint->dof_data[0].vmax;
  if(q1 > qmax)
    {
      q1-= 2*M_PI;
      if( (q1 < qmin) || (q1 > qmax) )
	{  isValid= 0; }
    }
  if(q1 < qmin)
    {
      q1+= 2*M_PI;
      if( (q1 < qmin) || (q1 > qmax) )
	{  isValid= 0; }
    }
  if(isValid)
    { q[armJoint->index_dof]=  q1;   }
  else
    {
      if(verbose)
	{
	  printf("%s: %d: Manipulation_JIDO::setArmQ(): q1 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q1,qmin,qmax);
	}
      p3d_destroy_config(_robotPt, q);
      return 1;
    }
  /////////////////////////////////////////////////////


    ////////////////////////q2////////////////////////////
    armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT2);
    if(armJoint==NULL)
      {
	p3d_destroy_config(_robotPt, q);
	return 1;
      }
    isValid= 1;
    qmin= armJoint->dof_data[0].vmin;
    qmax= armJoint->dof_data[0].vmax;
    if(q2 > qmax)
      {
	q2-= 2*M_PI;
	if( (q2 < qmin) || (q2 > qmax) )
	  {  isValid= 0; }
      }
    if(q2 < qmin)
      {
	q2+= 2*M_PI;
	if( (q2 < qmin) || (q2 > qmax) )
	  {  isValid= 0; }        }
    if(isValid)
      { q[armJoint->index_dof]=  q2;   }
    else
      {
	if(verbose)
	  {
	    printf("%s: %d: Manipulation_JIDO::setArmQ(): q2 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q2,qmin,qmax);
	  }
	p3d_destroy_config(_robotPt, q);
	return 1;
      }
    /////////////////////////////////////////////////////


      ////////////////////////q3////////////////////////////
      armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT3);
      if(armJoint==NULL)
	{
	  p3d_destroy_config(_robotPt, q);
	  return 1;
	}
      isValid= 1;
      qmin= armJoint->dof_data[0].vmin;
      qmax= armJoint->dof_data[0].vmax;
      if(q3 > qmax)
	{
	  q3-= 2*M_PI;
	  if( (q3 < qmin) || (q3 > qmax) )
	    {  isValid= 0; }
	}
      if(q3 < qmin)
	{
	  q3+= 2*M_PI;
	  if( (q3 < qmin) || (q3 > qmax) )
	    {  isValid= 0; }
	}
      if(isValid)
	{ q[armJoint->index_dof]=  q3;   }
      else
	{
	  if(verbose)
	    {
	      printf("%s: %d: Manipulation_JIDO::setArmQ(): q3 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q3,qmin,qmax);
	    }
	  p3d_destroy_config(_robotPt, q);
	  return 1;
	}
      /////////////////////////////////////////////////////


	////////////////////////q4////////////////////////////
	armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT4);
	if(armJoint==NULL)
	  {
	    p3d_destroy_config(_robotPt, q);
	    return 1;
	  }
	isValid= 1;
	qmin= armJoint->dof_data[0].vmin;
	qmax= armJoint->dof_data[0].vmax;
	if(q4 > qmax)
	  {
	    q4-= 2*M_PI;
	    if( (q4 < qmin) || (q4 > qmax) )
	      {  isValid= 0; }
	  }
	if(q4 < qmin)
	  {
	    q4+= 2*M_PI;
	    if( (q4 < qmin) || (q4 > qmax) )
	      {  isValid= 0; }
	  }
	if(isValid)
	  { q[armJoint->index_dof]=  q4;   }
	else
	  {
	    if(verbose)
	      {
		printf("%s: %d: Manipulation_JIDO::setArmQ(): q4 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q4,qmin,qmax);
	      }
	    p3d_destroy_config(_robotPt, q);
	    return 1;
	  }
	/////////////////////////////////////////////////////


	  ////////////////////////q5////////////////////////////
	  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT5);
	  if(armJoint==NULL)
	    {
	      p3d_destroy_config(_robotPt, q);
	      return 1;
	    }
	  isValid= 1;
	  qmin= armJoint->dof_data[0].vmin;
	  qmax= armJoint->dof_data[0].vmax;
	  if(q5 > qmax)
	    {
	      q5-= 2*M_PI;
	      if( (q5 < qmin) || (q5 > qmax) )
		{  isValid= 0; }
	    }
	  if(q5 < qmin)
	    {
	      q5+= 2*M_PI;
	      if( (q5 < qmin) || (q5 > qmax) )
		{  isValid= 0; }
	    }
	  if(isValid)
	    { q[armJoint->index_dof]=  q5;   }
	  else
	    {
	      if(verbose)
		{
		  printf("%s: %d: Manipulation_JIDO::setArmQ(): q5 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q5,qmin,qmax);
		}
	      p3d_destroy_config(_robotPt, q);
	      return 1;
	    }
	  /////////////////////////////////////////////////////


	    ////////////////////////q6////////////////////////////
	    armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_WRISTJOINT);
	    if(armJoint==NULL)
	      {
		p3d_destroy_config(_robotPt, q);
		return 1;
	      }
	    isValid= 1;
	    qmin= armJoint->dof_data[0].vmin;
	    qmax= armJoint->dof_data[0].vmax;
	    if(q6 > qmax)
	      {
		q6-= 2*M_PI;
		if( (q6 < qmin) || (q6 > qmax) )
		  {  isValid= 0; }
	      }
	    if(q6 < qmin)
	      {
		q6+= 2*M_PI;
		if( (q6 < qmin) || (q6 > qmax) )
		  {  isValid= 0; }
	      }
	    if(isValid)
	      { q[armJoint->index_dof]=  q6;   }
	    else
	      {
		if(verbose)
		  {
		    printf("%s: %d: Manipulation_JIDO::setArmQ(): q6 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q6,qmin,qmax);
		  }
		p3d_destroy_config(_robotPt, q);
		return 1;
	      }

	    deactivateCcCntrts(_robotPt, -1);
	    result= p3d_set_and_update_this_robot_conf(_robotPt, q);
	    p3d_get_robot_config_into(_robotPt, &_robotPt->ROBOT_POS);

	    p3d_destroy_config(_robotPt, q);

	    return !result;
}

//! Gets the robot's arm configuration (in radians).
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::getArmQ(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6){
  if(_robotPt==NULL)
    {
      printf("%s: %d: Manipulation_JIDO::getArmQ().\n",__FILE__,__LINE__);
      undefinedRobotMessage();
      return 1;
    }

  p3d_jnt *armJoint= NULL;

  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT1);
  if(armJoint==NULL)
    {  return 0; }
  *q1= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT2);
  if(armJoint==NULL)
    {  return 0; }
  *q2= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT3);
  if(armJoint==NULL)
    {  return 0; }
  *q3= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT4);
  if(armJoint==NULL)
    {  return 0; }
  *q4= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_ARMJOINT5);
  if(armJoint==NULL)
    {  return 0; }
  *q5= armJoint->dof_data[0].v;

  armJoint= p3d_get_robot_jnt_by_name(_robotPt, (char*)GP_WRISTJOINT);
  if(armJoint==NULL)
    {  return 0; }
  *q6= armJoint->dof_data[0].v;

  return 0;
}

//! Sets the robot's end effector pose with the given values (in meters and radians).
//! \param x position of end effector along X axis (in world coordinates)
//! \param y position of end effector along Y axis (in world coordinates)
//! \param z position of end effector along Z axis (in world coordinates)
//! \param rx first Euler angle (with Move3D convention) of end effector (in world coordinates)
//! \param ry second Euler angle (with Move3D convention) of end effector (in world coordinates)
//! \param rz third Euler angle (with Move3D convention) of end effector (in world coordinates)
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setArmX(double x, double y, double z, double rx, double ry, double rz){
  int result;
  configPt qcur= NULL;

  if(_robotPt==NULL)
    {
      printf("%s: %d: Manipulation_JIDO::setArmX().\n",__FILE__,__LINE__);
      undefinedRobotMessage();
      return 1;
    }

  // store the current configuration:
  qcur= p3d_alloc_config(_robotPt);
  p3d_get_robot_config_into(_robotPt, &qcur);  

  result= p3d_set_virtual_object_pose2(_robotPt, x, y, z, rx, ry, rz);
  deactivateCcCntrts(_robotPt, -1);
  if(result==TRUE) {
    p3d_destroy_config(_robotPt, qcur);
    return 0;
  }
  else {
//     printf("%s: %d: Manipulation_JIDO::setArmX(): the input pose is not reachable (no IK solution).\n",__FILE__,__LINE__);
    p3d_set_and_update_this_robot_conf(_robotPt, qcur); //restore the initial configuration
    p3d_destroy_config(_robotPt, qcur);
    return 1;
  }
}

//! Sets the robot's end effector pose with the given position values (in meters).
//! The rotation part of the end-effector is randomly chosen so that it yields no collision.
//! \param x position of end effector along X axis (in world coordinates)
//! \param y position of end effector along Y axis (in world coordinates)
//! \param z position of end effector along Z axis (in world coordinates)
//! \param nbTries number of times the function will try to generate a collision-free configuration
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setArmX(double x, double y, double z, unsigned int nbTries){
  int result;

  if(_robotPt==NULL)
    {
      printf("%s: %d: Manipulation_JIDO::setArmX().\n",__FILE__,__LINE__);
      undefinedRobotMessage();
      return 1;
    }

  unsigned int i;
  double rx, ry, rz;
  configPt qcur= NULL;


  // store the current configuration:
  qcur= p3d_alloc_config(_robotPt);
  p3d_get_robot_config_into(_robotPt, &qcur);  

  for(i=0; i<nbTries; ++i) {
     rx= p3d_random(-M_PI, M_PI);
     ry= p3d_random(-M_PI, M_PI);
     rz= p3d_random(-M_PI, M_PI);

     result= setArmX(x, y, z, rx, ry, rz);

     if(result==0 && !p3d_col_test_robot(_robotPt, 0) ) {
       p3d_destroy_config(_robotPt, qcur);
       return 0;
     }
  }

//printf("%s: %d: Manipulation_JIDO::setArmX(): the input pose is not reachable (no IK solution).\n",__FILE__,__LINE__);
 p3d_set_and_update_this_robot_conf(_robotPt, qcur); //restore the initial configuration
 p3d_destroy_config(_robotPt, qcur);

 return 1;

}

//! Gets the robot's end effector pose with the given values (in meters and radians).
//! \param robotPt pointer to the robot
//! \param x position of end effector along X axis (in world coordinates)
//! \param y position of end effector along Y axis (in world coordinates)
//! \param z position of end effector along Z axis (in world coordinates)
//! \param rx first Euler angle (with Move3D convention) of end effector (in world coordinates)
//! \param ry second Euler angle (with Move3D convention) of end effector (in world coordinates)
//! \param rz third Euler angle (with Move3D convention) of end effector (in world coordinates)
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::getArmX(double* x, double* y, double* z, double* rx, double* ry, double* rz){
  if(_robotPt==NULL)
    {
      printf("%s: %d: anipulation_JIDO::getArmX().\n",__FILE__,__LINE__);
      undefinedRobotMessage();
      return 1;
    }
  return p3d_get_virtual_object_pose2(_robotPt, x, y, z, rx, ry, rz);
}

void Manipulation_JIDO::setArmCartesian(bool v){
  _cartesian = v;
}

bool Manipulation_JIDO::getArmCartesian() {
  return _cartesian;
}

//! Prints some info about a robot's contraints
int Manipulation_JIDO::printConstraintInfo(){
  int i=0; 
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::printConstraintInfo().\n",__FILE__,__LINE__);
    undefinedRobotMessage();
    return 1;
  }

  printf("constraint info: \n");
  printf("robot \"%s\" has %d constraints: \n",_robotPt->name,_robotPt->cntrt_manager->ncntrts);
  for(i=0; i<_robotPt->cntrt_manager->ncntrts; i++) {
    printf("%s, active= %d\n", _robotPt->cntrt_manager->cntrts[i]->namecntrt, _robotPt->cntrt_manager->cntrts[i]->active);
  }
  return 0;
}

//! Sets the pose of the freeflyer robot "object" from cooridnates given in the frame
//! of the end effector (virtual object) of robot "robotPt"
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz, configPt q){
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::setPoseWrtEndEffector().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return 1;
  }
  double x1, y1, z1, rx1, ry1, rz1;
  p3d_matrix4 T1, T2, T3;
  p3d_set_virtual_object_pose(_robotPt, T1);
  p3d_mat4PosReverseOrder(T2, x, y, z, rx, ry, rz);
  p3d_mat4Mult(T1, T2, T3);
  p3d_mat4ExtractPosReverseOrder(T3, &x1, &y1, &z1, &rx1, &ry1, &rz1);
  this->setArmX(x, y, z, rx, ry, rz);
  q = p3d_get_robot_config(_robotPt);
  return 0;
}

//! Gets the robot's ROBOT_POS configuration.
//! \return the ROBOT_POS configuration in case of success, NULL otherwise
configPt Manipulation_JIDO::robotStart(){
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::robotStart().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return NULL;
  }

  return _robotPt->ROBOT_POS;
}

//! Gets the robot's ROBOT_GOTO configuration.
//! \return the ROBOT_GOTO configuration in case of success, NULL otherwise
configPt Manipulation_JIDO::robotGoto(){
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::robotGoto().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return NULL;
  }

  return _robotPt->ROBOT_GOTO;
}

//! Gets the robot's rest configuration.
//! \return the rest configuration in case of success, NULL otherwise
configPt Manipulation_JIDO::robotRest(){
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::robotRest().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return NULL;
  }

  return _qrest;
}


//! Computes a path for a given manipulation elementary task.
MANIPULATION_TASK_MESSAGE Manipulation_JIDO::armPlanTask(MANIPULATION_TASK_TYPE_STR task, configPt qStart, configPt qGoal, char* objectName, std::vector <int> &lp, std::vector < std::vector <double> > &positions){

  configPt qi = NULL, qf = NULL;
  p3d_rob *cur_robot= NULL;
  p3d_traj *traj = NULL;
  bool success;
  int i, ntest=0;
  double gain = 0;


  // variabel for ARM_PICK_GOTO
   double pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6;
   double q1, q2, q3, q4, q5, q6;
   unsigned int itraj;
   configPt q1_conf = NULL, q2_conf = NULL, qint = NULL;

//variable for ARM_PICK_TAKE_TO_FREE
  double x, y, z, rx, ry, rz;

  cur_robot= XYZ_ENV->cur_robot;
  XYZ_ENV->cur_robot= _robotPt;



printf("*****************************************************************************\n");
  if(_cartesian == false) {
    printf("armGotoQ : Articular Mode\n");
    deactivateCcCntrts(_robotPt, -1);
    p3d_multiLocalPath_disable_all_groupToPlan(_robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(_robotPt, (char*)"jido-arm_lin", 1) ;
  } else {
    printf("armGotoQ : Cartesian Mode\n");
    qi = p3d_alloc_config(_robotPt);
    deactivateCcCntrts(_robotPt, -1);
#ifdef LIGHT_PLANNER
    shootTheObjectArroundTheBase(_robotPt,
				 _robotPt->baseJnt,
				 _robotPt->curObjectJnt,
				 2.0);
#endif

    /* Sets the linear local planner for the arm  */
    p3d_multiLocalPath_disable_all_groupToPlan(_robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(_robotPt, (char*)"jido-ob_lin", 1) ;
    p3d_copy_config_into(_robotPt, qStart, &qi);
    /* Uptdate the Virual object for inverse kinematics */
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qi);
    p3d_set_and_update_this_robot_conf(_robotPt, qi);
    p3d_destroy_config(_robotPt, qi);
    qi = p3d_get_robot_config(_robotPt);
    //     g3d_draw_allwin_active();
    p3d_copy_config_into(_robotPt, qi, &qStart);
    p3d_destroy_config(_robotPt, qi);
  }
  p3d_set_and_update_this_robot_conf(_robotPt, qStart);

  switch(task) {
    case ARM_FREE:
       clearConfigTraj();
       printf("plan for ARM_FREE task\n");
       if(_cartesian == true) {
	qf = p3d_alloc_config(_robotPt);
	p3d_copy_config_into(_robotPt, qGoal, &qf);
	p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qf);
	p3d_set_and_update_this_robot_conf(_robotPt, qf);
	p3d_destroy_config(_robotPt, qf);
	qf = p3d_get_robot_config(_robotPt);
	//     g3d_draw_allwin_active();
	p3d_copy_config_into(_robotPt, qf, &qGoal);
	p3d_destroy_config(_robotPt, qf);

	if(_robotPt->nbCcCntrts!=0) {
	  p3d_activateCntrt(_robotPt, _robotPt->ccCntrts[0]);
	}
       }

        p3d_set_and_update_this_robot_conf(_robotPt, qStart);
        gpOpen_hand(_robotPt, _handProp);
        qi = p3d_get_robot_config(_robotPt);
        addConfigTraj(qi);

        p3d_set_and_update_this_robot_conf(_robotPt, qGoal);
        gpOpen_hand(_robotPt, _handProp);
        qf = p3d_get_robot_config(_robotPt);
        addConfigTraj(qf);

        copyConfigTrajToFORM();

//	cleanRoadmap();
	cleanTraj();

	printf("il y a %d configurations\n", _configTraj.size());
        for(itraj=0; itraj< _configTraj.size()-1; itraj++) {
                q1_conf = _configTraj[itraj];
                q2_conf = _configTraj[itraj+1];

                if(computeTrajBetweenTwoConfigs(_cartesian, q1_conf, q2_conf)!=0) {
		  printf("ERROR genomFindGraspConfigAndComputeTraj on traj %d",itraj);
		  return MANIPULATION_TASK_NO_TRAJ_FOUND;
		}
              
	        cleanRoadmap();
	}


	GP_ConcateneAllTrajectories(_robotPt);
        _robotPt->tcur= _robotPt->t[0];
	p3d_copy_config_into(_robotPt, qi, &qStart);
      break;
    case  ARM_PICK_GOTO:
      for(std::list<gpGrasp>::iterator igrasp=_graspList.begin(); igrasp!=_graspList.end(); ++igrasp)
      {  igrasp->tested= false;   }

      for(i=0; i<_nbGraspsToTestForPickGoto; ++i)
      {
        success= true;
        printf("plan for ARM_PICK_GOTO task\n");
        destroyTrajectories();

        clearConfigTraj();

        printf("il y a %d configurations\n", _configTraj.size());

        p3d_set_and_update_this_robot_conf(_robotPt, qStart);
        gpOpen_hand(_robotPt, _handProp);
        qi = p3d_get_robot_config(_robotPt);
        addConfigTraj(qi);

        if(findPregraspAndGraspConfiguration(_liftUpDistance,&pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6) != 0) {
	  printf("no solution to grasp\n");
          success= false;
	  return MANIPULATION_TASK_NO_GRASP;
	}
        // mark the unreachable grasp as tested for path planning:
        markGraspAsTested(_graspID);
	//p3d_activateCntrt(_robotPt, _robotPt->fkCntrts[0]);
        p3d_set_and_update_this_robot_conf(_robotPt, qi);
        gpOpen_hand(_robotPt, _handProp);
        setArmQ(pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6);
        qint = p3d_get_robot_config(_robotPt);
        addConfigTraj(qint);

        p3d_set_and_update_this_robot_conf(_robotPt, qi);
        gpOpen_hand(_robotPt, _handProp);
        setArmQ(q1, q2, q3, q4, q5, q6);
        qf = p3d_get_robot_config(_robotPt);
	p3d_get_robot_config_into(_robotPt, &qGoal);
        addConfigTraj(qf);

        copyConfigTrajToFORM();

	cleanRoadmap();
	cleanTraj();
	
	printf("il y a %d configurations\n", _configTraj.size());

        for(itraj=0; itraj< _configTraj.size()-1; itraj++) {
                q1_conf = _configTraj[itraj];
                q2_conf = _configTraj[itraj+1];

#ifdef FK_CNTRT
               if(_cartesian==true && _robotPt->nbFkCntrts!=0) {
                  p3d_desactivateCntrt(_robotPt, _robotPt->fkCntrts[0]);
               }
#endif

                if(computeTrajBetweenTwoConfigs(_cartesian, q1_conf, q2_conf)!=0) {
		  printf("ERROR genomFindGraspConfigAndComputeTraj on traj %d",itraj);
                  success= false;
                  cleanRoadmap();
                  break;
		}
	        cleanRoadmap();
	}
        if(success==true)
        {  break;  }

      }

      if(success==false)
      {  return MANIPULATION_TASK_NO_TRAJ_FOUND;  }

	GP_ConcateneAllTrajectories(_robotPt);

        _robotPt->tcur= _robotPt->t[0];
	p3d_copy_config_into(_robotPt, qi, &qStart);
      
    break;
    case  ARM_PICK_TAKE_TO_FREE:
        printf("plan for ARM_TAKE_TO_FREE task\n");
	  
        destroyTrajectories();
        clearConfigTraj();
        printf("il y a %d configurations\n", _configTraj.size());

	qi = p3d_copy_config(_robotPt, qStart);
        p3d_set_and_update_this_robot_conf(_robotPt, qi);
        gpOpen_hand(_robotPt, _handProp);
        qi = p3d_get_robot_config(_robotPt);
        addConfigTraj(qi);

        getArmX(&x, &y, &z, &rx, &ry, &rz);

	setArmX(x, y, z+0.1, rx, ry, rz);
 
        gpOpen_hand(_robotPt, _handProp);
        qint = p3d_get_robot_config(_robotPt);
        addConfigTraj(qint);

	
        qf = p3d_alloc_config(_robotPt);
        p3d_copy_config_into(_robotPt, qGoal, &qf);
        p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qf);
        gpOpen_hand(_robotPt, _handProp);
        p3d_set_and_update_this_robot_conf(_robotPt, qf);
        p3d_destroy_config(_robotPt, qf);
        qf = p3d_get_robot_config(_robotPt);
        addConfigTraj(qf);

        if(p3d_equal_config(_robotPt, qStart, qGoal)) {
	    printf("genomArmGotoQ: Start and goal configurations are the same.\n");
	    return MANIPULATION_TASK_INVALID_QSTART;
        }

        p3d_set_and_update_this_robot_conf(_robotPt, qi);

        copyConfigTrajToFORM();

	printf("il y a %d configurations\n", _configTraj.size());

        for(itraj=0; itraj< _configTraj.size()-1; itraj++) {
                q1_conf = _configTraj[itraj];
                q2_conf = _configTraj[itraj+1];

                // for lifting phase:
		if(itraj == 0){
                  p3d_col_deactivate_robot_robot(_object, _support);
		} else {
                  p3d_col_activate_robot_robot(_object, _support);
	        }
                //if(_cartesian==true && _robotPt->nbFkCntrts!=0) {
                //   p3d_desactivateCntrt(_robotPt, _robotPt->fkCntrts[0]);
                //}
                if(this->computeTrajBetweenTwoConfigs(_cartesian, q1_conf, q2_conf)!=0) {
		  printf("ERROR genomFindGraspConfigAndComputeTraj on traj %d",itraj);
		  return MANIPULATION_TASK_NO_TRAJ_FOUND;
	       }
	       cleanRoadmap();
        }

	GP_ConcateneAllTrajectories(_robotPt);
        _robotPt->tcur= _robotPt->t[0];
	p3d_copy_config_into(_robotPt, qi, &qStart);
      break;
    case ARM_PLACE_FROM_FREE:
        printf("plan for ARM_PLACE_FROM_FREE task\n");
        if(!_robotPt->isCarryingObject) {
	  printf("%s: %d: armPlanTask(): call grabObject() before calling ARM_PLACE_FROM_FREE.\n",__FILE__,__LINE__);
	  return MANIPULATION_TASK_ERROR_UNKNOWN;
        }

        clearConfigTraj();
        // destroy all the current trajectories of the robot:
        destroyTrajectories();

        // set the robot's configuration to qStart and save it:
        p3d_set_and_update_this_robot_conf(_robotPt, qStart);
        qi = p3d_get_robot_config(_robotPt);
        addConfigTraj(qi);

        // compute the possible placements of the object on the support
        if(findPlacementConfigurations(_liftUpDistance, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6) != 0) {
	  printf("nowhere to place the object\n");
	  return MANIPULATION_TASK_NO_PLACE;
	}

        // set the pre-placement configuration of the arm:
        p3d_set_and_update_this_robot_conf(_robotPt, qi);
        gpOpen_hand(_robotPt, _handProp);
        setArmQ(pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6);
        qint = p3d_get_robot_config(_robotPt);
        addConfigTraj(qint);

        // set the final placement configuration of the arm:
        p3d_set_and_update_this_robot_conf(_robotPt, qi);
        gpOpen_hand(_robotPt, _handProp);
        setArmQ(q1, q2, q3, q4, q5, q6);
        getArmX(&x, &y, &z, &rx, &ry, &rz);
	setArmX(x, y, z + _placementOffset, rx, ry, rz);
        qf = p3d_get_robot_config(_robotPt);
	p3d_get_robot_config_into(_robotPt, &qGoal);
        addConfigTraj(qf);

        copyConfigTrajToFORM();

	cleanRoadmap();
	cleanTraj();
	
	printf("il y a %d configurations\n", _configTraj.size());

        for(itraj=0; itraj< _configTraj.size()-1; itraj++) {
                q1_conf = _configTraj[itraj];
                q2_conf = _configTraj[itraj+1];

                // for the final placing motion on the support:
		if(itraj==_configTraj.size()-2) {
                  p3d_col_deactivate_robot_robot(_object, _support);
		} else {
                  p3d_col_activate_robot_robot(_object, _support);
	        }
                //if(_cartesian==true && _robotPt->nbFkCntrts!=0) {
                //   p3d_desactivateCntrt(_robotPt, _robotPt->fkCntrts[0]);
                //}
                if(computeTrajBetweenTwoConfigs(_cartesian, q1_conf, q2_conf)!=0) {
		  printf("ERROR genomFindGraspConfigAndComputeTraj on traj %d",itraj);
		  return MANIPULATION_TASK_NO_TRAJ_FOUND;
		}
	        cleanRoadmap();
	}


	GP_ConcateneAllTrajectories(_robotPt);
        _robotPt->tcur= _robotPt->t[0];
	p3d_copy_config_into(_robotPt, qi, &qStart);
    break;
    case  ARM_PICK_TAKE_TO_PLACE:
        printf("plan for ARM_PICK_TAKE_TO_PLACE task\n");
        if(!_robotPt->isCarryingObject) {
	  printf("%s: %d: armPlanTask(): call grabObject() before calling ARM_PICK_TAKE_TO_PLACE.\n",__FILE__,__LINE__);
	  return MANIPULATION_TASK_ERROR_UNKNOWN;
        }

        clearConfigTraj();
        // destroy all the current trajectories of the robot:
        destroyTrajectories();

        // set the robot's configuration to qStart and save it:
        p3d_set_and_update_this_robot_conf(_robotPt, qStart);
        qi = p3d_get_robot_config(_robotPt);
        addConfigTraj(qi);

       //introduce an in-between configuration just a little higher than the initial one:
        p3d_set_and_update_this_robot_conf(_robotPt, qStart);
        getArmX(&x, &y, &z, &rx, &ry, &rz);
	setArmX(x, y, z + _liftUpDistance, rx, ry, rz);
        qint = p3d_get_robot_config(_robotPt);
        addConfigTraj(qint);

       // compute the possible placements of the object on the support
        if(findPlacementConfigurations(_liftUpDistance, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6) != 0) {
	  printf("nowhere to place the object\n");
	  return MANIPULATION_TASK_NO_PLACE;
	}

        // set the pre-placement configuration of the arm:
        p3d_set_and_update_this_robot_conf(_robotPt, qi);
        gpOpen_hand(_robotPt, _handProp);
        setArmQ(pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6);
        qint = p3d_get_robot_config(_robotPt);
        addConfigTraj(qint);

        // set the final placement configuration of the arm:
        p3d_set_and_update_this_robot_conf(_robotPt, qi);
        gpOpen_hand(_robotPt, _handProp);
        setArmQ(q1, q2, q3, q4, q5, q6);
        getArmX(&x, &y, &z, &rx, &ry, &rz);
	setArmX(x, y, z + _placementOffset, rx, ry, rz);
        qf = p3d_get_robot_config(_robotPt);
	p3d_get_robot_config_into(_robotPt, &qGoal);
        addConfigTraj(qf);

        copyConfigTrajToFORM();

	cleanRoadmap();
	cleanTraj();

	printf("il y a %ld configurations\n", _configTraj.size());

        for(itraj=0; itraj < _configTraj.size()-1; itraj++) {
                q1_conf = _configTraj[itraj];
                q2_conf = _configTraj[itraj+1];

                // for lifting and placing the object:
		if(itraj == 0 || itraj==_configTraj.size()-2) {
                  p3d_col_deactivate_robot_robot(_object, _support);
		} else {
                  p3d_col_activate_robot_robot(_object, _support);
	        }
                //if(_cartesian==true && _robotPt->nbFkCntrts!=0) {
                //   p3d_desactivateCntrt(_robotPt, _robotPt->fkCntrts[0]);
                //}
                if(computeTrajBetweenTwoConfigs(_cartesian, q1_conf, q2_conf)!=0) {
		  printf("ERROR genomFindGraspConfigAndComputeTraj on traj %d",itraj);
		  return MANIPULATION_TASK_NO_TRAJ_FOUND;
	       }
	       cleanRoadmap();
	}

	GP_ConcateneAllTrajectories(_robotPt);
        _robotPt->tcur= _robotPt->t[0];
	p3d_copy_config_into(_robotPt, qi, &qStart);
    break;
    default:
      printf("%s: %d: Manipulation_JIDO::armPlanTask(): wrong task.\n",__FILE__,__LINE__);
      return MANIPULATION_TASK_INVALID_TASK;
    break;
  }
   
   /* COMPUTE THE SOFTMOTION TRAJECTORY */
  traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if(!traj) {
    printf("SoftMotion : ERREUR : no current traj\n");
    XYZ_ENV->cur_robot= cur_robot;
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  if(!traj || traj->nlp < 1) {
    printf("Optimization with softMotion not possible: current trajectory	contains one or zero local path\n");
    XYZ_ENV->cur_robot= cur_robot;
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  if(p3d_optim_traj_softMotion(traj, true, &gain, &ntest, lp, positions) == 1){
    printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
    XYZ_ENV->cur_robot= cur_robot;
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  
  p3d_copy_config_into(_robotPt, qStart, &_robotPt->ROBOT_POS);
  p3d_set_and_update_this_robot_conf(_robotPt, _robotPt->ROBOT_POS);
  p3d_copy_config_into(_robotPt, qGoal, &_robotPt->ROBOT_GOTO);
  ENV.setBool(Env::drawTraj, true);
  XYZ_ENV->cur_robot= cur_robot;

//   centerCamera();

  g3d_draw_allwin_active();

  return MANIPULATION_TASK_OK;
}

int Manipulation_JIDO::cleanRoadmap(){
  if(_robotPt!=NULL) {
    XYZ_ENV->cur_robot= _robotPt;
    deleteAllGraphs();
    FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  } else {
    return 1;
  }
  return 0;
}

int Manipulation_JIDO::cleanTraj(){
  if(_robotPt!=NULL) {
    XYZ_ENV->cur_robot= _robotPt;
    while(_robotPt->nt!=0)
      {
	p3d_destroy_traj(_robotPt, _robotPt->t[0]);
      }
    FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  } else {
    return 1;
  }
  return 0;
}

int Manipulation_JIDO::computeRRT(){
  int result;
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);

#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif

//   ENV.setBool(Env::biDir,true);
//   ENV.setInt(Env::NbTry, 100000);
//   ENV.setInt(Env::MaxExpandNodeFail, 30000);
//   ENV.setInt(Env::maxNodeCompco, 100000);
//   ENV.setExpansionMethod(Env::Extend);
//   ENV.setDouble(Env::extensionStep, 2.0);
  ENV.setBool(Env::biDir,true);
  ENV.setInt(Env::NbTry, 10000);
  ENV.setInt(Env::MaxExpandNodeFail, 10000);
  ENV.setInt(Env::maxNodeCompco, 10000);
  ENV.setExpansionMethod(Env::Extend);
  ENV.setDouble(Env::extensionStep, 2.0);


#if defined (USE_CXX_PLANNER)
  ENV.setBool(Env::withSmoothing, true);
  ENV.setBool(Env::withShortCut, true);
  ENV.setBool(Env::withDeformation, false);
  ENV.setInt(Env::nbCostOptimize, 30);

  ChronoOn();

  result= p3d_run_rrt(_robotPt->GRAPH, fct_stop, fct_draw);

  ChronoPrint("");
  ChronoOff();
#else
  result= p3d_specific_search((char*)"out.txt");
  optimiseTrajectory(100,6);
#endif
  if(!result){
    printf("ArmGotoQ: could not find a path.\n");
    this->printConstraintInfo();
    if(p3d_col_test()) {
      printf("ArmGotoQ: the current configuration is colliding.\n");
      p3d_obj *o1= NULL, *o2= NULL;
      p3d_col_get_report_obj(&o1, &o2 );
      if(o1!=NULL && o2!=NULL) {
	printf("ArmGotoQ: collision between \"%s\" and \"%s\".\n",o1->name,o2->name);
      }
    } else {
      printf("ArmGotoQ: the current configuration is not colliding.\n");
    }

    return 1;
  }
  return 0;
}

int Manipulation_JIDO::computeOptimTraj(){
  int ntest=0;
  double gain = 0;
  p3d_set_NB_OPTIM(30);
  p3d_traj *trajopt = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  int ir = p3d_get_desc_curnum(P3D_ROBOT);

  for(int i=1;i<=p3d_get_NB_OPTIM();i++){
    if(p3d_optim_traj(trajopt,&gain, &ntest)){
      /* position the robot at the beginning of the optimized trajectory */
      position_robot_at_beginning(ir, trajopt);
    }
  }
  return 0;
}

//! Sets everything so that the object motion will be linked to the robot's end-effector motion.
int Manipulation_JIDO::grabObject(char* objectName){
  double x, y, z, rx, ry, rz;
  double error;
  p3d_matrix4 wristPose, objectPose, T1, T2, Tinv, gframe;

  this->releaseObject();
  deactivateCcCntrts(_robotPt, -1);


  // We first have to test the consistency of the relative pose between object and hand with respect to the one
  // in the current grasp:
  getArmX(&x, &y, &z, &rx, &ry, &rz);
  p3d_mat4PosReverseOrder(wristPose, x, y, z, rx, ry, rz);
  getObjectPose(objectPose);

  p3d_mat4Mult(_grasp.frame, _handProp.Tgrasp_frame_hand, T1);
  p3d_mat4Mult(T1, _handProp.Thand_wrist, gframe); 
  p3d_matInvertXform(objectPose, Tinv);
  p3d_mat4Mult(Tinv, wristPose, T2);
  // the two following lines use empirical values, they may need to be adjusted
  error= p3d_mat4Distance(gframe, T2, 1, 0.3);
  if(error > 0.2) {
     printf("error= %f\n", error);
     printf("%s: %d: Manipulation_JIDO::grabObject(): there is an inconsistency between the current frame and the current wrist/object relative pose.\n",__FILE__,__LINE__);
     return 1;
  }

  configPt qi = NULL;
  qi = p3d_get_robot_config(_robotPt);
  p3d_copy_config_into(_robotPt, qi, &_robotPt->ROBOT_POS);
  p3d_destroy_config(_robotPt, qi);

  if(p3d_set_object_to_carry(_robotPt, objectName)!=0) {
    return 1;
  }
  p3d_grab_object2(_robotPt, 0);
  FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  g3d_draw_allwin_active();

  return 0;
}

//! Sets everything so that the object whose motion is linked to the robot's end-effector motion is no more linked.
int Manipulation_JIDO::releaseObject(){
  configPt qi = NULL;
  p3d_release_object(_robotPt);
  deactivateCcCntrts(_robotPt, -1);
  qi = p3d_alloc_config(_robotPt);
  p3d_copy_config_into(_robotPt, _robotPt->ROBOT_POS, &qi);
  /* Uptdate the Virual object for inverse kinematics */
  p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qi);
  p3d_set_and_update_this_robot_conf(_robotPt, qi);
  p3d_destroy_config(_robotPt, qi);
  qi = p3d_get_robot_config(_robotPt);
  p3d_copy_config_into(_robotPt, qi, &_robotPt->ROBOT_POS);
  p3d_destroy_config(_robotPt, qi);
  g3d_draw_allwin_active();
  return 0;
}

//! \return 0 in case of success, !=0 otherwise
int Manipulation_JIDO::armComputePRM() {

  configPt qi = NULL, qf = NULL;

  XYZ_ENV->cur_robot= _robotPt;
  deleteAllGraphs();


  if(_robotPt!=NULL) {
    while(_robotPt->nt!=0)
      {   p3d_destroy_traj(_robotPt, _robotPt->t[0]);  }
    FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  }

  if(_cartesian == 0) {
    /* plan in the C_space */
    p3d_multiLocalPath_disable_all_groupToPlan(_robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(_robotPt, (char*)"jido-arm_lin", 1) ;
    //              if(robotPt->nbCcCntrts!=0) {
    //                      p3d_desactivateCntrt(robotPt, robotPt->ccCntrts[0]);
    //              }
    deactivateCcCntrts(_robotPt, -1);
    //              p3d_desactivateAllCntrts(robotPt);

  } else {
    /* plan in the cartesian space */
    qi = p3d_alloc_config(_robotPt);
    qf = p3d_alloc_config(_robotPt);
    p3d_multiLocalPath_disable_all_groupToPlan(_robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(_robotPt, (char*)"jido-ob_lin", 1) ;
    p3d_copy_config_into(_robotPt, _robotPt->ROBOT_POS, &qi);
    p3d_copy_config_into(_robotPt, _robotPt->ROBOT_GOTO, &qf);
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qi);
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qf);
    p3d_copy_config_into(_robotPt, qi, &_robotPt->ROBOT_POS);
    p3d_copy_config_into(_robotPt, qf, &_robotPt->ROBOT_GOTO);
    p3d_destroy_config(_robotPt, qi);
    p3d_destroy_config(_robotPt, qf);
    if(_robotPt->nbCcCntrts!=0) {
      p3d_activateCntrt(_robotPt, _robotPt->ccCntrts[0]);
    }
  }

  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_BASIC);
  ENV.setInt(Env::NbTry,100000);
  p3d_set_tmax(300);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);

  p3d_set_and_update_this_robot_conf(_robotPt, _robotPt->ROBOT_POS);
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);

  // reactivate collisions for all other robots:
  //         for(i=0; i<(unsigned int) XYZ_ENV->nr; i++) {
  //                 if(XYZ_ENV->robot[i]==robotPt){
  //       continue;
  //     } else {
  //       p3d_col_activate_robot(XYZ_ENV->robot[i]);
  //     }
  //   }
  return 0;
}


//! Gets the pose of the object as a 4x4 matrix.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::getObjectPose(p3d_matrix4 pose){

  if(_object==NULL) {
    printf("%s: %d: Manipulation_JIDO::getObjectPose().\n",__FILE__,__LINE__);
    undefinedObjectMessage();
    return 1;
  }

  p3d_jnt *joint= NULL;

  joint= _object->joints[1];

  if(joint->type!=P3D_FREEFLYER) {
    printf("%s: %d: Manipulation_JIDO::getObjectPose(): the first joint of the object (\"%s\") should be of type P3D_FREEFLYER.\n", __FILE__, __LINE__,_object->name);
    return 1;
  }

  p3d_mat4Copy(joint->abs_pos, pose);

  return 0;
}



//! Gets the pose of the object as a position vector and Euler angles.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::getObjectPose(double *x, double *y, double *z, double *rx, double *ry, double *rz){
  if(_object==NULL) {
    printf("%s: %d: Manipulation_JIDO::getObjectPose().\n",__FILE__,__LINE__);
    undefinedObjectMessage();
    return 1;
  }

  p3d_get_freeflyer_pose2(_object, x, y, z, rx, ry, rz);

  return 0;
}

//! Computes a list of grasps for the currently selected object.
//! NB: works only for the gripper for now
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::computeGraspList(){
  if(_object==NULL) {
    printf("%s: %d: Manipulation_JIDO::computeGraspList().\n", __FILE__, __LINE__);
    undefinedObjectMessage();
    return 1;
  }

  bool recompute;
  int result= 0;
  configPt qbase= NULL;
  std::list<gpGrasp>::iterator igrasp;

  forbidWindowEvents();

  //! Check if the current grasp list corresponds to the current object and so if we need to recompute it or not:
  recompute= true;
  if(!_graspList.empty()) {
    if(_graspList.front().object_name==_object->name) {
      recompute= false;
    }
    else {
      recompute= true;
    }
  }


  if(recompute) {
    switch (_handProp.type) {
    case GP_GRIPPER:
      result = gpGet_grasp_list_gripper(std::string(_object->name), _graspList);
      break;
    default:
      result = GP_ERROR;
    break;
    }
  }


  if(_graspList.empty())
  {
    printf("%s: %d: Manipulation_JIDO::computeGraspList(): No grasp was found.\n",__FILE__,__LINE__);
    allowWindowEvents();
    return 1;
  }


  // remove the grasp if contacts are too close to an angulous edge:

/*
  
  igrasp= _graspList.begin();
  while(igrasp!=_graspList.end()) {
   if( igrasp->areContactsTooCloseToEdge(30*DEGTORAD, 0.02) ) {
     igrasp= _graspList.erase(igrasp);
     continue;
   }
   igrasp++;
  }*/

  // compute the visibility score of the grasps and sort them according to this score:
  if(_cameraJnt!=NULL)
  {
    qbase= p3d_alloc_config(_robotPt);
    p3d_get_robot_config_into(_robotPt, &qbase);
    gpGrasp_visibility_filter(_robotPt, _object, _cameraJnt, _cameraFOV, _cameraImageWidth, _cameraImageHeight, _graspList, GP_PA10, qbase, _handProp);
    p3d_destroy_config(_robotPt, qbase);
  }


  allowWindowEvents();

  if(result == GP_OK) {
    return 0;
  } else {
    return 1;
  }
}

//! Reduces the size of the current grasp list.
//! \param maxSize the desired maximum number of elements of the list
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::reduceGraspList(int maxSize){
  if(_graspList.empty()) {
    printf("%s: %d: Manipulation_JIDO::reduceGraspList(): the current grasp list is empty.\n", __FILE__, __LINE__);
    return 1;
  }

  maxSize= abs(maxSize);

  if((int)_graspList.size() < maxSize) {
    printf("%s: %d: Manipulation_JIDO::reduceGraspList(): the current grasp list has already less than %d elements.\n", __FILE__, __LINE__,maxSize);
    return 1;
  }

  gpReduce_grasp_list_size(_graspList, _graspList, maxSize);

  return 0;
}



//! Sets the _nbGraspsToTestForPickGoto variable.
//! \param n the desired value
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setNbGraspsToTestForPickGoto(int n){

  n= abs(n);

  _nbGraspsToTestForPickGoto= n;

  return 0;
}

//! Marks the grasp with specified ID as tested.
//! \param id the ID of the grasp (see gpGrasp.h)
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::markGraspAsTested(int id){
  if(_graspList.empty()) {
    printf("%s: %d: Manipulation_JIDO::markGraspAsTested(): the current grasp list is empty.\n", __FILE__, __LINE__);
    return 1;
  }

  std::list<gpGrasp>::iterator igrasp;

  for(igrasp=_graspList.begin(); igrasp!=_graspList.end(); ++igrasp)
  {
    if(igrasp->ID==id)
    {
      igrasp->tested= true;
      return 0;
    }
  }

  printf("%s: %d: Manipulation_JIDO::markGraspAsTested(): there is no grasp with ID= %d.\n", __FILE__, __LINE__, id);


  return 1;
}


//! Computes a list of stable poses for the currently selected object.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::computePlacementList(){
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::computePlacementList().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return 1;
  }
  if(_object==NULL) {
    printf("%s: %d: Manipulation_JIDO::computePlacementList().\n", __FILE__, __LINE__);
    undefinedObjectMessage();
    return 1;
  }
  if(_support==NULL) {
    printf("%s: %d: Manipulation_JIDO::computePlacementList().\n", __FILE__, __LINE__);
    undefinedSupportMessage();
    return 1;
  }

  std::list<p3d_rob*> robotList;

  //! Check if the current placement list corresponds to the current object:
  if(!_placementList.empty()) {
    if(_placementList.front().object_name==_object->name) {
      return 0;
    }
  }

  forbidWindowEvents();

  gpCompute_stable_placements(_object, _placementList);
  printf("%d poses found\n",_placementList.size());

  // do not test collisions against our robot:
  robotList.push_back(_robotPt);
//   robotList.push_back(_support);
  gpFind_placements_on_object(_object, _support, robotList, _placementList, _placementTranslationStep, _placementNbOrientations, 0.005, _placementOnSupportList);
  printf("%d poses on support found\n",_placementOnSupportList.size());


  // do not compute clearance with: our robot, the support, the human:
  robotList.clear();
  robotList.push_back(_robotPt);
  robotList.push_back(_support);
  if(_human!=NULL) {
    robotList.push_back(_human);
  } 
  gpCompute_placement_clearances(_object, robotList, _placementOnSupportList);
  printf("Clearance computation is finished.\n");

  allowWindowEvents();

  if(_placementOnSupportList.empty())
  {
    printf("%s: %d: Manipulation_JIDO::computePlacementList(): No stable pose was found.\n",__FILE__,__LINE__);
    return 1;
  }

  return 0;
}

//! Finds a configuration to grasp an object (or a robot) with a simple shape (cylinder, bottle, box, etc.).
//! All the grasps configurations are computed so that the hand grasps the object vertically.
//! NB: The function first searchs for an obstacle with the given name. If there is not, it looks for a robot with that
//! name. The object that we will try to grasp is then the first body of the robot.
//! \param robotPt pointer to the robot
//! \param object_name name of the object (or of a robot) to grasp
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return 0 in case of success,
//!         1 in case no grasp was found and the object is unreachable
//!         2 in case no grasp was found but the object is reachable
int Manipulation_JIDO::findSimpleGraspConfiguration(double *q1, double *q2, double *q3, double *q4, double *q5, double *q6)
{
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::findSimpleGraspConfiguration().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return 1;
  }
  if(_object==NULL) {
    printf("%s: %d: Manipulation_JIDO::findSimpleGraspConfiguration().\n", __FILE__, __LINE__);
    undefinedObjectMessage();
    return 1;
  }

  int i, result;
  int unreachable;
  double x, y, z, theta, radius, r1, r2, r3;
  p3d_vector3 p;
  p3d_matrix4 object_placement;
  p3d_polyhedre *polyhedron= NULL;
  p3d_vector3 cmass, cmass_abs; // object's center of mass
  p3d_matrix3 iaxes; // object's main inertia axes
  p3d_vector3 iaxes_abs[3];
  double iaabb[6]; // bounding box aligned on the object's inertia axes
  configPt q= NULL;
  gpHand_properties hand;

  hand.initialize(GP_GRIPPER);

//   XYZ_ENV->cur_robot= robotPt;
//   FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));



  polyhedron= _object->o[0]->pol[0]->poly;
//   p3d_get_poly_pos(OBJECT->pol[0]->poly, object_placement);
//   pqp_get_obj_pos(OBJECT, object_placement);
  p3d_get_body_pose(_object, 0, object_placement);

  Mass_properties mass_prop;
  gpCompute_mass_properties(polyhedron);
  gpCompute_inertia_axes(&mass_prop, iaxes);
  p3d_vectCopy(mass_prop.r, cmass);
  gpInertia_AABB(polyhedron, cmass, iaxes, iaabb);

/*
  printf("center of mass: \n\t %f %f %f \n", cmass[0], cmass[1], cmass[2] );
  printf("inertia axes: \n\t %f %f %f \n", iaxes[0][0], iaxes[0][1], iaxes[0][2] );
  printf("\t %f %f %f \n", iaxes[1][0], iaxes[1][1], iaxes[1][2] );
  printf("\t %f %f %f \n", iaxes[2][0], iaxes[2][1], iaxes[2][2] );
*/

  p3d_xformPoint(object_placement, cmass, cmass_abs);
  for(i=0; i<3; i++) {
    p[0]= iaxes[0][i];
    p[1]= iaxes[1][i];
    p[2]= iaxes[2][i];
    p3d_xformVect(object_placement, p, iaxes_abs[i]);
  }

  unreachable= TRUE;

//   g3d_win *win= NULL;
//   win= g3d_get_cur_win();
//   win->fct_draw2= &(genomDraw);
  q= p3d_alloc_config(_robotPt);
  p3d_get_robot_config_into(_robotPt, &q);

  radius= 0;
  for(i=0; i<3000; i++)   {
     radius+= 0.001;
     if(radius > 0.7) radius= 0.7;

     r1= p3d_random( radius*iaabb[0], radius*iaabb[1] );
     r2= p3d_random( radius*iaabb[2], radius*iaabb[3] );
     r3= p3d_random( radius*iaabb[4], radius*iaabb[5] );

     x= cmass_abs[0] + r1*iaxes_abs[0][0] + r2*iaxes_abs[0][1] + r3*iaxes_abs[0][2];
     y= cmass_abs[1] + r1*iaxes_abs[1][0] + r2*iaxes_abs[1][1] + r3*iaxes_abs[1][2];
     z= cmass_abs[2] + r1*iaxes_abs[2][0] + r2*iaxes_abs[2][1] + r3*iaxes_abs[2][2];

     theta= p3d_random(0, 2*M_PI);

     result= p3d_set_virtual_object_pose2(_robotPt, x, y, z, 0, 0, theta);

     if(result==0) {
        unreachable= FALSE;
        gpOpen_hand(_robotPt, hand);

        if(p3d_col_test()) {
          continue;
        }
        else {
          gpClose_hand(_robotPt, hand);

//           if(!p3d_col_test_robot_obj(robotPt, OBJECT)) {
          if(!p3d_col_test_robot_other(_robotPt, _object, 0)) {
            continue;
          }

          gpOpen_hand(_robotPt, hand);

          this->getArmQ(q1, q2, q3, q4, q5, q6);
          p3d_set_and_update_this_robot_conf(_robotPt, q);
	  p3d_get_robot_config_into(_robotPt, &_robotPt->ROBOT_POS);
          p3d_destroy_config(_robotPt, q);
          return 0;
        }
    }
  }

  p3d_destroy_config(_robotPt, q);

  if(unreachable==TRUE) {
    return 1;
  }
  else {
    return 2;
  }

}

//! Check if the object is graspable for the current configuration of the robot platform
//! \param objectName the object to test the graspability
//! \return false not graspable, true otherwise
bool Manipulation_JIDO::isObjectGraspable(char *objectName){
   setObjectToManipulate(objectName);
   double q1, q2, q3, q4, q5, q6;
   double pre_q1, pre_q2, pre_q3, pre_q4, pre_q5,  pre_q6;
   double distance = 0.0;
   if(findPregraspAndGraspConfiguration(distance, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6)== 1){
      return false;
   }
   return true;
}


//! Finds a configuration to grasp the object plus
//! an intermediate configuration (a configuration slightly before grasqping the object)
//! \param distance the distance (along the vertical) between the grasp configuration and the intermediate configuration
//! \param pre_q1 will be filled with value of joint #1
//! \param pre_q2 will be filled with value of joint #2
//! \param pre_q3 will be filled with value of joint #3
//! \param pre_q4 will be filled with value of joint #4
//! \param pre_q5 will be filled with value of joint #5
//! \param pre_q6 will be filled with value of joint #6
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::findPregraspAndGraspConfiguration(double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6){

 if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::findPregraspAndGraspConfiguration().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return 1;
  }
  if(_object==NULL) {
    printf("%s: %d: Manipulation_JIDO::findPregraspAndGraspConfiguration().\n", __FILE__, __LINE__);
    undefinedObjectMessage();
    return 1;
  }
  deactivateCcCntrts(_robotPt, -1);
  int result = 0;

  this->computeGraspList();

  /* met la pince mobile loin, treeeesss loiiiiinnnn */
  p3d_set_and_update_this_robot_conf(p3d_get_robot_by_name(GP_GRIPPER_ROBOT_NAME), p3d_get_robot_by_name(GP_GRIPPER_ROBOT_NAME)->ROBOT_GOTO);


//   p3d_matrix4 objectPose;
  configPt qcur= NULL, qgrasp= NULL, qpregrasp= NULL;
  p3d_rob *cur_robotPt= NULL;
  list<gpGrasp> untestedGrasps;
  list<gpGrasp>::iterator igrasp;

  cur_robotPt= XYZ_ENV->cur_robot;
  XYZ_ENV->cur_robot= _robotPt;

  qcur= p3d_alloc_config(_robotPt);
  qgrasp= p3d_alloc_config(_robotPt);
  qpregrasp= p3d_alloc_config(_robotPt);
  p3d_get_robot_config_into(_robotPt, &qcur);

  // only copy the grasps that were not tested for path planning:
  for(igrasp=_graspList.begin(); igrasp!=_graspList.end(); ++igrasp)
  {
    if(igrasp->tested==false)
    {
      untestedGrasps.push_back(*igrasp);
    }
  }

  result= gpFind_grasp_and_pregrasp_from_base_configuration(_robotPt, _object, untestedGrasps, GP_PA10, qcur, _grasp, _handProp, _liftUpDistance, qpregrasp, qgrasp);

  _graspID= _grasp.ID;

  if(result==GP_ERROR)
  {
    printf("%s: %d: Manipulation_JIDO::findPregraspAndGraspConfiguration(): No grasp is reachable from the current base configuration.\n",__FILE__,__LINE__);
    p3d_set_and_update_this_robot_conf(_robotPt, qcur);
    p3d_destroy_config(_robotPt, qcur);
    p3d_destroy_config(_robotPt, qpregrasp);
    p3d_destroy_config(_robotPt, qgrasp);
    XYZ_ENV->cur_robot= cur_robotPt;
    return 1;
  }


  p3d_set_and_update_this_robot_conf(_robotPt, qgrasp);
  gpOpen_hand(_robotPt, _handProp);
  this->getArmQ(q1, q2, q3, q4, q5, q6);

  p3d_set_and_update_this_robot_conf(_robotPt, qpregrasp);
//   gpOpen_hand(robotPt, hand);
  this->getArmQ(pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6);

  p3d_get_robot_config_into(_robotPt, &_robotPt->ROBOT_GOTO);

  p3d_set_and_update_this_robot_conf(_robotPt, qcur);
  p3d_get_robot_config_into(_robotPt, &_robotPt->ROBOT_POS);

  p3d_destroy_config(_robotPt, qcur);
  p3d_destroy_config(_robotPt, qgrasp);
  p3d_destroy_config(_robotPt, qpregrasp);
  XYZ_ENV->cur_robot= cur_robotPt;

  return 0;
}


//! Finds a configuration to place the object plus
//! an intermediate configuration (a configuration slightly before placing the object).
//! \param distance distance (along the vertical) between the placement and the intermediate configuration
//! \param pre_q1 will be filled with value of joint #1
//! \param pre_q2 will be filled with value of joint #2
//! \param pre_q3 will be filled with value of joint #3
//! \param pre_q4 will be filled with value of joint #4
//! \param pre_q5 will be filled with value of joint #5
//! \param pre_q6 will be filled with value of joint #6
//! \param q1 will be filled with value of joint #1
//! \param q2 will be filled with value of joint #2
//! \param q3 will be filled with value of joint #3
//! \param q4 will be filled with value of joint #4
//! \param q5 will be filled with value of joint #5
//! \param q6 will be filled with value of joint #6
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::findPlacementConfigurations(double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6){

 if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::findPlacementConfigurations().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return 1;
  }
  if(_object==NULL) {
    printf("%s: %d: Manipulation_JIDO::findPlacementConfigurations().\n", __FILE__, __LINE__);
    undefinedObjectMessage();
    return 1;
  }
  if(_support==NULL) {
    printf("%s: %d: Manipulation_JIDO::findPlacementConfigurations().\n", __FILE__, __LINE__);
    undefinedSupportMessage();
    return 1;
  }
  if(!_robotPt->isCarryingObject) {
    printf("%s: %d: Manipulation_JIDO::findPlacementConfigurations(): no object is currently grabbed.\n", __FILE__, __LINE__);
    return 1;
  }


  int result= 0;
  configPt qcur= NULL;
  configPt qpreplacement= NULL;
  configPt qplacement= NULL;

  result= computePlacementList();
  if(result==GP_ERROR) {
    printf("%s: %d: Manipulation_JIDO::findPlacementConfigurations(): No stable pose were found on the current support.\n",__FILE__,__LINE__);
    return 1;
  }

  qcur = p3d_get_robot_config(_robotPt);
  qpreplacement= p3d_alloc_config(_robotPt);
  qplacement= p3d_alloc_config(_robotPt);

  deactivateCcCntrts(_robotPt, -1);
  result= gpFind_placement_from_base_configuration(_robotPt, _object, _placementOnSupportList, GP_PA10, qcur, _grasp, _handProp, distance, qpreplacement, qplacement, _placement);

  if(result==GP_ERROR)
  {
    printf("%s: %d: Manipulation_JIDO::findPlacementConfigurations(): No placement is reachable from the current base configuration and the current grasp.\n",__FILE__,__LINE__);
    p3d_set_and_update_this_robot_conf(_robotPt, qcur);
    p3d_destroy_config(_robotPt, qcur);
    p3d_destroy_config(_robotPt, qpreplacement);
    p3d_destroy_config(_robotPt, qplacement);
    return 1;
  }



  p3d_set_and_update_this_robot_conf(_robotPt, qplacement);
  getArmQ(q1, q2, q3, q4, q5, q6);

  p3d_set_and_update_this_robot_conf(_robotPt, qpreplacement);
//   gpOpen_hand(robotPt, hand);
  getArmQ(pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6);

  p3d_get_robot_config_into(_robotPt, &_robotPt->ROBOT_GOTO);

  p3d_set_and_update_this_robot_conf(_robotPt, qcur);
  p3d_get_robot_config_into(_robotPt, &_robotPt->ROBOT_POS);

  p3d_destroy_config(_robotPt, qcur);
  p3d_destroy_config(_robotPt, qplacement);
  p3d_destroy_config(_robotPt, qpreplacement);
  

  return 0;
}


//! Finds a body from its name. The function first searchs for an obstacle with the given name.
//! If there is not, it looks for a robot with that name.
p3d_obj * Manipulation_JIDO::getObjectByName(char *object_name){
 int i;
  p3d_obj *object= NULL;

  object= NULL;
  object= p3d_get_obst_by_name(object_name);

  if(object==NULL) {
     for(i=0; i<XYZ_ENV->nr; i++) {
       if(strcmp(XYZ_ENV->robot[i]->name, object_name)==0) {
         object= XYZ_ENV->robot[i]->o[0];
         return object;
       }
     }
     if(i==XYZ_ENV->nr) {
       printf("%s: %d: genomGetObject(): There is no object or robot named \"%s\".\n", __FILE__, __LINE__, object_name);
       return NULL;
     }
  }
  else {
    return object;
  }

  return NULL;
}


//! Sets the pose of the object from coordinates given in the frame
//! of the end effector (virtual object) of the robot.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setObjectPoseWrtEndEffector(double x, double y, double z, double rx, double ry, double rz){
  if(_robotPt==NULL) {
    printf("%s: %d: genomSetObjectPoseWrtEndEffector().\n", __FILE__, __LINE__);
    undefinedRobotMessage();
    return 1;
  }
  if(_object==NULL) {
    printf("%s: %d: genomSetObjectPoseWrtEndEffector(): input object is NULL.\n", __FILE__, __LINE__);
    undefinedObjectMessage();
    return 1;
  }

//   double x1, y1, z1, rx1, ry1, rz1;
  p3d_matrix4 T1, T2, T3;

  p3d_set_virtual_object_pose(_robotPt, T1);

  p3d_mat4PosReverseOrder(T2, x, y, z, rx, ry, rz);
  p3d_mat4Mult(T1, T2, T3);

  p3d_set_freeflyer_pose(_object, T3);


//   p3d_mat4ExtractPosReverseOrder(T3, &x1, &y1, &z1, &rx1, &ry1, &rz1);
// 
//   setFreeflyerPose(object, x1, y1, z1, rx1, ry1, rz1);

  return 0;

}


//! This function updates the configuration of the robot so that it grasps the object.
//! It first tries to grasp it from its current base configuration and if no grasp is reachable
//! tries another base configuration.
//! \param robot_name name of the robot
//! \param hand_robot_name name of the hand robot uesd to compute the grasp list
//! \param object_name name of the object to grasp
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::dynamicGrasping(char *robot_name, char *hand_robot_name, char *object_name){
 int i, result, nb_iters_max;
  double pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6;
  double q1, q2, q3, q4, q5, q6;
  p3d_vector3 objectCenter;
  p3d_matrix4 objectPose;
  configPt qbase= NULL;
  p3d_obj *object= NULL;
  p3d_rob *robotPt= NULL;
  p3d_rob *hand_robotPt= NULL;
  p3d_rob *cur_robotPt= NULL;
  gpHand_properties hand_info;
  hand_info.initialize(GP_GRIPPER);


  robotPt= p3d_get_robot_by_name(robot_name);
  hand_robotPt= p3d_get_robot_by_name(hand_robot_name);

  cur_robotPt= XYZ_ENV->cur_robot;
  XYZ_ENV->cur_robot= robotPt;

  result= findPregraspAndGraspConfiguration(0.0, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6);

  object= getObjectByName(object_name);

  if(object==NULL)
  {
    printf("%s: %d: genomDynamicGrasping(): object \"%s\" is not defined.\n",__FILE__,__LINE__,object_name);
    XYZ_ENV->cur_robot= cur_robotPt;
    return 1;
  }

  p3d_get_poly_pos((poly_polyhedre*)object->pol[0]->poly, objectPose);
  objectCenter[0]= objectPose[0][3];
  objectCenter[1]= objectPose[1][3];
  objectCenter[2]= objectPose[2][3];
// p3d_rob *rob= p3d_get_robot_by_name("BOTTLE");
// if(rob!=NULL) genomSetObjectPoseWrtEndEffector(robotPt, rob, 0, 0, 0.06, 0.5, 0, 0);
 if(result==0) //success
  {
    this->setArmQ(q1, q2, q3, q4, q5, q6);
    gpSet_grasp_configuration(robotPt, _grasp, 0);
  }
  else //robot needs to move
  {
     nb_iters_max= 100;
     for(i=0; i<nb_iters_max; i++)
     {
        qbase= gpRandom_robot_base(robotPt, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter, GP_PA10);
        if(qbase==NULL)
        {  continue;  }
        p3d_set_and_update_this_robot_conf(robotPt, qbase);
        p3d_destroy_config(robotPt, qbase);
        qbase= NULL;
printf("pose %f %f %f\n",objectPose[0][3],objectPose[1][3],objectPose[2][3]);

        result= findPregraspAndGraspConfiguration(0.0, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6);


        if(result==0) {
          this->setArmQ(q1, q2, q3, q4, q5, q6);
          gpSet_grasp_configuration(robotPt, _grasp, 0);
          break;
        }
     }


     if(i==nb_iters_max)
     {
       printf("genomDynamicGrasping: No valid platform configuration was found.\n");
     }
  }

  XYZ_ENV->cur_robot= cur_robotPt;

  //to take a screenshot if CAPTURE==TRUE:
  static int count= 0;
  char file[128], filePPM[128], fileJPG[128], str[128];

  if(_capture==TRUE)
  {
    if(count < 10) sprintf(file, "./video/00000%d", count);
    else if(count < 100) sprintf(file, "./video/0000%d", count);
    else if(count < 1000) sprintf(file, "./video/000%d", count);
    else if(count < 10000) sprintf(file, "./video/00%d", count);
    else sprintf(file, "0%d", count);

    strcpy(filePPM, file);
    strcpy(fileJPG, file);
    strcat(filePPM, ".ppm");
    strcat(fileJPG, ".jpg");
    g3d_export_OpenGL_display(filePPM);

    sprintf(str,"convert -quality 95 %s %s; rm %s",filePPM, fileJPG,filePPM);
    system(str);
    count++;
  }

  return 0;
}


int Manipulation_JIDO::robotBaseGraspConfig(char *objectName, double *x, double *y, double *theta){
//   bool needs_to_move = false;
  p3d_rob *objectPt= NULL;
  objectPt= p3d_get_robot_by_name(objectName);
  p3d_matrix4 objectPose;
  p3d_vector3 objectCenter;
  int nb_iters_max, i;
  configPt qbase = NULL;//, qresult = NULL;
  gpGrasp grasp;
  gpHand_properties handProp;
  double x0, y0, theta0;
  handProp.initialize(GP_GRIPPER);
   p3d_rob *hand_robotPt= NULL;

   std::list<gpGrasp> graspList;

   configPt q0 = NULL;

   q0 = p3d_get_robot_config(_robotPt);

   double pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6;
   double q1, q2, q3, q4, q5, q6;

   gpGet_grasp_list_gripper(std::string(objectName), graspList);

        hand_robotPt= p3d_get_robot_by_name(GP_GRIPPER_ROBOT_NAME);

        gpCompute_mass_properties(objectPt->o[0]->pol[0]->poly);

        p3d_get_body_pose ( objectPt, 0, objectPose );
	objectCenter[0]= objectPose[0][3] + objectPt->o[0]->pol[0]->poly->cmass[0];
	objectCenter[1]= objectPose[1][3] + objectPt->o[0]->pol[0]->poly->cmass[1];
	objectCenter[2]= objectPose[2][3] + objectPt->o[0]->pol[0]->poly->cmass[2];
        qbase = p3d_get_robot_config(_robotPt);
	nb_iters_max= 300;
	for ( i=0; i<nb_iters_max; i++ )
	{
          if(i!=0) {
	        qbase= gpRandom_robot_base ( _robotPt, GP_INNER_RADIUS, GP_OUTER_RADIUS, objectCenter, GP_PA10 );
		if ( qbase==NULL )
			{   break;   }
          }
p3d_set_and_update_this_robot_conf(_robotPt, qbase);
          gpGet_platform_configuration ( _robotPt, x0, y0, theta0 );
	p3d_set_and_update_this_robot_conf(_robotPt, q0);
      gpSet_platform_configuration(_robotPt, x0, y0, theta0 );

      if(p3d_col_test_robot(_robotPt, 0)) {

continue;
      }
	g3d_draw_allwin_active();
		//qresult= NULL;
		//qresult= gpFind_grasp_from_base_configuration ( robotPt, objectPt, graspList, GP_PA10, qbase, grasp, handProp );
      if(findPregraspAndGraspConfiguration( 0.08, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6) != 0) {
// 	  printf("no solution to grasp\n");

          p3d_destroy_config ( _robotPt, qbase );
		qbase= NULL;


	} else {
           p3d_set_and_update_this_robot_conf(_robotPt, qbase);
           break;
	}

	}

	if ( qbase!=NULL )
		{  p3d_destroy_config ( _robotPt, qbase );  }

	if ( i==nb_iters_max )
	{
		printf ( "GP_FindGraspConfig: No valid platform configuration was found.\n" );
		return 1;
	}

//
        gpGet_platform_configuration ( _robotPt, x0, y0, theta0 );
	*x = x0;
	*y = y0;
	*theta = theta0;
	return 0;
}



//! Sets the pose of a robot whose first joint is a P3D_FREEFLYER.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setFreeflyerPose(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz)
{
  if(robotPt==NULL) {
    printf("%s: %d: genomSetFreeflyerPose(): input robot is NULL.\n", __FILE__, __LINE__);
    return 1;
  }

  configPt q= NULL;
  p3d_jnt *joint= NULL;

  joint= robotPt->joints[1];

  if(joint->type!=P3D_FREEFLYER) {
    printf("%s: %d: genomSetFreeflyerPose(): first joint of robot \"%s\" should be of type P3D_FREEFLYER.\n", __FILE__, __LINE__,robotPt->name);
    return 1;
  }


  q= p3d_alloc_config(robotPt);
  p3d_get_robot_config_into(robotPt, &q);

  q[joint->index_dof]    = x;
  q[joint->index_dof + 1]= y;
  q[joint->index_dof + 2]= z;
  q[joint->index_dof + 3]= rx;
  q[joint->index_dof + 4]= ry;
  q[joint->index_dof + 5]= rz;

  p3d_set_and_update_this_robot_conf(robotPt, q);

  p3d_destroy_config(robotPt, q);

  return 0;
}


//! Sets the pose of a robot whose first joint is a P3D_FREEFLYER.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setFreeflyerPoseByName(char *name, double x, double y, double z, double rx, double ry, double rz)
{

        p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
        int nr,i, cont=0;
        p3d_rob * r = NULL;
        p3d_rob* robotPt=NULL;


        nr = envPt->nr;

        for(i=0;i<nr && !cont;i++)
        {
                r = envPt->robot[i];
                if (strcmp(r->name,name)==0)
                {
                        robotPt  = r;
                }
        }

        if(robotPt==NULL) {
                //printf("%s: %d: genomSetFreeflyerPose(): input robot is NULL.\n", __FILE__, __LINE__);
                return 1;
        }

        configPt q= NULL;
        p3d_jnt *joint= NULL;

        joint= robotPt->joints[1];

        if(joint->type!=P3D_FREEFLYER) {
                printf("%s: %d: genomSetFreeflyerPose(): first joint of robot \"%s\" should be of type P3D_FREEFLYER.\n", __FILE__, __LINE__,robotPt->name);
                return 1;
        }


        q= p3d_alloc_config(robotPt);
        p3d_get_robot_config_into(robotPt, &q);

        q[joint->index_dof]    = x;
        q[joint->index_dof + 1]= y;
        q[joint->index_dof + 2]= z;
        q[joint->index_dof + 3]= rx;
        q[joint->index_dof + 4]= ry;
        q[joint->index_dof + 5]= rz;

        p3d_set_and_update_this_robot_conf(robotPt, q);

        p3d_destroy_config(robotPt, q);

        return 0;
}

//! Display function of the class Manipulation_JIDO.
void Manipulation_JIDO::draw(){
  _grasp.draw(0.015);

  _placement.draw(0.005);

  if(displayGrasps) {
    for ( std::list<gpGrasp>::iterator iter= _graspList.begin(); iter!=_graspList.end(); iter++ )
    { ( *iter ).draw(0.005);    }
  }

  if(displayPlacements) {
//     for ( std::list<gpPlacement>::iterator iter= _placementList.begin(); iter!=_placementList.end(); iter++ )
//     { ( *iter ).draw(0.005);    }

    for ( std::list<gpPlacement>::iterator iter= _placementOnSupportList.begin(); iter!=_placementOnSupportList.end(); iter++ )
    { ( *iter ).draw(0.005);    }
  }

}

void Manipulation_JIDO::setCapture(bool v){
  _capture = v;
}

bool Manipulation_JIDO::getCapture(){
  return _capture;
}

//! Centers the camera on the object's position.
int Manipulation_JIDO::centerCamera(){
  if(_object==NULL) {
   return 1;
  }
  p3d_matrix4 pose;
  g3d_win *win= NULL;

  win= g3d_get_cur_win();
  p3d_get_freeflyer_pose(_object, pose);

  win->vs.x= pose[0][3];
  win->vs.y= pose[1][3];
  win->vs.z= pose[2][3];
  win->vs.zo= 1.5;
  win->vs.az= M_PI_2;
  win->vs.GOURAUD = 1;
  win->vs.displayFloor = 1;

  return 0;
}

int Manipulation_JIDO::computeTrajBetweenTwoConfigs(bool cartesian, configPt qi, configPt qf) {
        if(qi==NULL) {
           printf("%s: %d: computeTrajBetweenTwoConfigs(): qi is NULL.\n", __FILE__, __LINE__);
          return 1;
        }
        if(qf==NULL) {
           printf("%s: %d: computeTrajBetweenTwoConfigs(): qf is NULL.\n", __FILE__, __LINE__);
          return 1;
        }

        forbidWindowEvents();

        p3d_copy_config_into(_robotPt, qi, &_robotPt->ROBOT_POS);
        p3d_copy_config_into(_robotPt, qf, &_robotPt->ROBOT_GOTO);

        if(cartesian == false) {
                /* plan in the C_space */
                p3d_multiLocalPath_disable_all_groupToPlan(_robotPt);
                p3d_multiLocalPath_set_groupToPlan_by_name(_robotPt, (char*)"jido-arm_lin", 1) ;
                if(_robotPt->nbCcCntrts!=0) {
                        p3d_desactivateCntrt(_robotPt, _robotPt->ccCntrts[0]);
                }
        } else {
                /* plan in the cartesian space */
                qi = p3d_alloc_config(_robotPt);
                qf = p3d_alloc_config(_robotPt);
                p3d_multiLocalPath_disable_all_groupToPlan(_robotPt);
                p3d_multiLocalPath_set_groupToPlan_by_name(_robotPt, (char*)"jido-ob_lin", 1) ;
                p3d_copy_config_into(_robotPt, _robotPt->ROBOT_POS, &qi);
                p3d_copy_config_into(_robotPt, _robotPt->ROBOT_GOTO, &qf);
                p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qi);
                p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qf);
                p3d_copy_config_into(_robotPt, qi, &_robotPt->ROBOT_POS);
                p3d_copy_config_into(_robotPt, qf, &_robotPt->ROBOT_GOTO);
                p3d_destroy_config(_robotPt, qi);
                p3d_destroy_config(_robotPt, qf);
                if(_robotPt->nbCcCntrts!=0) {
                        p3d_activateCntrt(_robotPt, _robotPt->ccCntrts[0]);
                }
        }


      if(p3d_equal_config(_robotPt, _robotPt->ROBOT_POS, _robotPt->ROBOT_GOTO)) {
	printf("genomArmGotoQ: Start and goal configurations are the same.\n");
        allowWindowEvents();
	return 1;
      }
        /* RRT */
      if(this->computeRRT() != 0) {
        allowWindowEvents();
	return 1;
      }
      printf("End RRT\n");

  allowWindowEvents();

  return 0;
}

//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setObjectToManipulate(char *objectName) {

  _object= p3d_get_robot_by_name(objectName);

  if(_object==NULL)
  {
    printf("%s: %d: Manipulation_JIDO::setObjectToManipulate(): there is no robot (the object to grasp) named \"%s\".\n", __FILE__, __LINE__, objectName);
    return 1;
  }
  return 0;
}

//! Sets the support i.e. the object the robot will try to place the manipulated object onto.
//! Typically, it is a table or any object with a large flat horizontal surface.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setSupport(char *supportName) {

  _support= p3d_get_robot_by_name(supportName);

  if(_support==NULL)
  {
    printf("%s: %d: Manipulation_JIDO::setSupport(): there is no robot (the support to place the grasped object on) named \"%s\".\n", __FILE__, __LINE__, supportName);
    return 1;
  }

  return 0;
}

//! Sets the human the robot may have to interact with.
//! For now, it is used to avoid unnecessary costly distance computation.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setHuman(char *humanName) {

  _human= p3d_get_robot_by_name(humanName);

  if(_human==NULL)
  {
    printf("%s: %d: Manipulation_JIDO::setHuman(): there is no robot (the human the robot has to interact with) named \"%s\".\n", __FILE__, __LINE__, humanName);
    return 1;
  }

  return 0;
}

//! Sets the camera joint of the robot.
//! It is used to retrieve the frame of the robot's gaze.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setCameraJnt(char *cameraJntName) {
  if(_robotPt==NULL) {
    printf("%s: %d: Manipulation_JIDO::setCameraJnt(): .", __FILE__, __LINE__);
    undefinedRobotMessage();
    return 1;
  }

  _cameraJnt= p3d_get_robot_jnt_by_name(_robotPt, (char*)cameraJntName);

  if(_cameraJnt==NULL)  {
    printf("%s: %d: Manipulation_JIDO::setCameraJnt(): the robot has no joint named \"%s\".\n", __FILE__, __LINE__, cameraJntName);
    return 1;
  }

  return 0;
}

//! Forbids all the interaction (keyboard and mouse) with the current window.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::forbidWindowEvents() {
  g3d_win *win= NULL;
  win= g3d_get_cur_win();

  if(win==NULL) {
    return 1;
  }

  win->vs.eventsEnabled= 0;
  
  return 0;
}

//! Allows the interaction (keyboard and mouse) with the current window.
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::allowWindowEvents() {
  g3d_win *win= NULL;
  win= g3d_get_cur_win();

  if(win==NULL) {
    return 1;
  }

  win->vs.eventsEnabled= 1;
  
  return 0;
}

//! Sets the robot's camera field of view angle.
//! It is used to compute the synthetic views of what the robot is meant to see from its camera.
//! \param fov the field of view angle value, in degrees
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setCameraFOV(double fov) {
  
  _cameraFOV= fov;

  return 0;
}

//! Sets the robot's camera image size.
//! It is used to compute the synthetic views of what the robot is meant to see from its camera.
//! \param width image width
//! \param height image height
//! \return 0 in case of success, 1 otherwise
int Manipulation_JIDO::setCameraImageSize(int width, int height) {
  
  _cameraImageWidth= width;
  _cameraImageHeight= height;

  return 0;
}

#ifdef DPG
//! \brief Check if the current path is in collision or not
//! \return 1 in case of collision, 0 otherwise
int  Manipulation_JIDO::checkCollisionOnTraj(){
  p3d_rob *robotPt = NULL;
  robotPt= (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  //   configPt currentPos = p3d_get_robot_config(robotPt);
  //   double armPos[6] = {currentPos[5], currentPos[6], currentPos[7], currentPos[8], currentPos[9], currentPos[10]};
  if(checkCollisionOnTraj(robotPt, 0, 0)){
    printf("There is collision\n");
    return 1;
  }else{
    printf("There is no collision\n");
    return 0;
  }
}


//! \brief Check if the current path is in collision or not
//! \return 1 in case of collision, 0 otherwise
int Manipulation_JIDO::checkCollisionOnTraj(p3d_rob* robotPt, int cartesian, int currentLpId) {
  configPt qi = NULL, qf = NULL;
  static p3d_traj *traj = NULL;
  //   int ntest=0;
  //   double gain;
  
  XYZ_ENV->cur_robot= robotPt;
  //initialize and get the current linear traj
  if (!traj){
    if(robotPt->nt < robotPt->tcur->num - 2){
      return 1;
    }else{
      traj = robotPt->t[robotPt->tcur->num - 2];
    }
  }
  if(cartesian == 0) {
    /* plan in the C_space */
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-arm_lin", 1) ;
    deactivateCcCntrts(robotPt, -1);
  } else {
    /* plan in the cartesian space */
    qi = p3d_alloc_config(robotPt);
    qf = p3d_alloc_config(robotPt);
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-ob_lin", 1) ;
    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &qi);
    p3d_copy_config_into(robotPt, robotPt->ROBOT_GOTO, &qf);
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qi);
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qf);
    p3d_copy_config_into(robotPt, qi, &robotPt->ROBOT_POS);
    p3d_copy_config_into(robotPt, qf, &robotPt->ROBOT_GOTO);
    p3d_destroy_config(robotPt, qi);
    p3d_destroy_config(robotPt, qf);
    if(robotPt->nbCcCntrts!=0) {
      p3d_activateCntrt(robotPt, robotPt->ccCntrts[0]);
    }
  }
  if (currentLpId > traj->nlp){
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  p3d_localpath* currentLp = traj->courbePt;
  for(int i = 0; i < currentLpId/2; i++){
    currentLp = currentLp->next_lp;
  }
  return checkForCollidingPath(robotPt, traj, currentLp);
}

//! Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
//! \return 0 in case of success, !=0 otherwise
int Manipulation_JIDO::replanCollidingTraj(p3d_rob* robotPt, int cartesian, int currentLpId, int lp[], Gb_q6 positions[],  int *nbPositions) {
  configPt qi = NULL, qf = NULL;
  static p3d_traj *traj = NULL;
  int ntest=0;
  double gain;
  
  XYZ_ENV->cur_robot = robotPt;
  //initialize and get the current linear traj
  if (!traj){
    if(robotPt->nt < robotPt->tcur->num - 2){
      return MANIPULATION_TASK_INVALID_TRAJ_ID;
    }else{
      traj = robotPt->t[robotPt->tcur->num - 2];
    }
  }
  if(cartesian == 0) {
    /* plan in the C_space */
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-arm_lin", 1) ;
    deactivateCcCntrts(robotPt, -1);
  } else {
    /* plan in the cartesian space */
    qi = p3d_alloc_config(robotPt);
    qf = p3d_alloc_config(robotPt);
    p3d_multiLocalPath_disable_all_groupToPlan(robotPt);
    p3d_multiLocalPath_set_groupToPlan_by_name(robotPt, (char*)"jido-ob_lin", 1) ;
    p3d_copy_config_into(robotPt, robotPt->ROBOT_POS, &qi);
    p3d_copy_config_into(robotPt, robotPt->ROBOT_GOTO, &qf);
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qi);
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(robotPt, qf);
    p3d_copy_config_into(robotPt, qi, &robotPt->ROBOT_POS);
    p3d_copy_config_into(robotPt, qf, &robotPt->ROBOT_GOTO);
    p3d_destroy_config(robotPt, qi);
    p3d_destroy_config(robotPt, qf);
    if(robotPt->nbCcCntrts!=0) {
      p3d_activateCntrt(robotPt, robotPt->ccCntrts[0]);
    }
  }
  if (currentLpId > traj->nlp){
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  p3d_localpath* currentLp = traj->courbePt;
  for(int i = 0; i < currentLpId/2; i++){
    currentLp = currentLp->next_lp;
  }
  configPt currentConfig = p3d_get_robot_config(robotPt);
  int j = 0, returnValue = 0, optimized = traj->isOptimized;
  if(optimized){
    p3dAddTrajToGraph(robotPt, robotPt->GRAPH, traj);
  }
  do{
    printf("Test %d\n", j);
    j++;
    returnValue = replanForCollidingPath(robotPt, traj, robotPt->GRAPH, currentConfig, currentLp, optimized);
    traj = robotPt->tcur;
    currentLp = traj->courbePt;
  }while(returnValue != 1 && returnValue != 0 && returnValue != -2 && j < 10);
  if (optimized && j > 1){
    optimiseTrajectory(100,6);
  }
  if(j > 1){//There is a new traj
    /* COMPUTE THE SOFTMOTION TRAJECTORY */
    traj = robotPt->tcur;
    if(!traj) {
      printf("SoftMotion : ERREUR : no current traj\n");
      return MANIPULATION_TASK_ERROR_UNKNOWN;
    }
    if(!traj || traj->nlp < 1) {
      printf("Optimization with softMotion not possible: current trajectory contains one or zero local path\n");
      return MANIPULATION_TASK_OK;
    }
    //     if(p3d_optim_traj_softMotion(traj, true, &gain, &ntest, lp, positions, nbPositions) == 1){
    //       printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
    //       return 1;
    //     }
    //peut etre ajouter un return specific pour savoir qu'il y'a une nouvelle traj
  }
  return MANIPULATION_TASK_OK;
}
#endif


void printManipulationError(MANIPULATION_TASK_MESSAGE message) {
  switch(message)
  {
    case MANIPULATION_TASK_OK:
     printf("MANIPULATION_TASK_OK\n");
    break;
    case MANIPULATION_TASK_NOT_INITIALIZED:
     printf("MANIPULATION_TASK_NOT_INITIALIZED\n");
    break;
    case MANIPULATION_TASK_NO_TRAJ_FOUND:
     printf("MANIPULATION_TASK_NO_TRAJ_FOUND\n");
    break;
    case MANIPULATION_TASK_INVALID_QSTART:
     printf("MANIPULATION_TASK_INVALID_QSTART \n");
    break;
    case MANIPULATION_TASK_INVALID_QGOAL:
     printf("MANIPULATION_TASK_INVALID_QGOAL\n");
    break;
    case MANIPULATION_TASK_INVALID_TRAJ_ID:
     printf("MANIPULATION_TASK_INVALID_TRAJ_ID\n");
    break;
    case MANIPULATION_TASK_INVALID_TASK:
     printf("MANIPULATION_TASK_INVALID_TASK\n");
    break;
    case MANIPULATION_TASK_UNKNOWN_OBJECT:
     printf("MANIPULATION_TASK_UNKNOWN_OBJECT\n");
    break;
    case MANIPULATION_TASK_NO_GRASP:
     printf("MANIPULATION_TASK_NO_GRASP\n");
    break;
    case MANIPULATION_TASK_NO_PLACE:
     printf("MANIPULATION_TASK_NO_PLACE\n");
    break;
    case MANIPULATION_TASK_ERROR_UNKNOWN:
     printf("MANIPULATION_TASK_ERROR_UNKNOWN\n");
    break;

   }


}


void printManipulationMessage(MANIPULATION_TASK_MESSAGE message)
{
  switch(message)
  {
    case MANIPULATION_TASK_OK:
     printf("MANIPULATION_TASK_OK\n");
    break;
    case MANIPULATION_TASK_NOT_INITIALIZED:
     printf("MANIPULATION_TASK_NOT_INITIALIZED\n");
    break;
    case MANIPULATION_TASK_NO_TRAJ_FOUND:
     printf("MANIPULATION_TASK_NO_TRAJ_FOUND\n");
    break;
    case MANIPULATION_TASK_INVALID_QSTART:
     printf("MANIPULATION_TASK_INVALID_QSTART\n");
    break;
    case MANIPULATION_TASK_INVALID_QGOAL:
     printf("MANIPULATION_TASK_INVALID_QGOAL\n");
    break;
    case MANIPULATION_TASK_INVALID_TRAJ_ID:
     printf("MANIPULATION_TASK_INVALID_TRAJ_ID\n");
    break;
    case MANIPULATION_TASK_INVALID_TASK:
     printf("MANIPULATION_TASK_INVALID_TASK\n");
    break;
    case MANIPULATION_TASK_UNKNOWN_OBJECT:
     printf("MANIPULATION_TASK_UNKNOWN_OBJECT\n");
    break;
    case MANIPULATION_TASK_NO_GRASP:
     printf("MANIPULATION_TASK_NO_GRASP\n");
    break;
    case MANIPULATION_TASK_NO_PLACE:
     printf("MANIPULATION_TASK_NO_PLACE\n");
    break;
    case MANIPULATION_TASK_ERROR_UNKNOWN:
     printf("MANIPULATION_TASK_ERROR_UNKNOWN\n");
    break;
  }

}
