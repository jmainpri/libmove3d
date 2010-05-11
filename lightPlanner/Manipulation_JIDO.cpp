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
#ifdef DPG
#include "p3d_chanEnv_proto.h"
#endif
#include "planner_cxx/plannerFunctions.hpp"
#include <list>
#include <string>
#include <iostream>

using namespace std;

Manipulation_JIDO::Manipulation_JIDO(p3d_rob * robotPt)//: // _capture(false)
{
  if(robotPt->nbCcCntrts != 1){
    cout << "Error: the number of arm  != 1" << endl;
    return;
  }
  _robotPt = robotPt;

  for(int i=0; i<6; i++){
    _QCUR[i]= 0.0;
    _QGOAL[i]= 0.0;
    _XCUR[i]= 0.0;
    _XGOAL[i]= 0.0;
  }
  _object = NULL;
  _capture = false;
  _cartesian = false;
  _objectGrabed = false;
}

Manipulation_JIDO::~Manipulation_JIDO(){
 
}

void Manipulation_JIDO::clear(){
  _robotPt = NULL;
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
      printf("%s: %d: genomSetArmQ(): robot is NULL.\n",__FILE__,__LINE__);
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
	  printf("%s: %d: genomSetArmQ(): q1 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q1,qmin,qmax);
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
	    printf("%s: %d: genomSetArmQ(): q2 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q2,qmin,qmax);
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
	      printf("%s: %d: genomSetArmQ(): q3 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q3,qmin,qmax);
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
		printf("%s: %d: genomSetArmQ(): q4 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q4,qmin,qmax);
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
		  printf("%s: %d: genomSetArmQ(): q5 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q5,qmin,qmax);
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
		    printf("%s: %d: genomSetArmQ(): q6 value (%f) is out of range (%f %f).\n",__FILE__,__LINE__,q6,qmin,qmax);
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
      printf("%s: %d: genomGetArmQ(): robot is NULL.\n",__FILE__,__LINE__);
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

  if(_robotPt==NULL)
    {
      printf("%s: %d: genomSetArmX(): robot is NULL.\n",__FILE__,__LINE__);
      return 1;
    }

  result= p3d_set_virtual_object_pose2(_robotPt, x, y, z, rx, ry, rz);

  if(result==TRUE) {
    return 0;
  }
  else {
    return 1;
  }
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
      printf("%s: %d: genomGetArmX(): robot is NULL.\n",__FILE__,__LINE__);
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
    printf("genomPrintConstraintInfo(): input p3d_rob is NULL.\n");
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
    printf("%s: %d: genomSetObjectPoseWrtEndEffector(): input robot is NULL.\n", __FILE__, __LINE__);
    return 1;
  }

  double x1, y1, z1, rx1, ry1, rz1;
  p3d_matrix4 T1, T2, T3;
  p3d_get_virtual_object_pose(_robotPt, T1);
  p3d_mat4PosReverseOrder(T2, x, y, z, rx, ry, rz);
  p3d_mat4Mult(T1, T2, T3);
  p3d_mat4ExtractPosReverseOrder(T3, &x1, &y1, &z1, &rx1, &ry1, &rz1);

  this->setArmX(x, y, z, rx, ry, rz);
  q = p3d_get_robot_config(_robotPt);

  return 0;
}

int Manipulation_JIDO::armPlanTask(MANIPULATION_TASK_TYPE_STR task,char* objectName, int lp[], Gb_q6 positions[],  int *nbPositions){

  configPt qi = NULL, qf = NULL;
  p3d_rob *cur_robot= NULL;
  p3d_traj *traj = NULL;
  int ntest=0;
  double gain = 0;

  cur_robot= XYZ_ENV->cur_robot;
  XYZ_ENV->cur_robot= _robotPt;

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
    p3d_copy_config_into(_robotPt, _robotPt->ROBOT_POS, &qi);
    /* Uptdate the Virual object for inverse kinematics */
    p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qi);
    p3d_set_and_update_this_robot_conf(_robotPt, qi);
    p3d_destroy_config(_robotPt, qi);
    qi = p3d_get_robot_config(_robotPt);
    //     g3d_draw_allwin_active();
    p3d_copy_config_into(_robotPt, qi, &_robotPt->ROBOT_POS);
    p3d_destroy_config(_robotPt, qi);
  }
  p3d_set_and_update_this_robot_conf(_robotPt, _robotPt->ROBOT_POS);
  
  switch(task) {
    case ARM_FREE:
      printf("plan for ARM_FREE task\n");
       if(_cartesian == true) {
	qf = p3d_alloc_config(_robotPt);
	p3d_copy_config_into(_robotPt, _robotPt->ROBOT_GOTO, &qf);
	p3d_update_virtual_object_config_for_pa10_6_arm_ik_constraint(_robotPt, qf);
	p3d_set_and_update_this_robot_conf(_robotPt, qf);
	p3d_destroy_config(_robotPt, qf);
	qf = p3d_get_robot_config(_robotPt);
	//     g3d_draw_allwin_active();
	p3d_copy_config_into(_robotPt, qf, &_robotPt->ROBOT_GOTO);
	p3d_destroy_config(_robotPt, qf);

	if(_robotPt->nbCcCntrts!=0) {
	  p3d_activateCntrt(_robotPt, _robotPt->ccCntrts[0]);
	}
       }
      if(p3d_equal_config(_robotPt, _robotPt->ROBOT_POS, _robotPt->ROBOT_GOTO)) {
	printf("genomArmGotoQ: Start and goal configurations are the same.\n");
	return 1;
      }
        /* RRT */
      if(this->computeRRT() != 0) {
	XYZ_ENV->cur_robot= cur_robot;
	return 1;
      }
      printf("End path optimization\n");
      break;
    case  ARM_PICK_GOTO:
    case  ARM_PICK_TAKE_TO_FREE:
    case  ARM_PICK_TAKE_TO_PLACE:
    case  ARM_PLACE_FROM_FREE:
    default:
      printf("wrong task\n");
      return 1;
  }


   /* COMPUTE THE SOFTMOTION TRAJECTORY */
  traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if(!traj) {
    printf("SoftMotion : ERREUR : no current traj\n");
    XYZ_ENV->cur_robot= cur_robot;
    return 1;
  }
  if(!traj || traj->nlp < 1) {
    printf("Optimization with softMotion not possible: current trajectory	contains one or zero local path\n");
    XYZ_ENV->cur_robot= cur_robot;
    return 1;
  }
  if(p3d_optim_traj_softMotion(traj, true, &gain, &ntest, lp, positions, nbPositions) == 1){
    printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
    XYZ_ENV->cur_robot= cur_robot;
    return 1;
  }

  ENV.setBool(Env::drawTraj, true);
  XYZ_ENV->cur_robot= cur_robot;
  g3d_draw_allwin_active();
  return 0;
}

int Manipulation_JIDO::cleanRoadmap(){
  if(_robotPt!=NULL) {
    XYZ_ENV->cur_robot= _robotPt;
    deleteAllGraphs();
    FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  } else {
    return 1;
  }
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
}


int Manipulation_JIDO::computeRRT(){
  int result;
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);

#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif

  ENV.setBool(Env::biDir,true);
  ENV.setInt(Env::NbTry, 100000);
  ENV.setInt(Env::MaxExpandNodeFail, 30000);
  ENV.setInt(Env::maxNodeCompco, 100000);
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
  result= p3d_specific_search("out.txt");
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


int Manipulation_JIDO::grabObject(configPt qGrab, char* objectName){
  this->releaseObject();
  deactivateCcCntrts(_robotPt, -1);

  configPt qi = NULL;
  qi = p3d_get_robot_config(_robotPt);
  p3d_copy_config_into(_robotPt, qi, &_robotPt->ROBOT_POS);
  p3d_destroy_config(_robotPt, qi);

  if(p3d_set_object_to_carry(_robotPt, objectName)!=0) {
   
    return 1;
  }
  p3d_grab_object2(_robotPt,0);
  FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  g3d_draw_allwin_active();
  return 0;
}

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

int Manipulation_JIDO::getFreeflyerPose(char *name, p3d_matrix4 pose){
  p3d_env *envPt = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  int nr,i, cont=0;
  p3d_rob * r = NULL;
  p3d_rob* robotPt=NULL;
  p3d_jnt *joint= NULL;

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
    printf("%s: %d: genomGetFreeflyerPose(): there is no robot named \"%s\".\n", __FILE__, __LINE__,name);
    return 1;
  }
  joint= robotPt->joints[1];
  if(joint->type!=P3D_FREEFLYER) {
    printf("%s: %d: genomGetFreeflyerPose(): first joint of robot \"%s\" should be of type P3D_FREEFLYER.\n", __FILE__, __LINE__,robotPt->name);
    return 1;
  }
  p3d_mat4Copy(joint->abs_pos, pose);
  return 0;
}




