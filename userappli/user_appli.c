#include "UserAppli-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"
#include "P3d-pkg.h"
#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

//Offset added to the Y axis of the grasp frames to compute the approach configuration of the robot
#define APROACH_OFFSET 0.120
//The robot lenght (length of the arms + lenght of the torso)
#define ROBOT_MAX_LENGTH 1.462
//Distance to prevent the platform localisation errors
double SAFETY_DIST = 0.05;
//Use Linear lp or R&s lp for the base
int USE_LIN = 0;

static int saveSpecifiedConfigInFile(configPt conf);
static int trueFunction(void);
static void switchObjectsTypes(void);
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject, int cntrtToActivate);
static double* getJntDofValue(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos);
static void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf);
static void unFixAllJointsExceptBaseAndObject(p3d_rob * robot);
static void getObjectBaseAttachMatrix(p3d_matrix4 base, p3d_matrix4 object, p3d_matrix4 result);
static p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);
static void setSafetyDistance(p3d_rob* robot, double dist);
static void p3d_fuseGraphs(p3d_rob* robot, p3d_graph* mainGraph, p3d_graph* subGraph);

static p3d_edge* p3d_getLpEdge(p3d_rob* robot, p3d_graph* graph, p3d_localpath* lp);

void openChainPlannerOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
//   p3d_set_NB_TRY(1000);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

void closedChainPlannerOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
  ENV.setInt(Env::NbTry,1000);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

void pathGraspOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
	CB_DiffusionMethod_obj(NULL, 0);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  ENV.setBool(Env::biDir,true);//bidirectionnal
  ENV.setInt(Env::NbTry,100000);
  ENV.setInt(Env::maxNodeCompco,1000);
}

void globalPlanner(void) {
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
}

void findPath(void) {
  p3d_specific_search((char*)"");
}

void deactivateHandsVsObjectCol(p3d_rob* robot) {
  for (int i = 0; i < robot->graspNbJoints; i++) {
    p3d_col_deactivate_obj_obj(robot->graspJoints[i]->o, robot->objectJnt->o);
		p3d_col_deactivate_obj_env(robot->graspJoints[i]->o);
  }
}

void activateHandsVsObjectCol(p3d_rob* robot) {
  for (int i = 0; i < robot->graspNbJoints; i++) {
    p3d_col_activate_obj_obj(robot->graspJoints[i]->o, robot->objectJnt->o);
		p3d_col_activate_obj_env(robot->graspJoints[i]->o);
  }
}

static int trueFunction(void) {
  g3d_draw_allwin_active();
  return TRUE;
}

void optimiseTrajectory(void) {
  p3d_set_NB_OPTIM(50);
  CB_start_optim_obj(NULL, 0);
}

void viewTraj(void) {
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  switchBBActivationForGrasp();
  g3d_show_tcur_rob(robot, trueFunction);
  switchBBActivationForGrasp();
}

FILE* file = NULL;
 static int saveConfigInFile(void){
   p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
   configPt q = p3d_get_robot_config(robot);
	 if (!q){
		 return false;
	 }
   for(int i = 6; i < robot->nb_dof - 6; i++){
     fprintf(file,"%.4lf \n", q[i]);
   }
   fprintf(file,"\n");
   return true;
 }

static int saveSpecifiedConfigInFile(configPt conf){
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
	if(!conf){
		return false;
	}
  for(int i = 6; i < robot->nb_dof - 6; i++){
    fprintf(file,"%.4lf \n", conf[i]);
  }
  fprintf(file,"\n");
  return true;
}

void saveTrajInFile(const char* fileName, p3d_traj* traj, int smallIntervals){
//  switchBBActivationForGrasp();
  if(traj != NULL){
    //openFile
    if (file == NULL){
      file = fopen(fileName, "a");
    }
		if(smallIntervals){
			fprintf(file, "#############  Justin Movement ###########\n\n");
      //traj->rob->tcur = traj;
			//p3d_set_env_graphic_dmax(4);
      //g3d_show_tcur_rob(traj->rob, saveConfigInFile); // in case of discretized configs
			double trajSize = traj->range_param;
			for(int i = 0; i < 50; i++){
				saveSpecifiedConfigInFile(p3d_config_at_param_along_traj(traj,trajSize*i/49));
			}
		}else{
			fprintf(file, "#############  Platform Movement ###########\n\n");
			p3d_localpath* curlp = traj->courbePt;
			if(curlp->type_lp == LINEAR){
				saveSpecifiedConfigInFile(curlp->specific.lin_data->q_init);
			}else if(curlp->type_lp == REEDS_SHEPP){
//				for(p3d_rs_data* rsData = curlp->specific.rs_data; rsData; rsData = rsData->next_rs){
					saveSpecifiedConfigInFile(curlp->specific.rs_data->q_init);
//				}
			}
			for(p3d_localpath* curlp = traj->courbePt; curlp; curlp = curlp->next_lp){
				if(curlp->type_lp == LINEAR){
					saveSpecifiedConfigInFile(curlp->specific.lin_data->q_end);
				}else if(curlp->type_lp == REEDS_SHEPP){
					for(p3d_rs_data* rsData = curlp->specific.rs_data; rsData; rsData = rsData->next_rs){
						saveSpecifiedConfigInFile(rsData->q_end);
					}
				}
			}
		}
    //close File
    fclose(file);
    file = NULL;
  }
//  switchBBActivationForGrasp();
}

void showConfig(configPt conf){
  p3d_set_and_update_robot_conf(conf);
  g3d_refresh_allwin_active();
  sleep(1);
}

static void switchObjectsTypes(void) {
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  for (int i = 0 ; i < robotPt->graspNbJoints; i++) {
    p3d_obj* jntObj = (robotPt->graspJoints[i])->o;
    for (int j = 0; j < jntObj->np; j++) {
      if (jntObj->pol[j]->TYPE == P3D_GRAPHIC) {
        jntObj->pol[j]->TYPE = P3D_GHOST;
      } else if (jntObj->pol[j]->TYPE == P3D_GHOST) {
        jntObj->pol[j]->TYPE = P3D_GRAPHIC;
      }
    }
  }
}

void switchBBActivationForGrasp(void) {
  switchObjectsTypes();
  p3d_col_stop();
  p3d_col_start(p3d_col_mode_kcd);
}

static void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf) {
  p3d_set_and_update_robot_conf(conf);
  for (int i = 0; i < robot->njoints + 1; i++) {
    p3d_jnt * joint = robot->joints[i];
    if (joint->type != P3D_BASE && joint->type != P3D_FIXED && joint != robot->objectJnt && joint != robot->baseJnt) {
      fixJoint(robot, joint, joint->jnt_mat);
    }
  }
  p3d_update_this_robot_pos(robot);
}

static void unFixAllJointsExceptBaseAndObject(p3d_rob * robot) {
  for (int i = 0; i < robot->njoints + 1; i++) {
    p3d_jnt * joint = robot->joints[i];
    if (joint->type != P3D_BASE && joint->type != P3D_FIXED && joint != robot->objectJnt && joint != robot->baseJnt) {
      unFixJoint(robot, joint);
    }
  }
}

void fixJoint(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos) {
  double * dVal = getJntDofValue(robot, joint, initPos);
  for (int i = 0; i < joint->dof_equiv_nbr; i++) {
    if (robot->isUserDof[joint->index_dof + i]) {
      p3d_jnt_set_dof(joint, i, dVal[i]);
      joint->dof_data[i].is_user = FALSE;
    }
  }
  MY_FREE(dVal, double, joint->dof_equiv_nbr);
}

static double* getJntDofValue(p3d_rob * robot, p3d_jnt * joint, p3d_matrix4 initPos){
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  p3d_mat4ExtractPosReverseOrder(initPos, &x, &y, &z, &rx, &ry, &rz);
  switch(joint->type){
    case P3D_BASE:{
      double * dVal = MY_ALLOC(double, 6);
      dVal[0] = x;
      dVal[1] = y;
      dVal[2] = z;
      dVal[3] = rx;
      dVal[4] = ry;
      dVal[5] = rz;
      return dVal;
    }
    case P3D_ROTATE:{
      double * dVal = MY_ALLOC(double, 1);
      dVal[0] = rz;
      return dVal;
    }
    case P3D_TRANSLATE:{
      double * dVal = MY_ALLOC(double, 1);
      dVal[0] = x;
      return dVal;
    }
    case P3D_PLAN:{
      double * dVal = MY_ALLOC(double, 3);
      dVal[0] = x;
      dVal[1] = y;
      dVal[2] = rz;
      return dVal;
    }
    case P3D_FREEFLYER:{
      double * dVal = MY_ALLOC(double, 6);
      dVal[0] = x;
      dVal[1] = y;
      dVal[2] = z;
      dVal[3] = rx;
      dVal[4] = ry;
      dVal[5] = rz;
      return dVal;
    }
    case P3D_FIXED:{
      return NULL;
    }
    case P3D_KNEE:{
      double * dVal = MY_ALLOC(double, 3);
      dVal[0] = rx;
      dVal[1] = ry;
      dVal[2] = rz;
      return dVal;
    }
    default:{
      return NULL;
    }
  }
}

void unFixJoint(p3d_rob * robot, p3d_jnt * joint) {
  for (int i = 0; i < joint->dof_equiv_nbr; i++) {
    if (robot->isUserDof[joint->index_dof + i]) {
      joint->dof_data[i].is_user = TRUE;
    }
  }
}

static p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt) {
  for (int i = 0; i < robot->cntrt_manager->ncntrts; i++) {
    //Check if the constraint is already created
    p3d_cntrt *cntrt = robot->cntrt_manager->cntrts[i];
    if (cntrt->npasjnts == 1 && cntrt->nactjnts == 1 && cntrt->pasjnts[0]->num == passiveJnt->num && cntrt->actjnts[0]->num == activeJnt->num) {
      return cntrt;
    }
  }
  return NULL;
}

static void setAndActivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt) {
  int passiveJntId[1] = {passiveJnt->num}, activeJntId[1] = {activeJnt->num};
  p3d_cntrt * cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
  //If the constraint is already created
  if (cntrt != NULL) {
    //Activate it
    p3d_activateCntrt(robot, cntrt);
  } else if (!p3d_constraint("p3d_fix_jnts_relpos", -1, passiveJntId, -1, activeJntId, -1, NULL, -1, NULL, -1, 1)) {
    printf("Error in creatin the p3d_fix_jnts_relpos÷\n");
  } else {
    cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
  }
  //set the attach Matrix
  getObjectBaseAttachMatrix(activeJnt->abs_pos, passiveJnt->abs_pos, cntrt->Tatt);
  return;
}

static void desactivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt) {
  p3d_cntrt * cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
  p3d_desactivateCntrt(robot, cntrt);
}

static void getObjectBaseAttachMatrix(p3d_matrix4 base, p3d_matrix4 object, p3d_matrix4 result) {
  p3d_matrix4 tmp;
  p3d_matInvertXform(base, tmp);
  p3d_mat4Mult(tmp, object, result);
}

static void adaptClosedChainConfigToBasePos(p3d_rob *robot, p3d_matrix4 base, configPt refConf) {
  p3d_matrix4 relMatrix, newObjectPos, basePos;
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
	p3d_mat4Copy(base, basePos);
  //On met le robot dans la configuration passee dans le P3D afin de trouver la matrice de transformation entre la base et l'objet.
  p3d_set_and_update_robot_conf(refConf);
  getObjectBaseAttachMatrix(robot->baseJnt->abs_pos, robot->objectJnt->abs_pos, relMatrix);
  //Pour la configuration courante de la base, la position de l'objet est base * relMatrix.
  p3d_mat4Mult(basePos, relMatrix, newObjectPos);
  p3d_mat4ExtractPosReverseOrder(newObjectPos, &x, &y, &z, &rx, &ry, &rz);
  p3d_jnt_set_dof(robot->objectJnt, 0, x - robot->objectJnt->pos0[0][3]);
  p3d_jnt_set_dof(robot->objectJnt, 1, y - robot->objectJnt->pos0[1][3]);
  p3d_jnt_set_dof(robot->objectJnt, 2, z - robot->objectJnt->pos0[2][3]);
  p3d_jnt_set_dof(robot->objectJnt, 3, rx);
  p3d_jnt_set_dof(robot->objectJnt, 4, ry);
  p3d_jnt_set_dof(robot->objectJnt, 5, rz);
  //On change la configuration du joint de la base.
  p3d_mat4ExtractPosReverseOrder(basePos, &x, &y, &z, &rx, &ry, &rz);
  p3d_jnt_set_dof(robot->baseJnt, 0, x);
  p3d_jnt_set_dof(robot->baseJnt, 1, y);
  p3d_jnt_set_dof(robot->baseJnt, 2, rz);
  //Test de validitee et collision.
  if (p3d_update_this_robot_pos_with_partial_reshoot(robot)) {
    if (!p3d_col_test()) {
      //Sauvegarde de la configuration.
      p3d_get_robot_config_into(robot, &refConf);
    }
  }else{
		configPt tmp = setTwoArmsRobotGraspPosWithoutBase(robot, robot->objectJnt->abs_pos, robot->ccCntrts[0]->Tatt, robot->ccCntrts[1]->Tatt, -1);
		if(tmp != NULL){
			//Sauvegarde de la configuration.
			p3d_copy_config_into(robot, tmp, &refConf);
			p3d_destroy_config(robot, tmp);
		}
	}
}


void disableAutoCol(p3d_rob* robot){
  p3d_col_deactivate_rob(robot);
}

void enableAutoCol(p3d_rob* robot){
  p3d_col_activate_rob(robot);
}

static void setSafetyDistance(p3d_rob* robot, double dist){
  if(dist != 0){
    disableAutoCol(robot);
  }else{
    enableAutoCol(robot);
  }
  p3d_set_env_object_tolerance(dist);
  p3d_col_set_tolerance(dist);
}

/** ////////// Setters /////////////*/

void setLinearLp(int useLinear){
  USE_LIN = useLinear;
}

void setSafetyDistance(double safetyDistance){
  SAFETY_DIST = safetyDistance;
}

/** ////////// Fin Setters /////////////*/

/** ////////// Fonctions Principales /////////////*/

void computeOfflineOpenChain(p3d_rob* robot, p3d_matrix4 objectInitPos){
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
  CB_del_param_obj(NULL, 0);
  deactivateCcCntrts(robot, -1);
  fixAllJointsExceptBaseAndObject(robot, robot->openChainConf);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  p3d_update_this_robot_pos(robot);
  configPt conf = p3d_get_robot_config(robot);
  print_config(robot, conf);
  openChainPlannerOptions();
  globalPlanner();
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
}

void computeOfflineClosedChain(p3d_rob* robot, p3d_matrix4 objectInitPos){
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  switchBBActivationForGrasp();
}


void pickAndMoveObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2) {
  configPt graspConf, approachConf, finalConf;
  ChronoOn();
  setTwoArmsRobotGraspAndApproachPosWithHold(robot, objectInitPos, att1, att2, &graspConf, &approachConf);
  finalConf = setTwoArmsRobotGraspPosWithHold(robot, objectGotoPos, att1, att2, -1);
	if(finalConf == NULL){
		printf("No position found\n");
		return;
	}
//   showConfig(approachConf);
//   showConfig(graspConf);
//   showConfig(finalConf);
  ChronoPrint("Positions");
  pickAndMoveObjectByConf(robot, objectInitPos, approachConf, graspConf, finalConf);
  ChronoPrint("Total");
  ChronoOff();
}

void pickAndMoveObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf, configPt finalConf) {

  p3d_traj* approachTraj = pickObjectByConf(robot, objectInitPos, p3d_copy_config(robot, approachConf));
  ChronoPrint("Pick");
  p3d_traj* graspTraj = graspObjectByConf(robot, objectInitPos, p3d_copy_config(robot, approachConf), p3d_copy_config(robot, graspConf));
  ChronoPrint("Grasp");
  p3d_concat_traj(approachTraj, graspTraj);
  p3d_traj* carryTraj = moveObjectByConf(robot, graspConf, p3d_copy_config(robot, finalConf));
  ChronoPrint("Carry");
  p3d_concat_traj(approachTraj, carryTraj);
  p3d_destroy_config(robot, approachConf);
  p3d_destroy_config(robot, graspConf);
  p3d_destroy_config(robot, finalConf);
}

//Moving Justin Only
p3d_traj* gotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2){
	p3d_set_and_update_robot_conf(robot->ROBOT_POS);
	configPt conf = setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, -1);
	if(conf == NULL){
		printf("No position found\n");
		return NULL;
	}
	return gotoObjectByConf(robot, objectStartPos, conf);
}
p3d_traj* gotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf){
	p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
  deactivateCcCntrts(robot, -1);
	p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
	p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
	
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->objectJnt, objectStartPos);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, conf);
//	showConfig(robot->ROBOT_POS);
//	showConfig(robot->ROBOT_GOTO);
  pathGraspOptions();
  findPath();
  optimiseTrajectory();
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
	p3d_col_env_set_traj_method(testcolMethod);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
}

p3d_traj* carryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
	int cntrtToActivate = -1;
	if(att1[0][0] == att1[0][1] == att1[0][2] == 0){//null attach frame
		cntrtToActivate = 0;
	}else if(att2[0][0] == att2[0][1] == att2[0][2] == 0){
		cntrtToActivate  = 1;
	}
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
	configPt conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectGotoPos, att1, att2, cntrtToActivate);
	if(conf == NULL){
		printf("No position found\n");
		return NULL;
	}
	return carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
}
p3d_traj* carryObjectByConf(p3d_rob * robot,  p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate){
	deactivateHandsVsObjectCol(robot);
	//   Linear
  p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
  activateCcCntrts(robot, cntrtToActivate);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  unFixJoint(robot, robot->objectJnt);
  unFixAllJointsExceptBaseAndObject(robot);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  pathGraspOptions();
  findPath();
  optimiseTrajectory();
	activateHandsVsObjectCol(robot);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
}

//Moving Base Only
p3d_traj* platformGotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2){
	//try to reach the object without moving the base.
	p3d_set_and_update_this_robot_conf_without_cntrt(robot, robot->ROBOT_POS);
	if(!setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, -1)){
		configPt conf = setTwoArmsRobotGraspApproachPosWithHold(robot, objectStartPos, att1, att2);
		if(conf == NULL){
			printf("no position found\n");
			return NULL;
		}
		return platformGotoObjectByConf(robot, objectStartPos, conf);
	}else{
		printf("I can reach the object without moving the base\n");
		return NULL;
	}
}
p3d_traj* platformGotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf){
	//Plannification des bras pour aller en position de trasfert
	//Trouver la configuration de transfert a partir de la config initiale.
	deactivateCcCntrts(robot, -1);
	configPt transfertConf = setBodyConfigForBaseMovement(robot, robot->ROBOT_POS, robot->openChainConf);
  p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  fixJoint(robot, robot->objectJnt, objectStartPos);
  p3d_update_this_robot_pos(robot);
  p3d_copy_config_into(robot, transfertConf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, transfertConf);
  pathGraspOptions();	
  findPath();
  optimiseTrajectory();
  p3d_traj* justinTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	
	//Plannification de la base
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
	p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
	p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
  fixAllJointsExceptBaseAndObject(robot, robot->openChainConf);
  fixJoint(robot, robot->objectJnt, objectStartPos);
	unFixJoint(robot, robot->baseJnt);
  p3d_update_this_robot_pos(robot);
  configPt newConf = setBodyConfigForBaseMovement(robot, conf, robot->openChainConf);
	showConfig(newConf);
	p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, newConf, &(robot->ROBOT_GOTO));
  setSafetyDistance(robot, (double)SAFETY_DIST);
  p3d_col_deactivate_obj_env(robot->objectJnt->o);
	pathGraspOptions();
  findPath();
  optimiseTrajectory();
  setSafetyDistance(robot, 0);
  p3d_col_activate_obj_env(robot->objectJnt->o);
	p3d_col_env_set_traj_method(testcolMethod);
  p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if (justinTraj) {
    p3d_concat_traj(justinTraj, baseTraj);
  }
	unFixAllJointsExceptBaseAndObject(robot);
  return justinTraj;
}

p3d_traj* platformCarryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
	int cntrtToActivate = -1;
	if(att1[0][0] == att1[0][1] == att1[0][2] == 0){//null attach frame
		cntrtToActivate = 0;
	}else if(att2[0][0] == att2[0][1] == att2[0][2] == 0){
		cntrtToActivate  = 1;
	}
	configPt conf = setTwoArmsRobotGraspPosWithHold(robot, objectGotoPos, att1, att2, cntrtToActivate);
	if(conf == NULL){
		printf("No position found\n");
		return NULL;
	}
	return platformCarryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
}
p3d_traj* platformCarryObjectByConf(p3d_rob * robot,  p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate){
	//Extract traj
	//   Linear
	deactivateHandsVsObjectCol(robot);
  p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
  activateCcCntrts(robot, -1);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  unFixJoint(robot, robot->objectJnt);
  unFixAllJointsExceptBaseAndObject(robot);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
  pathGraspOptions();
  findPath();
  optimiseTrajectory();
  p3d_traj* extractTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	
	//base Moving
	p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
	p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
	if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
  CB_del_param_obj(NULL, 0);
  p3d_set_and_update_robot_conf(conf);
  p3d_copy_config_into(robot, robot->closedChainConf, &adaptedConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  unFixJoint(robot, robot->baseJnt);
  fixAllJointsExceptBaseAndObject(robot, robot->closedChainConf);
  deactivateCcCntrts(robot, cntrtToActivate);
  setAndActivateTwoJointsFixCntrt(robot, robot->objectJnt, robot->baseJnt);
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
	//   showConfig(robot->ROBOT_POS);
	//   showConfig(robot->ROBOT_GOTO);
  p3d_destroy_config(robot, adaptedConf);
  setSafetyDistance(robot, (double)SAFETY_DIST);
  closedChainPlannerOptions();
  findPath();
  optimiseTrajectory();
  setSafetyDistance(robot, 0);
	activateHandsVsObjectCol(robot);
	p3d_col_env_set_traj_method(testcolMethod);
	p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	if (extractTraj) {
    p3d_concat_traj(extractTraj, baseTraj);
  }
  return extractTraj;
}

traj* carryObject(p3d_rob* robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
	int cntrtToActivate = -1;
	if(att1[0][0] == att1[0][1] == att1[0][2] == 0){//null attach frame
		cntrtToActivate = 0;
	}else if(att2[0][0] == att2[0][1] == att2[0][2] == 0){
		cntrtToActivate  = 1;
	}
	//try to reach the object without moving the base.
	p3d_set_and_update_robot_conf(robot->ROBOT_POS);
	configPt conf = NULL;
	if(!(conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectGotoPos, att1, att2, cntrtToActivate))){
		p3d_traj* carry = platformCarryObjectByMat(robot, objectGotoPos, att1, att2);
		p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
		p3d_traj* deposit = carryObjectByMat(robot, objectGotoPos, att1, att2);
		if (carry) {
			p3d_concat_traj(carry, deposit);
		}
		return carry;
	}else{
		return carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
	}	
}



void pickObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2) {
  configPt approachConf = setTwoArmsRobotGraspApproachPosWithHold(robot, objectInitPos, att1, att2);
	if(approachConf == NULL){
		printf("No position found\n");
		return;
	}
  showConfig(approachConf);
  pickObjectByConf(robot, objectInitPos, approachConf);
}

p3d_traj* pickObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf) {
	//Plannification des bras pour aller en position de trasfert
	//Trouver la configuration de transfert a partir de la config initiale.
	deactivateCcCntrts(robot, -1);
	configPt transfertConf = setBodyConfigForBaseMovement(robot, robot->ROBOT_POS, robot->openChainConf);
  p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
//  p3d_set_and_update_robot_conf(transfertConf);
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  p3d_update_this_robot_pos(robot);
  p3d_copy_config_into(robot, transfertConf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, transfertConf);
  pathGraspOptions();
  findPath();
	
  optimiseTrajectory();
  p3d_traj* JustinTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	
//Plannification de la base
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
  fixAllJointsExceptBaseAndObject(robot, robot->openChainConf);
  fixJoint(robot, robot->objectJnt, objectInitPos);
	unFixJoint(robot, robot->baseJnt);
  p3d_update_this_robot_pos(robot);
  configPt conf = setBodyConfigForBaseMovement(robot, approachConf, robot->openChainConf);
	showConfig(conf);
	p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  setSafetyDistance(robot, (double)SAFETY_DIST);
  p3d_col_deactivate_obj_env(robot->objectJnt->o);
	showConfig(robot->ROBOT_POS);
	showConfig(robot->ROBOT_GOTO);	
//  openChainPlannerOptions();
	pathGraspOptions();
  findPath();
  optimiseTrajectory();
  setSafetyDistance(robot, 0);
  p3d_col_activate_obj_env(robot->objectJnt->o);
  p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
//
////Plannification des bras
//  p3d_local_set_planner((p3d_localplanner_type)1);
//  CB_del_param_obj(NULL, 0);
//  p3d_set_and_update_robot_conf(approachConf);
//  unFixAllJointsExceptBaseAndObject(robot);
//  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
//  fixJoint(robot, robot->objectJnt, objectInitPos);
//  p3d_update_this_robot_pos(robot);
//  p3d_copy_config_into(robot, conf, &(robot->ROBOT_POS));
//  p3d_copy_config_into(robot, approachConf, &(robot->ROBOT_GOTO));
//  p3d_destroy_config(robot, approachConf);
//  p3d_destroy_config(robot, conf);
//  pathGraspOptions();
//  findPath();
//
//  optimiseTrajectory();
//  p3d_traj* JustinTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if (JustinTraj) {
    p3d_concat_traj(JustinTraj, baseTraj);
  }
	unFixAllJointsExceptBaseAndObject(robot);//
//  unFixJoint(robot, robot->objectJnt);
//  unFixJoint(robot, robot->baseJnt);
  return JustinTraj;
}

void moveObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2) {
  configPt finalConf = setTwoArmsRobotGraspPosWithHold(robot, objectGotoPos, att1, att2, -1);
	if(finalConf == NULL){
		printf("No position found\n");
		return;
	}
  showConfig(finalConf);
  moveObjectByConf(robot, robot->ROBOT_POS, finalConf);
}
p3d_traj* moveObjectByConf(p3d_rob * robot, configPt initConf, configPt finalConf) {
  deactivateHandsVsObjectCol(robot);

//Extract traj
//   Linear
  p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
  activateCcCntrts(robot, -1);
  p3d_set_and_update_robot_conf(initConf);
  configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  unFixJoint(robot, robot->objectJnt);
  unFixAllJointsExceptBaseAndObject(robot);
  p3d_set_and_update_robot_conf(initConf);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, initConf, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
	showConfig(robot->ROBOT_POS);
	if(p3d_col_test()){
		return NULL;
	}
	showConfig(robot->ROBOT_GOTO);
  pathGraspOptions();
  findPath();
  optimiseTrajectory();
  p3d_traj* extractTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);

//trasferTraj
// R&S+linear
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
  CB_del_param_obj(NULL, 0);
  p3d_set_and_update_robot_conf(finalConf);
  p3d_copy_config_into(robot, robot->closedChainConf, &adaptedConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  unFixJoint(robot, robot->baseJnt);
  fixAllJointsExceptBaseAndObject(robot, robot->closedChainConf);
  deactivateCcCntrts(robot, -1);
  setAndActivateTwoJointsFixCntrt(robot, robot->objectJnt, robot->baseJnt);
  p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
//   showConfig(robot->ROBOT_POS);
//   showConfig(robot->ROBOT_GOTO);
  p3d_destroy_config(robot, adaptedConf);
  setSafetyDistance(robot, (double)SAFETY_DIST);
  closedChainPlannerOptions();
  findPath();
  optimiseTrajectory();
  setSafetyDistance(robot, 0);
  deactivateHandsVsObjectCol(robot);
  p3d_traj* trasfertTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if (extractTraj) {
    p3d_concat_traj(extractTraj, trasfertTraj);
  }
//Depose Traj
  //   Linear
  p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
  activateCcCntrts(robot, -1);
  desactivateTwoJointsFixCntrt(robot, robot->objectJnt, robot->baseJnt);
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_update_this_robot_pos(robot);
  p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, finalConf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, finalConf);
//   showConfig(robot->ROBOT_POS);
//   showConfig(robot->ROBOT_GOTO);
  pathGraspOptions();
  findPath();
  optimiseTrajectory();
  p3d_traj* deposeTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if (extractTraj) {
    p3d_concat_traj(extractTraj, deposeTraj);
  }
  unFixJoint(robot, robot->baseJnt);
  deactivateCcCntrts(robot, -1);
  return extractTraj;
}

void graspObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2) {
  configPt graspConf, approachConf;
//   setTwoArmsRobotGraspAndApproachPos(robot, objectInitPos, att1, att2, &graspConf, &approachConf);
  //setTwoArmsRobotGraspAndApproachPosWithHold(robot, objectInitPos, att1, att2, &graspConf, &approachConf);
	graspConf = setTwoArmsRobotGraspApproachPosWithHold(robot, objectInitPos, att1, att2);
	if(graspConf == NULL){
		printf("No position found\n");
		return;
	}
  showConfig(graspConf);
  graspObjectByConf(robot, objectInitPos, approachConf, graspConf);
}

p3d_traj* graspObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf) {
  p3d_local_set_planner((p3d_localplanner_type)1);
  CB_del_param_obj(NULL, 0);
  deactivateCcCntrts(robot, -1);
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
	approachConf = setBodyConfigForBaseMovement(robot, graspConf, robot->openChainConf);
  p3d_copy_config_into(robot, approachConf, &(robot->ROBOT_GOTO));
  p3d_copy_config_into(robot, graspConf, &(robot->ROBOT_POS));
  p3d_destroy_config(robot, approachConf);
  p3d_destroy_config(robot, graspConf);
	showConfig(robot->ROBOT_POS);
	showConfig(robot->ROBOT_GOTO);
//  switchBBActivationForGrasp();
  pathGraspOptions();
//   p3d_specificSuperGraphLearn();
  findPath();
  optimiseTrajectory();
//  switchBBActivationForGrasp();
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
  return p3d_invert_traj(robot, (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ));
}

/** //////////// Fin Fonctions Principales /////////////*/

/** //////////// Compute Robot Pos /////////////*/

/**
 * @brief Get the robot grasp approach an the robot grasp configuration given an attach matrix for each arm and the object position. Verify also the hold configuration for the transfert
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @param graspConf the retruned grasp config of the robot
 * @param approachConf the retruned approach config of the robot
 */
void setTwoArmsRobotGraspAndApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf) {
  if (robot->nbCcCntrts != 2) {
    printf("There is more than 2 arms\n");
    return;
  }

  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  switchBBActivationForGrasp();
  do{
    do{
      p3d_col_activate_obj_env(robot->objectJnt->o);
      setSafetyDistance(robot, 0);
      *graspConf = getRobotGraspConf(robot, objectPos, att, TRUE, -1);
			if(graspConf == NULL){
				return;
			}

      setSafetyDistance(robot, (double)SAFETY_DIST);
      p3d_col_deactivate_obj_env(robot->objectJnt->o);
      deactivateCcCntrts(robot, -1);
      configPt conf = setBodyConfigForBaseMovement(robot, *graspConf, robot->openChainConf);
      p3d_set_and_update_robot_conf(conf);
      p3d_destroy_config(robot, conf);
    }while (p3d_col_test());
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  }while (p3d_col_test());
  p3d_col_activate_obj_env(robot->objectJnt->o);
  setSafetyDistance(robot, 0);
  /*Shift attach position over wrist X axis*/
  att[0][1][3] += -APROACH_OFFSET;
  att[1][1][3] += APROACH_OFFSET;
  p3d_set_and_update_robot_conf(*graspConf);
  *approachConf = getRobotGraspConf(robot, objectPos, att, FALSE, -1);
  MY_FREE(att, p3d_matrix4, 2);
  switchBBActivationForGrasp();
  return;
}

/**
 * @brief Get the robot grasp approach configuration given an attach matrix for each arm and the object position. Verify also the hold configuration for the transfert
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt setTwoArmsRobotGraspApproachPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2) {
  if (robot->nbCcCntrts != 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  /*Shift attach position over wrist X axis*/
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
//  att[0][1][3] += -APROACH_OFFSET;
//  att[1][1][3] += APROACH_OFFSET;
  configPt q;
//  switchBBActivationForGrasp();
  do{
    do{
      p3d_col_activate_obj_env(robot->objectJnt->o);
      setSafetyDistance(robot, 0);
      q = getRobotGraspConf(robot, objectPos, att, TRUE, -1);
			if(q == NULL){
				return NULL;
			}
      setSafetyDistance(robot, (double)SAFETY_DIST);
      p3d_col_deactivate_obj_env(robot->objectJnt->o);
      deactivateCcCntrts(robot, -1);
      configPt conf = setBodyConfigForBaseMovement(robot, q, robot->openChainConf);
      p3d_set_and_update_robot_conf(conf);
      p3d_destroy_config(robot, conf);
    }while (p3d_col_test());
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  }while (p3d_col_test());
  MY_FREE(att, p3d_matrix4, 2);
	setSafetyDistance(robot, 0);
//  switchBBActivationForGrasp();
  return q;
}

/**
 * @brief Get the robot grasp configuration given an attach matrix for each arm and the object position. Verify also the hold configuration for the transfert
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt setTwoArmsRobotGraspPosWithHold(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) {
  if (robot->nbCcCntrts != 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q;
//  switchBBActivationForGrasp();
	deactivateHandsVsObjectCol(robot);
  do{
    p3d_col_activate_obj_env(robot->objectJnt->o);
    setSafetyDistance(robot, 0);
    q = getRobotGraspConf(robot, objectPos, att, TRUE, -1);
		if(q == NULL){
			activateHandsVsObjectCol(robot);
			return NULL;
		}
    setSafetyDistance(robot, (double)SAFETY_DIST);
    p3d_col_deactivate_obj_env(robot->objectJnt->o);
    configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
    adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
    p3d_set_and_update_robot_conf(adaptedConf);
    p3d_destroy_config(robot, adaptedConf);
  }while (p3d_col_test());
  MY_FREE(att, p3d_matrix4, 2);
	 setSafetyDistance(robot, 0);
//  switchBBActivationForGrasp();
	activateHandsVsObjectCol(robot);
  return q;
}

/**
 * @brief Get the robot grasp without reshooting the base configuration given an attach matrix for each arm and the object position
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt setTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) {
  if (robot->nbCcCntrts != 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
	deactivateHandsVsObjectCol(robot);
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att, FALSE, cntrtToActivate);
	activateHandsVsObjectCol(robot);
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}

configPt setTwoArmsRobotGraspApproachPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, int cntrtToActivate) {
  if (robot->nbCcCntrts != 2) {
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att, FALSE, cntrtToActivate);
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}

/**
 * @brief Get the robot grasp configuration given the attach matrix
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att the attach matrix. The number of matrix is robot->nbCcCntrts
 * @return the robot config
 */
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject, int cntrtToActivate) {
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  configPt q;
  p3d_mat4ExtractPosReverseOrder(objectPos, &x, &y, &z, &rx, &ry, &rz);
  //Backup and set the attach matrix
  p3d_matrix4 bakTatt[robot->nbCcCntrts];
  for (int i = 0; i < robot->nbCcCntrts; i++) {
    p3d_mat4Copy(robot->ccCntrts[i]->Tatt, bakTatt[i]);
    p3d_mat4Copy(att[i], robot->ccCntrts[i]->Tatt);
  }
  q = p3d_getRobotBaseConfigAroundTheObject(robot, robot->baseJnt, robot->objectJnt, x, y, z, rx, ry, rz, -1, ROBOT_MAX_LENGTH, shootObject, cntrtToActivate);
  //Restore the attach matrix
  for (int i = 0; i < robot->nbCcCntrts; i++) {
    p3d_mat4Copy(bakTatt[i], robot->ccCntrts[i]->Tatt);
  }
  return q;
}

/** //////////// Fin Compute Robot Pos /////////////*/


static void p3d_globalPDRSequence(void){
  int nbTry = ENV.getInt(Env::NbTry);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
  CB_global_search_obj(NULL,0);
  p3d_set_cycles(1);
  p3d_set_is_visibility_discreet(1);
  p3d_set_test_reductib(1);
  ENV.setInt(Env::NbTry,(int)(nbTry/20));
  CB_global_search_obj(NULL,0);
  ENV.setInt(Env::NbTry,nbTry);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

#ifdef MULTIGRAPH

void p3d_specificSuperGraphLearn(void) {
  double *arraytimes = MY_ALLOC(double, p3d_get_NB_specific());
  int nfail = 0;
  configPt qs = NULL, qg = NULL, qStart = NULL, qGoal = NULL;
  double tu = 0.0, ts = 0.0, mgTime = 0.0, gTime = 0.0;
  p3d_rob *robotPt = (p3d_rob *)(p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_graph * graph = NULL, *tmp = NULL;
  p3d_flatSuperGraphNode *startNode = NULL, *goalNode = NULL;

  p3d_SetDiffuStoppedByWeight(0);
  p3d_SetStopValue(FALSE);

  qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  if (ENV.getBool(Env::expandToGoal) == true) {
    qg = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
  }
  if (!XYZ_GRAPH) {
    graph = p3d_create_graph();
  } else {
    graph = XYZ_GRAPH;
  }

  //compute the specific planner for each part of the system
  for (int j = 0; j < robotPt->mg->nbGraphs; j++) {
    p3d_set_user_drawnjnt(robotPt->mg->mgJoints[j]->joints[robotPt->mg->mgJoints[j]->nbJoints - 1]);
    tmp = p3d_setMultiGraphAndActiveDof(robotPt, j);
    qStart = p3d_copy_config(robotPt, qs);
    if (ENV.getBool(Env::expandToGoal) == true) {
      qGoal = p3d_copy_config(robotPt, qg);
    }
    p3d_loopSpecificLearn(robotPt, qStart, qGoal, (char*)"", 0, arraytimes, &nfail);
    if (p3d_graph_to_traj(robotPt)) {
      g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
    }
    gTime += robotPt->mg->graphs[j]->time;
  }
  //create the flat super Graph
  if (graph->nnode == 0) {
    robotPt->GRAPH = NULL;
    XYZ_GRAPH = NULL;
    ChronoOn();
    p3d_setAllDofActive(robotPt);
    p3d_flatMultiGraph(robotPt, 1);
    p3d_setAllDofActive(robotPt);
    ChronoTimes(&tu, &ts);
    ChronoOff();
    mgTime += tu;
    //check if there is solution in this graph or not.
    startNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qs);
    goalNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qg);
    robotPt->mg->fsg->search_start = startNode;
    robotPt->mg->fsg->search_goal = goalNode;
    if (!p3d_graph_search(robotPt->mg->fsg, p3d_mgHeurist, p3d_valid, p3d_mgEnd, MGGRAPH)) {
      //continuer la recherche
      ChronoOn();
      p3d_delFlatSuperGraph(robotPt, robotPt->mg->fsg);
      p3d_flatMultiGraph(robotPt, 0);
      p3d_setAllDofActive(robotPt);
      ChronoTimes(&tu, &ts);
      ChronoOff();
      mgTime += tu;
      startNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qs);
      goalNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qg);
      robotPt->mg->fsg->search_start = startNode;
      robotPt->mg->fsg->search_goal = goalNode;
      if (!p3d_graph_search(robotPt->mg->fsg, p3d_mgHeurist, p3d_valid, p3d_mgEnd, MGGRAPH)) {
        p3d_doIncrementalConstruction(1);
        printf("Continue search\n");
      }
    }
  } else {
    ChronoOn();
    p3d_setAllDofActive(robotPt);
    for (int j = 0; j < robotPt->mg->nbGraphs; j++) {
      p3d_fillFlatMultiGraph(robotPt, NULL, NULL, j, 2);
    }
    p3d_setAllDofActive(robotPt);
    ChronoTimes(&tu, &ts);
    ChronoOff();
    mgTime += tu;
    startNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qs);
    goalNode = p3d_isConfigInSuperGraph(robotPt, robotPt->mg->fsg, qg);
    if(robotPt->mg->fsg){
      robotPt->mg->fsg->search_start = startNode;
      robotPt->mg->fsg->search_goal = goalNode;
      //check if there is solution in this graph or not.
      if (!p3d_graph_search(robotPt->mg->fsg, p3d_mgHeurist, p3d_valid, p3d_mgEnd, MGGRAPH)) {
        p3d_doIncrementalConstruction(1);
        printf("Continue search\n");
      }
    }
  }
  if (p3d_doIncrementalConstruction(-1)) {
    p3d_set_multiGraph(TRUE);
    p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
    p3d_setAllDofActive(robotPt);
    p3d_doIncrementalConstruction(0);
  }

  p3d_convertFsgToGraph(graph, robotPt->mg->fsg);
  XYZ_GRAPH = graph;
  robotPt->GRAPH = graph;
  graph->time = gTime;
  graph->mgTime = mgTime;
  p3d_print_info_graph(graph);
  p3d_addStartAndGoalNodeToGraph(qs, qg, NULL, NULL, graph, robotPt);
  if(p3d_graph_to_traj(robotPt)){
    printf("A path is found\n");
    g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
  }
}

void p3d_globalSuperGraphLearn(void){
  p3d_graph * final = NULL;
  double graphsTime = 0.0, tu = 0.0, ts = 0.0;
  int graphsNodes = 0, graphsEdges = 0;
  p3d_rob * robotPt = XYZ_ROBOT;
  int nbNodes[3] = {20,20,20};

  if (!XYZ_GRAPH) final = p3d_create_graph();
  else           final = XYZ_GRAPH;

  for(int j = 0; j < robotPt->mg->nbGraphs; j++){
    p3d_set_user_drawnjnt(robotPt->mg->mgJoints[j]->joints[robotPt->mg->mgJoints[j]->nbJoints - 1]);
    p3d_set_NB_NODES(nbNodes[j]);
    p3d_setMultiGraphAndActiveDof(robotPt, j);
//     p3d_globalPDRSequence();
    CB_global_search_obj(NULL,0);
//     CB_print_obj(NULL,0);
    graphsTime += XYZ_GRAPH->time;
    graphsNodes += XYZ_GRAPH->nnode;
    graphsEdges += XYZ_GRAPH->nedge;
    if(STAT){
      setTotalCountVar(XYZ_GRAPH);
      mergeStat(XYZ_GRAPH->stat, final->stat);
    }
  }

  ChronoOn();
  p3d_setAllDofActive(robotPt);
  p3d_flatMultiGraph(robotPt, 0);
  p3d_setAllDofActive(robotPt);
  p3d_convertFsgToGraph(final, robotPt->mg->fsg);
  ChronoTimes(&tu, &ts);
  ChronoOff();
  final->mgTime += tu;
  final->time += graphsTime;
  XYZ_GRAPH = final;
  robotPt->GRAPH = final;
  p3d_set_user_drawnjnt(-1);
  MY_ALLOC_INFO("After p3d_learn");
  p3d_print_info_graph(final);
  if (STAT){
    final->stat->mgNodes = final->nnode;
    final->nnode = final->stat->planConfAdd;
    final->stat->mgEdges = final->nedge/2;
    final->nedge = final->stat->planEdge*2;
    final->stat->mgTime = final->mgTime;
  }
  printf("Graphs Nodes = %d\n", graphsNodes);
  printf("Graphs Edges = %d\n", graphsEdges);
  printf("SG merge time = %f\n", final->mgTime);
}
#endif

static void p3d_reset_graph(p3d_graph * graph) {
  p3d_del_graph(graph);
  p3d_reinit_array_exhausted_nodes();
  MY_ALLOC_INFO("Apres destruction du graphe");
}

void p3d_computeTests(void){
  printf("===========================================\n");
  printf("GLOBAL\n");
  printf("===========================================\n");
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  for(int i = 0; i < 1; i++) {
    p3d_reset_graph(XYZ_GRAPH);
    XYZ_GRAPH = NULL;
    p3d_resetMultiGraph(XYZ_ROBOT);
    printf("##########  TEST N %d  ############\n", i+1);
//     CB_global_search_obj(NULL,0);
    p3d_globalSuperGraphLearn();
//         p3d_globalPDRSequence();
    setTotalCountVar(XYZ_GRAPH);
    mergeStat(XYZ_GRAPH->stat, XYZ_ENV->stat);
    printStatsGraph(XYZ_GRAPH->stat, 1);
  }
  printStatsEnv(XYZ_ENV->stat, 1);
}
#ifdef DPG
//Ne traite pas le cas ou c'est le debut ou la fin du lp qui sont en collision. C'est du changement statique de l'environement. Ne marche pas lors de l'execution. True if a traj is found false otherwise
int checkForCollidingLpAlongPath(void) {
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_traj *traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  p3d_graph* mainGraph = XYZ_GRAPH;
  if (!traj) {
    return FALSE;
  }
  p3d_localpath *cur = traj->courbePt;
  int ntest = 0;
  double dist = 0;
  double nodeDist = 0.0;
  double trajLength =  p3d_compute_traj_length(traj);
  int counter = 0;
  int counterMax = 20;
  bool optimTrajInCollision = false, graphTrajInCollision = false;

  if(traj->isOptimized){//is a otimized trajectory
   //if the optimized traj is in collision use the graph trajectory
    for (; cur != NULL; cur = cur->next_lp){
      if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {
//         cur = traj->trajInGraph;
        optimTrajInCollision = true;
        break;
      }
    }
    //else exit this function
    if(optimTrajInCollision == false){
      return TRUE;
    }else{
      p3dAddTrajToGraph(robot, mainGraph, traj);
    }
  }
  
//Essai : regarder si il n'y a pas un chemin valid deja construit dans le graph
  if(optimTrajInCollision){
    if(p3d_graph_to_traj(robot)){
      g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
      optimTrajInCollision = false;
      if(traj->isOptimized){
        optimiseTrajectory();
      }
    }
  }

  for (; cur != NULL; cur = cur->next_lp) {
    if (p3d_unvalid_localpath_test(robot, cur, &ntest)) {//le lp est en collision
      graphTrajInCollision = true;
      printf((char*)"lp in collision\n");
      configPt box[2], q = p3d_alloc_config(robot);
      p3d_localpath * tmpPrev = cur;
      double tmpDist = dist;
      //find the first node (start of a localpath)
      do {
        box[0] = p3d_config_at_distance_along_traj(traj, tmpDist);
        p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, box[0]);
        if (tmpPrev->prev_lp != NULL){
          tmpPrev = tmpPrev->prev_lp;
          tmpDist -= tmpPrev->length_lp;
        }else{
          //la configuration de depart
          box[0] = p3d_config_at_distance_along_traj(traj, 0);
          break;
        }
      } while (p3d_col_test());
      //save the selected localpath
      configPt tmpQ = p3d_config_at_distance_along_traj(traj, 0);
      if(!p3d_equal_config(robot, box[0], tmpQ)){
        tmpPrev = tmpPrev->next_lp;
      }
      p3d_destroy_config(robot, tmpQ);
      //find the last node (end of a localpath)
      p3d_localpath * tmpNext = cur;
      tmpDist = dist + cur->length_lp;
      do {
        box[1] = p3d_config_at_distance_along_traj(traj, tmpDist);
        p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, box[1]);
        if (tmpNext->next_lp != NULL) {
          tmpNext = tmpNext->next_lp;
          tmpDist += tmpNext->length_lp;
        }else{
          //la configuration d'arrivee
          box[1] = p3d_config_at_distance_along_traj(traj, trajLength);
          break;
        }
      } while (p3d_col_test());
      //save the selected localpath
      tmpQ = p3d_config_at_distance_along_traj(traj, trajLength);
      if(!p3d_equal_config(robot, box[1], tmpQ)){
        tmpNext = tmpNext->prev_lp;
      }
      p3d_destroy_config(robot, tmpQ);
      //tagger les edges comme invalides.
      p3d_edge* edge = p3d_getLpEdge(robot, robot->GRAPH, tmpPrev);
      p3d_unvalid_edge(robot->GRAPH, edge);
      for(p3d_localpath* tmplp = tmpPrev->next_lp; tmplp != tmpNext->next_lp; tmplp = tmplp->next_lp){
        p3d_edge* edge = p3d_getLpEdge(robot, robot->GRAPH, tmplp);
        edge->unvalid = TRUE;
        if(!robot->GRAPH->oriented){//unvalid the other edge too
          for(p3d_list_edge* lEdge = edge->Nf->edges; lEdge; lEdge = lEdge->next){
            if(lEdge->E->Nf == edge->Ni){
              lEdge->E->unvalid = TRUE;
              break;
            }
          }
        }
      }
      showConfig(box[0]);
      showConfig(box[1]);
      //sauvegarde
      int random = p3d_get_RANDOM_CHOICE();
      int sampling = p3d_get_SAMPLING_CHOICE();
      int motion = p3d_get_MOTION_PLANNER();
      int biDirection = ENV.getBool(Env::biDir);
      int nbTry = ENV.getInt(Env::NbTry);
      int comp = ENV.getInt(Env::maxNodeCompco);
      p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
      p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
      p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
      ENV.setInt(Env::NbTry,10000000);
      ENV.setBool(Env::biDir, true);
      ENV.setInt(Env::maxNodeCompco,10000000);
      configPt qPos = robot->ROBOT_POS;
      configPt qGoto = robot->ROBOT_GOTO;
      robot->ROBOT_POS = box[0];
      robot->ROBOT_GOTO = box[1];
      XYZ_GRAPH = NULL;
      robot->GRAPH = NULL;

      int success = p3d_specific_search((char*)"");

      //restore
      robot->ROBOT_POS = qPos;
      robot->ROBOT_GOTO = qGoto;
      p3d_set_RANDOM_CHOICE(random);
      p3d_set_SAMPLING_CHOICE(sampling);
      p3d_set_MOTION_PLANNER(motion);
      ENV.setInt(Env::NbTry,nbTry);
      ENV.setBool(Env::biDir, biDirection);
      ENV.setInt(Env::maxNodeCompco,comp);
      if (!success){
        printf("Impossible to avoid collision\n");
        return FALSE;
      }
      //Fuse the generated graph with the main one
      p3d_graph* subGraph = XYZ_GRAPH;
      p3d_fuseGraphs(robot, mainGraph, subGraph);
      XYZ_GRAPH = mainGraph;
      robot->GRAPH = mainGraph;
      cur = tmpNext;
    }
    dist += cur->length_lp;
  }
  if(graphTrajInCollision || optimTrajInCollision){ //If it's the optimized traj in collision, we change the trajectory withe the graph traj.
    p3d_addStartAndGoalNodeToGraph(robot->ROBOT_POS, robot->ROBOT_GOTO, robot->ikSolPos, robot->ikSolGoto, robot->GRAPH, robot);
    if(p3d_graph_to_traj(robot)){
      g3d_add_traj((char*)"Globalsearch", p3d_get_desc_number(P3D_TRAJ));
      checkForCollidingLpAlongPath();
    }else{
      printf("oula Mino il y'a soucis!!! \n");
      return FALSE;
    }
  }
  if(traj->isOptimized){
    optimiseTrajectory();
  }
  return TRUE;
}
#endif

static p3d_edge* p3d_getLpEdge(p3d_rob* robot, p3d_graph* graph, p3d_localpath* lp){
  p3d_node * startNode = NULL, *goalNode = NULL;
  startNode = p3d_TestConfInGraph(graph, lp->config_at_param(robot, lp, 0));
  goalNode = p3d_TestConfInGraph(graph, lp->config_at_param(robot, lp, lp->length_lp));
  if(startNode && goalNode){
    for(p3d_list_edge* lEdge = startNode->edges; lEdge; lEdge = lEdge->next){
      if (lEdge->E->Nf == goalNode){
        return lEdge->E;
      }
    }
  }
  return NULL;
}

void p3dAddTrajToGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj){
  p3d_node* initNode = NULL, *endNode = NULL;
  configPt qInit, qEnd;
  bool nodeAlreadyConnected = false;
  for(p3d_localpath* lp = traj->courbePt; lp; lp = lp->next_lp){
    qInit = lp->config_at_param(robot, lp, 0);
    qEnd = lp->config_at_param(robot, lp, lp->length_lp);
    nodeAlreadyConnected = false;
    initNode = NULL;
    endNode = NULL;
    initNode = p3d_TestConfInGraph(graph, qInit);
    if(!initNode){
      printf("QInit n'est pas dans le graph\n");//If qinit is not already in the graph, there is a problem !!!
      return;
    }
    endNode = p3d_TestConfInGraph(graph, qEnd);
    if(!endNode){
      endNode = p3d_APInode_make_multisol(graph, qEnd, lp->ikSol);
      p3d_insert_node(graph, endNode);
    }
    //connect qInit and qEnd if its not connected yet
    for(p3d_list_edge* lEdge = initNode->edges; lEdge; lEdge = lEdge->next){
      if(lEdge->E->Nf == endNode){
        nodeAlreadyConnected = true;
        break;
      }
    }
    if(!nodeAlreadyConnected){
      p3d_add_node_compco(endNode, initNode->comp, TRUE);
      double dist = p3d_dist_q1_q2_multisol(robot, qInit, qEnd, lp->ikSol);//take the distance between the two nodes
      p3d_create_edges(graph, initNode, endNode, dist);//create edges between the two nodes
    }
  }
}

static void p3d_fuseGraphs(p3d_rob* robot, p3d_graph* mainGraph, p3d_graph* subGraph){
  p3d_node* map[2][subGraph->nnode];
  for(int i = 0; i < subGraph->nnode; i++){
    map[0][i] = map[1][i] = NULL;
  }
  p3d_list_node *subLNode = subGraph->nodes;
  for(int i = 0; subLNode; subLNode = subLNode->next, i++){
    p3d_node* existingNode = p3d_TestConfInGraph(mainGraph, subLNode->N->q);
    if(!existingNode){
      existingNode = p3d_APInode_make_multisol(mainGraph, subLNode->N->q, subLNode->N->iksol);
      p3d_insert_node(mainGraph, existingNode);
      p3d_create_compco(mainGraph, existingNode);
    }
    map[0][i] = subLNode->N;
    map[1][i] = existingNode;
    //find in the map the neighbours of subLNode->N and connect the resulting nodes in the main graph
    for(p3d_list_node *lNode = subLNode->N->neighb; lNode; lNode = lNode->next){
      for(int j = 0; j < i; j++){
        if (map[0][j] == lNode->N){
          //existingNode and map[1][j] have to be connected (and their compco fused)
          int* ikSol = NULL;
          if (existingNode->numcomp < map[1][j]->numcomp) {
            p3d_merge_comp(mainGraph, existingNode->comp, &(map[1][j]->comp));// merge the two compco
            p3d_get_non_sing_iksol(robot->cntrt_manager, existingNode->iksol, map[1][j]->iksol, &ikSol);
            int dist = p3d_dist_q1_q2_multisol(robot, existingNode->q, map[1][j]->q, ikSol);
            p3d_create_edges(mainGraph, existingNode, map[1][j], dist);//link the two nodes
          } else if (existingNode->numcomp > map[1][j]->numcomp){
            p3d_merge_comp(mainGraph, map[1][j]->comp, &(existingNode->comp));// merge the two compco
            p3d_get_non_sing_iksol(robot->cntrt_manager, existingNode->iksol, map[1][j]->iksol, &ikSol);
            int dist = p3d_dist_q1_q2_multisol(robot, existingNode->q, map[1][j]->q, ikSol);
            p3d_create_edges(mainGraph, existingNode, map[1][j], dist);//link the two nodes
          }
        }
      }
    }
  }
}


// void p3d_computeCollisionTime(void){
//   p3d_rob * r = (p3d_rob *)p3d_get_desc_curid(P3D_ROBOT);
//   configPt q = p3d_alloc_config(r);
//   configPt q2 = p3d_alloc_config(r);
//   double tu = 0.0, ts = 0.0, nTu = 0.0;
//   p3d_localpath * localpathPt = NULL;
//   int ntest;
//
// //Shoot
//   ChronoOn();
//   ChronoTimes(&tu, &ts);
//   printf("Time before shoot = %f\n", tu);
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//   }
//   ChronoTimes(&tu, &ts);
//   printf("Time after shoot = %f\n", tu);
//   ChronoOff();
//
// //Collision Shoot
//   ChronoOn();
//   ChronoTimes(&nTu, &ts);
//   printf("Time before col = %f\n", nTu);
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//     p3d_col_test();
//   }
//   ChronoTimes(&nTu, &ts);
//   printf("Time after col = %f\n", nTu);
//   ChronoOff();
//   printf("Collision time = %f over 1000 = %f\n", (nTu - tu)/NB_SHOOTS, (nTu - tu)*1000/NB_SHOOTS);
// 
//   tu = nTu - tu;
// //localPath Creation
//   ChronoOn();
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//     p3d_col_test();
//     p3d_standard_shoot(r, q2,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q2);
//     p3d_get_robot_config_into(r, &q2);
//     p3d_col_test();
//     localpathPt = p3d_local_planner_multisol(r, q, q2, NULL);
//   }
//   ChronoTimes(&nTu, &ts);
//   printf("Time after lp Create = %f\n", nTu - 2*tu);
//   ChronoOff();
//   tu = nTu - 2*tu;
// 
// //localpath Col
//   ChronoOn();
//   for(int i = 0; i < NB_SHOOTS; i++){
//     p3d_standard_shoot(r, q,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q);
//     p3d_get_robot_config_into(r, &q);
//     p3d_col_test();
//     p3d_standard_shoot(r, q2,1);
//     p3d_set_and_update_this_robot_conf_with_partial_reshoot(r, q2);
//     p3d_get_robot_config_into(r, &q2);
//     p3d_col_test();
//     localpathPt = p3d_local_planner_multisol(r, q, q2, NULL);
//     p3d_unvalid_localpath_test(r, localpathPt, &ntest);
//   }
//   ChronoTimes(&nTu, &ts);
//   printf("Time after lp Create = %f\n", nTu - tu);
//   ChronoOff();
//   printf("Collision time = %f over 1000 = %f\n", (nTu - tu)/NB_SHOOTS, (nTu - tu)*1000/NB_SHOOTS);
// }

