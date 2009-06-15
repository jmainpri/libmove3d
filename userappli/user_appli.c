#include "UserAppli-pkg.h"
#include "Planner-pkg.h"
// #include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
// #include "Util-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

#define APROACH_OFFSET 120

static int trueFunction(void);
static void switchBBActivationForGrasp(void);
static void switchObjectsTypes(void);
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject);
static void fixJoint(p3d_rob * robot, p3d_jnt * joint,  p3d_matrix4 initPos);
static void unFixJoint(p3d_rob * robot, p3d_jnt * joint);
static void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf);
static void unFixAllJointsExceptBaseAndObject(p3d_rob * robot);
static void getObjectBaseAttachMatrix(p3d_matrix4 base, p3d_matrix4 object, p3d_matrix4 result);
static p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt);

void openChainPlannerOptions(void){
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
  p3d_set_NB_TRY(1000);
  p3d_set_multiGraph(TRUE);
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

void closedChainPlannerOptions(void){
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
  p3d_set_NB_TRY(1000);
  p3d_set_multiGraph(FALSE);
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

void pathGraspOptions(void){
    p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
    p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
    p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
    p3d_set_multiGraph(FALSE);
    p3d_SetIsBidirectDiffu(TRUE);//bidirectionnal
    p3d_set_NB_TRY(100000);
    p3d_set_COMP_NODES(100000);
}

void globalPlanner(void){
  p3d_learn(p3d_get_NB_NODES(), NULL, NULL);
}

void findPath(void){
  p3d_specific_search("");
}

void deactivateHandsVsObjectCol(p3d_rob* robot){
  for(int i = 0; i < robot->graspNbJoints; i++){
    p3d_col_deactivate_obj_obj(robot->graspJoints[i]->o, robot->objectJnt->o);
  }
}

void activateHandsVsObjectCol(p3d_rob* robot){
  for(int i = 0; i < robot->graspNbJoints; i++){
    p3d_col_activate_obj_obj(robot->graspJoints[i]->o, robot->objectJnt->o);
  }
}

static int trueFunction(void){
  return TRUE;
}

void optimiseTrajectory(void){
  p3d_set_NB_OPTIM(50);
  CB_start_optim_obj(NULL,0);
}

void viewTraj(void){
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  switchBBActivationForGrasp();
  g3d_show_tcur_rob(robot,trueFunction);
  switchBBActivationForGrasp();
}

static void switchObjectsTypes(void){
  p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  for(int i = 0 ; i < robotPt->graspNbJoints; i++){
    p3d_obj* jntObj = (robotPt->graspJoints[i])->o;
    for(int j = 0; j < jntObj->np; j++){
      if (jntObj->pol[j]->TYPE == P3D_GRAPHIC){
        jntObj->pol[j]->TYPE = P3D_GHOST;
      }else if (jntObj->pol[j]->TYPE == P3D_GHOST){
        jntObj->pol[j]->TYPE = P3D_GRAPHIC;
      }
    }
  }
}

static void switchBBActivationForGrasp(void){
  switchObjectsTypes();
  p3d_col_stop();
  p3d_col_start(p3d_col_mode_kcd);
}

static void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf){
  p3d_set_and_update_robot_conf(conf);
  for(int i = 0; i < robot->njoints; i++){
    p3d_jnt * joint = robot->joints[i];
  	if(joint->type != P3D_BASE && joint->type != P3D_FIXED && joint != robot->objectJnt && joint != robot->baseJnt){      
      fixJoint(robot, joint, joint->jnt_mat);
    }
  }
}

static void unFixAllJointsExceptBaseAndObject(p3d_rob * robot){
  for(int i = 0; i < robot->njoints; i++){
    p3d_jnt * joint = robot->joints[i];
  	if(joint->type != P3D_BASE && joint->type != P3D_FIXED && joint != robot->objectJnt && joint != robot->baseJnt){      
      unFixJoint(robot, joint);
    }
  }
}

static void fixJoint(p3d_rob * robot, p3d_jnt * joint,  p3d_matrix4 initPos){
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  p3d_mat4ExtractPosReverseOrder(initPos, &x, &y, &z, &rx, &ry, &rz);
  double dVal[6] = {x,y,z,rx,ry,rz};
	for(int i = 0; i < joint->dof_equiv_nbr; i++){
    if(robot->isUserDof[joint->index_dof + i]){
      joint->dof_data[i].v = dVal[i];
    	joint->dof_data[i].is_user = FALSE;
    }
  }
}

static void unFixJoint(p3d_rob * robot, p3d_jnt * joint){
	for(int i = 0; i < joint->dof_equiv_nbr; i++){
    if(robot->isUserDof[joint->index_dof + i]){
    	joint->dof_data[i].is_user = TRUE;
    }
  }
}

static p3d_cntrt * findTwoJointsFixCntrt(p3d_rob* robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt){
	for(int i = 0; i < robot->cntrt_manager->ncntrts; i++){
		//Check if the constraint is already created
		p3d_cntrt *cntrt = robot->cntrt_manager->cntrts[i];
		if(cntrt->npasjnts == 1 && cntrt->nactjnts == 1 && cntrt->pasjnts[0]->num == passiveJnt->num && cntrt->actjnts[0]->num == activeJnt->num){
			return cntrt;
		}
	}
	return NULL;
}

static void setAndActivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt){
	int passiveJntId[1] = {passiveJnt->num}, activeJntId[1] = {activeJnt->num};
	p3d_cntrt * cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
	//If the constraint is already created
	if (cntrt != NULL){
		//Activate it
		if(p3d_update_constraint(cntrt, 1)) {
			if (cntrt->enchained != NULL)
				p3d_reenchain_cntrts(cntrt);
			p3d_col_deactivate_one_cntrt_pairs(cntrt);
		}
	}else	if(!p3d_constraint("p3d_fix_jnts_relpos", -1, passiveJntId, -1, activeJntId, -1, NULL, -1, NULL, -1, 1)){
		printf("Error in creatin the p3d_fix_jnts_relpos÷\n");
	}else{
		cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
	}
	//set the attach Matrix
	getObjectBaseAttachMatrix(activeJnt->abs_pos, passiveJnt->abs_pos, cntrt->Tatt);
	return;
}

static void desactivateTwoJointsFixCntrt(p3d_rob * robot, p3d_jnt* passiveJnt, p3d_jnt* activeJnt){
	p3d_cntrt * cntrt = findTwoJointsFixCntrt(robot, passiveJnt, activeJnt);
	if(p3d_update_constraint(cntrt, 0)) {
		if (cntrt->enchained != NULL)
			p3d_unchain_cntrts(cntrt);
		p3d_update_jnts_state(robot->cntrt_manager,cntrt, 0);
		p3d_col_activate_one_cntrt_pairs(cntrt);
	}
}

static void getObjectBaseAttachMatrix(p3d_matrix4 base, p3d_matrix4 object, p3d_matrix4 result){
	p3d_matrix4 tmp;
	p3d_matInvertXform(base, tmp);
	p3d_mat4Mult(tmp, object, result);
}

static void adaptClosedChainConfigToBasePos(p3d_rob *robot, p3d_matrix4 base, configPt refConf){
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
	//p3d_jnt_set_mat_pos(robot->objectJnt, &newObjectPos);
	//On change la configuration du joint de la base.
	p3d_mat4ExtractPosReverseOrder(basePos, &x, &y, &z, &rx, &ry, &rz);
	p3d_jnt_set_dof(robot->baseJnt, 0, x);
	p3d_jnt_set_dof(robot->baseJnt, 1, y);
	p3d_jnt_set_dof(robot->baseJnt, 2, rz);
	//Test de validitee et collision.
	if(p3d_update_this_robot_pos_with_partial_reshoot(robot)){
		if(!p3d_col_test()){
			//Sauvegarde de la configuration.
			p3d_get_robot_config_into(robot, &refConf);
			return;
		}
	}
	//Si un de ces tests ne marche pas on tire aleatoirement une configuration du torse/bras
  configPt tmp =	setTwoArmsRobotGraspPosWithoutBase(robot, robot->objectJnt->abs_pos, robot->ccCntrts[0]->Tatt, robot->ccCntrts[1]->Tatt);
	//Sauvegarde de la configuration.
	p3d_copy_config_into(robot, tmp, &refConf);
	p3d_destroy_config(robot, tmp);
}

void pickAndMoveObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
  configPt graspConf, approachConf, finalConf;
  setTwoArmsRobotGraspAndApproachPos(robot, objectInitPos, att1, att2, &graspConf, &approachConf);
  finalConf = setTwoArmsRobotGraspPos(robot, objectGotoPos, att1, att2);
  pickAndMoveObjectByConf(robot, objectInitPos, approachConf, graspConf, finalConf);
}

void pickAndMoveObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf, configPt finalConf){
  p3d_traj* approachTraj = pickObjectByConf(robot, objectInitPos, approachConf);
  p3d_traj* graspTraj = graspObjectByConf(robot, objectInitPos, approachConf, graspConf);
  p3d_concat_traj(approachTraj, graspTraj);
  p3d_traj* carryTraj = moveObjectByConf(robot, graspConf, finalConf);
  p3d_concat_traj(approachTraj, carryTraj);
}

void pickObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2){
  configPt approachConf = setTwoArmsRobotGraspApproachPos(robot, objectInitPos, att1, att2);
  p3d_set_and_update_robot_conf(approachConf);
  g3d_refresh_allwin_active();
  sleep(2);
  pickObjectByConf(robot, objectInitPos, approachConf);
}

p3d_traj* pickObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf){
//Plannification de la base
  printf("Base Planning\n");
  deactivateCcCntrts(robot);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  fixAllJointsExceptBaseAndObject(robot, robot->openChainConf);
  configPt conf = setBodyConfigForBaseMovement(robot, approachConf, robot->openChainConf);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  openChainPlannerOptions();
  findPath();
  optimiseTrajectory();
  p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);

//Plannification des bras
	printf("Body Planning\n");
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, approachConf, &(robot->ROBOT_GOTO));
  p3d_set_and_update_robot_conf(approachConf);
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  CB_del_param_obj(NULL, 0);
  p3d_specificSuperGraphLearn();
  optimiseTrajectory();
  p3d_traj* JustinTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if(baseTraj){
  	p3d_concat_traj(baseTraj, JustinTraj);
  }
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
  return baseTraj;
}

void moveObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
  configPt finalConf = setTwoArmsRobotGraspPos(robot, objectGotoPos, att1, att2);
  p3d_set_and_update_robot_conf(finalConf);
  g3d_refresh_allwin_active();
  sleep(1);
  moveObjectByConf(robot, robot->ROBOT_POS, finalConf);
}
p3d_traj* moveObjectByConf(p3d_rob * robot, configPt initConf, configPt finalConf){
  deactivateHandsVsObjectCol(robot);
	
//Extract traj	
  activateCcCntrts(robot);
	p3d_set_and_update_robot_conf(initConf);
	configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
	adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  unFixJoint(robot, robot->objectJnt);
	unFixAllJointsExceptBaseAndObject(robot);
	p3d_set_and_update_robot_conf(initConf);
	fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, initConf, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
	pathGraspOptions();
  findPath();
  optimiseTrajectory();
	p3d_traj* extractTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
/*
//trasferTraj
	p3d_set_and_update_robot_conf(finalConf);
  p3d_copy_config_into(robot, robot->closedChainConf, &adaptedConf);
	adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
	unFixJoint(robot, robot->baseJnt);
	fixAllJointsExceptBaseAndObject(robot, robot->closedChainConf);
	deactivateCcCntrts(robot);
	setAndActivateTwoJointsFixCntrt(robot, robot->objectJnt, robot->baseJnt);
	p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
	p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
	p3d_destroy_config(robot, adaptedConf);
	closedChainPlannerOptions();
	findPath();
  optimiseTrajectory();
	p3d_traj* trasfertTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	if(extractTraj){
  	p3d_concat_traj(extractTraj, trasfertTraj);
  }
//Depose Traj
	p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
	p3d_copy_config_into(robot, finalConf, &(robot->ROBOT_GOTO));	
	activateCcCntrts(robot);
//desactiver la contrainte entre la base et l'objet
	desactivateTwoJointsFixCntrt(robot, robot->objectJnt, robot->baseJnt);
	unFixAllJointsExceptBaseAndObject(robot);
	fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
	pathGraspOptions();
  findPath();
  optimiseTrajectory();
	p3d_traj* deposeTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
	if(extractTraj){
  	p3d_concat_traj(extractTraj, deposeTraj);
  }*/
  return extractTraj;
}

void graspObjectByMat(p3d_rob * robot, p3d_matrix4 objectInitPos, p3d_matrix4 att1, p3d_matrix4 att2){
  configPt graspConf, approachConf;
  setTwoArmsRobotGraspAndApproachPos(robot, objectInitPos, att1, att2, &graspConf, &approachConf);
  p3d_set_and_update_robot_conf(approachConf);
  g3d_refresh_allwin_active();
  sleep(1);
  p3d_set_and_update_robot_conf(graspConf);
  g3d_refresh_allwin_active();
  sleep(1);
  graspObjectByConf(robot, objectInitPos, approachConf, graspConf);
}

p3d_traj* graspObjectByConf(p3d_rob * robot, p3d_matrix4 objectInitPos, configPt approachConf, configPt graspConf){
  deactivateCcCntrts(robot);
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  robot->ROBOT_POS = approachConf;
  robot->ROBOT_GOTO = graspConf;
	switchBBActivationForGrasp();
  pathGraspOptions();
  findPath();
  optimiseTrajectory();
  pathGraspOptions();
	switchBBActivationForGrasp();
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
}

/**
 * @brief Get the robot grasp approach an the robot grasp configuration given an attach matrix for each arm and the object position
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @param graspConf the retruned grasp config of the robot
 * @param approachConf the retruned approach config of the robot
 */
void setTwoArmsRobotGraspAndApproachPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2, configPt* graspConf, configPt* approachConf){
  if(robot->nbCcCntrts != 2){
    printf("There is more than 2 arms\n");
    return;
  }

  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  *graspConf = getRobotGraspConf(robot, objectPos, att, TRUE);
  /*Shift attach position over wrist X axis*/
  att[0][1][3] += -APROACH_OFFSET;
  att[1][1][3] += APROACH_OFFSET;
  p3d_set_and_update_robot_conf(*graspConf);
  *approachConf = getRobotGraspConf(robot, objectPos, att, FALSE);
  MY_FREE(att, p3d_matrix4, 2);
  return;
}


/**
 * @brief Get the robot grasp approach configuration given an attach matrix for each arm and the object position
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt setTwoArmsRobotGraspApproachPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2){
  if(robot->nbCcCntrts != 2){
    printf("There is more than 2 arms\n");
    return NULL;
  }
  /*Shift attach position over wrist X axis*/
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  att[0][1][3] += -APROACH_OFFSET;
  att[1][1][3] += APROACH_OFFSET;
  configPt q = getRobotGraspConf(robot, objectPos, att, TRUE);
  MY_FREE(att, p3d_matrix4, 2);
  return q;
}


/**
 * @brief Get the robot grasp configuration given an attach matrix for each arm and the object position
 * @param robot the robot
 * @param objectPos the object position matrix
 * @param att1 the attach matrix for the first arm
 * @param att2 the attach matrix for the second arm
 * @return the robot config
 */
configPt setTwoArmsRobotGraspPos(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2){
  if(robot->nbCcCntrts != 2){
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att, TRUE);
  MY_FREE(att, p3d_matrix4, 2);
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
configPt setTwoArmsRobotGraspPosWithoutBase(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 att1, p3d_matrix4 att2){
  if(robot->nbCcCntrts != 2){
    printf("There is more than 2 arms\n");
    return NULL;
  }
  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att, FALSE);
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
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shootObject){
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  configPt q;
  switchBBActivationForGrasp();
  p3d_mat4ExtractPosReverseOrder(objectPos, &x, &y, &z, &rx, &ry, &rz);
  //Backup and set the attach matrix
  p3d_matrix4 bakTatt[robot->nbCcCntrts];
  for(int i = 0; i < robot->nbCcCntrts; i++){
    p3d_mat4Copy(robot->ccCntrts[i]->Tatt, bakTatt[i]);
    p3d_mat4Copy(att[i], robot->ccCntrts[i]->Tatt);
  }
  q = p3d_getRobotBaseConfigAroundTheObject(robot, x, y, z, rx, ry, rz, shootObject);
  //Restore the attach matrix
  for(int i = 0; i < robot->nbCcCntrts; i++){
    p3d_mat4Copy(bakTatt[i], robot->ccCntrts[i]->Tatt);
  }
  switchBBActivationForGrasp();
  return q;
}

#ifdef MULTIGRAPH

void p3d_specificSuperGraphLearn(void){
double *arraytimes = MY_ALLOC(double, p3d_get_NB_specific());
  int nfail = 0;
  configPt qs = NULL, qg = NULL;
  double tu = 0.0, ts = 0.0, mgTime = 0.0, gTime = 0.0;
  p3d_rob *robotPt = (p3d_rob *) (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_graph * graph = NULL, *tmp = NULL;

  p3d_SetDiffuStoppedByWeight(0);
  p3d_SetStopValue(FALSE);

  qs = p3d_copy_config(robotPt, robotPt->ROBOT_POS);
  if (p3d_GetIsExpansionToGoal() == TRUE) {
    qg = p3d_copy_config(robotPt, robotPt->ROBOT_GOTO);
  }

  if (!XYZ_GRAPH){
    graph = p3d_create_graph();
  }else{
    graph = XYZ_GRAPH;
  }

  //compute the specific planner for each part of the system
  for(int j = 0; j < robotPt->mg->nbGraphs; j++){
    p3d_set_user_drawnjnt(robotPt->mg->mgJoints[j]->joints[robotPt->mg->mgJoints[j]->nbJoints - 1]);
    tmp = p3d_setMultiGraphAndActiveDof(robotPt, j);
    p3d_loopSpecificLearn(robotPt, qs, qg, "", 0, arraytimes, &nfail);
    if (p3d_graph_to_traj(robotPt)) {
      g3d_add_traj("Globalsearch", p3d_get_desc_number(P3D_TRAJ));
    }
    gTime += robotPt->mg->graphs[j]->time;
  }
  //create the flat super Graph
  if (graph->nnode == 0){
    robotPt->GRAPH = NULL;
    XYZ_GRAPH = NULL;
    ChronoOn();
    p3d_setAllDofActive(robotPt);
    p3d_flatMultiGraph(robotPt, 1);
    p3d_setAllDofActive(robotPt);
    p3d_convertFsgToGraph(graph, robotPt->mg->fsg);
    ChronoTimes(&tu, &ts);
    ChronoOff();
    mgTime += tu;
    XYZ_GRAPH = graph;
    robotPt->GRAPH = graph;
    p3d_addStartAndGoalNodeToGraph(qs, qg, NULL, NULL, graph, robotPt);
    //check if there is solution in this graph or not.
    if (!p3d_graph_to_traj(robotPt)) {
      //continuer la recherche
      ChronoOn();
      p3d_delFlatSuperGraph(robotPt, robotPt->mg->fsg);
      p3d_flatMultiGraph(robotPt, 0);
      p3d_setAllDofActive(robotPt);
      p3d_del_graph(graph);
      graph = p3d_create_graph();
      p3d_convertFsgToGraph(graph, robotPt->mg->fsg);
      ChronoTimes(&tu, &ts);
      ChronoOff();
      mgTime += tu;
      XYZ_GRAPH = graph;
      robotPt->GRAPH = graph;
      p3d_addStartAndGoalNodeToGraph(qs, qg, NULL, NULL, graph, robotPt);
      if (!p3d_graph_to_traj(robotPt)) {
        p3d_doIncrementalConstruction(1);
      }else{
        g3d_add_traj("Globalsearch", p3d_get_desc_number(P3D_TRAJ));
        p3d_print_info_graph(graph);
      }
    }/*else{//pour les futures requetes
      p3d_delFlatSuperGraph(robotPt, robotPt->mg->fsg);
    }*/
  }else{
    XYZ_GRAPH = graph;
    ChronoOn();
    p3d_setAllDofActive(robotPt);
    for(int j = 0; j < robotPt->mg->nbGraphs; j++){
      p3d_fillFlatMultiGraph(robotPt, NULL, NULL, j, 2);
    }
    p3d_setAllDofActive(robotPt);
    p3d_convertFsgToGraph(graph, robotPt->mg->fsg);
    ChronoTimes(&tu, &ts);
    ChronoOff();
    mgTime += tu;
    XYZ_GRAPH = graph;
    robotPt->GRAPH = graph;
    p3d_addStartAndGoalNodeToGraph(qs, qg, NULL, NULL, graph, robotPt);
    //check if there is solution in this graph or not.
    if (!p3d_graph_to_traj(robotPt)) {
      p3d_doIncrementalConstruction(1);
    }else{
      p3d_print_info_graph(graph);
    }
  }
  if(p3d_doIncrementalConstruction(-1)){
    p3d_set_multiGraph(FALSE);
    p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
    p3d_setAllDofActive(robotPt);
    graph = XYZ_GRAPH;
    p3d_addStartAndGoalNodeToGraph(qs, qg, NULL, NULL, graph, robotPt);
    p3d_doIncrementalConstruction(0);
  }
}

#endif

void checkForCollidingLpAlongPath(void){
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  p3d_traj *traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if(!traj){
    return;
  }
  p3d_localpath *cur = traj->courbePt;
  int ntest = 0;
  double dist = 0;
  double nodeDist = 0.0;
  int counter = 0;
  int counterMax = 20;

  p3d_node * node = NULL, *nodeP = NULL, *nodeN = NULL;
  for(; cur != NULL; cur = cur->next_lp){
    if(p3d_unvalid_localpath_test(robot, cur, &ntest)){//le lp est en collision
      printf("lp in collision\n");
      configPt box[2], q = p3d_alloc_config(robot);
      p3d_localpath * tmpPrev = cur;
      double tmpDist = dist;
      do{
        tmpPrev = tmpPrev->prev_lp;
        box[0] = p3d_config_at_distance_along_traj(traj, tmpDist);
        p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, box[0]);
        tmpDist -= tmpPrev->length_lp;
        if(tmpDist < 0){
          //la configuration de dÃ©part
          box[0] = p3d_config_at_distance_along_traj(traj, 0);
          break;
        }
      }while(p3d_col_test());
      tmpPrev = tmpPrev->next_lp;
      p3d_localpath * tmpNext = cur;
      tmpDist = dist + cur->length_lp;
      do{
        tmpNext = tmpNext->next_lp;
        box[1] = p3d_config_at_distance_along_traj(traj, tmpDist);
        p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, box[1]);
        tmpDist += tmpNext->length_lp;
        if(tmpDist < 0){
          //la configuration de dÃ©part
          box[1] = p3d_config_at_distance_along_traj(traj, 0);
          break;
        }
      }while(p3d_col_test());
      tmpNext = tmpNext->prev_lp;
      do{
        if(node != NULL){
          p3d_APInode_desalloc(robot->GRAPH, node);
          node = NULL;
          q = p3d_alloc_config(robot);
        }
        if(nodeP != NULL){
          p3d_APInode_desalloc(robot->GRAPH, nodeP);
          nodeP = NULL;
        }
        if(nodeN != NULL){
          p3d_APInode_desalloc(robot->GRAPH, nodeN);
          nodeN = NULL;
        }
        do{
          p3d_shoot_inside_box(robot, q, box, 0);
          p3d_set_and_update_this_robot_conf_with_partial_reshoot(robot, q);
        }while(p3d_col_test());
        node = p3d_APInode_make_multisol(robot->GRAPH, q, NULL);
        nodeP = p3d_APInode_make_multisol(robot->GRAPH, box[0], NULL);
        nodeN = p3d_APInode_make_multisol(robot->GRAPH, box[1], NULL);
        counter++;
      }while((!p3d_APInode_linked_multisol(robot->GRAPH, node, nodeP, &nodeDist) || !p3d_APInode_linked_multisol(robot->GRAPH, node, nodeN, &nodeDist)) && counter < counterMax);
      if(counter >= counterMax){//first Step Failed do an BiRRT search
//         //sauvegarde
//         int random = p3d_get_RANDOM_CHOICE();
//         int sampling = p3d_get_SAMPLING_CHOICE();
//         int motion = p3d_get_MOTION_PLANNER();
//         int biDirection = p3d_GetIsBidirectDiffu();
//         int nbTry = p3d_get_NB_TRY();
//         int comp = p3d_get_COMP_NODES();
//         p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
//         p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
//         p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
//         p3d_SetIsBidirectDiffu(TRUE);//bidirectionnal
//         p3d_set_NB_TRY(100000);
//         p3d_set_COMP_NODES(100000);
//         configPt qPos = robot->ROBOT_POS, qGoto = robot->ROBOT_GOTO;
//         robot->ROBOT_POS = box[0];
//         robot->ROBOT_GOTO = box[1];
//         
//         p3d_specific_search("");
// 
//         //restore
//         robot->ROBOT_POS = qPos;
//         robot->ROBOT_GOTO = qGoto;
//         p3d_set_RANDOM_CHOICE(random);
//         p3d_set_SAMPLING_CHOICE(sampling);
//         p3d_set_MOTION_PLANNER(motion);
//         p3d_SetIsBidirectDiffu(biDirection);//bidirectionnal
//         p3d_set_NB_TRY(nbTry);
//         p3d_set_COMP_NODES(comp);
        printf("can't reconnect the path");
      }else{//TODO risque de segFault si on est dans le premier ou dernier lp
        p3d_localpath *lp1 = p3d_local_planner_multisol(robot, nodeP->q, node->q, tmpPrev->ikSol);
        p3d_localpath *lp2 = p3d_local_planner_multisol(robot, node->q, nodeN->q, tmpNext->ikSol);
        tmpPrev->prev_lp->next_lp = lp1;
        lp1->prev_lp = tmpPrev->prev_lp;
        lp1->next_lp = lp2;
        lp2->prev_lp = lp1;
        lp2->next_lp = tmpNext->next_lp;
        tmpNext->next_lp->prev_lp = lp2;
        p3d_localpath * tmp = tmpPrev;
        while(tmpPrev != tmpNext->next_lp){
          tmp = tmpPrev->next_lp;
          tmpPrev->destroy(robot, tmpPrev);
          tmpPrev = tmp;
        }
      }
      if(node != NULL){
        p3d_APInode_desalloc(robot->GRAPH, node);
        node = NULL;
      }
      if(nodeP != NULL){
        p3d_APInode_desalloc(robot->GRAPH, nodeP);
        nodeP = NULL;
      }
      if(nodeN != NULL){
        p3d_APInode_desalloc(robot->GRAPH, nodeN);
        nodeN = NULL;
      }
    }
    dist += cur->length_lp;
  }
}
