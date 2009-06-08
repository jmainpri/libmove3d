#include "UserAppli-pkg.h"
#include "Planner-pkg.h"
// #include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Move3d-pkg.h"
// #include "Util-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

static int trueFunction(void);
static void switchBBActivationForGrasp(void);
static void switchObjectsTypes(void);
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shoot);
static void fixJoint(p3d_rob * robot, p3d_jnt * joint,  p3d_matrix4 initPos);
static void unFixJoint(p3d_rob * robot, p3d_jnt * joint);
static void fixAllJointsExceptBaseAndObject(p3d_rob * robot, configPt conf);
static void unFixAllJointsExceptBaseAndObject(p3d_rob * robot);


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

void pathGraspOptions(int start){
  if(start){
    switchBBActivationForGrasp();
    p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
    p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
    p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
    p3d_set_multiGraph(FALSE);
    p3d_SetIsBidirectDiffu(TRUE);//bidirectionnal
    p3d_set_NB_TRY(100000);
    p3d_set_COMP_NODES(100000);
  }else{
    switchBBActivationForGrasp();
  }
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
  p3d_traj* carryTraj = moveObjectByConf(robot, finalConf);
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
  fixAllJointsExceptBaseAndObject(robot, robot->defaultConf);
  configPt conf = setBodyConfigForBaseMovement(robot, approachConf, robot->defaultConf);
  
  p3d_set_and_update_robot_conf(conf);
  g3d_refresh_allwin_active();
  sleep(2);
  
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  
  p3d_set_and_update_robot_conf(robot->ROBOT_GOTO);
  g3d_refresh_allwin_active();
  sleep(2);
  
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
  p3d_concat_traj(baseTraj, JustinTraj);
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
  //unfixJntObjectCntrt(robot->cntrt_manager, robot->objectJnt->num);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
}

void moveObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
  configPt finalConf = setTwoArmsRobotGraspPos(robot, objectGotoPos, att1, att2);
  p3d_set_and_update_robot_conf(finalConf);
  g3d_refresh_allwin_active();
  sleep(1);
  moveObjectByConf(robot, finalConf);
}
p3d_traj* moveObjectByConf(p3d_rob * robot, configPt finalConf){
  activateCcCntrts(robot);
  //unfixJntObjectCntrt(robot->cntrt_manager, robot->objectJnt->num);
  unFixJoint(robot, robot->objectJnt);
  robot->ROBOT_GOTO = finalConf;
  closedChainPlannerOptions();
  deactivateHandsVsObjectCol(robot);
  findPath();
  optimiseTrajectory();
  activateHandsVsObjectCol(robot);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
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
  //fixJntObjectCntrt(robot->cntrt_manager, robot->objectJnt->num, objectInitPos);
  fixJoint(robot, robot->objectJnt, objectInitPos);
  robot->ROBOT_POS = approachConf;
  robot->ROBOT_GOTO = graspConf;
  pathGraspOptions(1);
  findPath();
  optimiseTrajectory();
  pathGraspOptions(0);
  //unfixJntObjectCntrt(robot->cntrt_manager, robot->objectJnt->num);
  unFixJoint(robot, robot->objectJnt);
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
  *graspConf = getRobotGraspConf(robot, objectPos, att, 1);
  /*Shift attach position over wrist X axis*/
  att[0][1][3] += -100;
  att[1][1][3] += 100;
  p3d_set_and_update_robot_conf(*graspConf);
  *approachConf = getRobotGraspConf(robot, objectPos, att, 0);
  att[0][1][3] -= -100;
  att[1][1][3] -= 100;
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
  att1[1][3] += -100;
  att2[1][3] += 100;

  p3d_matrix4 * att = MY_ALLOC(p3d_matrix4, 2);
  p3d_mat4Copy(att1, att[0]);
  p3d_mat4Copy(att2, att[1]);
  configPt q = getRobotGraspConf(robot, objectPos, att,1);
  att1[1][3] -= -100;
  att2[1][3] -= 100;
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
  configPt q = getRobotGraspConf(robot, objectPos, att,1);
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
static configPt getRobotGraspConf(p3d_rob* robot, p3d_matrix4 objectPos, p3d_matrix4 *att, int shoot){
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  configPt q;
  switchBBActivationForGrasp();
  p3d_mat4ExtractPosReverseOrder(objectPos, &x, &y, &z, &rx, &ry, &rz);
  q = p3d_getRobotBaseConfigAroundTheObject(robot, x, y, z, rx, ry, rz, shoot);
  switchBBActivationForGrasp();
  return q;
}

/**
 * @brief Set the robot Start configuration given the object matrix position
 * @param robot The robot
 * @param objectPos The object matrix position
 */
void setRobotStartPosByObjectMat(p3d_rob* robot, p3d_matrix4 objectPos){
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  p3d_mat4ExtractPosReverseOrder(objectPos, &x, &y, &z, &rx, &ry, &rz);
  setRobotStartPosByObjectPos(robot, x, y, z, rx, ry, rz);
}

/**
 * @brief Set the robot Start configuration given the object position and euler angles.
 * @param robot The robot
 * @param x the object x coordinate
 * @param y the object y coordinate
 * @param z the object z coordinate
 * @param rx the object rotation around x axis
 * @param ry the object rotation around y axis
 * @param rz the object rotation around z axis
 */
void setRobotStartPosByObjectPos(p3d_rob* robot, double x, double y, double z, double rx, double ry, double rz){
  switchBBActivationForGrasp();
  configPt q = p3d_getRobotBaseConfigAroundTheObject(robot, x, y, z, rx, ry, rz, 1);
  switchBBActivationForGrasp();
  if(robot->ROBOT_POS != NULL){
    p3d_destroy_config(robot, robot->ROBOT_POS);
  }
  robot->ROBOT_POS = q;
}

/**
 * @brief Set the robot Goto configuration given the object matrix position
 * @param robot The robot
 * @param objectPos The object matrix position
 */
void setRobotGotoPosByObjectMat(p3d_rob* robot, p3d_matrix4 objectPos){
  double x = 0, y = 0, z = 0, rx = 0, ry = 0, rz = 0;
  p3d_mat4ExtractPosReverseOrder(objectPos, &x, &y, &z, &rx, &ry, &rz);
  setRobotGotoPosByObjectPos(robot, x, y, z, rx, ry, rz);
}

/**
 * @brief Set the robot Goto configuration given the object position and euler angles.
 * @param robot The robot
 * @param x the object x coordinate
 * @param y the object y coordinate
 * @param z the object z coordinate
 * @param rx the object rotation around x axis
 * @param ry the object rotation around y axis
 * @param rz the object rotation around z axis
 */
void setRobotGotoPosByObjectPos(p3d_rob* robot, double x, double y, double z, double rx, double ry, double rz){
  switchBBActivationForGrasp();
  configPt q = p3d_getRobotBaseConfigAroundTheObject(robot, x, y, z, rx, ry, rz, 1);
  switchBBActivationForGrasp();
  if(robot->ROBOT_GOTO != NULL){
    p3d_destroy_config(robot, robot->ROBOT_GOTO);
  }
  robot->ROBOT_GOTO = q;
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
