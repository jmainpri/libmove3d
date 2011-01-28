#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "Move3d-pkg.h"
#include "P3d-pkg.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/robotPos.h"
#include "../graphic/proto/g3d_draw_traj_proto.h"

#include "env.hpp"

///////////////////////////
//// Static functions /////
///////////////////////////
static int saveCurrentConfigInFile(p3d_rob* robot, p3d_localpath* curLp);
static int saveSpecifiedConfigInFile(configPt conf);
static void rrtOptions(void);
static int findPath(void);

extern double SAFETY_DIST;
extern double USE_LIN;
#define OPTIMSTEP 200
#define OPTIMTIME 10
#define MAXPLANTIME 300 //5 mins
/** @brief File used to save the trajectory*/
static FILE* trajFile = NULL;

#ifdef MULTILOCALPATH
/** @brief Multilocalpath Id*/
static int BaseMLP = -1;
static int HeadMLP = -1;
static int UpBodyMLP = -1;
static int ObjectMLP = -1;

/**
  @brief Function to initialize the multilocalpaths
*/
void initLightPlannerForMLP(p3d_rob* robot){
  for(int i = 0; i < robot->mlp->nblpGp; i++){
    if(!strcmp(robot->mlp->mlpJoints[i]->gpName, "base")){
      BaseMLP = i;
    }else if(!strcmp(robot->mlp->mlpJoints[i]->gpName, "head")){
      HeadMLP = i;
    }else if(!strcmp(robot->mlp->mlpJoints[i]->gpName, "upBody")){
      UpBodyMLP = i;
    }else if(!strcmp(robot->mlp->mlpJoints[i]->gpName, "object")){
      ObjectMLP = i;
    }
  }
}
#endif

/**
 * @brief Function that save the current configuration of the robot into an opened file (trajFile here)
 * @return True if the configuration is correctly written, false otherwise
 */
static int saveCurrentConfigInFile(p3d_rob* robot, p3d_localpath* curLp){
  configPt q = p3d_get_robot_config(robot);
  if (!q){
    return false;
  }
  int result = saveSpecifiedConfigInFile(q);
  p3d_destroy_config(robot, q);
  return result;
}

/**
 * @brief Function that save the configuration given into an opened file (trajFile here)
 * @param conf the configuration to write
 * @return True if the configuration is correctly written, false otherwise
 */
static int saveSpecifiedConfigInFile(configPt conf){
  p3d_rob *robot = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
  if(!conf || !trajFile){
    return false;
  }
  for(int i = 6; i < robot->nb_dof - 6; i++){
    fprintf(trajFile,"%.4lf \n", conf[i]);
  }
  fprintf(trajFile,"\n");
  return true;
}

/**
 * @brief Save the given trajectory into a file. Depending if the param smallIntervals is true or false, each localpath of the trajectory is divided into multiple configurations at each dmax, or only the initial or final configuration is writed down.
 * @param fileName The file where the trajectory will be writen
 * @param traj the trajectory
 * @param smallIntervals Decompose each localpath into multiple configurations or not
 * @param dmax The step if the param smallIntervals is true.
 */
void saveTrajInFile(const char* fileName, p3d_traj* traj, int smallIntervals,double dmax){
  if(traj != NULL){
    //openFile
    if (trajFile == NULL){
      trajFile = fopen(fileName, "a");
    }
    if(smallIntervals){
      fprintf(trajFile, "#############  Justin Movement ###########\n\n");
      traj->rob->tcur = traj;
      p3d_set_env_graphic_dmax(dmax);
      g3d_show_tcur_rob(traj->rob, saveCurrentConfigInFile); // in case of discretized configs
    }else{
      fprintf(trajFile, "#############  Platform Movement ###########\n\n");
      p3d_localpath* curlp = traj->courbePt;
      if(curlp->type_lp == LINEAR){
        saveSpecifiedConfigInFile(curlp->specific.lin_data->q_init);
      }else if(curlp->type_lp == REEDS_SHEPP){
        saveSpecifiedConfigInFile(curlp->specific.rs_data->q_init);
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
    fclose(trajFile);
    trajFile = NULL;
  }
}

/** ////////////////////////////////////////////
 * //////////// Planning Options //////////////
 * //////////////////////////////////////////*/

/**
 * @brief Delete all previously generated graphs
 */
void deleteAllGraphs(void){
#ifdef MULTIGRAPH
  if(p3d_get_multiGraph()){
    p3d_resetMultiGraph(XYZ_ROBOT);
  }
#endif
  p3d_del_graph(XYZ_GRAPH);
  p3d_reinit_array_exhausted_nodes();
  p3d_reinit_array_farther_nodes();
  XYZ_GRAPH = NULL;
}

/**
 * @brief Parameters for the planner
 */
static void rrtOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
#ifdef WITH_XFORMS
  CB_DiffusionMethod_obj(NULL, 0); //0 rrt Connect, 1 rrt extend
#endif
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  ENV.setBool(Env::biDir,true);//bidirectionnal
  ENV.setInt(Env::NbTry,1000000);
  ENV.setInt(Env::maxNodeCompco,100000);
}

/**
 * @brief Parameters for the planner
 */
static void offlinePlannerOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
  ENV.setInt(Env::NbTry,100000);
  p3d_set_tmax(MAXPLANTIME);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

/**
 * @brief Parameters for the planner
 */
static void offlineMgPlannerOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_ISOLATE_LINKING);
  ENV.setInt(Env::NbTry,100000);
  p3d_set_tmax(MAXPLANTIME);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(TRUE);
#endif
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);
}

/**
 * @brief Launch the planner
 */
static int findPath(void) {
  return p3d_specific_search((char*)"");
}

/**
 * @brief Set the number of optimisations steps and optimise the current trajectory of the robot
 * @param nbSteps number of steps for random smoothing
 * @param maxTime The maximum time that the planner have to spend to smooth the trajectory
 */
void optimiseTrajectory(p3d_rob* robot, p3d_traj* traj, int nbSteps, double maxTime) {
  p3d_set_NB_OPTIM(nbSteps);
  if(maxTime <= 0){
    p3d_set_use_optimization_time(false);
  }else{
  p3d_set_use_optimization_time(true);
  p3d_set_optimization_time(maxTime);
  }
  p3d_optimize_traj(robot, traj, true, false, true, NULL);
}
/** ////////////////////////////////////////////
 * ////////////// Query functions //////////////
 * //////////////////////////////////////////*/


/**
 * @brief Plan a trajectory for the robot platform from its start configuration to a final configuration where the object is reachable.
 * @param robot The robot to plan
 * @param objectStartPos The start position of the object to manipulate
 * @param att1 The attach matrix for the first constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @param att2 The attach matrix for the second constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @return the computed trajectory
 */
p3d_traj* platformGotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
  //try to reach the object without moving the base.
  p3d_set_and_update_this_robot_conf_without_cntrt(robot, robot->ROBOT_POS);
  if(!setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, cntrtToActivate, true)){
    configPt conf = setTwoArmsRobotGraspApproachPosWithHold(robot, objectStartPos, att1, att2, cntrtToActivate);
    if(conf == NULL){
      printf("no position found\n");
      return NULL;
    }
    p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
    return platformGotoObjectByConf(robot, objectStartPos, conf);
  }else{
    printf("I can reach the object without moving the base\n");
    return NULL;
  }
}

/**
 * @brief Plan a trajectory for the robot platform from its start configuration to a final configuration. First move the robot (all exept the base and the object) to the moving configuration, then move the base to the computed goal configuration
 * @param robot The robot
 * @param objectStartPos The start object position
 * @param conf The final configuration
 * @return The computed trajectory
 */
p3d_traj* platformGotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf){
  //Plannification des bras pour aller en position de trasfert
  //Trouver la configuration de transfert a partir de la config initiale.
  deactivateCcCntrts(robot, -1);
  configPt transfertConf = setBodyConfigForBaseMovement(robot, robot->ROBOT_POS, robot->openChainConf);
#ifdef MULTILOCALPATH
	p3d_multiLocalPath_disable_all_groupToPlan(robot);
    p3d_multiLocalPath_set_groupToPlan(robot, UpBodyMLP, 1) ;
#else
  p3d_local_set_planner((p3d_localplanner_type)1);
#endif
  deleteAllGraphs();
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  fixJoint(robot, robot->curObjectJnt, objectStartPos);
  p3d_update_this_robot_pos(robot);
  p3d_copy_config_into(robot, transfertConf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, transfertConf);
  rrtOptions();
  findPath();
  p3d_traj* justinTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  optimiseTrajectory(robot, justinTraj, OPTIMSTEP, OPTIMTIME);


  //Plannification de la base
#ifdef MULTILOCALPATH
	p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_set_groupToPlan(robot, BaseMLP, 1);
	p3d_multiLocalPath_set_groupToPlan(robot, HeadMLP, 1);
#else
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
#endif
  p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
  p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
  fixAllJointsExceptBaseAndObject(robot, robot->openChainConf);
  fixJoint(robot, robot->curObjectJnt, objectStartPos);
  unFixJoint(robot, robot->baseJnt);
  p3d_update_this_robot_pos(robot);
  configPt newConf = setBodyConfigForBaseMovement(robot, conf, robot->openChainConf);
  p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, newConf, &(robot->ROBOT_GOTO));
  setSafetyDistance(robot, (double)SAFETY_DIST);
  p3d_col_deactivate_obj_env(robot->curObjectJnt->o);
  deleteAllGraphs();
  rrtOptions();
  findPath();
  p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  optimiseTrajectory(robot, baseTraj, OPTIMSTEP, OPTIMTIME);
  setSafetyDistance(robot, 0);
  p3d_col_activate_obj_env(robot->curObjectJnt->o);
  p3d_col_env_set_traj_method(testcolMethod);
  if (justinTraj) {
    p3d_concat_traj(justinTraj, baseTraj);
  }
  unFixAllJointsExceptBaseAndObject(robot);
  return justinTraj;
}

/**
 * @brief Plan a trajectory for the robot from its start configuration to a final configuration grasping the object. If the base is too far from the object move the base first, then grasp the object.
 * @param robot The robot
 * @param objectStartPos The start position of the object to manipulate
 * @param att1 The attach matrix for the first constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @param att2 The attach matrix for the second constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @return the computed trajectory
 */
traj* pickObject(p3d_rob* robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
  //try to reach the object without moving the base.
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = NULL;
  if(!(conf = setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, cntrtToActivate, true))){
    return platformGotoObjectByMat(robot, objectStartPos, att1, att2);
  }else{
    return gotoObjectByConf(robot, objectStartPos, conf, true);
  }
}

/**
 * @brief Plan a trajectory for the robot from its start configuration to a final configuration grasping the object.
 * @param robot The robot
 * @param objectStartPos The start position of the object to manipulate
 * @param att1 The attach matrix for the first constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @param att2 The attach matrix for the second constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @return the computed trajectory
 */
p3d_traj* gotoObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, cntrtToActivate, true);
  if(conf == NULL){
    printf("No position found\n");
    return NULL;
  }
  return gotoObjectByConf(robot, objectStartPos, conf, true);
}

/**
 * @brief Plan a trajectory for the robot without the platform from its start configuration to a final configuration.
 * @param robot The robot
 * @param objectStartPos The start object position
 * @param conf The final configuration
 * @return The computed trajectory
 */
p3d_traj* gotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf, bool biDir){
#ifdef MULTILOCALPATH
	p3d_multiLocalPath_disable_all_groupToPlan(robot);
    p3d_multiLocalPath_set_groupToPlan(robot, UpBodyMLP, 1) ;
#else
  p3d_local_set_planner((p3d_localplanner_type)1);
#endif
  //Select and activate the right graph
  XYZ_GRAPH = robot->preComputedGraphs[1];
  robot->GRAPH = robot->preComputedGraphs[1];
  deactivateCcCntrts(robot, -1);
  for (int i  = 0; i < robot->nbCcCntrts; i++) {
    desactivateTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->ccCntrts[i]->pasjnts[robot->ccCntrts[i]->npasjnts - 1]);
  }
//  p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
//  p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->curObjectJnt, objectStartPos);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  rrtOptions();
  if (!biDir) {
    ENV.setBool(Env::biDir,false);//monodirectionnal
  }
  int success = findPath();
  //optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
//  unFixJoint(robot, robot->curObjectJnt);
//  unFixJoint(robot, robot->baseJnt);
//  p3d_col_env_set_traj_method(testcolMethod);
  if(success)
    return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  else {
    return NULL;
  }

}

/**
 * @brief Plan a trajectory for the robot from its start configuration to a final configuration grasping the object.
 * @param robot The robot
 * @param objectStartPos The start position of the object to manipulate
 * @param att1 The attach matrix for the first constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @param att2 The attach matrix for the second constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @return the computed trajectory
 */
p3d_traj* touchObjectByMat(p3d_rob * robot, p3d_matrix4 objectStartPos, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectStartPos, att1, att2, FALSE, cntrtToActivate, true);
  if(conf == NULL){
    printf("No position found\n");
    return NULL;
  }
  return touchObjectByConf(robot, objectStartPos, conf);
}

/**
 * @brief Plan a trajectory for the robot without the platform from its start configuration to a final configuration.
 * @param robot The robot
 * @param objectStartPos The start object position
 * @param conf The final configuration
 * @return The computed trajectory
 */
p3d_traj* touchObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf){
  deactivateHandsVsObjectCol(robot);
#ifdef MULTILOCALPATH
	p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_set_groupToPlan(robot, UpBodyMLP, 1) ;
#else
  p3d_local_set_planner((p3d_localplanner_type)1);
#endif
  //Select and activate the right graph
  XYZ_GRAPH = NULL;
  robot->GRAPH = NULL;
  deactivateCcCntrts(robot, -1);
  p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
  p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
  
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->curObjectJnt, objectStartPos);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  rrtOptions();
#ifdef WITH_XFORMS
  CB_DiffusionMethod_obj(NULL, 1);
#endif
  findPath();
  p3d_traj* traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  optimiseTrajectory(robot, traj, OPTIMSTEP, OPTIMTIME);
  unFixJoint(robot, robot->curObjectJnt);
  unFixJoint(robot, robot->baseJnt);
  p3d_col_env_set_traj_method(testcolMethod);
  activateHandsVsObjectCol(robot);
  return traj;
}

/**
 * @brief Plan a trajectory for the robot carring the object from its start configuration to a final configuration where the end configuration of the plate is reachable. If the base is too far from the object final configuration, the base moves first, then deposit the object. Otherwise The robot just transfert the object without moving the base
 * @param robot The robot
 * @param objectGotoPos The goal position of the object to manipulate
 * @param att1 The attach matrix for the first constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @param att2 The attach matrix for the second constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @return the computed trajectory
 */
traj* carryObject(p3d_rob* robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
  //try to reach the object without moving the base.
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = NULL;
  if(!(conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectGotoPos, att1, att2, FALSE, cntrtToActivate, true))){
    configPt conf = setTwoArmsRobotGraspPosWithHold(robot, objectGotoPos, att1, att2, cntrtToActivate);
    if(conf == NULL){
      printf("No position found\n");
      return NULL;
    }
    p3d_traj* carry = platformCarryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
    p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
    p3d_traj* deposit = carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate, TRUE, true);
    if (carry && deposit) {
      p3d_concat_traj(carry, deposit);
    }
    return carry;
  }else{
    return carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate, TRUE, true);
  }
}

/**
 * @brief Plan a trajectory for the robot grasping the object from its start configuration to a final configuration given the goal position of the object.
 * @param robot The robot
 * @param objectGotoPos The goal position of the object to manipulate
 * @param att1 The attach matrix for the first constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @param att2 The attach matrix for the second constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @return the computed trajectory
 */
p3d_traj* carryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectGotoPos, att1, att2, FALSE, cntrtToActivate, true);
  if(conf == NULL){
    printf("No position found\n");
    return NULL;
  }
  return carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate, TRUE, true);
}

/**
 * @brief Plan a trajectory for the robot carring the object without the platform from its start configuration to a final configuration.
 * @param robot The robot
 * @param objectGotoPos The goal object position
 * @param conf The final configuration
 * @param cntrtToActivate the constraint to activate
 * @return The computed trajectory
 */
p3d_traj* carryObjectByConf(p3d_rob * robot, p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate, int cartesian, bool biDir){
  deactivateHandsVsObjectCol(robot);
#ifdef MULTILOCALPATH
	p3d_multiLocalPath_disable_all_groupToPlan(robot);
    p3d_multiLocalPath_set_groupToPlan(robot, UpBodyMLP, 1);
	p3d_multiLocalPath_set_groupToPlan(robot, ObjectMLP, 1);
#else
  //   Linear
  p3d_local_set_planner((p3d_localplanner_type)1);
#endif
//   deleteAllGraphs();
  //Select and activate the right graph
  XYZ_GRAPH = robot->preComputedGraphs[3];
  robot->GRAPH = robot->preComputedGraphs[3];
//Select and activate the right constraint
  if (cartesian || cntrtToActivate == -1) {
    activateCcCntrts(robot, cntrtToActivate, true);
    unFixJoint(robot, robot->curObjectJnt);
  }else {
    //create a constraint linking the object to the corresponding arms
    for (int i  = 0; i < robot->nbCcCntrts; i++) {
      desactivateTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->ccCntrts[i]->pasjnts[robot->ccCntrts[i]->npasjnts - 1]);
    }
    setAndActivateTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->ccCntrts[cntrtToActivate]->pasjnts[robot->ccCntrts[cntrtToActivate]->npasjnts - 1]);    
  }

#ifndef GRASP_PLANNING
  unFixAllJointsExceptBaseAndObject(robot);
#endif
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  if(cartesian){
    shootTheObjectArroundTheBase(robot, robot->baseJnt,robot->curObjectJnt, -2);
  }
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  rrtOptions();
  if (!biDir) {
    ENV.setBool(Env::biDir,false);//monodirectionnal
  }
  int success = findPath();
  //optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  //activateHandsVsObjectCol(robot);
  if (cartesian) {
    shootTheObjectInTheWorld(robot, robot->curObjectJnt);
  }
  if (success) {
    return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ); 
  }else {
    return NULL;
  }

  
}

/**
 * @brief Plan a trajectory for the robot platform grasping the object from its start configuration to a final configuration given the goal position of the object. First move the robot (all exept the base) to the carry configuration, then move the base to the computed goal configuration
 * @param robot The robot
 * @param objectGotoPos The goal position of the object to manipulate
 * @param att1 The attach matrix for the first constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @param att2 The attach matrix for the second constraint. If the first line of the attach matrix is equal to 0, that mean that the robot have to grasp the object only using the other constraint.
 * @return the computed trajectory
 */
p3d_traj* platformCarryObjectByMat(p3d_rob * robot, p3d_matrix4 objectGotoPos, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
  configPt conf = setTwoArmsRobotGraspPosWithHold(robot, objectGotoPos, att1, att2, cntrtToActivate);
  if(conf == NULL){
    printf("No position found\n");
    return NULL;
  }
  return platformCarryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
}

/**
 * @brief Plan a trajectory for the robot platform carring the object from its start configuration to a final configuration. First move the robot and the object to the moving configuration, then move the base to the computed goal configuration
 * @param robot The robot
 * @param objectGotoPos The goal object position
 * @param conf The final configuration
 * @param cntrtToActivate the constraint to activate
 * @return The computed trajectory
 */
p3d_traj* platformCarryObjectByConf(p3d_rob * robot,  p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate){
  //Extract traj
  //   Linear
  activateCcCntrts(robot, cntrtToActivate, true);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  p3d_traj* extractTraj = carryObjectByConf(robot, objectGotoPos, adaptedConf, cntrtToActivate, TRUE, true);
  //base Moving
  shootTheObjectInTheWorld(robot, robot->curObjectJnt);
  deactivateHandsVsObjectCol(robot);
  p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
  p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
#ifdef MULTILOCALPATH
	p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_set_groupToPlan(robot, BaseMLP, 1);
	p3d_multiLocalPath_set_groupToPlan(robot, HeadMLP, 1);
	p3d_multiLocalPath_set_groupToPlan(robot, ObjectMLP, 1);
#else
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
#endif
  deleteAllGraphs();
  p3d_set_and_update_robot_conf(conf);
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, robot->closedChainConf, &adaptedConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  unFixJoint(robot, robot->baseJnt);
  fixAllJointsExceptBaseAndObject(robot, robot->closedChainConf);
  deactivateCcCntrts(robot, cntrtToActivate);
  setAndActivateTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->baseJnt);
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, adaptedConf);
  setSafetyDistance(robot, (double)SAFETY_DIST);
  rrtOptions();
  findPath();
  p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  optimiseTrajectory(robot, baseTraj, OPTIMSTEP, OPTIMTIME);
  setSafetyDistance(robot, 0);
  activateHandsVsObjectCol(robot);
  desactivateTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->baseJnt);
  p3d_col_env_set_traj_method(testcolMethod);
  if (extractTraj) {
    p3d_concat_traj(extractTraj, baseTraj);
  }
  return extractTraj;
}

/** ////////////////////////////////////////////
 * ////////////// Offline functions ////////////
 * //////////////////////////////////////////*/

void preComputePlatformGotoObject(p3d_rob * robot, p3d_matrix4 objectStartPos){
  //Select and activate the right graph
  XYZ_GRAPH = robot->preComputedGraphs[0];
  robot->GRAPH = robot->preComputedGraphs[0];
  //Configure the sampling options
  p3d_set_and_update_robot_conf_multisol(robot->openChainConf, NULL);
  unFixJoint(robot, robot->baseJnt);
  fixJoint(robot, robot->curObjectJnt, objectStartPos);
  fixAllJointsExceptBaseAndObject(robot, robot->openChainConf);
  deactivateCcCntrts(robot, -1);
  setSafetyDistance(robot, (double)SAFETY_DIST);
  p3d_col_deactivate_obj_env(robot->curObjectJnt->o);
  offlinePlannerOptions();
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
  p3d_col_activate_obj_env(robot->curObjectJnt->o);
  setSafetyDistance(robot, 0);
  p3d_set_tmax(0);
  //Save the graph
  robot->preComputedGraphs[0] = XYZ_GRAPH;
}
void preComputeGotoObject(p3d_rob * robot, p3d_matrix4 objectStartPos){
  //Select and activate the right graph
  XYZ_GRAPH = robot->preComputedGraphs[1];
  robot->GRAPH = robot->preComputedGraphs[1];
  //Configure the sampling options
  p3d_set_and_update_robot_conf_multisol(robot->ROBOT_POS, NULL);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->abs_pos);
  fixJoint(robot, robot->curObjectJnt, objectStartPos);
#ifndef GRASP_PLANNING
  unFixAllJointsExceptBaseAndObject(robot);
#endif
  deactivateCcCntrts(robot, -1);
  offlineMgPlannerOptions();
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
  p3d_set_tmax(0);
  //Save the graph
  robot->preComputedGraphs[1] = XYZ_GRAPH;
}
void preComputePlatformCarryObject(p3d_rob * robot){
  //Select and activate the right graph
  XYZ_GRAPH = robot->preComputedGraphs[2];
  robot->GRAPH = robot->preComputedGraphs[2];
  //Configure the sampling options
  p3d_set_and_update_robot_conf_multisol(robot->closedChainConf, NULL);
  unFixJoint(robot, robot->baseJnt);
  unFixJoint(robot, robot->curObjectJnt);
  fixAllJointsExceptBaseAndObject(robot, robot->closedChainConf);
  deactivateCcCntrts(robot, -1);
  setAndActivateTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->baseJnt);
  setSafetyDistance(robot, (double)SAFETY_DIST);
  offlinePlannerOptions();
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
  setSafetyDistance(robot, 0);
  p3d_set_tmax(0);
  //Save the graph
  robot->preComputedGraphs[2] = XYZ_GRAPH;
}
void preComputeCarryObject(p3d_rob * robot, p3d_matrix4 att1, p3d_matrix4 att2){
  int cntrtToActivate = -1;
  if(att1[0][0] == 0 && att1[0][1] == 0 && att1[0][2] == 0){//null attach frame
    cntrtToActivate = 1;
  }else if(att2[0][0] == 0 && att2[0][1] == 0 && att2[0][2] == 0){
    cntrtToActivate  = 0;
  }

  //Configure the sampling options
  p3d_set_and_update_robot_conf_multisol(robot->ROBOT_POS, NULL);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->abs_pos);
  unFixJoint(robot, robot->curObjectJnt);
  unFixAllJointsExceptBaseAndObject(robot);
  activateCcCntrts(robot, cntrtToActivate, true);
  deactivateHandsVsObjectCol(robot);
  shootTheObjectArroundTheBase(robot, robot->baseJnt,robot->curObjectJnt, -2);
  offlinePlannerOptions();
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
  p3d_set_tmax(0);
  activateHandsVsObjectCol(robot);
  //Save the graph
  robot->preComputedGraphs[3] = XYZ_GRAPH;
}

/** ////////////////////////////////////////////
 * ////////////// Grasping functions ///////////
 * ///////////////////////////////////////////*/
//#define GRASP_PLANNING
#if defined(GRASP_PLANNING)
// gpGrasp DEBUGGRASP;
// int WHICHHANDDEBUG = 0;
// void debugLightPlanner(){
// 
//   p3d_matrix4 handFrame, tAtt;
//   gpHand_properties hand;
//   switch(WHICHHANDDEBUG){
//     case 1:{
//       hand.initialize(GP_SAHAND_RIGHT);
//       break;
//     }
//     case 2:{
//       hand.initialize(GP_SAHAND_LEFT);
//       break;
//     }
//   }
//   
//   p3d_mat4Mult(DEBUGGRASP.frame, hand.Tgrasp_frame_hand, handFrame);
//   p3d_matrix4 objectpos, objHandFrame;
//   p3d_mat4Copy(XYZ_ROBOT->curObjectJnt->abs_pos, objectpos);
//   p3d_mat4Mult(objectpos, handFrame, objHandFrame);
// 
//   p3d_mat4Mult(objHandFrame, XYZ_ROBOT->ccCntrts[WHICHHANDDEBUG - 1]->Tatt2, tAtt);
//   g3d_draw_frame(objHandFrame, 0.1);
// 
//   g3d_draw_frame(tAtt, 0.1);
//   DEBUGGRASP.draw(0.05);
// }

p3d_traj* graspTheObject(p3d_rob * robot, p3d_matrix4 objectStartPos, int* whichArm, gpGrasp* curGrasp, bool cartesian){
//   configPt startConfig = p3d_copy_config(robot, robot->ROBOT_POS);
  
  gpHand_properties leftHand, rightHand;
  leftHand.initialize(GP_SAHAND_LEFT);
  rightHand.initialize(GP_SAHAND_RIGHT);
  p3d_matrix4 tAtt = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
  int newGraspingArm = 0;
  //Check if the robot is already carring an object. if yes compute the tatt
  if(*whichArm == 1 || *whichArm == 2){
    p3d_mat4Copy(robot->ccCntrts[*whichArm - 1]->Tatt, tAtt);
    newGraspingArm = *whichArm;
  }else if(*whichArm > 2){
    printf("The two arms are grasping the object or there is more than two arms\n");
    return NULL;
  }
  configPt graspConfig = p3d_alloc_config(robot);
  configPt approachConfig = p3d_alloc_config(robot);

  gpFix_hand_configuration(robot, rightHand, 1);
  gpFix_hand_configuration(robot, leftHand, 2);
  gpSet_hand_rest_configuration(robot, rightHand, 1);
  gpSet_hand_rest_configuration(robot, leftHand, 2);
  configPt startConfig = p3d_get_robot_config(robot);
  
  if(selectHandAndGetGraspApproachConfigs(robot, robot->curObjectJnt->abs_pos, tAtt, &graspConfig, &approachConfig, curGrasp, &newGraspingArm, cartesian)){
    printf("Error no configuration is found, move the object\n");
    p3d_destroy_config(robot, graspConfig);
    p3d_destroy_config(robot, approachConfig);
    return NULL;
  }
  *whichArm = newGraspingArm;
//   WHICHHANDDEBUG = *newGraspingArm;
  p3d_copy_config_into(robot, startConfig, &(robot->ROBOT_POS));
  p3d_set_and_update_this_robot_conf(robot, startConfig);
  gpDeactivate_hand_selfcollisions(robot, 1);
  gpDeactivate_hand_selfcollisions(robot, 2);
  p3d_traj* approachTraj = gotoObjectByConf(robot, robot->curObjectJnt->abs_pos, approachConfig, true);
  p3d_copy_config_into(robot, approachConfig, &(robot->ROBOT_POS));
  gpActivate_hand_selfcollisions(robot, newGraspingArm);
  p3d_set_and_update_this_robot_conf(robot, approachConfig);
  p3d_traj* graspTraj = gotoObjectByConf(robot, robot->curObjectJnt->abs_pos, graspConfig, true);

  if (approachTraj) {
    p3d_concat_traj(approachTraj, graspTraj);
  }

//   DEBUGGRASP = grasp;
//   g3d_win* win= g3d_get_cur_win();
//   win->fct_draw2= &(debugLightPlanner);
//   g3d_draw_allwin();
//   g3d_draw_allwin_active();

  return approachTraj;
}

p3d_traj* carryTheObject(p3d_rob * robot, p3d_matrix4 objectGotoPos, gpGrasp grasp, int whichArm, bool cartesian){
  //Stick the robotObject to the virtual object
  p3d_set_object_to_carry_to_arm(robot, whichArm, (char*)GP_OBJECT_NAME_DEFAULT);
  p3d_matrix4 tAtt;
  configPt graspConfig = p3d_alloc_config(robot);
//  configPt approachConfig = p3d_alloc_config(robot);
  gpHand_properties handProp;
  if(whichArm == 1){
    handProp.initialize(GP_SAHAND_RIGHT);
  }else if(whichArm == 2){
    handProp.initialize(GP_SAHAND_LEFT);
  }
  configPt startConfig = p3d_copy_config(robot, robot->ROBOT_POS);
  showConfig(startConfig);
//   if(getCollisionFreeGraspAndApproach(robot, objectGotoPos, handProp, grasp, whichArm, tAtt, &graspConfig, &approachConfig)){
//     printf("The robot can not reach the goto configuration of the object using this grasp\n");
//     return NULL;
//   }
  p3d_copy_config_into(robot, startConfig, &(robot->ROBOT_POS));
  p3d_set_and_update_this_robot_conf(robot, startConfig);
  p3d_mat4Copy(tAtt, robot->ccCntrts[whichArm -1]->Tatt);
  gpDeactivate_hand_selfcollisions(robot, 1);
  gpDeactivate_hand_selfcollisions(robot, 2);
  p3d_traj *carry = carryObjectByConf(robot, objectGotoPos, graspConfig, whichArm -1, cartesian, true);
  return carry;
}

#endif


//! Finds the best position where to exchange an object between two hands.
//! \param object pointer to the object (a freeflying robot)
//! \param Oi initial object position
//! \param Of final object position
//! \param Ai initial position of the hand that will pick the object up
//! \param Af final position of the hand that will pick the object up
//! \param Bi initial position of the hand that will place the object at its final position
//! \param Bf final position of the hand that will place the object at its final position
//! \param E best position where the two hands can exchange the object
//! \return 0 in case of success, 1 otherwise
int findBestExchangePosition(p3d_rob *object, p3d_vector3 Oi, p3d_vector3 Of, p3d_vector3 Ai, p3d_vector3 Af, p3d_vector3 Bi, p3d_vector3 Bf, p3d_vector3 result)
{
  if(object==NULL)
  {
    printf("%s: %d: findBestExchangePosition(): input p3d_rob is NULL.\n", __FILE__, __LINE__);
    return 1;
  }

  unsigned int i, j, k, Nx, Ny, Nz;
  int minFound= FALSE;
  double  dx, dy, dz, cost, minCost;
  double dimX, dimY, dimZ;
  double xmin, xmax, ymin, ymax, zmin, zmax;
  double dOiOf, dAiOi, dOfBf, dOiE, dBiE, dEOf, dEAf;
  p3d_vector3 OiOf, AiOi, OfBf, OiE, BiE, EOf, EAf;
  p3d_vector3 origin, E, Ebest;
  p3d_matrix4 pose;
  configPt q= NULL;

  q= p3d_alloc_config(object);
  p3d_get_robot_config_into(object, &q);

  xmin= MIN( MIN(MIN(Oi[0],Of[0]),MIN(Ai[0],Af[0])), MIN(Bi[0],Bf[0]) );
  xmax= MAX( MAX(MAX(Oi[0],Of[0]),MAX(Ai[0],Af[0])), MAX(Bi[0],Bf[0]) );
  ymin= MIN( MIN(MIN(Oi[1],Of[1]),MIN(Ai[1],Af[1])), MIN(Bi[1],Bf[1]) );
  ymax= MAX( MAX(MAX(Oi[1],Of[1]),MAX(Ai[1],Af[1])), MAX(Bi[1],Bf[1]) );
  zmin= MIN( MIN(MIN(Oi[2],Of[2]),MIN(Ai[2],Af[2])), MIN(Bi[2],Bf[2]) );
  zmax= MAX( MAX(MAX(Oi[2],Of[2]),MAX(Ai[2],Af[2])), MAX(Bi[2],Bf[2]) );

  if(xmin < 0) xmin*= 1.2; else xmin*= 0.8;
  if(ymin < 0) ymin*= 1.2; else ymin*= 0.8;
  if(zmin < 0) zmin*= 1.2; else zmin*= 0.8;

  if(xmax > 0) xmax*= 1.2; else xmax*= 0.8;
  if(ymax > 0) ymax*= 1.2; else ymax*= 0.8;
  if(zmax > 0) zmax*= 1.2; else zmax*= 0.8;

  p3d_vectSub(Oi, Of, OiOf); 
  p3d_vectSub(Ai, Oi, AiOi); 
  p3d_vectSub(Of, Bf, OfBf); 

  dOiOf= p3d_vectNorm(OiOf);
  dAiOi= p3d_vectNorm(AiOi);
  dOfBf= p3d_vectNorm(OfBf);

  origin[0]= xmin;
  origin[1]= ymin;
  origin[2]= zmin;
  dimX= xmax - xmin;
  dimY= ymax - ymin;
  dimZ= zmax - zmin;

  Nx= Ny= Nz= 30;

  dx= dimX/((double) Nx);
  dy= dimY/((double) Ny);
  dz= dimZ/((double) Nz);

  minCost= 1e9;

  for(i=0; i<Nx; ++i)
  {
    E[0]= origin[0] + i*dx;

    for(j=0; j<Ny; ++j)
    {
      E[1]= origin[1] + j*dy;
      for(k=0; k<Nz; ++k)
      {
        E[2]= origin[2] + k*dz;

        p3d_mat4Pos(pose, E[0], E[1], E[2], 0, 0, 0);
        p3d_set_freeflyer_pose(object, pose);
        if(p3d_col_test_robot_statics(object, 0))
        {  continue;  }
        p3d_mat4Pos(pose, E[0], E[1], E[2], 90, 0, 0);
        p3d_set_freeflyer_pose(object, pose);
        if(p3d_col_test_robot_statics(object, 0))
        {  continue;  }
        p3d_mat4Pos(pose, E[0], E[1], E[2], -90, 0, 0);
        p3d_set_freeflyer_pose(object, pose);
        if(p3d_col_test_robot_statics(object, 0))
        {  continue;  }
        p3d_mat4Pos(pose, E[0], E[1], E[2], 0, 90, 0);
        p3d_set_freeflyer_pose(object, pose);
        if(p3d_col_test_robot_statics(object, 0))
        {  continue;  }
        p3d_mat4Pos(pose, E[0], E[1], E[2], 0, -90, 0);
        p3d_set_freeflyer_pose(object, pose);
        if(p3d_col_test_robot_statics(object, 0))
        {  continue;  }
        p3d_mat4Pos(pose, E[0], E[1], E[2], 0, 0, 90);
        p3d_set_freeflyer_pose(object, pose);
        if(p3d_col_test_robot_statics(object, 0))
        {  continue;  }
        p3d_mat4Pos(pose, E[0], E[1], E[2], 0, 0, -90);
        p3d_set_freeflyer_pose(object, pose);
        if(p3d_col_test_robot_statics(object, 0))
        {  continue;  }

        p3d_vectSub(E, Oi, OiE);
        p3d_vectSub(E, Bi, BiE);
        p3d_vectSub(Of, E, EOf);
        p3d_vectSub(Af, E, EAf);

        dOiE= p3d_vectNorm(OiE);
        dBiE= p3d_vectNorm(BiE);
        dEOf= p3d_vectNorm(EOf);
        dEAf= p3d_vectNorm(EAf);

        cost= dAiOi + dOiE + dBiE + dEOf + dOfBf + dEAf;

//         cost+= fabs(dAiOi + dOiE -dBiE) + fabs(dEOf + dOfBf - dEAf);
        if(cost < minCost)
        {
           minFound= TRUE;
           minCost= cost;
           Ebest[0]= E[0];
           Ebest[1]= E[1];
           Ebest[2]= E[2];
        }


      }
    }
  }


  if(minFound==TRUE)
  {
    result[0]= Ebest[0];
    result[1]= Ebest[1];
    result[2]= Ebest[2];
  }
  else
  {
    result[0]= 0.5*(Oi[0] + Of[0]);
    result[1]= 0.5*(Oi[1] + Of[1]);
    result[2]= 0.5*(Oi[2] + Of[2]);
  }

// printf("origin: %f %f %f \n",origin[0],origin[1],origin[2]);
// printf("[%f %f] [%f %f]  [%f %f] \n",xmin,xmax,ymin,ymax,zmin,zmax);
// printf("Oi: %f %f %f \n",Oi[0],Oi[1],Oi[2]);
// printf("Of: %f %f %f \n",Of[0],Of[1],Of[2]);
// printf("best exchange: %f %f %f \n",result[0],result[1],result[2]);


  p3d_set_and_update_this_robot_conf(object, q);
  p3d_destroy_config(object, q);

  return 0;
}

//! Displays the grid that is computed in findBestExchangePosition.
//! The collisions are not tested in this version to make it computable at each frame display.
//! \param object pointer to the object (a freeflying robot)
//! \param Oi initial object position
//! \param Of final object position
//! \param Ai initial position of the hand that will pick the object up
//! \param Af final position of the hand that will pick the object up
//! \param Bi initial position of the hand that will place the object at its final position
//! \param Bf final position of the hand that will place the object at its final position
//! \param E best position where the two hands can exchange the object
//! \return 0 in case of success, 1 otherwise
int findBestExchangePositionGraphic(p3d_rob *object, p3d_vector3 Oi, p3d_vector3 Of, p3d_vector3 Ai, p3d_vector3 Af, p3d_vector3 Bi, p3d_vector3 Bf, p3d_vector3 result)
{
  if(object==NULL)
  {
    printf("%s: %d: findBestExchangePositionGraphic(): input p3d_rob is NULL.\n", __FILE__, __LINE__);
    return 1;
  }

  unsigned int i, j, k, Nx, Ny, Nz;
  int minFound= FALSE;
  double  dx, dy, dz, cost, minCost;
  double dimX, dimY, dimZ;
  double xmin, xmax, ymin, ymax, zmin, zmax;
  double dOiOf, dAiOi, dOfBf, dOiE, dBiE, dEOf, dEAf;
  p3d_vector3 OiOf, AiOi, OfBf, OiE, BiE, EOf, EAf;
  p3d_vector3 origin, E, Ebest;
  configPt q= NULL;

  q= p3d_alloc_config(object);
  p3d_get_robot_config_into(object, &q);

  xmin= 1.2*MIN( MIN(MIN(Oi[0],Of[0]),MIN(Ai[0],Af[0])), MIN(Bi[0],Bf[0]) );
  xmax= 1.2*MAX( MAX(MAX(Oi[0],Of[0]),MAX(Ai[0],Af[0])), MAX(Bi[0],Bf[0]) );
  ymin= MIN( MIN(MIN(Oi[1],Of[1]),MIN(Ai[1],Af[1])), MIN(Bi[1],Bf[1]) );
  ymax= MAX( MAX(MAX(Oi[1],Of[1]),MAX(Ai[1],Af[1])), MAX(Bi[1],Bf[1]) );
  zmin= 1.2*MIN( MIN(MIN(Oi[2],Of[2]),MIN(Ai[2],Af[2])), MIN(Bi[2],Bf[2]) );
  zmax= 1.2*MAX( MAX(MAX(Oi[2],Of[2]),MAX(Ai[2],Af[2])), MAX(Bi[2],Bf[2]) );

  if(xmin < 0) xmin*= 1.2; else xmin*= 0.8;
  if(ymin < 0) ymin*= 1.2; else ymin*= 0.8;
  if(zmin < 0) zmin*= 1.2; else zmin*= 0.8;

  if(xmax > 0) xmax*= 1.2; else xmax*= 0.8;
  if(ymax > 0) ymax*= 1.2; else ymax*= 0.8;
  if(zmax > 0) zmax*= 1.2; else zmax*= 0.8;

  p3d_vectSub(Oi, Of, OiOf); 
  p3d_vectSub(Ai, Oi, AiOi); 
  p3d_vectSub(Of, Bf, OfBf); 

  dOiOf= p3d_vectNorm(OiOf);
  dAiOi= p3d_vectNorm(AiOi);
  dOfBf= p3d_vectNorm(OfBf);

  origin[0]= xmin;
  origin[1]= ymin;
  origin[2]= zmin;
  dimX= xmax - xmin;
  dimY= ymax - ymin;
  dimZ= zmax - zmin;

  Nx= Ny= Nz= 30;
  Nx= Ny= Nz= 40;
  Ny= 100;

  Nx= (unsigned int)(dimX/0.01);
  Ny= (unsigned int)(dimY/0.01);
  Nz= (unsigned int)(dimZ/0.01);


  dx= dimX/((double) Nx);
  dy= dimY/((double) Ny);
  dz= dimZ/((double) Nz);

  minCost= 1e9;
#ifdef GRASP_PLANNING
  double maxCost= -1e9;
  gpVector3D point;
  std::vector<gpVector3D> points;

  for(i=0; i<Nx; ++i)
  {
    E[0]= origin[0] + i*dx;

    for(j=0; j<Ny; ++j)
    {
      E[1]= origin[1] + j*dy;
      for(k=0; k<Nz; ++k)
      {
        E[2]= origin[2] + k*dz;

        p3d_vectSub(E, Oi, OiE);
        p3d_vectSub(E, Bi, BiE);
        p3d_vectSub(Of, E, EOf);
        p3d_vectSub(Af, E, EAf);

        dOiE= p3d_vectNorm(OiE);
        dBiE= p3d_vectNorm(BiE);
        dEOf= p3d_vectNorm(EOf);
        dEAf= p3d_vectNorm(EAf);

        cost= dAiOi + dOiE + dBiE + dEOf + dOfBf + dEAf;

        point.set(E[0], E[1], E[2]);
        point.cost= cost;
        points.push_back(point);

        if(cost > maxCost) { maxCost= cost; }

//         cost+= fabs(dAiOi + dOiE -dBiE) + fabs(dEOf + dOfBf - dEAf);
        if(cost < minCost)
        {
           minFound= TRUE;
           minCost= cost;
           Ebest[0]= E[0];
           Ebest[1]= E[1];
           Ebest[2]= E[2];
        }
      }
    }
  }


  for(unsigned int i=0; i<points.size(); ++i)
  {
    points[i].cost= (points[i].cost - minCost)/(maxCost - minCost);
  }

  double color[4];
  glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
  glEnable(GL_BLEND);
  glDisable(GL_LIGHTING);
  glPointSize(10);
  glBegin(GL_POINTS);  
    for(unsigned int i=0; i<points.size(); ++i)
    {
      g3d_rgb_from_hue(points[i].cost, color);
      glColor4f(color[0], color[1], color[2], 0.3);
      glVertex3d(points[i].x, points[i].y, points[i].z);
    }
  glEnd();
  glPopAttrib();


  if(minFound==TRUE)
  {
    result[0]= Ebest[0];
    result[1]= Ebest[1];
    result[2]= Ebest[2];
  }
  else
  {
    result[0]= 0.5*(Oi[0] + Of[0]);
    result[1]= 0.5*(Oi[1] + Of[1]);
    result[2]= 0.5*(Oi[2] + Of[2]);
  }
	
#endif
// printf("origin: %f %f %f \n",origin[0],origin[1],origin[2]);
// printf("[%f %f] [%f %f]  [%f %f] \n",xmin,xmax,ymin,ymax,zmin,zmax);
// printf("Oi: %f %f %f \n",Oi[0],Oi[1],Oi[2]);
// printf("Of: %f %f %f \n",Of[0],Of[1],Of[2]);
// printf("best exchange: %f %f %f \n",result[0],result[1],result[2]);


  p3d_set_and_update_this_robot_conf(object, q);
  p3d_destroy_config(object, q);

  return 0;
}

//! This function is the same as findBestExchangePosition but it receives matrices instead of vectors.
//! Only the translation part of the matrices is used in the function.
int findBestExchangePosition2(p3d_rob *object, p3d_matrix4 Oi, p3d_matrix4 Of, p3d_matrix4 Ai, p3d_matrix4 Af, p3d_matrix4 Bi, p3d_matrix4 Bf, p3d_matrix4 result)
{
  if(object==NULL)
  {
    printf("%s: %d: findBestExchangePosition2(): input p3d_rob is NULL.\n", __FILE__, __LINE__);
    return 1;
  }
 
  int i;

  p3d_vector3 oi, of, ai, af, bi, bf, e;

  for(i=0; i<3; ++i)
  {
    oi[i]= Oi[i][3];
    of[i]= Of[i][3];
    ai[i]= Ai[i][3];
    af[i]= Af[i][3];
    bi[i]= Bi[i][3];
    bf[i]= Bf[i][3];
  }

  findBestExchangePosition(object, oi, of, ai, af, bi, bf, e);
  for(i=0; i<3; ++i){
    result[i][3] = e[i];
  }

  return 0;
}
