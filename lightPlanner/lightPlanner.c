#ifdef LIGHT_PLANNER

#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "Move3d-pkg.h"
#include "P3d-pkg.h"
#include "../lightPlanner/proto/lightPlanner.h"
#include "../lightPlanner/proto/lightPlannerApi.h"
#include "../lightPlanner/proto/robotPos.h"
#include "../graphic/proto/g3d_draw_traj_proto.h"
///////////////////////////
//// Static functions /////
///////////////////////////
static int saveCurrentConfigInFile(p3d_rob* robot, p3d_localpath* curLp);
static int saveSpecifiedConfigInFile(configPt conf);
static void pathGraspOptions(void);
static void closedChainPlannerOptions(void);
static void findPath(void);

extern double SAFETY_DIST;
extern double USE_LIN;
#define OPTIMSTEP 200
#define OPTIMTIME 4 //4 seconds
/** @brief File used to save the trajectory*/
static FILE* trajFile = NULL;

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
}

/**
 * @brief Parameters for the planner
 */
static void pathGraspOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
  CB_DiffusionMethod_obj(NULL, 1); //0 rrt Connect, 1 rrt extend
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
static void closedChainPlannerOptions(void) {
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

/**
 * @brief Launch the planner
 */
static void findPath(void) {
  p3d_specific_search((char*)"");
}

/**
 * @brief Set the number of optimisations steps and optimise the current trajectory of the robot
 * @param nbSteps number of steps for random smoothing
 * @param maxTime The maximum time that the planner have to spend to smooth the trajectory
 */
void optimiseTrajectory(int nbSteps, double maxTime) {
  p3d_set_NB_OPTIM(nbSteps);
  if(maxTime <= 0){
    p3d_set_use_optimization_time(false);
  }else{
  p3d_set_use_optimization_time(true);
  p3d_set_optimization_time(maxTime);
  }
  CB_start_optim_obj(NULL, 0);
}
/** ////////////////////////////////////////////
 * //////////// Planning functions ////////////
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
  //try to reach the object without moving the base.
  p3d_set_and_update_this_robot_conf_without_cntrt(robot, robot->ROBOT_POS);
  if(!setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, cntrtToActivate)){
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
  p3d_local_set_planner((p3d_localplanner_type)1);
  deleteAllGraphs();
  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  fixJoint(robot, robot->objectJnt, objectStartPos);
  p3d_update_this_robot_pos(robot);
  p3d_copy_config_into(robot, transfertConf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, transfertConf);
  pathGraspOptions();
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
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
  p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, newConf, &(robot->ROBOT_GOTO));
  setSafetyDistance(robot, (double)SAFETY_DIST);
  p3d_col_deactivate_obj_env(robot->objectJnt->o);
  deleteAllGraphs();
  pathGraspOptions();
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
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
  //try to reach the object without moving the base.
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = NULL;
  if(!(conf = setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, cntrtToActivate))){
    return platformGotoObjectByMat(robot, objectStartPos, att1, att2);
  }else{
    return gotoObjectByConf(robot, objectStartPos, conf);
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
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = setTwoArmsRobotGraspApproachPosWithoutBase(robot, objectStartPos, att1, att2, cntrtToActivate);
  if(conf == NULL){
    printf("No position found\n");
    return NULL;
  }
  return gotoObjectByConf(robot, objectStartPos, conf);
}

/**
 * @brief Plan a trajectory for the robot without the platform from its start configuration to a final configuration.
 * @param robot The robot
 * @param objectStartPos The start object position
 * @param conf The final configuration
 * @return The computed trajectory
 */
p3d_traj* gotoObjectByConf(p3d_rob * robot,  p3d_matrix4 objectStartPos, configPt conf){
  p3d_local_set_planner((p3d_localplanner_type)1);
  deleteAllGraphs();
  deactivateCcCntrts(robot, -1);
  p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
  p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);

  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->objectJnt, objectStartPos);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, conf);
  pathGraspOptions();
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  unFixJoint(robot, robot->objectJnt);
  unFixJoint(robot, robot->baseJnt);
  p3d_col_env_set_traj_method(testcolMethod);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
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
  //try to reach the object without moving the base.
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = NULL;
  if(!(conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectGotoPos, att1, att2, cntrtToActivate))){
    configPt conf = setTwoArmsRobotGraspPosWithHold(robot, objectGotoPos, att1, att2, cntrtToActivate);
    if(conf == NULL){
      printf("No position found\n");
      return NULL;
    }
    p3d_traj* carry = platformCarryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
    p3d_copy_config_into(robot, robot->ROBOT_GOTO, &(robot->ROBOT_POS));
    p3d_traj* deposit = carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
    if (carry && deposit) {
      p3d_concat_traj(carry, deposit);
    }
    return carry;
  }else{
    return carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
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
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectGotoPos, att1, att2, cntrtToActivate);
  if(conf == NULL){
    printf("No position found\n");
    return NULL;
  }
  return carryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
}

/**
 * @brief Plan a trajectory for the robot carring the object without the platform from its start configuration to a final configuration.
 * @param robot The robot
 * @param objectGotoPos The goal object position
 * @param conf The final configuration
 * @param cntrtToActivate the constraint to activate
 * @return The computed trajectory
 */
p3d_traj* carryObjectByConf(p3d_rob * robot, p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate){
  deactivateHandsVsObjectCol(robot);
  //   Linear
  p3d_local_set_planner((p3d_localplanner_type)1);
  deleteAllGraphs();
  activateCcCntrts(robot, cntrtToActivate);
  unFixJoint(robot, robot->objectJnt);
  unFixAllJointsExceptBaseAndObject(robot);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  shootTheObjectArroundTheBase(robot, robot->baseJnt,robot->objectJnt, -2);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  pathGraspOptions();
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  activateHandsVsObjectCol(robot);
  shootTheObjectInTheWorld(robot, robot->objectJnt);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
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
  configPt conf = setTwoArmsRobotGraspPosWithHold(robot, objectGotoPos, att1, att2, cntrtToActivate);
  if(conf == NULL){
    printf("No position found\n");
    return NULL;
  }
  return platformCarryObjectByConf(robot, objectGotoPos, conf, cntrtToActivate);
}

/**
 * @brief Plan a trajectory for the robot platform carring the object from its start configuration to a final configuration. First move the robot (all exept the base and the object) to the moving configuration, then move the base to the computed goal configuration
 * @param robot The robot
 * @param objectGotoPos The goal object position
 * @param conf The final configuration
 * @param cntrtToActivate the constraint to activate
 * @return The computed trajectory
 */
p3d_traj* platformCarryObjectByConf(p3d_rob * robot,  p3d_matrix4 objectGotoPos, configPt conf, int cntrtToActivate){
  //Extract traj
  //   Linear
  activateCcCntrts(robot, cntrtToActivate);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  p3d_traj* extractTraj = carryObjectByConf(robot, objectGotoPos, adaptedConf, cntrtToActivate);
  //base Moving
  shootTheObjectInTheWorld(robot, robot->objectJnt);
  deactivateHandsVsObjectCol(robot);
  p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
  p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);
  if(USE_LIN){
    p3d_local_set_planner((p3d_localplanner_type)1);
  }
  else{
    p3d_local_set_planner((p3d_localplanner_type)0);
  }
  deleteAllGraphs();
  p3d_set_and_update_robot_conf(conf);
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_POS));
  p3d_copy_config_into(robot, robot->closedChainConf, &adaptedConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  unFixJoint(robot, robot->baseJnt);
  fixAllJointsExceptBaseAndObject(robot, robot->closedChainConf);
  deactivateCcCntrts(robot, cntrtToActivate);
  setAndActivateTwoJointsFixCntrt(robot, robot->objectJnt, robot->baseJnt);
  p3d_copy_config_into(robot, adaptedConf, &(robot->ROBOT_GOTO));
  p3d_destroy_config(robot, adaptedConf);
  setSafetyDistance(robot, (double)SAFETY_DIST);
  closedChainPlannerOptions();
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  setSafetyDistance(robot, 0);
  activateHandsVsObjectCol(robot);
  desactivateTwoJointsFixCntrt(robot, robot->objectJnt, robot->baseJnt);
  p3d_col_env_set_traj_method(testcolMethod);
  p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
  if (extractTraj) {
    p3d_concat_traj(extractTraj, baseTraj);
  }
  return extractTraj;
}

#endif
