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
static void rrtOptions(void);
static void findPath(void);

extern double SAFETY_DIST;
extern double USE_LIN;
#define OPTIMSTEP 200
#define OPTIMTIME 10
#define MAXPLANTIME 120 //2 mins
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
}

/**
 * @brief Parameters for the planner
 */
static void rrtOptions(void) {
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);
  CB_DiffusionMethod_obj(NULL, 0); //0 rrt Connect, 1 rrt extend
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
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  p3d_traj* justinTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);

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
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  setSafetyDistance(robot, 0);
  p3d_col_activate_obj_env(robot->curObjectJnt->o);
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
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
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
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
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
  p3d_traj_test_type testcolMethod = p3d_col_env_get_traj_method();
  p3d_col_env_set_traj_method(TEST_TRAJ_OTHER_ROBOTS_CLASSIC_ALL);

  unFixAllJointsExceptBaseAndObject(robot);
  fixJoint(robot, robot->curObjectJnt, objectStartPos);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  rrtOptions();
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  unFixJoint(robot, robot->curObjectJnt);
  unFixJoint(robot, robot->baseJnt);
  p3d_col_env_set_traj_method(testcolMethod);
  return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
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
  configPt conf = setTwoArmsRobotGraspPosWithoutBase(robot, objectStartPos, att1, att2, cntrtToActivate);
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
  CB_DiffusionMethod_obj(NULL, 1);
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  unFixJoint(robot, robot->curObjectJnt);
  unFixJoint(robot, robot->baseJnt);
  p3d_col_env_set_traj_method(testcolMethod);
  activateHandsVsObjectCol(robot);
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
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
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
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(robot);
  p3d_multiLocalPath_enable_all_groupToPlan(robot);
#endif
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
//Select and activate the right graph
  activateCcCntrts(robot, cntrtToActivate);
  unFixJoint(robot, robot->curObjectJnt);
  unFixAllJointsExceptBaseAndObject(robot);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  fixJoint(robot, robot->baseJnt, robot->baseJnt->jnt_mat);
  shootTheObjectArroundTheBase(robot, robot->baseJnt,robot->curObjectJnt, -2);
  p3d_copy_config_into(robot, conf, &(robot->ROBOT_GOTO));
  rrtOptions();
  findPath();
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  activateHandsVsObjectCol(robot);
  shootTheObjectInTheWorld(robot, robot->curObjectJnt);
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
  activateCcCntrts(robot, cntrtToActivate);
  p3d_set_and_update_robot_conf(robot->ROBOT_POS);
  configPt adaptedConf = p3d_copy_config(robot, robot->closedChainConf);
  adaptClosedChainConfigToBasePos(robot, robot->baseJnt->abs_pos, adaptedConf);
  p3d_traj* extractTraj = carryObjectByConf(robot, objectGotoPos, adaptedConf, cntrtToActivate);
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
  optimiseTrajectory(OPTIMSTEP, OPTIMTIME);
  setSafetyDistance(robot, 0);
  activateHandsVsObjectCol(robot);
  desactivateTwoJointsFixCntrt(robot, robot->curObjectJnt, robot->baseJnt);
  p3d_col_env_set_traj_method(testcolMethod);
  p3d_traj* baseTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
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
  unFixAllJointsExceptBaseAndObject(robot);
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
  activateCcCntrts(robot, cntrtToActivate);
  shootTheObjectArroundTheBase(robot, robot->baseJnt,robot->curObjectJnt, -2);
  offlinePlannerOptions();
  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
  p3d_set_tmax(0);
  //Save the graph
  robot->preComputedGraphs[3] = XYZ_GRAPH;
}
