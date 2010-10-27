#include "ManipulationPlanner.hpp"

#include "lightPlanner.h"
#include "lightPlannerApi.h"
//#include "p3d_chanEnv_proto.h"
#ifdef CXX_PLANNER
#include "planner_cxx/plannerFunctions.hpp"
#if defined (USE_CXX_PLANNER)
#include "planEnvironment.hpp"
#endif
#endif

#include "robotPos.h"

#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"

#include <list>
#define MPDEBUG 0
using namespace std;

/* ******************************* */
/* ******* (De)Constructor ******* */
/* ******************************* */
ManipulationPlanner::ManipulationPlanner(p3d_rob *robotPt) {
  _robot = robotPt;
//   _cameraJnt = NULL;
//   _cameraFOV = 60;
//   _cameraImageWidth = 200;
//   _cameraImageHeight = 200;
// 
//   _liftUpDistance = 0.15;
//   _nbGraspsToTestForPickGoto = 1;
//   _placementTranslationStep = 0.2;
//   _placementNbOrientations = 8;
//   _placementOffset = -0.005;
// 
//   _capture          = false;
//   displayGrasps     = false;
//   displayPlacements = false;

  _optimizeSteps = 100;
  _optimizeTime = 4.0; // 4 secondes
  _approachFreeOffset = 0.1; //0.1 meters
  _approachGraspOffset = 0.1; //0.1 meters

#ifdef MULTILOCALPATH
  _BaseMLP = -1;
  _HeadMLP = -1;
  _UpBodyMLP = -1;
  _UpBodySmMLP = -1;


  for (int i = 0; _robot && i < _robot->mlp->nblpGp; i++) {
    if (!strcmp(_robot->mlp->mlpJoints[i]->gpName, "base")) {
      _BaseMLP = i;
    } else if (!strcmp(_robot->mlp->mlpJoints[i]->gpName, "head")) {
      _HeadMLP = i;
    } else if (!strcmp(_robot->mlp->mlpJoints[i]->gpName, "upBody")) {
      _UpBodyMLP = i;
    } else if (!strcmp(_robot->mlp->mlpJoints[i]->gpName, "upBodySm")) {
      _UpBodySmMLP = i;
    }
  }
  if ((_UpBodyMLP == -1) || (_UpBodySmMLP == -1)/* || (_ObjectMLP==-1) || (_ObjectSmMLP==-1)*/) {
    printf("%s: %d: ManipulationPlanner::ManipulationPlanner: cannot find all necessary multiLocalpth groups\n", __FILE__, __LINE__);
    return;
  }
  for (uint i = 0; i < _robot->armManipulationData->size(); i++) {
//   _handProp.at(0).setArmType(GP_LWR);
  if ((*_robot->armManipulationData)[i].getCartesianGroup() == -1) {
    printf("%s: %d: ManipulationPlanner::ManipulationPlanner: the arm type is set to GP_LWR. Use setArmType() to change it.\n",__FILE__,__LINE__);
    return;
    }
  }
#endif
}

ManipulationPlanner::~ManipulationPlanner() {
  //Nothing To do
}

/* ******************************* */
/* *********** Cleaning ********** */
/* ******************************* */
void ManipulationPlanner::clear() {
  _robot = NULL;
}

int ManipulationPlanner::cleanRoadmap() {
  if (_robot != NULL) {
    XYZ_ENV->cur_robot = _robot;
    deleteAllGraphs();
//     FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
  } else {
    return 1;
  }
  return 0;
}

int ManipulationPlanner::cleanTraj() {
  if (_robot != NULL) {
    XYZ_ENV->cur_robot = _robot;
    while (_robot->nt != 0) {
      p3d_destroy_traj(_robot, _robot->t[0]);
    }
#ifdef WITH_XFORMS
    FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
#endif
  } else {
    return 1;
  }
  return 0;
}

/* ******************************* */
/* ******* (Ge)Setters *********** */
/* ******************************* */
void ManipulationPlanner::setOptimizeSteps(int nbSteps){
  if(nbSteps > 0){
    _optimizeSteps = nbSteps;
  }else{
    printf("%s: %d: ManipulationPlanner::setOptimizeSteps() unvalid nbSteps.\n", __FILE__, __LINE__);
    return;
  }
}
int ManipulationPlanner::getOptimizeSteps(void) const{
  return _optimizeSteps;
#ifdef XFORMS
#endif
}

void ManipulationPlanner::setOptimizeTime(double time){
  if(time > 0){
    _optimizeTime = time;
  }else{
    printf("%s: %d: ManipulationPlanner::setOptimizeTime() unvalid time.\n", __FILE__, __LINE__);
    return;
  }
}
double ManipulationPlanner::getOptimizeTime(void) const{
  return _optimizeTime;
}

void ManipulationPlanner::setApproachFreeOffset(double offset){
  _approachFreeOffset = offset;
}
double ManipulationPlanner::getApproachFreeOffset(void) const{
  return _approachFreeOffset;
}

void ManipulationPlanner::setApproachGraspOffset(double offset){
  _approachGraspOffset = offset;
}
double ManipulationPlanner::getApproachGraspOffset(void) const{
  return _approachGraspOffset;
}
/* ******************************* */
/* ******* Planning Modes ******** */
/* ******************************* */
void ManipulationPlanner::checkConfigForCartesianMode(configPt q) {
  bool deleteConfig = false;
  if(q == NULL){
    q = p3d_get_robot_config(_robot);
    deleteConfig = true;
  }
  for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
    ArmManipulationData armData  = (*_robot->armManipulationData)[i];
    if (armData.getCartesian()) {
      /* Uptdate the Virual object for inverse kinematics */
      p3d_update_virtual_object_config_for_arm_ik_constraint(_robot, i, q);
      activateCcCntrts(_robot, i, false);
    }else{
      deactivateCcCntrts(_robot, i);
    }
  }
  p3d_set_and_update_this_robot_conf(_robot, q);
  p3d_get_robot_config_into(_robot, &q);
  if(deleteConfig){
    p3d_destroy_config(_robot, q);
    q = NULL;
  }
}

void ManipulationPlanner::setArmCartesian(int armId, bool cartesian) {
  ArmManipulationData armData  = (*_robot->armManipulationData)[armId];

  armData.setCartesian(cartesian);
  //activate the corresponding group, constraint and set the sampling bounds of the object to sample
  if (cartesian) {
    activateCcCntrts(_robot, armId, 0);
    shootTheObjectArroundTheBase(_robot, _robot->baseJnt, armData.getManipulationJnt(), 2.0);
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_set_groupToPlan(_robot, armData.getCartesianGroup(), 1);
#endif
  } else {
    deactivateCcCntrts(_robot, armId);
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_set_groupToPlan(_robot, armData.getCartesianGroup(), 0);
#endif
    shootTheObjectInTheWorld(_robot, armData.getManipulationJnt());
  }
}
bool ManipulationPlanner::getArmCartesian(int armId) const{
  return (*_robot->armManipulationData)[armId].getCartesian();
}

/* ******************************* */
/* ******* Motion Planning ******* */
/* ******************************* */
/** Concatenes all the current trajectories of the robot into the first one.
 * NB: only the first trajectory will remain (and grown up); the others are destroyed.
 * \param trajs A vector of trajectories
 * \param concatTraj The concatenated trajectory to return
 * \return Message of success or fail */
MANIPULATION_TASK_MESSAGE ManipulationPlanner::concatTrajectories (std::vector<p3d_traj*>& trajs, p3d_traj** concatTraj){
  if ( _robot==NULL ) {
    PrintInfo ( ( "concateneAllTrajectories: robot is NULL.\n" ) );
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  if ( trajs.empty() ) {
    PrintInfo ( ( "concateneAllTrajectories: the trajectory vector is empty.\n" ) );
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  *concatTraj = p3d_create_traj_by_copy(trajs[0]);
  for(int i = 1; i < (int)trajs.size(); i++){
    p3d_concat_traj(*concatTraj, trajs[i]);
  }
  g3d_add_traj((char*)"Task", (*concatTraj)->num);
  return MANIPULATION_TASK_OK;
}

int ManipulationPlanner::computeRRT(int smoothingSteps, double smootingTime, bool biDir) {
  int result;
  p3d_set_MOTION_PLANNER(P3D_DIFFUSION);

#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  ENV.setBool(Env::biDir, true);
  ENV.setInt(Env::NbTry, 10000);
  ENV.setInt(Env::MaxExpandNodeFail, 10000);
  ENV.setInt(Env::maxNodeCompco, 10000);
  ENV.setExpansionMethod(Env::Extend);
  ENV.setDouble(Env::extensionStep, 2.0);


#if defined (USE_CXX_PLANNER)
  ENV.setBool(Env::withSmoothing, true);
  ENV.setBool(Env::withShortCut, true);
  ENV.setBool(Env::withDeformation, false);
  ENV.setInt(Env::nbCostOptimize, smoothingSteps);
  ENV.setInt(Env::timeOptimize, smootingTime);

  ChronoOn();

  result = p3d_run_rrt(_robot->GRAPH, fct_stop, fct_draw);

  ChronoPrint("");
  ChronoOff();
#else
  result = p3d_specific_search((char *)"");
  optimiseTrajectory(smoothingSteps, smootingTime);
#endif
  if (!result) {
    printf("ArmGotoQ: could not find a path.\n");
    ManipulationUtils::printConstraintInfo(_robot);
    return 1;
  }
  return 0;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armComputePRM(double ComputeTime) {
  this->cleanRoadmap();
  
  checkConfigForCartesianMode(NULL);
#ifdef MULTILOCALPATH
  p3d_multiLocalPath_disable_all_groupToPlan(_robot);
  p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);
#endif
  fixJoint(_robot, _robot->baseJnt,_robot->baseJnt->abs_pos);
  fixAllHands(NULL, true);
  /*TODO Fix all free flyers to hands and disable object trasportation*/
  
  p3d_set_RANDOM_CHOICE(P3D_RANDOM_SAMPLING);
  p3d_set_SAMPLING_CHOICE(P3D_UNIFORM_SAMPLING);
  p3d_set_MOTION_PLANNER(P3D_BASIC);
  ENV.setInt(Env::NbTry, 100000);
  double bakComputeTime = p3d_get_tmax();
  p3d_set_tmax(ComputeTime);
#ifdef MULTIGRAPH
  p3d_set_multiGraph(FALSE);
#endif
  p3d_set_ik_choice(IK_NORMAL);
  p3d_set_is_visibility_discreet(0);
  p3d_set_test_reductib(0);
  p3d_set_cycles(0);

  p3d_learn(p3d_get_NB_NODES(), fct_stop, fct_draw);
  p3d_set_tmax(bakComputeTime);
  return MANIPULATION_TASK_OK;
}

p3d_traj* ManipulationPlanner::computeTrajBetweenTwoConfigs(configPt qi, configPt qf) {
  ManipulationUtils::forbidWindowEvents();
  p3d_copy_config_into(_robot, qi, &_robot->ROBOT_POS);
  p3d_copy_config_into(_robot, qf, &_robot->ROBOT_GOTO);

  /* RRT */

  if (this->computeRRT(_optimizeSteps, _optimizeTime, 1) != 0) {
    ManipulationUtils::allowWindowEvents();
    return NULL;
  }
  printf("End RRT\n");

    ManipulationUtils::allowWindowEvents();

  return  (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  //Generate the Grasping the openGrasping and the approach configurations
  ManipulationData configs(_robot);
  
  status = findArmGraspsConfigs(armId, object, configs);
  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armPickGoto(armId, qStart, object, configs.getGraspConfig(), configs.getOpenConfig(), configs.getApproachFreeConfig(), trajs);
  }
  return status;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs){
  p3d_traj* traj = NULL;
  fixAllHands(qStart, false);
  if((traj = computeTrajBetweenTwoConfigs(qStart, approachFreeConfig))){
    trajs.push_back(traj);
    if((traj = computeTrajBetweenTwoConfigs(approachFreeConfig, openConfig))){
      trajs.push_back(traj);
      (*_robot->armManipulationData)[armId].unFixHand(_robot);
      if((traj = computeTrajBetweenTwoConfigs(openConfig, graspConfig))){
        trajs.push_back(traj);
        return MANIPULATION_TASK_OK;
      }
    }
  }
  return MANIPULATION_TASK_NO_TRAJ_FOUND;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qGoal, p3d_rob* object, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK; 
  return status;
}
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToPlace(int armId, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  return status;
}
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  return status;
}
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickAndPlace(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  return status;
}

#ifdef DPG
//! \brief Check if the current path is in collision or not
//! \return 1 in case of collision, 0 otherwise
int ManipulationPlanner::checkCollisionOnTraj() {
  //   configPt currentPos = p3d_get_robot_config(robotPt);
  //   double armPos[6] = {currentPos[5], currentPos[6], currentPos[7], currentPos[8], currentPos[9], currentPos[10]};
  if (checkCollisionOnTraj(0)) {
    printf("There is collision\n");
    return 1;
  } else {
    printf("There is no collision\n");
    return 0;
  }
}


//! \brief Check if the current path is in collision or not
//! \return 1 in case of collision, 0 otherwise
int  ManipulationPlanner::checkCollisionOnTraj(int currentLpId) {
  p3d_traj *traj = NULL;

  XYZ_ENV->cur_robot = _robot;
  //initialize and get the current linear traj
  if (!traj) {
    if (_robot->nt < _robot->tcur->num - 2) {
      printf("BioMove3D: checkCollisionOnTraj unvalid traj number : nbTraj = %d, robot tcur = %d\n", _robot->nt, _robot->tcur->num);
      return 1;
    } else {
      traj = _robot->t[_robot->tcur->num - 2];
    }
  }
  checkConfigForCartesianMode(NULL);
  if (currentLpId > _robot->tcur->nlp) {
    printf("BioMove3D: checkCollisionOnTraj given lpId  = %d > tcur nlp = %d\n", currentLpId, _robot->tcur->nlp);
    currentLpId = 0;
  }
  p3d_localpath* currentLp = traj->courbePt;
  for (int i = 0; i < currentLpId / 2; i++) {
    currentLp = currentLp->next_lp;
  }
  return checkForCollidingPath(_robot, traj, currentLp);
}

/** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
\return MANIPULATION_TASK_OK for success */
MANIPULATION_TASK_MESSAGE ManipulationPlanner::replanCollidingTraj(int currentLpId, std::vector <p3d_traj*> &trajs){
  p3d_traj* traj = NULL;
  XYZ_ENV->cur_robot = _robot;
  //initialize and get the current linear traj
  if (!traj) {
    if (_robot->nt < _robot->tcur->num - 2) {
      return MANIPULATION_TASK_INVALID_TRAJ_ID;
    } else {
      traj = _robot->t[_robot->tcur->num - 2];
    }
  }
  checkConfigForCartesianMode(NULL);
  if (currentLpId > _robot->tcur->nlp) {
    printf("BioMove3D: checkCollisionOnTraj given lpId  = %d > tcur nlp = %d\n", currentLpId, _robot->tcur->nlp);
    currentLpId = 0;
  }
  p3d_localpath* currentLp = traj->courbePt;
  for (int i = 0; i < currentLpId / 2; i++) {
    currentLp = currentLp->next_lp;
  }
  configPt currentConfig = p3d_get_robot_config(_robot);
  int j = 0, returnValue = 0, optimized = traj->isOptimized;
  if (optimized) {
    p3dAddTrajToGraph(_robot, _robot->GRAPH, traj);
  }
//   printf("nbTraj before : %d\n", _robot->nt);
  do {
    printf("Test %d\n", j);
    j++;
    returnValue = replanForCollidingPath(_robot, traj, _robot->GRAPH, currentConfig, currentLp, optimized);
    traj = _robot->tcur;
    currentLp = traj->courbePt;
  } while (returnValue != 1 && returnValue != 0 && returnValue != -2 && j < 10);

    printf("nbTraj after : %d, returnValue = %d\n", _robot->nt, returnValue);

  if (returnValue == 1 && j == 0) { //no collision on traj
    return armPlanTask(ARM_FREE, 0, currentConfig, _robot->ROBOT_GOTO,(char *) "",(char *) "", trajs);
  }
  if (optimized || j > 1) {
    optimiseTrajectory(_optimizeSteps, _optimizeTime);
  }
  trajs.push_back((p3d_traj*) p3d_get_desc_curid(P3D_TRAJ));
  return MANIPULATION_TASK_OK;
}

#ifdef MULTILOCALPATH
/** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
\return MANIPULATION_TASK_OK for success */
MANIPULATION_TASK_MESSAGE  ManipulationPlanner::replanCollidingTraj(int currentLpId, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs) {
  p3d_traj *traj = NULL;
  MANIPULATION_TASK_MESSAGE returnMessage = MANIPULATION_TASK_OK;
  std::vector <p3d_traj*> trajs;

  p3d_multiLocalPath_disable_all_groupToPlan(_robot);
  p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);
  
  returnMessage = replanCollidingTraj(currentLpId, trajs);
  if (returnMessage == MANIPULATION_TASK_OK ) {//There is a new traj
    /* COMPUTE THE SOFTMOTION TRAJECTORY */
    if(concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK){
    /* COMPUTE THE SOFTMOTION TRAJECTORY */
      MANPIPULATION_TRAJECTORY_CONF_STR conf;
      SM_TRAJ smTraj;
      computeSoftMotion(traj, conf, smTraj);
      confs.push_back(conf);
      smTrajs.push_back(smTraj);
    }else{
      returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
    }
    //peut etre ajouter un return specific pour savoir qu'il y'a une nouvelle traj
  }
  return returnMessage;
}
#endif
#endif

#ifdef MULTILOCALPATH
int ManipulationPlanner::computeSoftMotion(p3d_traj* traj, MANPIPULATION_TRAJECTORY_CONF_STR &confs, SM_TRAJ &smTraj) {

  if (!traj) {
    printf("SoftMotion : ERREUR : no generated traj\n");
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  if (!traj || traj->nlp < 1) {
    printf("Optimization with softMotion not possible: current trajectory contains one or zero local path\n");
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  if(p3d_local_get_planner() != 9){
    printf("Optimization with softMotion not possible: current trajectory is not multi-localpath one\n");
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
  if (p3d_convert_traj_to_softMotion(traj, true, confs.first, confs.second, smTraj) == 1) {
    printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
    return MANIPULATION_TASK_ERROR_UNKNOWN;
  }
 return 0;
}
#endif
/* ******************************* */
/* ******** Task Planning ******** */
/* ******************************* */
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, const char* objectName, const char* supportName, std::vector <p3d_traj*> &trajs) {
  if(!_robot){
    printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  
  configPt qi = p3d_copy_config(_robot, qStart), qf = p3d_copy_config(_robot, qGoal);
  p3d_rob* cur_robot = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  p3d_rob* object = p3d_get_robot_by_name(objectName);
  p3d_rob* support = p3d_get_robot_by_name(supportName);
  p3d_sel_desc_id(P3D_ROBOT, _robot);
  p3d_traj *traj = NULL;
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  if(armId < 0 || armId >= (int)_robot->armManipulationData->size()){
    status = MANIPULATION_TASK_INVALID_TASK;
  }else{
    ENV.setBool(Env::drawTraj, false);
    checkConfigForCartesianMode(qi);
    checkConfigForCartesianMode(qf);
    fixManipulationJoints(armId, qi, object);
    fixManipulationJoints(armId, qf, object);
    fixAllHands(qi, false);
    fixAllHands(qf, false);
    p3d_set_and_update_this_robot_conf(_robot, qi);
    switch (task) {
      case ARM_FREE:{
        printf("plan for ARM_FREE task\n");
        if(MPDEBUG){
          ManipulationUtils::copyConfigToFORM(_robot, qi);
          ManipulationUtils::copyConfigToFORM(_robot, qf);
        }
        if ((traj = computeTrajBetweenTwoConfigs(qi, qf)) == NULL) {
          printf("ERROR armPlanTask(ARM_FREE) on traj");
          status = MANIPULATION_TASK_NO_TRAJ_FOUND;
          break;
        }
        trajs.push_back(traj);
        break;
      }
      case ARM_PICK_GOTO:{
        printf("plan for ARM_PICK_GOTO task\n");
        status = armPickGoto(armId, qi, object, trajs);
        break;
      }
      case ARM_PICK_TAKE_TO_FREE:{
        printf("plan for ARM_PICK_TAKE_TO_FREE task\n");
        status = armPickTakeToFree(armId, qf, object, trajs);
        break;
      }
      case ARM_PICK_TAKE_TO_PLACE:{
        printf("plan for ARM_PICK_TAKE_TO_PLACE task\n");
        status = armPickTakeToPlace(armId, object, support, trajs);
        break;
      }
      case ARM_PLACE_FROM_FREE:{
        printf("plan for ARM_PLACE_FROM_FREE task\n");
        status = armPlaceFromFree(armId, qi, object, support, trajs);
        break;
      }
      case ARM_PICK_AND_PLACE:{
        printf("plan for ARM_PICK_AND_PLACE task\n");
        status = armPickAndPlace(armId, qi, qf, object, support, trajs);
        break;
      }
      default:{
        printf("%s: %d: ManipulationPlanner::armPlanTask(): wrong task.\n", __FILE__, __LINE__);
        status = MANIPULATION_TASK_INVALID_TASK;
        break;
      }
    }
  }
  unfixManipulationJoints(armId);
  p3d_sel_desc_id(P3D_ROBOT,cur_robot);
  g3d_draw_allwin_active();
  if(status == MANIPULATION_TASK_OK){
    printf("BioMove3D: armPlanTask OK\n");
  }else{
    printf("BioMove3D: armPlanTask Fail\n");
  }
  return status;
}

#ifdef MULTILOCALPATH
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, const char* objectName, const char* supportName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs){
  std::vector <p3d_traj*> trajs;
  p3d_traj* traj = NULL;
  MANIPULATION_TASK_MESSAGE returnMessage;

  if(!_robot){
    printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  p3d_multiLocalPath_disable_all_groupToPlan(_robot);
  p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);
  if ((returnMessage = armPlanTask(task, armId, qStart, qGoal, objectName, supportName, trajs)) == MANIPULATION_TASK_OK){
    //concatene
    if(concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK){
    /* COMPUTE THE SOFTMOTION TRAJECTORY */
      MANPIPULATION_TRAJECTORY_CONF_STR conf;
      SM_TRAJ smTraj;
      computeSoftMotion(traj, conf, smTraj);
      confs.push_back(conf);
      smTrajs.push_back(smTraj);
    }else{
      returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
    }
  }
  
  return returnMessage;
}
#endif
/* ******************************* */
/* ******* Hands / Grasping ****** */
/* ******************************* */
void ArmManipulationData::fixHand(p3d_rob* robot, bool rest){
  if(rest){
    gpSet_hand_rest_configuration(robot, _handProp, _handProp.type);
  }
  gpFix_hand_configuration(robot, _handProp, _handProp.type);
  gpDeactivate_hand_selfcollisions(robot, _handProp.type);
}
void ArmManipulationData::unFixHand(p3d_rob* robot){
  gpUnFix_hand_configuration(robot, _handProp, _handProp.type);
  gpActivate_hand_selfcollisions(robot, _handProp.type);
}

void ManipulationPlanner::fixAllHands(configPt q, bool rest) const{
  if(q != NULL){
    p3d_set_and_update_this_robot_conf(_robot, q);
  }
  for(uint i = 0; i < (*_robot->armManipulationData).size(); i++){
    (*_robot->armManipulationData)[i].fixHand(_robot, rest);
  }
  if(q != NULL){
    p3d_get_robot_config_into(_robot, &q);
  }
}
void ManipulationPlanner::unFixAllHands(void){
  for(uint i = 0; i < (*_robot->armManipulationData).size(); i++){
    (*_robot->armManipulationData)[i].unFixHand(_robot);
  }
}

void ManipulationPlanner::unfixManipulationJoints(int armId){
  for(uint i = 0; i < (*_robot->armManipulationData).size(); i++){
    unFixJoint(_robot, (*_robot->armManipulationData)[i].getManipulationJnt());
  }
}

void ManipulationPlanner::fixManipulationJoints(int armId, configPt q, p3d_rob* object){
  p3d_matrix4 pos;
  if(object){
    p3d_mat4Copy(object->joints[1]->abs_pos, pos);
  }else{
    p3d_mat4Copy(p3d_mat4IDENTITY, pos);
  }
  if(q){
    p3d_set_and_update_this_robot_conf(_robot, q);
  }
  for(uint i = 0; i < (*_robot->armManipulationData).size(); i++){
    fixJoint(_robot, (*_robot->armManipulationData)[i].getManipulationJnt(), pos);
  }
  if(q){
    p3d_get_robot_config_into(_robot, &q);
  }
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::findArmGraspsConfigs(int armId, p3d_rob* object, ManipulationData& configs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  (*_robot->armManipulationData)[armId].setCarriedObject(object);
  if(armId == -1){
    //TODO Multi arm Grasping
  }else{
    if(armId == -2){//Compute closest arm
      if ((armId = getClosestWristToTheObject(_robot, object)) == -2){
        printf("ERROR findArmGraspsConfigs on getClosestWristToTheObject");
        return MANIPULATION_TASK_NO_GRASP;
      }
    }
    gpHand_properties armHandProp = (*_robot->armManipulationData)[armId].getHandProperties();
    list<gpGrasp> graspList/*, tmp*/;
    //Compute the grasp list for the given hand and object
    gpGet_grasp_list(object->name, armHandProp.type, graspList);
    if (graspList.size() == 0){
      status = MANIPULATION_TASK_NO_GRASP;
    }else{
      status = MANIPULATION_TASK_NO_GRASP;
      int counter = 0;
      bool validConf = false;
      for(list<gpGrasp>::iterator iter = graspList.begin(); iter != graspList.end(); iter++){
        ManipulationData data(_robot);
        p3d_matrix4 tAtt;
        fixAllHands(NULL, true);
        status = getGraspOpenApproachExtractConfs(object, armId, armHandProp, (*iter), tAtt, data);
        if(status == MANIPULATION_TASK_OK){
          if(data.getGraspConfigCost() < configs.getGraspConfigCost()){
            configs = data;
            validConf = true;
//             break;
            if(data.getGraspConfigCost() < _robot->configCostThreshold){
                break;
            }
          }
          if(MPDEBUG){
            ManipulationUtils::copyConfigToFORM(_robot, data.getGraspConfig());
          }
        }
         counter++;
      }
      if(MPDEBUG){
        printf("NbTest Before Config : %d\n", counter);
      }
      if(validConf){
        status = MANIPULATION_TASK_OK;
      }
    }
  }

  if(MPDEBUG){
    showConfig(configs.getOpenConfig());
    showConfig(configs.getGraspConfig());
    showConfig(configs.getApproachGraspConfig());
    showConfig(configs.getApproachFreeConfig());
    printf("MinConfig Cost = %f\n", configs.getGraspConfigCost());
    ManipulationUtils::copyConfigToFORM(_robot, configs.getGraspConfig());
    ManipulationUtils::copyConfigToFORM(_robot, configs.getOpenConfig());
    ManipulationUtils::copyConfigToFORM(_robot, configs.getApproachGraspConfig());
    ManipulationUtils::copyConfigToFORM(_robot, configs.getApproachFreeConfig());
  }
  (*_robot->armManipulationData)[armId].setCarriedObject((p3d_rob*)NULL);
  return status;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::getGraspOpenApproachExtractConfs(p3d_rob* object, int armId, gpHand_properties& armHandProp, gpGrasp& grasp, p3d_matrix4 tAtt, ManipulationData& configs) const{
  p3d_matrix4 handFrame;
  ArmManipulationData mData = (*_robot->armManipulationData)[armId];
  p3d_mat4Mult(grasp.frame, armHandProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpHand_properties handProp = mData.getHandProperties();
  gpSet_grasp_configuration(_robot, grasp, handProp.type);
  gpFix_hand_configuration(_robot, handProp, handProp.type);
  //Compute Grasp configuration
  q = setRobotGraspPosWithoutBase(_robot, object->joints[1]->abs_pos, tAtt, FALSE, armId, true);
  if(q){
    //If it exist, try to find better Rest Arm config and 
    double restArmCost = setRobotArmsRest(_robot, object->joints[1]->abs_pos, armId, tAtt, _robot->openChainConf, q);
    double graspArmCost = computeRobotGraspArmCost(_robot, armId, grasp, q, _robot->openChainConf, object->joints[1]->abs_pos)/270;
    double confCost = (restArmCost + graspArmCost * 2) / 3;

    deactivateCcCntrts(_robot, armId);
    gpSet_grasp_configuration(_robot, grasp, q, mData.getHandProperties().type);
    configs.setGraspConfig(q);
    configs.setGraspConfigCost(confCost);
    printf("configuration Cost = %f\n", confCost);
    configs.setGrasp(new gpGrasp(grasp));
    //Check the open configuration of the hand
    gpSet_grasp_open_configuration(_robot, grasp, q, mData.getHandProperties().type);
    p3d_set_and_update_this_robot_conf(_robot, q);
    if(!p3d_col_test()){
      configs.setOpenConfig(q);
      //Extract configuration
     _robot->isCarryingObject = TRUE;
      q[(*_robot->armManipulationData)[armId].getManipulationJnt()->index_dof + 2] += getApproachGraspOffset(); //Z axis of the manipulation joint
      p3d_set_and_update_this_robot_conf(_robot, q);
      gpSet_grasp_configuration(_robot, grasp, q, mData.getHandProperties().type);
      q = setRobotCloseToConfGraspApproachOrExtract(_robot, q, object->joints[1]->abs_pos, tAtt, false, armId, true);
      deactivateCcCntrts(_robot, armId);
      configPt tmpConf = p3d_get_robot_config(_robot);
      tmpConf[(*_robot->armManipulationData)[armId].getManipulationJnt()->index_dof + 2] -= getApproachGraspOffset();
      p3d_set_and_update_this_robot_conf(_robot, tmpConf);
      p3d_destroy_config(_robot, tmpConf);
     _robot->isCarryingObject = FALSE;
      if(q){
        fixAllHands(NULL, true);
        configs.setApproachGraspConfig(q);
        p3d_matrix4 tAttTmp;
        p3d_vector3 tAttY;
        p3d_mat4Copy(tAtt, tAttTmp);
        if(!strcmp(mData.getCcCntrt()->namecntrt, "p3d_kuka_arm_ik")){
          p3d_mat4ExtractColumnY(tAtt, tAttY);
        }else if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_lwr_arm_ik")){
          p3d_mat4ExtractColumnZ(tAtt, tAttY);
        }
        tAttTmp[0][3] -= getApproachFreeOffset() * tAttY[0];
        tAttTmp[1][3] -= getApproachFreeOffset() * tAttY[1];
        tAttTmp[2][3] -= getApproachFreeOffset() * tAttY[2];
        p3d_copy_config_into(_robot, configs.getOpenConfig(), &q);
        gpUnFix_hand_configuration(_robot, handProp, mData.getHandProperties().type);
        gpSet_grasp_open_configuration(_robot, grasp, mData.getHandProperties().type);
        gpFix_hand_configuration(_robot, handProp, mData.getHandProperties().type);
        p3d_set_and_update_this_robot_conf(_robot, q);
        q = setRobotCloseToConfGraspApproachOrExtract(_robot, q, object->joints[1]->abs_pos, tAttTmp, false, armId, true);
        deactivateCcCntrts(_robot, armId);
        if(q){
          fixAllHands(NULL, true);
          configs.setApproachFreeConfig(q);
          p3d_destroy_config(_robot, q);
          return MANIPULATION_TASK_OK; //success
        }
      }
    }
  }
  p3d_destroy_config(_robot, q);
  deactivateCcCntrts(_robot, armId);
  return MANIPULATION_TASK_NO_GRASP;
}
