#include <list>
#include <algorithm>

#include "move3d-headless.h"

#include "ManipulationPlanner.hpp"
#include "ManipulationArmData.hpp"

#include "lightPlanner.h"
#include "lightPlannerApi.h"

#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

static bool MPDEBUG=true;

SM_TRAJ ManipPlannerLastTraj;

using namespace std;

/* ******************************* */
/* ******* (De)Constructor ******* */
/* ******************************* */
ManipulationPlanner::ManipulationPlanner(p3d_rob *robot) : _robot(robot), _manipData(robot), _manipConf(robot)
{
    // Set planner funtions
    _plannerMethod = rrtQuerry;
    _smoothingMethod = optimiseTrajectory;
  
    // Manipulation planner
    _planningTime = 60; // 15
    _optimizeSteps = 200;
    _optimizeTime = 15.0; // 4 secondes
    _safetyDistanceValue = 0.0;
    _placementTry = 15;
    _useBaseMotion = false;
  
    ENV.setBool(Env::smoothSoftMotionTraj,true);
    ENV.setBool(Env::drawExploration,false);

    setDebugSoftMotionMode(true);
#ifdef MULTILOCALPATH
    setDebugSoftMotionMode(true);
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
        printf("%s: %d: ManipulationPlanner::ManipulationPlanner: cannot find all necessary multiLocalpth groups in robot %s\n", __FILE__, __LINE__, _robot->name);
        return;
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

void ManipulationPlanner::setDefaultPlanner()
{
    _plannerMethod = rrtQuerry;
    _smoothingMethod = optimiseTrajectory;

    // Manipulation planner
    _planningTime = 15; // 15
    _optimizeSteps = 200;
    _optimizeTime = 4.0; // 4 secondes
    _safetyDistanceValue = 0.0;
    _placementTry = 5;

    ENV.setExpansionMethodSlot(0);

    if (_robot)
    {
//        p3d_jnt_set_dof_bounds_deg ( _robot->joints[1], 1, -6, 2 );
//        p3d_jnt_set_dof_rand_bounds_deg ( _robot->joints[1], 1, -6, 2 );
    }
}

void ManipulationPlanner::setNavigationPlanner()
{
//    _plannerMethod = rrtQuerry;
//    _smoothingMethod = optimiseTrajectory;

    // Manipulation planner
    _planningTime = 60; // 15
    _optimizeSteps = 200;
    _optimizeTime = 15.0; // 4 secondes
    _safetyDistanceValue = 0.0;
    _placementTry = 5;

    ENV.setExpansionMethodSlot(2);

    if (_robot)
    {
//        p3d_jnt_set_dof_bounds_deg ( _robot->joints[1], 1, -5, -2 );
//        p3d_jnt_set_dof_rand_bounds_deg ( _robot->joints[1], 1, -5, -2 );
    }
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
void ManipulationPlanner::setDebugMode(bool value){
  MPDEBUG = value;
}

#ifdef MULTILOCALPATH
void ManipulationPlanner::setDebugSoftMotionMode(bool value){
  ENV.setBool(Env::writeSoftMotionFiles, value);
}
void ManipulationPlanner::setSmoothingSoftMotionMode(bool value){
   ENV.setBool(Env::smoothSoftMotionTraj, value);
}
bool ManipulationPlanner::getSmoothingSoftMotionMode(){
   ENV.getBool(Env::smoothSoftMotionTraj);
}

#endif
void ManipulationPlanner::setPlanningMethod(p3d_traj* (*funct)(p3d_rob* robot, configPt qs, configPt qg)){
  _plannerMethod = funct;
}

void ManipulationPlanner::resetPlanningMethod(){
  _plannerMethod = rrtQuerry;
}

void ManipulationPlanner::setSmoothingMethod(void (*funct)(p3d_rob* robot, p3d_traj* traj, int nbSteps, double maxTime)){
  _smoothingMethod = funct;
}

void ManipulationPlanner::resetSmoothingMethod(){
  _smoothingMethod = optimiseTrajectory;
}

void ManipulationPlanner::setReplanningMethod(p3d_traj* (*funct)(p3d_rob* robotPt, p3d_traj* traj, p3d_vector3 target, int deformationViaPoint)) {
  _replanningMethod = funct;
}

void ManipulationPlanner::setPlanningTime(double time){
  if (time > 0) {
      _planningTime = time;
  } else {
      printf("%s: %d: ManipulationPlanner::setPlanningTime() unvalid time.\n", __FILE__, __LINE__);
      return;
  }
}
double ManipulationPlanner::getPlanningTime(void) const{
  return _planningTime;
}

void ManipulationPlanner::setOptimizeSteps(int nbSteps) {
    if (nbSteps > 0) {
        _optimizeSteps = nbSteps;
    } else {
        printf("%s: %d: ManipulationPlanner::setOptimizeSteps() unvalid nbSteps.\n", __FILE__, __LINE__);
        return;
    }
}
int ManipulationPlanner::getOptimizeSteps(void) const {
    return _optimizeSteps;
}

void ManipulationPlanner::setOptimizeTime(double time) {
    if (time > 0) {
        _optimizeTime = time;
    } else {
        printf("%s: %d: ManipulationPlanner::setOptimizeTime() unvalid time.\n", __FILE__, __LINE__);
        return;
    }
}
double ManipulationPlanner::getOptimizeTime(void) const {
    return _optimizeTime;
}

void ManipulationPlanner::setSafetyDistanceValue(double value){
  _safetyDistanceValue = value;
}
double ManipulationPlanner::getSafetyDistanceValue(void) const{
  return _safetyDistanceValue;
}

void ManipulationPlanner::setPlacementTry(int nbTry){
  _placementTry = nbTry;
}
int ManipulationPlanner::getPlacementTry(void){
  return _placementTry;
}

void ManipulationPlanner::setUseBaseMotion(bool useBase)
{
  _useBaseMotion = useBase;
}

bool ManipulationPlanner::getUseBaseMotion(void)
{
  return _useBaseMotion;
}

/* ******************************* */
/* ******* Hands / Grasping ****** */
/* ******************************* */
void ManipulationPlanner::fitConfigurationToRobotBounds(configPt q)
{
  for (int i = 0; i < _robot->njoints; i++) 
  {
    p3d_jnt* jnt = _robot->joints[i];
    
    for (int j = 0; j < jnt->dof_equiv_nbr; j++) 
    {
      double vmin = -P3D_HUGE, vmax = P3D_HUGE;
      p3d_jnt_get_dof_bounds(jnt, j, &vmin, &vmax);
      if(q[jnt->index_dof + j] < vmin){
        cout << "WARNING : Configuration correction : Joint " << jnt->name ;
        cout << " has value = " << q[jnt->index_dof + j] << " and its minimal bound = " << vmin << endl;
        q[jnt->index_dof + j] = vmin;
      }
      if(q[jnt->index_dof + j] > vmax){
        cout << "WARNING : Configuration correction : Joint " << jnt->name ;
        cout << " has value = " << q[jnt->index_dof + j] << " and its maximal bound = " << vmax << endl;
        q[jnt->index_dof + j] = vmax;
      }
    }
  }
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::computeManipulationData(int armId,p3d_rob* object, gpGrasp& grasp)
{
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  // Generate new manip configs
  ManipulationData manipConfigs(_robot);

  _manipData = manipConfigs;
  
  status = _manipConf.findArmGraspsConfigs(armId,object, grasp, _manipData);

  ManipulationUtils::printManipulationMessage(status);
  return status;
}

/* ******************************* */
/* ******* Motion Planning Modes * */
/* ******************************* */
void ManipulationPlanner::checkConfigForCartesianMode(configPt q, p3d_rob* object)
{
    bool deleteConfig = false;
    if (q == NULL) {
        q = p3d_get_robot_config(_robot);
        deleteConfig = true;
    }
  
    // For Each Arm
    for (int i = 0; i < int((*_robot->armManipulationData).size()); i++) 
    {
        ArmManipulationData& armData  = (*_robot->armManipulationData)[i];
      
        desactivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(), 
                                     armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
        if (armData.getCartesian()) {
          cout << "Arm Data " << i <<  " is set cartesian" << endl;
          /* Uptdate the Virual object for inverse kinematics */
          /* Be carfull the Arm will be unfixed */
          p3d_update_virtual_object_config_for_arm_ik_constraint(_robot, i, q);
          activateCcCntrts(_robot, i, false);
          ManipulationUtils::unfixManipulationJoints(_robot, i);
          if(object){
            armData.getManipulationJnt()->dist = object->joints[1]->dist;
          }
        } else {
          deactivateCcCntrts(_robot, i);
          p3d_update_virtual_object_config_for_arm_ik_constraint(_robot, i, q);
          p3d_set_and_update_this_robot_conf(_robot, q);
          setAndActivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(), 
                                          armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
        }
    }
    p3d_set_and_update_this_robot_conf(_robot, q);
    p3d_get_robot_config_into(_robot, &q);
    if (deleteConfig) {
      p3d_destroy_config(_robot, q);
      q = NULL;
    }
}

void ManipulationPlanner::setArmCartesian(int armId, bool cartesian) 
{
    ArmManipulationData& armData  = (*_robot->armManipulationData)[armId];

    armData.setCartesian(cartesian);
    // activate the corresponding group, constraint 
    // and set the sampling bounds of the object to sample
    if (cartesian) {
        activateCcCntrts(_robot, armId, 0);
        shootTheObjectArroundTheBase(_robot, _robot->baseJnt, armData.getManipulationJnt(), 2.0);      
    } else {
        deactivateCcCntrts(_robot, armId);
        shootTheObjectInTheWorld(_robot, armData.getManipulationJnt());
    }
}
bool ManipulationPlanner::getArmCartesian(int armId) const {
    return (*_robot->armManipulationData)[armId].getCartesian();
}

/* ******************************* */
/* ******* Motion Planning ******* */
/* ******************************* */
//! Compute an RRT with
//! @param smoothingSteps : number of smoothing steps
//! @param smootingTime : maximum smoothing Time
//! @param biDir : is the RRT bidirectional
MANIPULATION_TASK_MESSAGE ManipulationPlanner::computeRRT(int smoothingSteps, double smootingTime, bool biDir) {
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;

  configPt qs = _robot->ROBOT_POS;
  configPt qg = _robot->ROBOT_GOTO;
  
  double planningTime = p3d_get_tmax();
  p3d_set_tmax(_planningTime);
  p3d_traj* traj = _plannerMethod(_robot,qs,qg);
  p3d_set_tmax(planningTime);

  if(traj){
    _smoothingMethod(_robot, traj, smoothingSteps, smootingTime);
  }

  if (!traj) {
    //Check the possible errors of planning
    //Check the start and goal configuration
    if(!p3d_is_collision_free(_robot, _robot->ROBOT_POS)){
      status = MANIPULATION_TASK_INVALID_QSTART;
      ManipulationUtils::printManipulationMessage(status);
      ManipulationUtils::printConstraintInfo(_robot);
      p3d_print_col_pair();
    }else if(!p3d_is_collision_free(_robot, _robot->ROBOT_GOTO)){
      status = MANIPULATION_TASK_INVALID_QGOAL;
      ManipulationUtils::printManipulationMessage(status);
      ManipulationUtils::printConstraintInfo(_robot);
      p3d_print_col_pair();
    }else{
      status = MANIPULATION_TASK_NO_TRAJ_FOUND;
      ManipulationUtils::printManipulationMessage(status);
    }
    return status;
  }
  return status;
}

//! Compute a PRM
//! the PRM algortihm sample configuration in all C-Space
//! @param ComputeTime : number of smoothing steps
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armComputePRM(double ComputeTime) {

    this->cleanRoadmap();

    checkConfigForCartesianMode(NULL, NULL);
#ifdef MULTILOCALPATH
    p3d_multiLocalPath_disable_all_groupToPlan(_robot, FALSE);
    p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1, FALSE);
#endif
    fixJoint(_robot, _robot->baseJnt,_robot->baseJnt->abs_pos);
    ManipulationUtils::fixAllHands(_robot, NULL, true);
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


//! Computes a trajectory between two configurations
//! It first moves the qi and qf configuration to the ROBOT_POS and ROBOT_GOTO
//! This method calls the compute RRT method from this class
p3d_traj* ManipulationPlanner::computeTrajBetweenTwoConfigs(configPt qi, configPt qf, MANIPULATION_TASK_MESSAGE* status) {

  
    if( p3d_equal_config(_robot, qi, qf) ){
      *status = MANIPULATION_TASK_EQUAL_QSTART_QGOAL;
      ManipulationUtils::printManipulationMessage(*status);
      print_config(_robot, qi);
      print_config(_robot, qf);
      return NULL;
    }
    ManipulationUtils::forbidWindowEvents();
    p3d_copy_config_into(_robot, qi, &_robot->ROBOT_POS);
    p3d_copy_config_into(_robot, qf, &_robot->ROBOT_GOTO);

    /* RRT */

    if (( *status = this->computeRRT(_optimizeSteps, _optimizeTime, 1)) != MANIPULATION_TASK_OK) {
        ManipulationUtils::allowWindowEvents();
        return NULL;
    }
    printf("End RRT\n");

    ManipulationUtils::allowWindowEvents();
    return (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
}

/* ******************************* */
/* ******* Trajectories ********** */
/* ******************************* */
/** Concatenes all the current trajectories of the robot into the first one.
 * NB: only the first trajectory will remain (and grown up); the others are destroyed.
 * \param trajs A vector of trajectories
 * \param concatTraj The concatenated trajectory to return
 * \return Message of success or fail */
MANIPULATION_TASK_MESSAGE ManipulationPlanner::cutTrajInSmall ( p3d_traj* inputTraj, p3d_traj* outputTraj ) {
  
  if ( _robot==NULL ) {
    PrintInfo ( ( "cutTrajInSmall: robot is NULL.\n" ) );
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  
  if ( !inputTraj ) {
    PrintInfo ( ( "cutTrajInSmall: the trajectory is NULL.\n" ) );
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  
  vector<p3d_localpath> trajArray;
  
  const double dmax = 0.10;
  
  outputTraj = p3d_create_empty_trajectory(_robot);
  inputTraj->range_param = p3d_compute_traj_rangeparam(inputTraj);
  
  p3d_localpath* lastPath = NULL;
  
  unsigned int i=0;
  
  for(double param = 0; param < inputTraj->range_param; param += dmax )
  {
    configPt q_init = p3d_config_at_param_along_traj(inputTraj,param);
    configPt q_goal = p3d_config_at_param_along_traj(inputTraj,param+dmax);
      
    if (p3d_equal_config(_robot, q_init, q_goal )) {
      break;
    }
    
    p3d_localpath* path = p3d_local_planner(_robot,q_init,q_goal);
    cout << "LocalPath nb : " << i++ << endl;
      
    if( lastPath != NULL )
    {
      while (lastPath->next_lp != NULL) 
      {
          lastPath = lastPath->next_lp;
      }
      lastPath->next_lp = path;
      path->prev_lp = lastPath;
    }
    else {
      outputTraj->courbePt = path;
    }

    lastPath = path;
    
    p3d_destroy_config(_robot,q_init);
    p3d_destroy_config(_robot,q_goal);
  }

  outputTraj->range_param = p3d_compute_traj_rangeparam(outputTraj);
  outputTraj->nlp = p3d_compute_traj_nloc(outputTraj);
  
  cout << "Number of localpaths : " << outputTraj->nlp << endl;
  
  return MANIPULATION_TASK_OK;
}

//! Concatenes all the current trajectories of the robot into the first one.
//! NB: only the first trajectory will remain (and grown up); the others are destroyed.
//! \param trajs A vector of trajectories
//! \param concatTraj The concatenated trajectory to return
//! \return Message of success or fail 
MANIPULATION_TASK_MESSAGE ManipulationPlanner::concatTrajectories (std::vector<p3d_traj*>& trajs, p3d_traj** concatTraj) {
    if ( _robot==NULL ) {
        PrintInfo ( ( "concateneAllTrajectories: robot is NULL.\n" ) );
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }
    if ( trajs.empty() || !trajs[0] ) {
        PrintInfo ( ( "concateneAllTrajectories: the trajectory vector is empty.\n" ) );
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }
      
    cout << "Concat Traj" << endl;
    cout << trajs[0] << endl;
    *concatTraj = p3d_create_traj_by_copy(trajs[0]);
    for (int i = 1; i < (int)trajs.size(); i++) {
      cout << trajs[i] << endl;
      if(trajs[i]){
        if (p3d_concat_traj(*concatTraj, trajs[i]) == TRUE)
        {
          cout << trajs[i] << endl;
          cout << "Concat traj fails" << endl;  
          return MANIPULATION_TASK_NO_TRAJ_FOUND;
        }
      }
    }
    _robot->tcur = (*concatTraj);
    g3d_add_traj((char*)"Task", (*concatTraj)->num, _robot, (*concatTraj) );

    return MANIPULATION_TASK_OK;
}

#ifdef MULTILOCALPATH
int ManipulationPlanner::computeSoftMotion(p3d_traj* traj, MANPIPULATION_TRAJECTORY_CONF_STR &confs, SM_TRAJ &smTraj) {
    bool approximate = false;
    if (!traj) {
        printf("SoftMotion : ERREUR : no generated traj\n");
        return MANIPULATION_TASK_ERROR_UNKNOWN;
    }
    if (!traj || traj->nlp < 1) {
        printf("Optimization with softMotion not possible: current trajectory contains one or zero local path\n");
        return MANIPULATION_TASK_ERROR_UNKNOWN;
    }
    if (p3d_local_get_planner() != 9) {
        printf("Optimization with softMotion not possible: current trajectory is not multi-localpath one\n");
        return MANIPULATION_TASK_ERROR_UNKNOWN;
    }
    for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
        ArmManipulationData& armData  = (*_robot->armManipulationData)[i];
        if (armData.getCartesian()) {
          approximate = true;
	}
    }
    if (p3d_convert_traj_to_softMotion(traj, ENV.getBool(Env::smoothSoftMotionTraj), true, approximate,confs.first, confs.second, smTraj) == 1) {
        printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
        return MANIPULATION_TASK_ERROR_UNKNOWN;
    }
    return 0;
}
#endif

/* ******************************* */
/* ******* Manipulation Planning * */
/* ******************************* */

//! The arm to free point method takes a goto configuration
//! and then computes a trajectory from the start configuration of the specified arm
//
//! @param armId : which arm is planned
//! @param qStart : the configuration from which to start
//! @param objGoto : the Cartesian goal pose
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armToFree(int armId, configPt qStart, configPt qGoal, bool useSafetyDistance, p3d_rob* object, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  p3d_traj* traj = NULL;
  
  ManipulationUtils::fixAllHands(_robot, qStart, false);
  
  if(!_useBaseMotion) {
    fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);
  }
  else {
    unFixJoint(_robot, _robot->baseJnt);
  }
  checkConfigForCartesianMode(qStart, object);
  checkConfigForCartesianMode(qGoal, object);
  
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  if(object){
    gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
    gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); 
    
    p3d_set_object_to_carry_to_arm(_robot, armId, object->name);
  }
  if (MPDEBUG) {
    ManipulationUtils::copyConfigToFORM(_robot, qStart);
    ManipulationUtils::copyConfigToFORM(_robot, qGoal);
  }
  if(useSafetyDistance){
    setSafetyDistance(_robot, getSafetyDistanceValue());
  }
  if((traj = computeTrajBetweenTwoConfigs(qStart, qGoal, &status))){
    trajs.push_back(traj);
  }
  if(useSafetyDistance){
    setSafetyDistance(_robot, 0);
  }
  if(object){
    gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
    gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1
    (*_robot->armManipulationData)[armId].setCarriedObject((p3d_rob*) NULL);
    _robot->isCarryingObject = FALSE;
  }
  return status;
}

//! The arm to free point method takes a goto cartesian pose
//! and then computes a trajectory from the start configuration of the specified arm
//! A configuration is computed using the current CCntrt Tatt of the arm
//
//! @param armId : which arm is planned
//! @param qStart : the configuration from which to start
//! @param objGoto : the Cartesian goal pose
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armToFreePoint(int armId, configPt qStart, std::vector<double> &objGoto, p3d_rob* object, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  gpGrasp grasp;
  double confCost = -1;
  ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
  p3d_matrix4 tAtt, bakTatt;
  if(object){
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, bakTatt);
    desactivateTwoJointsFixCntrt(_robot,mData.getManipulationJnt(), mData.getCcCntrt()->pasjnts[ mData.getCcCntrt()->npasjnts-1 ]);
    for(int i = 0; i < mData.getManipulationJnt()->dof_equiv_nbr; i++){
      qStart[mData.getManipulationJnt()->index_dof + i] = p3d_jnt_get_dof(object->joints[1], i);
    }
    p3d_set_and_update_this_robot_conf(_robot, qStart);
    p3d_compute_Tatt(mData.getCcCntrt());
  }
  p3d_mat4Copy(mData.getCcCntrt()->Tatt, tAtt);
  configPt qGoal = _manipConf.getFreeHoldingConf(object, armId, grasp, tAtt, confCost, objGoto, NULL);
  if(qGoal){
    status = armToFree(armId, qStart, qGoal, true, object ,trajs);
  }else{
    status = MANIPULATION_TASK_NO_TRAJ_FOUND;
  }
  
  if(object){
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, bakTatt);
  }
  p3d_destroy_config(_robot, qGoal);
  return status;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armExtract(int armId, configPt qStart, p3d_rob* object, std::vector <p3d_traj*> &trajs) {
 MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  int updateTatt = false;

  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  g3d_draw_allwin_active();
  activateCcCntrts(_robot, armId, false);
  g3d_draw_allwin_active();
  configPt qGoal = _manipConf.getExtractConf(armId, qStart, (*_robot->armManipulationData)[armId].getCcCntrt()->Tatt);

  if (!qGoal) {
    status = MANIPULATION_TASK_NO_GRASP;
  }else{
    
  }

  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armToFree(armId, qStart, qGoal, false, NULL, trajs);
  }

  return status;
}

//! The arm Goto Pick method takes in an object and a configuration
//! and then computes a vector of trajectories
//! it calls the armPickGoto on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : the configuration from which to grasp the object
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, gpGrasp& grasp, std::vector <p3d_traj*> &trajs){
  
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  status = computeManipulationData(armId,object, grasp);
  ManipulationUtils::copyConfigToFORM(_robot, qStart);
  ManipulationUtils::copyConfigToFORM(_robot, _manipData.getApproachFreeConfig());
  ManipulationUtils::copyConfigToFORM(_robot, _manipData.getOpenConfig());
  ManipulationUtils::copyConfigToFORM(_robot, _manipData.getGraspConfig());
  
  if (status == MANIPULATION_TASK_OK) 
  {
    if( p3d_equal_config(_robot, qStart, _manipData.getApproachFreeConfig()) )
    {
      cout << "computeManipulationData : Start equal Approach" << endl;
      status = MANIPULATION_TASK_NO_GRASP;
    }
    if( p3d_equal_config(_robot, _manipData.getApproachFreeConfig(), _manipData.getOpenConfig()) )
    {
      cout << "computeManipulationData : Approach equal Open" << endl;
      status = MANIPULATION_TASK_NO_GRASP;
    }
  }
  
  if (status == MANIPULATION_TASK_OK)
  {
    //Compute the path between theses configurations
    status = armPickGoto(armId, qStart, object, _manipData.getGraspConfig(), _manipData.getOpenConfig(), _manipData.getApproachFreeConfig(), trajs);
  }
  else {
      cout << "Fail to findArmGraspsConfigs(armId,object,_manipData)" << endl;
  }

  return status;
}

//! Same as above but with more parameters
//
//! @param configPt graspConfig : the configuration to grasp the object
//! @param configPt openConfig : the configuration with the open hand
//! @param configPt approachFreeConfig : an aproaching configuration
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs) {

    p3d_traj* traj = NULL;
    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    ManipulationUtils::fixAllHands(_robot, qStart, false);
    
    if(!_useBaseMotion) {
      fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);
    }
    else {
      unFixJoint(_robot, _robot->baseJnt);
    }
    
    checkConfigForCartesianMode(qStart, object);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, qStart);
    }
    checkConfigForCartesianMode(approachFreeConfig, object);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, approachFreeConfig);
    }
    checkConfigForCartesianMode(openConfig, object);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, openConfig);
    }
    checkConfigForCartesianMode(graspConfig, object);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, graspConfig);
    }
    //fixJoint(_robot, armData.getManipulationJnt() , armData.getManipulationJnt()->abs_pos);

    // Compute to Approach config
    setSafetyDistance(_robot, getSafetyDistanceValue());
    printf("tSafetyDistance= %f\n", getSafetyDistanceValue());
    setSafetyDistance(_robot, 0);
  
    if ((traj = computeTrajBetweenTwoConfigs(qStart, approachFreeConfig, &status))
        /*|| (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL)*/ )
    {
      trajs.push_back(traj);
      setSafetyDistance(_robot, 0);
      // Compute to Open config
      if ((traj = computeTrajBetweenTwoConfigs(approachFreeConfig, openConfig, &status)) 
          || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
      {
        trajs.push_back(traj);

        // Set the hand as group to plan
        (*_robot->armManipulationData)[armId].unFixHand(_robot);
#ifdef MULTILOCALPATH
        p3d_multiLocalPath_disable_all_groupToPlan(_robot, FALSE);
        p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1, FALSE);
#endif
        // Compute to Grasp Config
        gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
        gpDeactivate_object_fingertips_collisions(_robot, object->joints[1]->o, handProp, armId);
        
        // Compute Open to Grasp trajectory
        if ((traj = computeTrajBetweenTwoConfigs(openConfig, graspConfig, &status)) 
            || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
        {
          trajs.push_back(traj);

          // This state should be set
          // After acknolegment by another module
          gpActivate_object_fingertips_collisions(_robot, object->joints[1]->o, handProp, armId);
          status = MANIPULATION_TASK_OK;
        }
        else {
          cout << "Warning::ManipulationPlanner::armPickGoto computeTraj failed between openConfig and graspConfig" << endl;
        }
        gpActivate_object_fingertips_collisions(_robot, object->joints[1]->o, handProp, armId);
      }
      else {
        cout << "Warning::ManipulationPlanner::armPickGoto computeTraj failed between approachFreeConfig and openConfig" << endl;
      }
    }
    else {
      cout << "Warning::ManipulationPlanner::armPickGoto computeTraj failed between qStart and approachFreeConfig" << endl;
    }
    setSafetyDistance(_robot, 0);
    return status;
}

//! The Arm Pick Take To Free method takes in an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qGoal : the configuration to bring the object to
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFreePointCheckEscape(int armId, configPt qStart, 
                                                                                 std::vector<double> &objGoto , 
                                                                      p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs){
  
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  int updateTatt = false;
  ArmManipulationData& armData = (*_robot->armManipulationData)[armId];
  configPt qGoal = NULL, approachGraspConfig = NULL; 
  
  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  
  p3d_matrix4 tAtt;
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    updateTatt = true;
  }

  if(!_manipData.getGrasp()){
    _manipData.setGrasp(new gpGrasp());
  }
  activateCcCntrts(_robot, armId, false);
  double confCost = -1;
  
  // Compute the free conf and verify that 
  // the escape object exists
  ManipulationData manipData(_robot);
  int nbTry = 0;
  status = MANIPULATION_TASK_NO_PLACE;
  while (status != MANIPULATION_TASK_OK && nbTry < _placementTry){
    ++nbTry;
    status = _manipConf.getHoldingOpenApproachExtractConfs(object, objGoto, NULL, armId, 
                                                           *(_manipData.getGrasp()), tAtt, manipData);
  }
  
  cout << "Grasp conf : " << manipData.getGraspConfig() << endl;
  
  // Compute the approch grasp config for the start config.
  if(!_manipData.getApproachGraspConfig())
  {
    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    p3d_set_and_update_this_robot_conf(_robot, qStart);
    
    approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_manipData.getGrasp()), qStart, tAtt);
    
    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    p3d_set_and_update_this_robot_conf(_robot, qStart);
    armData.setCarriedObject((p3d_rob*) NULL);
    _robot->isCarryingObject = FALSE;
    
    if(!approachGraspConfig){
      status = MANIPULATION_TASK_NO_GRASP;
    }
  }else{
    approachGraspConfig = p3d_copy_config(_robot, _manipData.getApproachGraspConfig());
  }
  
  _manipData.clear();
  _manipData = manipData;
  
  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armPickTakeToFree(armId, qStart, _manipData.getGraspConfig(), object, support, 
                               /*_manipData.getApproachGraspConfig()*/approachGraspConfig, *_manipData.getGrasp(), trajs);
  }
  
  return status;
}

//! The Arm Pick Take To Free method takes in an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qGoal : the configuration to bring the object to
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFreePoint(int armId, configPt qStart, std::vector<double> &objGoto , 
                                                                      p3d_rob* object, p3d_rob* support, 
                                                                      std::vector <p3d_traj*> &trajs){
  
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  int updateTatt = false;
  
  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  
  p3d_matrix4 tAtt;
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    updateTatt = true;
  }
  if(!_manipData.getGrasp()){
    _manipData.setGrasp(new gpGrasp());
  }
  activateCcCntrts(_robot, armId, false);
  double confCost = -1;
  
  configPt qGoal = _manipConf.getFreeHoldingConf(object, armId, 
                                        *_manipData.getGrasp(), tAtt, confCost,
                                        objGoto, NULL);

  if(updateTatt){
    _manipData.setAttachFrame(tAtt);
  }
  if (qGoal) {
    if(!_manipData.getApproachGraspConfig()){
      configPt approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_manipData.getGrasp()), qStart, tAtt);
      if(approachGraspConfig){
        _manipData.setApproachGraspConfig(approachGraspConfig);
      }else{
         status = MANIPULATION_TASK_NO_GRASP;
      }
    }
  } else {
    status = MANIPULATION_TASK_NO_GRASP;
  }
  
  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armPickTakeToFree(armId, qStart, qGoal, object, support, 
                               _manipData.getApproachGraspConfig(), *_manipData.getGrasp(), trajs);
  }

  return status;
}


//! The Arm Pick Take To Free method takes an object in a goal stable pose
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qGoal : the configuration to bring the object to
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
 MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, 
                                                                  p3d_rob* object, p3d_rob* support, 
                                                                  std::vector <p3d_traj*> &trajs) {
    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;

  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);

  p3d_matrix4 tAtt, bakTatt;
  bool updateTatt = false;
  ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    //NULL frame compute it.
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, bakTatt);
    p3d_compute_Tatt(mData.getCcCntrt());
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, tAtt);
    _manipData.setAttachFrame(tAtt);
    updateTatt = true;
  }

  if(!_manipData.getGrasp()){
    gpGrasp* grasp = new gpGrasp();
    gpSet_grasp_configuration(_robot, *grasp, qStart, armId);
    _manipData.setGrasp(grasp);
  }

  activateCcCntrts(_robot, armId, false);

  if(!_manipData.getApproachGraspConfig()){
    configPt approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_manipData.getGrasp()), qStart, tAtt);
    if(approachGraspConfig){
      _manipData.setApproachGraspConfig(approachGraspConfig);
    }else{
        status = MANIPULATION_TASK_NO_GRASP;
    }
  }
  if (status == MANIPULATION_TASK_OK) {
      // Compute the path between theses configurations
      status = armPickTakeToFree(armId, qStart, qGoal, object, support,
                                  _manipData.getApproachGraspConfig(),
                                  *_manipData.getGrasp(),
                                  trajs);
  }
  if(updateTatt){
    p3d_mat4Copy(bakTatt, mData.getCcCntrt()->Tatt);
  }
  return status;
}

//! The Arm Pick Take To Free method takes in an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qGoal : the configuration to bring the object to
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal,  p3d_rob* object, p3d_rob* support, configPt approachGraspConfig , gpGrasp &grasp , std::vector <p3d_traj*> &trajs) {
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_NO_TRAJ_FOUND;

  p3d_traj* traj = NULL;
  ArmManipulationData& armData = (*_robot->armManipulationData)[armId];
  ManipulationUtils::fixAllHands(_robot, qStart, false );
  
  if(!_useBaseMotion) {
    fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);
  }
  else {
    unFixJoint(_robot, _robot->baseJnt);
  }

  deactivateCcCntrts(_robot,armId);

  p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
  p3d_matrix4 tAtt;
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] != 0 || tAtt[0][1] != 0 || tAtt[0][2] != 0 || tAtt[0][3] != 0){
      p3d_mat4Copy(tAtt, armData.getCcCntrt()->Tatt);
  }

  checkConfigForCartesianMode(qStart, object);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, qStart);
  }
  checkConfigForCartesianMode(approachGraspConfig, object);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfig);
  }
  checkConfigForCartesianMode(qGoal, object);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, qGoal);
  }

  p3d_set_and_update_this_robot_conf(_robot, qStart);
  gpHand_properties handProp = armData.getHandProperties();
  std::vector <double> handConf;
  gpGet_hand_configuration(_robot, handProp, armId, handConf);

  gpSet_hand_configuration(_robot, handProp, handConf, approachGraspConfig, armId);
  p3d_set_and_update_this_robot_conf(_robot, approachGraspConfig);
  p3d_get_robot_config_into(_robot, &approachGraspConfig);

  gpSet_hand_configuration(_robot, handProp, handConf, qGoal, armId);
  p3d_set_and_update_this_robot_conf(_robot, qGoal);
  p3d_get_robot_config_into(_robot, &qGoal);

  gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1
  if(support){
    p3d_col_deactivate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
  }
  //Compute to Approach config
  if ((traj = computeTrajBetweenTwoConfigs(qStart, approachGraspConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
  {
    trajs.push_back(traj);
    if(support){
      p3d_col_activate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
    }
    // Compute to Open config
    if ((traj = computeTrajBetweenTwoConfigs(approachGraspConfig, qGoal, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
    {
        trajs.push_back(traj);
        cout << "Manipulation : traj found" << endl;
        status =  MANIPULATION_TASK_OK;
    }
  }
  gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1
  if(support){
    p3d_col_activate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
  }
// Warning comment to see traj in Linear mode
  _robot->isCarryingObject = false;
  armData.setCarriedObject((p3d_rob*)NULL);
  return status;
}

//! The Arm Take To Place method takes an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param placement : pointer to the p3d_rob that represent the placement object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armTakeToPlace(int armId, configPt qStart, p3d_rob* object, p3d_rob* support, p3d_rob* placement, std::vector <p3d_traj*> &trajs) {
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    return status;
}

//! The Arm Take To Place method takes an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : the start configuration
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param objGoto : the position and orientation where to deposit the object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armTakeToPlace(int armId, configPt qStart, p3d_rob* object,  p3d_rob* support, std::vector<double> &objGoto, p3d_rob* placement, std::vector <p3d_traj*> &trajs) {
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  int updateTatt = false;
  ArmManipulationData& armData = (*_robot->armManipulationData)[armId];
  configPt qGoal = NULL, approachGraspConfig = NULL; 

  if(!object){
    return MANIPULATION_TASK_UNKNOWN_OBJECT;
  }
  if(!placement){
    return MANIPULATION_TASK_NO_PLACE;
  }

  deactivateCcCntrts(_robot, armId);
  desactivateTwoJointsFixCntrt(_robot, armData.getManipulationJnt(), 
                               armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
  
  (*_robot->armManipulationData)[armId].setCarriedObject(object);
  _robot->isCarryingObject = TRUE;
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  (*_robot->armManipulationData)[armId].setCarriedObject((p3d_rob*) NULL);
  _robot->isCarryingObject = FALSE;

  p3d_matrix4 tAtt;
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    updateTatt = true;
  }
  if(!_manipData.getGrasp()){
    _manipData.setGrasp(new gpGrasp());
  }
  activateCcCntrts(_robot, armId, false);
  double confCost = -1;
  
  status = MANIPULATION_TASK_NO_PLACE;
  int nbTry = 0;
  ManipulationData manipData(_robot);
  while (status != MANIPULATION_TASK_OK && nbTry < _placementTry){
    ++nbTry;
    status = _manipConf.getHoldingOpenApproachExtractConfs(object, objGoto, placement, armId, 
                                                           *(_manipData.getGrasp()), tAtt, manipData);
  }

  // Compute the approch grasp config for the start config.
  if(!_manipData.getApproachGraspConfig()){
    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    p3d_set_and_update_this_robot_conf(_robot, qStart);
    approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_manipData.getGrasp()), qStart, tAtt);
    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    p3d_set_and_update_this_robot_conf(_robot, qStart);
    armData.setCarriedObject((p3d_rob*) NULL);
    _robot->isCarryingObject = FALSE;
    if(!approachGraspConfig){
      status = MANIPULATION_TASK_NO_GRASP;
    }
  }else{
    approachGraspConfig = p3d_copy_config(_robot, _manipData.getApproachGraspConfig());
  }
  
  // Set current _manipData from manipData
  _manipData.clear();
  _manipData = manipData;
  
  if (status == MANIPULATION_TASK_OK){
    // Compute the path between theses configurations
    status = armTakeToPlace(armId, qStart, approachGraspConfig, _manipData.getApproachGraspConfig(), _manipData.getGraspConfig(), object, support, placement, trajs);
  }
  else {
      cout << "Fail to findArmGraspsConfigs(armId,object,_manipData)" << endl;
  }
  return status;
}

//! The Arm Take To Place method takes an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : the start configuration
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armTakeToPlace(int armId, configPt qStart, configPt approachGraspConfig, configPt approachGraspConfigPlacement, configPt qGoal, p3d_rob* object,  p3d_rob* support, p3d_rob* placement, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_NO_TRAJ_FOUND;

  p3d_traj* traj = NULL;
  ArmManipulationData& armData = (*_robot->armManipulationData)[armId];
  ManipulationUtils::fixAllHands(_robot, qStart, false );
  
  if(!_useBaseMotion) {
    fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);
  }
  else {
    unFixJoint(_robot, _robot->baseJnt);
  }
  deactivateCcCntrts(_robot,armId);

  p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
  p3d_matrix4 tAtt;
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] != 0 || tAtt[0][1] != 0 || tAtt[0][2] != 0 || tAtt[0][3] != 0){
      p3d_mat4Copy(tAtt, armData.getCcCntrt()->Tatt);
  }

  checkConfigForCartesianMode(qStart, object);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, qStart);
      showConfig_2(qStart);
  }
  checkConfigForCartesianMode(approachGraspConfig, object);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfig);
      showConfig_2(approachGraspConfig);
  }
  checkConfigForCartesianMode(approachGraspConfigPlacement, object);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfigPlacement);
      showConfig_2(approachGraspConfigPlacement);
  }
  checkConfigForCartesianMode(qGoal, object);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, qGoal);
      showConfig_2(qGoal);
  }

  p3d_set_and_update_this_robot_conf(_robot, qStart);
  gpHand_properties handProp = armData.getHandProperties();
  std::vector <double> handConf;
  gpGet_hand_configuration(_robot, handProp, armId, handConf);

  gpSet_hand_configuration(_robot, handProp, handConf, approachGraspConfig, armId);
  p3d_set_and_update_this_robot_conf(_robot, approachGraspConfig);
  p3d_get_robot_config_into(_robot, &approachGraspConfig);

  gpSet_hand_configuration(_robot, handProp, handConf, approachGraspConfigPlacement, armId);
  p3d_set_and_update_this_robot_conf(_robot, approachGraspConfigPlacement);
  p3d_get_robot_config_into(_robot, &approachGraspConfigPlacement);
  
  gpSet_hand_configuration(_robot, handProp, handConf, qGoal, armId);
  p3d_set_and_update_this_robot_conf(_robot, qGoal);
  p3d_get_robot_config_into(_robot, &qGoal);

  gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1
  if(support){
    p3d_col_deactivate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
  }
  //Compute to Approach config
  if ((traj = computeTrajBetweenTwoConfigs(qStart, approachGraspConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL)){
    trajs.push_back(traj);
    if(support){
      p3d_col_activate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
    }
    // Compute to Open config
//     print_config(_robot, approachGraspConfigPlacement);
    if ((traj = computeTrajBetweenTwoConfigs(approachGraspConfig, approachGraspConfigPlacement, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL)){
      trajs.push_back(traj);
      if(placement){
        p3d_col_deactivate_pair_of_objects(object->joints[1]->o, placement->joints[1]->o);
      }
      if ((traj = computeTrajBetweenTwoConfigs(approachGraspConfigPlacement, qGoal, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL)){
        trajs.push_back(traj);
        if(placement){
          p3d_col_activate_pair_of_objects(object->joints[1]->o, placement->joints[1]->o);
        }
        cout << "Manipulation : traj found" << endl;
        status =  MANIPULATION_TASK_OK;
      }
    }
  }
  gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1
  if(support){
    p3d_col_activate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
  }
// Warning comment to see traj in Linear mode
  _robot->isCarryingObject = false;
  armData.setCarriedObject((p3d_rob*)NULL);
  return status;
}

//! The Arm Pick Place from Free method takes an object from a grasp configuration
//! to a placement. It's computes a vector of trajectories
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : The grasp configuration
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param placement : pointer to the p3d_rob that represent the placement object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs) {
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    return status;
}

//! The Arm Pick Place from Free method takes an object from a grasp configuration
//! to a placement. It's computes a vector of trajectories
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : The grasp configuration
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param placement : pointer to the p3d_rob that represent the placement object
//! @param objGoto : XYZ position where to place the object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, std::vector<double> &objGoto, p3d_rob* placement, std::vector <p3d_traj*> &trajs) {
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  int updateTatt = false;
  configPt qGoal = NULL;

  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);

  p3d_matrix4 tAtt;
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    updateTatt = true;
  }
  if(!_manipData.getGrasp()){
    _manipData.setGrasp(new gpGrasp());
  }
  activateCcCntrts(_robot, armId, false);
  double confCost = -1;
  
  (*_robot->armManipulationData)[armId].setCarriedObject(object);
  _robot->isCarryingObject = TRUE;
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  (*_robot->armManipulationData)[armId].setCarriedObject((p3d_rob*) NULL);
  _robot->isCarryingObject = FALSE;
  
  status = MANIPULATION_TASK_NO_PLACE;
  int nbTry = 0;
  
  while(status != MANIPULATION_TASK_OK && nbTry < _placementTry){
    ++nbTry;
    status = _manipConf.getHoldingOpenApproachExtractConfs(object, objGoto, placement, armId, *(_manipData.getGrasp()), tAtt, _manipData);
  }

  if (status == MANIPULATION_TASK_OK)
  {
    //Compute the path between theses configurations
    status = armPlaceFromFree(armId, qStart, object, placement, _manipData.getApproachGraspConfig(), _manipData.getGraspConfig(),trajs);
  }
  else {
      cout << "Fail to findArmGraspsConfigs(armId,object,_manipData)" << endl;
  }

  return status;
}

//! The Arm Pick Place from Free method takes an object from a grasp configuration
//! to a placement. It's computes a vector of trajectories
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : The grasp configuration
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param placement : pointer to the p3d_rob that represent the placement object
//! @param approachGraspConfig : The approach grasp configuration
//! @param depositConfig : The deposit configuration
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, p3d_rob* placement, configPt approachGraspConfig, configPt depositConfig, std::vector <p3d_traj*> &trajs) {

    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_NO_TRAJ_FOUND;
    p3d_traj* traj = NULL;
    ArmManipulationData& armData = (*_robot->armManipulationData)[armId];
    
    ManipulationUtils::fixAllHands( _robot, qStart, false );
    fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

    deactivateCcCntrts(_robot,armId);

    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    p3d_matrix4 tAtt;
    _manipData.getAttachFrame(tAtt);
    if(tAtt[0][0] != 0 || tAtt[0][1] != 0 || tAtt[0][2] != 0 || tAtt[0][3] != 0){
       p3d_mat4Copy(tAtt, armData.getCcCntrt()->Tatt);
    }
    
    checkConfigForCartesianMode(qStart, object);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, qStart);
    }
    checkConfigForCartesianMode(approachGraspConfig, object);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfig);
    }
    checkConfigForCartesianMode(depositConfig, object);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, depositConfig);
    }

    p3d_set_and_update_this_robot_conf(_robot, qStart);
    gpHand_properties handProp = armData.getHandProperties();
    std::vector <double> handConf;
    gpGet_hand_configuration(_robot, handProp, armId, handConf);

    gpSet_hand_configuration(_robot, handProp, handConf, approachGraspConfig, armId);
    p3d_set_and_update_this_robot_conf(_robot, approachGraspConfig);
    p3d_get_robot_config_into(_robot, &approachGraspConfig);

    gpSet_hand_configuration(_robot, handProp, handConf, depositConfig, armId);
    p3d_set_and_update_this_robot_conf(_robot, depositConfig);
    p3d_get_robot_config_into(_robot, &depositConfig);

    gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1

    //Compute to Approach config
    if ((traj = computeTrajBetweenTwoConfigs(qStart, approachGraspConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
    {
        trajs.push_back(traj);
        if(placement){
          p3d_col_deactivate_pair_of_objects(object->joints[1]->o, placement->joints[1]->o);
        }
        if ((traj = computeTrajBetweenTwoConfigs(approachGraspConfig, depositConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
        {
            trajs.push_back(traj);
            cout << "Manipulation : traj found" << endl;
            status =  MANIPULATION_TASK_OK;
        }
        if(placement){
          p3d_col_activate_pair_of_objects(object->joints[1]->o, placement->joints[1]->o);
        }
    }
    gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1

  // Warning comment to see traj in Linear mode

  _robot->isCarryingObject = false;
  armData.setCarriedObject((p3d_rob*)NULL);
  
  desactivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(),
                               armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
    return status;
}

//! The Arm Escape Object method takes Escape the robot from the grasped object
//! when it placed. The hand openning and the arm motion are both planned
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : The grasp configuration
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armEscapeObject(int armId, configPt qStart, p3d_rob* object, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  int updateTatt = false;
  ArmManipulationData& armData = (*_robot->armManipulationData)[armId];

  if(!object){
    return MANIPULATION_TASK_UNKNOWN_OBJECT;
  }
  
  if(qStart[6] == 0.0 && qStart[7] == 0.0)
  {
    print_config(_robot,qStart);
    printf("problem armEscapeObject\n");
  }
  
  deactivateCcCntrts(_robot, armId);
  desactivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
  
  p3d_set_and_update_this_robot_conf(_robot, qStart);

  p3d_matrix4 tAtt;
  _manipData.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    p3d_compute_Tatt(armData.getCcCntrt());
    _manipData.setAttachFrame(armData.getCcCntrt()->Tatt);
    _manipData.getAttachFrame(tAtt);
  }
  if(!_manipData.getGrasp()){
    gpGrasp grasp;
    grasp.hand_type = armData.getHandProperties().type;
    grasp.config.resize(armData.getHandProperties().nb_dofs);
    grasp.openConfig.resize(armData.getHandProperties().nb_dofs);
    gpSet_grasp_configuration(_robot, grasp, qStart, armId);
    _manipData.setGrasp(&grasp);
  }
//   activateCcCntrts(_robot, armId, false);
  _robot->isCarryingObject = false;
//   if(!_manipData.getOpenConfig()){
    configPt openGraspConfig = _manipConf.getOpenGraspConf(object, armId, *_manipData.getGrasp(), qStart);
    if (openGraspConfig) {
      _manipData.setOpenConfig(openGraspConfig);
      p3d_destroy_config(_robot, openGraspConfig);
    }else{
      status = MANIPULATION_TASK_NO_GRASP;
    }
//   }

  if(status == MANIPULATION_TASK_OK){
//     if(!_manipData.getApproachFreeConfig()){
      configPt approachFreeConfig = _manipConf.getApproachFreeConf(object, armId, *(_manipData.getGrasp()), qStart, tAtt);
      if(approachFreeConfig){
        _manipData.setApproachFreeConfig(approachFreeConfig);
        p3d_destroy_config(_robot, approachFreeConfig);
      }else{
          status = MANIPULATION_TASK_NO_GRASP;
      }
//     }
  }
  deactivateCcCntrts(_robot, armId);
  
  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armEscapeObject(armId, qStart, _manipData.getOpenConfig(), _manipData.getApproachFreeConfig(), object, trajs);
  }

  return status;
}

//! The Arm Escape Object method takes Escape the robot from the grasped object
//! when it placed. The hand openning and the arm motion are both planned
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : The grasp configuration
//! @param openGraspConfig : The open grasp configuration
//! @param approachFreeConfig : The approach free configuration
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armEscapeObject(int armId, configPt qStart, configPt openGraspConfig, configPt approachFreeConfig, p3d_rob* object, std::vector <p3d_traj*> &trajs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  p3d_traj* traj = NULL;

  fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

  checkConfigForCartesianMode(qStart, NULL);
  checkConfigForCartesianMode(openGraspConfig, NULL);
  checkConfigForCartesianMode(approachFreeConfig, NULL);
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
  gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
  
  if(qStart[6] == 0.0 && qStart[7] == 0.0)
  {
    printf("problem armEscapeObject\n");
  }

  if (MPDEBUG) {
      ManipulationUtils::copyConfigToFORM(_robot, qStart);
      ManipulationUtils::copyConfigToFORM(_robot, openGraspConfig);
      ManipulationUtils::copyConfigToFORM(_robot, approachFreeConfig);
  }
  setSafetyDistance(_robot, getSafetyDistanceValue());
  ManipulationUtils::unFixAllHands(_robot);
  if ((traj = computeTrajBetweenTwoConfigs(qStart, openGraspConfig, &status)) || 
      (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL)){
    trajs.push_back(traj);
    ManipulationUtils::fixAllHands(_robot, qStart, false);
    if ((traj = computeTrajBetweenTwoConfigs(openGraspConfig, approachFreeConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL)){
      trajs.push_back(traj);
      cout << "Manipulation : traj found" << endl;
      status =  MANIPULATION_TASK_OK;
    }
  }
  setSafetyDistance(_robot, 0);
  
  gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId); //the hand name is hand1 for arm0 and hand 2 for arm1
  
  return status;
}

/* ******************************* */
/* ******** Task Planning ******** */
/* ******************************* */
#ifdef MULTILOCALPATH
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armReplan(p3d_vector3 target, int qSwitchId, SM_TRAJ &smTraj) 
{
  p3d_traj* traj = NULL;
  MANIPULATION_TASK_MESSAGE returnMessage;
  
  if (!_robot) {
    printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  
  p3d_multiLocalPath_disable_all_groupToPlan(_robot, FALSE);
  p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1, FALSE);
  
  if ((traj = _replanningMethod(_robot, _robotPath, target, qSwitchId)) != NULL) 
  { 
    smTraj.clear();
    /* COMPUTE THE SOFTMOTION TRAJECTORY */
    MANPIPULATION_TRAJECTORY_CONF_STR conf;
    computeSoftMotion(traj/*s.at(i)*/, conf, smTraj);
    returnMessage = MANIPULATION_TASK_OK;
  } 
  else {
    returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
  }
  
  return returnMessage;
}
#endif

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, const char* placementName, gpGrasp& grasp, std::vector <p3d_traj*> &trajs) {
  
  if (!_robot) {
    printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  configPt qi = p3d_copy_config(_robot, qStart), qf = p3d_copy_config(_robot, qGoal);
  p3d_rob* cur_robot = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  p3d_rob* object = p3d_get_robot_by_name(objectName);
  if(!object && task != ARM_FREE && task != ARM_EXTRACT ){
    p3d_destroy_config(_robot, qi);
    p3d_destroy_config(_robot, qf);
    return MANIPULATION_TASK_UNKNOWN_OBJECT;
  }
  p3d_rob* support = p3d_get_robot_by_name(supportName);
  p3d_rob* placement = p3d_get_robot_by_name(placementName);
  p3d_sel_desc_id(P3D_ROBOT, _robot);
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  if (armId < 0 || armId >= (int)_robot->armManipulationData->size()) {
    status = MANIPULATION_TASK_INVALID_TASK;
  } else {
    //ENV.setBool(Env::drawTraj, false);
    checkConfigForCartesianMode(qi, object);
    checkConfigForCartesianMode(qf, object);
    ManipulationUtils::fixAllHands(_robot, qi, false);
    ManipulationUtils::fixAllHands(_robot, qf, false);
//     p3d_set_and_update_this_robot_conf(_robot, qi);
    if(object){
      gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
      gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    }
    if(!p3d_is_collision_free(_robot, qi)){
      p3d_destroy_config(_robot, qi);
      p3d_destroy_config(_robot, qf);
      cout << "qStart in collision" << endl;
      p3d_print_col_pair();
      if(object){
        gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
        gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
      }
      return MANIPULATION_TASK_INVALID_QSTART;
    }
    if(object){
      gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
      gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    }
    //Remove collision tolerence for the object
    if(object){
      p3d_set_collision_tolerance_inhibition(object, TRUE);
    }
    //Clear the graph
    cleanRoadmap();
    
    // Change wither the manip. conf. planner uses mobile base
    // degree of freedom
    _manipConf.setMobileBaseMode( _useBaseMotion );
    
    switch (task) {
      case ARM_FREE: {
        printf("plan for task ");
        if(!ManipulationUtils::isValidVector(objGoto)){
          printf("ARM_FREE (armToFree)\n");
          status = armToFree(armId, qi, qf, true, object, trajs);
        }else{
          printf("ARM_FREE (armToFreePoint)\n");
          status = armToFreePoint(armId, qi, objGoto, object, trajs);
        }
        break;
      }
      case ARM_PICK_GOTO: {
        printf("plan for ARM_PICK_GOTO task\n");
        status = armPickGoto(armId, qi, object, grasp, trajs);
        break;
      }
      case ARM_TAKE_TO_FREE: {
        printf("plan for ARM_PICK_TAKE_TO_FREE task\n");
        if(!ManipulationUtils::isValidVector(objGoto)){
          status = armPickTakeToFree(armId, qi, qf, object, support, trajs);
        }else{
         // status = armPickTakeToFreePoint(armId, qi, objGoto, object, support, trajs);
          status = armPickTakeToFreePointCheckEscape(armId, qi, objGoto, object, support, trajs);
        }
        break;
      }
      case ARM_TAKE_TO_PLACE: {
        printf("plan for ARM_PICK_TAKE_TO_PLACE task\n");
        if(!ManipulationUtils::isValidVector(objGoto)){
//           status = armPlaceFromFree(armId, qi, object, placement, trajs);
        }else{
          status = armTakeToPlace(armId, qi, object, support, objGoto, placement, trajs);
        }
        break;
      }
      case ARM_PLACE_FROM_FREE: {
        printf("plan for ARM_PLACE_FROM_FREE task\n");
        if(!ManipulationUtils::isValidVector(objGoto)){
//           status = armPlaceFromFree(armId, qi, object, placement, trajs);
        }else{
          status = armPlaceFromFree(armId, qi, object, objGoto, placement, trajs);
        }
        break;
      }
      case ARM_EXTRACT: {
        printf("plan for ARM_EXTRACT task\n");
        status = armExtract(armId, qi, object, trajs);
        break;
      }
      case ARM_ESCAPE_OBJECT:{
        printf("plan for ARM_ESCAPE_OBJECT task\n");
        status = armEscapeObject(armId, qi, object, trajs);
        break;
      }
      default: {
        printf("%s: %d: ManipulationPlanner::armPlanTask(): wrong task.\n", __FILE__, __LINE__);
        status = MANIPULATION_TASK_INVALID_TASK;
        break;
      }
    }
    ManipulationUtils::unfixManipulationJoints(_robot, armId);
    p3d_sel_desc_id(P3D_ROBOT,cur_robot);
    //g3d_draw_allwin_active();
    if (status == MANIPULATION_TASK_OK) {
      printf("BioMove3D: armPlanTask OK\n");
    } else {
      printf("BioMove3D: armPlanTask Fail\n");
    }
  }
  if(object){
    p3d_set_collision_tolerance_inhibition(object, FALSE);
  }
  p3d_destroy_config(_robot, qi);
  p3d_destroy_config(_robot, qf);
  return status;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, const char* placementName, std::vector <p3d_traj*> &trajs){
  gpGrasp grasp;
  return armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, placementName, grasp, trajs);
}

//! Replans a path form the variable _robotPath
#ifdef MULTILOCALPATH
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, const char* placementName, gpGrasp& grasp, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs) {
    std::vector <p3d_traj*> trajs;
    p3d_traj* traj = NULL;
    MANIPULATION_TASK_MESSAGE returnMessage;

    if (!_robot) {
        printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }

    p3d_multiLocalPath_disable_all_groupToPlan(_robot, FALSE);
    p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1, FALSE);

    if ((returnMessage = armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, placementName, grasp, trajs)) == MANIPULATION_TASK_OK) {
      //concatene
      if (concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
          cout << "Copy path to _robotPath" << endl;
          _robotPath = p3d_create_traj_by_copy(traj); // Stores the robot path
          cout << _robotPath->courbePt << endl;
          cout << traj->courbePt << endl;
          smTrajs.clear();
//         for(unsigned i = 0; i < trajs.size(); i++){
        /* COMPUTE THE SOFTMOTION TRAJECTORY */
          MANPIPULATION_TRAJECTORY_CONF_STR conf;
          SM_TRAJ smTraj;
          computeSoftMotion(traj/*s.at(i)*/, conf, smTraj);
          confs.push_back(conf);
          smTrajs.push_back(smTraj);
        
          ManipPlannerLastTraj = smTraj;
//         }
      } else {
        returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
      }
    }
    return returnMessage;
}

// return 0 if OK
// generate the discretised evolution of the specified jnt 
// in the robot frame at the rate sampling_time
//int ManipulationPlanner::getEvolutionOfSpecificJntInRobotFrame(p3d_jnt* jnt, double sampling_time, std::vector< std::vector<SM_COND> > discTraj)
//{
////  configPt conf;
////  p3d_matrix4 r7mat4;
////
////  for(double time=0.0; time < _robot->tcur->
////  conf = p3d_config_at_param_along_traj(_robot->tcur , time);
////  p3d_set_and_update_this_robot_conf(_robot, conf);
//// 
////  base = _robot->baseJnt->abs_pos; //base dans monde
////  r7mat4 = jnt->abs_pos; // r7 dans monde
////  // compute the pose of the jnt int the base frame of the robot
////  
////
//// for(unsigned int i=0; i<discTraj.size(); i++) {
////   discTraj[i].resize(positions.size());
//// }
//// 
//// // j is axis id
//// // i is the time
//// for(unsigned int i=0; i< positions.size(); i++) {
////   for(unsigned int j=0; j<positions[0].size(); j++) {
////     discTraj[j][i].x = positions[i][j];
////     if(i>0) {
////       discTraj[j][i].v = (discTraj[j][i].x - discTraj[j][i-1].x)/sampling_time;
////       if(i>1) {
////	 discTraj[j][i].a = (discTraj[j][i].v - discTraj[j][i-1].v)/sampling_time;
////       } else {
////	 discTraj[j][i].a = 0.0;
////       }
////     } else {
////       discTraj[j][i].v = 0.0;
////       discTraj[j][i].a = 0.0;
////     }
////   }
//// }
//  return 0;
//}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, const char* placementName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs) 
{
  gpGrasp grasp;
  return armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, placementName, grasp, confs, smTrajs);
}


//
// New Function to generate traje
//
MANIPULATION_TASK_MESSAGE armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, const char* placementName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, SM_TRAJ &smTraj_q, SM_TRAJ &smTraj_x, Gb_th th_Rrob_Robj, Gb_th th_Robj_R7)
{
// gpGrasp grasp;
// std::vector<SM_TRAJ> smTrajs;
// MANIPULATION_TASK_MESSAGE resp = armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, placementName, grasp, confs, smTrajs);
// smTraj_q = smTrajs[0];
//
// //TODO
// // fill  SM_TRAJ &smTraj_x, Gb_th th_Rrob_Robj, Gb_th th_Robj_R7
// std::vector< std::vector<SM_COND> > discTraj;
// p3d_jnt * r7_jnt = p3d_get_robot_jnt_by_name(_robot, "armJoint7");
// if(r7_jnt == NULL) {
//   printf(" well, the unknown error is ............... that the %s joint is not inside your robot ! I can not generate the cartesian trajectory but I have setted the articular one \n");
//   return MANIPULATION_TASK_ERROR_UNKNOWN;
// }
// double sampling_time = 0.01;
// getEvolutionOfSpecificJntInRobotFrame(r7_jnt, sampling_time, discTraj);
//
//
// smTraj.approximate(discTraj, SAMPLING_TIME, 0.01, 0.1, 36, true);
//
// return resp;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::planNavigation(configPt qStart, configPt qGoal, bool fixAllArm, std::vector <p3d_traj*> &trajs){
  
  
    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    p3d_traj* traj = NULL;

    setNavigationPlanner();

    if(fixAllArm) {
      cout << "planNavigation: arms are fixed "<< endl;
    } else {
      cout << "planNavigation: arms are not fixed "<< endl;
    }
    //bool fixAllArm = true;

    if (!_robot) {
      printf("%s: %d: ManipulationPlanner::planNavigation(): No robot initialized.\n", __FILE__, __LINE__);
      return MANIPULATION_TASK_NOT_INITIALIZED;
    }
    configPt qi = p3d_copy_config(_robot, qStart), qf = p3d_copy_config(_robot, qGoal);
    p3d_rob* cur_robot = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);


    p3d_set_and_update_robot_conf(qStart);
    if(fixAllArm == true) {
      for (int i = 2; i < _robot->njoints + 1; i++) {
	p3d_jnt * joint = _robot->joints[i];
	
	fixJoint(_robot, joint, joint->jnt_mat);
	p3d_update_this_robot_pos(_robot);
      }
    }

  for (int i = 2; i < _robot->njoints + 1; i++) {
    p3d_jnt * joint = _robot->joints[i];
    
    if(strcmp(joint->name, "Torso") == 0) {
      fixJoint(_robot, joint, joint->jnt_mat);
      cout << "planNavigation: Torso joint are  fixed "<< endl;
    }
    p3d_update_this_robot_pos(_robot);
  }
    p3d_update_this_robot_pos(_robot);

    //Clear the graph
//    cleanRoadmap();


    for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
        deactivateCcCntrts(_robot, i);
    }

    if((traj = computeTrajBetweenTwoConfigs(qStart, qGoal, &status))){
      trajs.push_back(traj);
    }

//    setDefaultPlanner();
  return status;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::planNavigation(configPt qStart, configPt qGoal,  bool fixAllArm, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs){
    std::vector <p3d_traj*> trajs;
    p3d_traj* traj = NULL;
    MANIPULATION_TASK_MESSAGE returnMessage;

    if (!_robot) {
        printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }

    p3d_multiLocalPath_disable_all_groupToPlan(_robot, FALSE);
    p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1, FALSE);

    if ((returnMessage = planNavigation(qStart, qGoal, fixAllArm, trajs)) == MANIPULATION_TASK_OK) {
      //concatene
      if (concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
        _robotPath = _robot->tcur; // Stores the robot path
        smTrajs.clear();

        /* COMPUTE THE SOFTMOTION TRAJECTORY */
          MANPIPULATION_TRAJECTORY_CONF_STR conf;
          SM_TRAJ smTraj;
          computeSoftMotion(traj, conf, smTraj);
          confs.push_back(conf);
          smTrajs.push_back(smTraj);

          std::cout << " SoftMotion trajectory OK" << endl;

      } else {
        returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
      }
    }
    return returnMessage;
}


#endif
