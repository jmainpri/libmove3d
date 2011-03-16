#include "ManipulationPlanner.hpp"

#include "lightPlanner.h"
#include "lightPlannerApi.h"

#ifdef CXX_PLANNER
#include "planner_cxx/plannerFunctions.hpp"
#endif

#include "robotPos.h"

#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

#include <list>
#include <algorithm>

static bool MPDEBUG=false;

using namespace std;

/* ******************************* */
/* ******* (De)Constructor ******* */
/* ******************************* */
ManipulationPlanner::ManipulationPlanner(p3d_rob *robot) :_robot(robot), _configs(robot), _manipConf(robot){
    // Set planner funtions
    _plannerMethod = rrtQuerry;
    _smoothingMethod = optimiseTrajectory;
  
    // Manipulation planner
    _optimizeSteps = 100;
    _optimizeTime = 4.0; // 4 secondes
    _safetyDistanceValue = 0.0;
    setMaxNumberOfTryForIK(10000);

    setDebugSoftMotionMode(false);

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
        if ((*_robot->armManipulationData)[i].getCartesianGroup() == -1) {
            printf("%s: %d: ManipulationPlanner::ManipulationPlanner: the arm cartesian group is not setted.\n",__FILE__,__LINE__);
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
void ManipulationPlanner::setDebugMode(bool value){
  MPDEBUG = value;
}

void ManipulationPlanner::setDebugSoftMotionMode(bool value){
  ENV.setBool(Env::writeSoftMotionFiles, value);
}

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

/* ******************************* */
/* ******* Hands / Grasping ****** */
/* ******************************* */
void ManipulationPlanner::fitConfigurationToRobotBounds(configPt q){
  for (int i = 0; i < _robot->njoints; i++) {
    p3d_jnt* jnt = _robot->joints[i];
    for (int j = 0; j < jnt->dof_equiv_nbr; j++) {
      double vmin = -P3D_HUGE, vmax = P3D_HUGE;
      p3d_jnt_get_dof_bounds(jnt, j, &vmin, &vmax);
      if(q[jnt->index_dof + j] < vmin){
        cout << "WARNING : Configuration correction : Joint " << jnt->name << " has value = " << q[jnt->index_dof + j] << " and its minimal bound = " << vmin << endl;
        q[jnt->index_dof + j] = vmin;
      }
      if(q[jnt->index_dof + j] > vmax){
        cout << "WARNING : Configuration correction : Joint " << jnt->name << " has value = " << q[jnt->index_dof + j] << " and its maximal bound = " << vmax << endl;
        q[jnt->index_dof + j] = vmax;
      }
    }
  }
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::computeManipulationData(int armId,p3d_rob* object, gpGrasp& grasp){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  // Generate new manip configs
  ManipulationData manipConfigs(_robot);

  _configs = manipConfigs;
  status = findArmGraspsConfigs(armId,object, grasp, _configs);

  ManipulationUtils::printManipulationMessage(status);
  return status;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::findArmGraspsConfigs(int armId, p3d_rob* object, gpGrasp& grasp, ManipulationData& configs){
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;

  if (grasp.ID != 0){//a valid grasp is given
    ManipulationData data(_robot);
    p3d_matrix4 tAtt;
    ManipulationUtils::fixAllHands(_robot, NULL, true);
    status = getGraspOpenApproachExtractConfs(object, armId, grasp, tAtt, data);
    if(status == MANIPULATION_TASK_OK){
      configs = data;
      //             break;
      if(MPDEBUG){
          ManipulationUtils::copyConfigToFORM(_robot, data.getGraspConfig());
      }
    }else{
      data.clear();
      status = MANIPULATION_TASK_NO_GRASP;
    }
  }else{

    if (armId == -1) {
      //TODO Multi arm Grasping
    } else {
      if (armId == -2) {//Compute closest arm
        if ((armId = getClosestWristToTheObject(_robot, object)) == -2) {
          printf("ERROR findArmGraspsConfigs on getClosestWristToTheObject");
          return MANIPULATION_TASK_NO_GRASP;
        }
      }
      gpHand_properties armHandProp = (*_robot->armManipulationData)[armId].getHandProperties();
      list<gpGrasp> graspList;

      graspList.clear();
      //Compute the grasp list for the given hand and object
      gpGet_grasp_list(object->name, armHandProp.type, graspList);
      status = MANIPULATION_TASK_NO_GRASP;

      if (graspList.size() != 0)  {
        int counter = 0;
        bool validConf = false;

        for (list<gpGrasp>::iterator iter = graspList.begin(); iter != graspList.end(); iter++)
        {
          ManipulationData data(_robot);
          p3d_matrix4 tAtt;
          ManipulationUtils::fixAllHands(_robot, NULL, true);
          status = getGraspOpenApproachExtractConfs(object, armId, (*iter), tAtt, data);
          //cout << "status = " << _ErrorMap[status] << endl;
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
          }else{
            data.clear();
          }

          counter++;
        }
        if (MPDEBUG) {
          printf("NbTest Before Config : %d\n", counter);
        }
        if (validConf) {
          status = MANIPULATION_TASK_OK;
        }
      }
    }
  }

  if (MPDEBUG && (status == MANIPULATION_TASK_OK) ) {
    showConfig_2(configs.getOpenConfig());
    showConfig_2(configs.getGraspConfig());
    showConfig_2(configs.getApproachGraspConfig());
    showConfig_2(configs.getApproachFreeConfig());
    printf("MinConfig Cost = %f\n", configs.getGraspConfigCost());

    ManipulationUtils::copyConfigToFORM(_robot, configs.getGraspConfig());
    ManipulationUtils::copyConfigToFORM(_robot, configs.getOpenConfig());
    ManipulationUtils::copyConfigToFORM(_robot, configs.getApproachGraspConfig());
    ManipulationUtils::copyConfigToFORM(_robot, configs.getApproachFreeConfig());
  }
  (*_robot->armManipulationData)[armId].setCarriedObject((p3d_rob*)NULL);
  return status;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::getGraspOpenApproachExtractConfs(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt,  ManipulationData& configs) const {

    const bool debug_configs = false;
    double confCost = -1;


    if( debug_configs )
    {
      cout << "----- **************** -----" << endl;
      cout << "-----  getGraspConf()  -----" << endl;
    }
    configPt q = NULL;
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    gpHand_properties handProp = mData.getHandProperties();
    p3d_matrix4 handFrame;
    p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
    q = _manipConf.getGraspConf(object, armId, grasp, tAtt, confCost);

    if (q){
      if( debug_configs/* || MPDEBUG*/ ) {
        cout << "FOUND Grasp Config!!!!!" << endl;
        cout << confCost << endl;
        ManipulationUtils::copyConfigToFORM(_robot, q);
        //showConfig_2(q);
      }

      configs.setGraspConfig(q);
      configs.setGraspConfigCost(confCost);
      configs.setGrasp(&grasp);
      configs.setAttachFrame(tAtt);

      p3d_destroy_config(_robot, q);
      q = NULL;

      q = _manipConf.getOpenGraspConf(object, armId, grasp, configs.getGraspConfig());

      if (q) {
        if( debug_configs || MPDEBUG ) {
          cout << "FOUND Open Config!!!!!" << endl;

        }
        configs.setOpenConfig(q);
        p3d_destroy_config(_robot, q);
        q = NULL;

        if ( debug_configs )
        {
          cout << "-----------------------------------------------------" << endl;
          cout << "-----------------------------------------------------" << endl;
          cout << " getApproachFreeConf() " << endl;
        }
        setSafetyDistance(_robot, getSafetyDistanceValue());
        q = _manipConf.getApproachFreeConf(object, armId, grasp, configs.getGraspConfig(), tAtt);
        setSafetyDistance(_robot, 0);
        if (q) {
          if( debug_configs || MPDEBUG ){
            cout << "FOUND Approach Free Config!!!!!" << endl;
          }
          configs.setApproachFreeConfig( configs.getOpenConfig() );
          configs.setApproachFreeConfig(q);
          p3d_destroy_config(_robot, q);
          q = NULL;
          setSafetyDistance(_robot, getSafetyDistanceValue());
          q = _manipConf.getApproachGraspConf(object, armId, grasp, configs.getGraspConfig(), tAtt);
          setSafetyDistance(_robot, 0);
          if (q) {
            if( debug_configs || MPDEBUG ){
              cout << "FOUND Approach Grasp Config!!!!!" << endl;
            }
            configs.setApproachGraspConfig(q);
            p3d_destroy_config(_robot, q);
            q = NULL;

            deactivateCcCntrts(_robot, armId);
            if( debug_configs || MPDEBUG ){
              cout << "SUCCES ALL CONFIG FOUND" << endl;
            }
            return MANIPULATION_TASK_OK; //success
          }
        }
      }
    }
    deactivateCcCntrts(_robot, armId);
    return MANIPULATION_TASK_NO_GRASP;
}

/* ******************************* */
/* ******* Motion Planning Modes * */
/* ******************************* */
void ManipulationPlanner::checkConfigForCartesianMode(configPt q, p3d_rob* object) {
    bool deleteConfig = false;
    if (q == NULL) {
        q = p3d_get_robot_config(_robot);
        deleteConfig = true;
    }
    for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
        ArmManipulationData& armData  = (*_robot->armManipulationData)[i];
        desactivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
        if (armData.getCartesian()) {
          /* Uptdate the Virual object for inverse kinematics */
          p3d_update_virtual_object_config_for_arm_ik_constraint(_robot, i, q);
          activateCcCntrts(_robot, i, false);
          ManipulationUtils::unfixManipulationJoints(_robot, i);
          armData.getManipulationJnt()->dist = _robot->joints[_robot->mlp->mlpJoints[armData.getHandGroup()]->joints[0]]->dist;
        } else {
          deactivateCcCntrts(_robot, i);
          if(object){
            ManipulationUtils::fixManipulationJoints(_robot, i, q, object);
          }else{
            p3d_update_virtual_object_config_for_arm_ik_constraint(_robot, i, q);
            p3d_set_and_update_this_robot_conf(_robot, q);
            setAndActivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
          }
        }
    }
    p3d_set_and_update_this_robot_conf(_robot, q);
    p3d_get_robot_config_into(_robot, &q);
    if (deleteConfig) {
        p3d_destroy_config(_robot, q);
        q = NULL;
    }
}

void ManipulationPlanner::setArmCartesian(int armId, bool cartesian) {
    ArmManipulationData& armData  = (*_robot->armManipulationData)[armId];

    armData.setCartesian(cartesian);
    //activate the corresponding group, constraint and set the sampling bounds of the object to sample
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

  p3d_traj* traj = _plannerMethod(_robot,qs,qg);

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
    p3d_multiLocalPath_disable_all_groupToPlan(_robot);
    p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);
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
//        print_config(_robot, qi);
//        print_config(_robot, qf);
//        showConfig_2(qi);
//        showConfig_2(qf);
//        cout << "ManipulationPlanner::computeTrajBetweenTwoConfigs::p3d_equal_config(_robot, qi, qf)" << endl;
      *status = MANIPULATION_TASK_EQUAL_QSTART_QGOAL;
      ManipulationUtils::printManipulationMessage(*status);
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

/** Concatenes all the current trajectories of the robot into the first one.
 * NB: only the first trajectory will remain (and grown up); the others are destroyed.
 * \param trajs A vector of trajectories
 * \param concatTraj The concatenated trajectory to return
 * \return Message of success or fail */
MANIPULATION_TASK_MESSAGE ManipulationPlanner::concatTrajectories (std::vector<p3d_traj*>& trajs, p3d_traj** concatTraj) {
    if ( _robot==NULL ) {
        PrintInfo ( ( "concateneAllTrajectories: robot is NULL.\n" ) );
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }
    if ( trajs.empty() || !trajs[0]) {
        PrintInfo ( ( "concateneAllTrajectories: the trajectory vector is empty.\n" ) );
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }
    *concatTraj = p3d_create_traj_by_copy(trajs[0]);
    for (int i = 1; i < (int)trajs.size(); i++) {
        p3d_concat_traj(*concatTraj, trajs[i]);
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
    if (p3d_convert_traj_to_softMotion(traj, true, approximate,confs.first, confs.second, smTraj) == 1) {
        printf("p3d_optim_traj_softMotion : cannot compute the softMotion trajectory\n");
        return MANIPULATION_TASK_ERROR_UNKNOWN;
    }
    return 0;
}
#endif

/* ******************************* */
/* ******* Manipulation Planning * */
/* ******************************* */
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
  configPt qGoal = _manipConf.getFreeHoldingConf(object, armId, grasp, (*_robot->armManipulationData)[armId].getCcCntrt()->Tatt, confCost, objGoto, NULL);
  if(qGoal){
    status = armToFree(armId, qStart, qGoal, true, object ,trajs);
  }else{
    status = MANIPULATION_TASK_NO_TRAJ_FOUND;
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
  fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

  checkConfigForCartesianMode(qStart, NULL);
  checkConfigForCartesianMode(qGoal, NULL);
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
  
  if (status == MANIPULATION_TASK_OK)
  {
    //Compute the path between theses configurations
    status = armPickGoto(armId, qStart, object, 
                         _configs.getGraspConfig(), 
                         _configs.getOpenConfig(), 
                         _configs.getApproachFreeConfig(), trajs);
  }
  else {
      cout << "Fail to findArmGraspsConfigs(armId,object,_configs)" << endl;
  }

  return status;
}

//! Same as above but with more parameters
//! @param configPt graspConfig : the configuration to grasp the object
//! @param configPt openConfig : the configuration with the open hand
//! @param configPt approachFreeConfig : an aproaching configuration
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs) {

    p3d_traj* traj = NULL;
    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    ManipulationUtils::fixAllHands(_robot, qStart, false);
    fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

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
    if ((traj = computeTrajBetweenTwoConfigs(qStart, approachFreeConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
    {
      trajs.push_back(traj);
      setSafetyDistance(_robot, 0);
      // Compute to Open config
      if ((traj = computeTrajBetweenTwoConfigs(approachFreeConfig, openConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
      {
        trajs.push_back(traj);

        // Set the hand as group to plan
        (*_robot->armManipulationData)[armId].unFixHand(_robot);

        p3d_multiLocalPath_disable_all_groupToPlan(_robot);
        p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);
        // Compute to Grasp Config
        gpHand_properties handProp = (*_robot->armManipulationData)[armId].getHandProperties();
        gpDeactivate_object_fingertips_collisions(_robot, object->joints[1]->o, handProp, armId);
        if ((traj = computeTrajBetweenTwoConfigs(openConfig, graspConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL))
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
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFreePoint(int armId, configPt qStart, std::vector<double> &objGoto , p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs){
  
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  int updateTatt = false;
  
  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);
  
  p3d_matrix4 tAtt;
  _configs.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    updateTatt = true;
  }
  if(!_configs.getGrasp()){
    _configs.setGrasp(new gpGrasp());
  }
  activateCcCntrts(_robot, armId, false);
  double confCost = -1;
  
  configPt qGoal = _manipConf.getFreeHoldingConf(object, armId, 
                                        *_configs.getGrasp(), 
                                        tAtt, confCost,
                                        objGoto, NULL);
  if(updateTatt){
    _configs.setAttachFrame(tAtt);
  }
  if (qGoal) {
    if(!_configs.getApproachGraspConfig()){
      configPt approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_configs.getGrasp()), qStart, tAtt);
      if(approachGraspConfig){
        _configs.setApproachGraspConfig(approachGraspConfig);
      }else{
         status = MANIPULATION_TASK_NO_GRASP;
      }
    }
  } else {
    status = MANIPULATION_TASK_NO_GRASP;
  }
  
  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armPickTakeToFree(armId, qStart, qGoal, object, support, _configs.getApproachGraspConfig(), *_configs.getGrasp(), trajs);
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
 MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* support, std::vector <p3d_traj*> &trajs) {

    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;

  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);

  p3d_matrix4 tAtt, bakTatt;
  bool updateTatt;
  ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
  _configs.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    //NULL frame compute it.
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, bakTatt);
    p3d_compute_Tatt(mData.getCcCntrt());
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, tAtt);
    _configs.setAttachFrame(tAtt);
    updateTatt = true;
  }

  if(!_configs.getGrasp()){
    gpGrasp* grasp = new gpGrasp();
    gpSet_grasp_configuration(_robot, *grasp, qStart, armId);
    _configs.setGrasp(grasp);
  }

  activateCcCntrts(_robot, armId, false);

  if(!_configs.getApproachGraspConfig()){
    configPt approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_configs.getGrasp()), qStart, tAtt);
    if(approachGraspConfig){
      _configs.setApproachGraspConfig(approachGraspConfig);
    }else{
        status = MANIPULATION_TASK_NO_GRASP;
    }
  }

  if (status == MANIPULATION_TASK_OK) {
      //Compute the path between theses configurations
      status = armPickTakeToFree(armId, qStart, qGoal, object, support,
                                  _configs.getApproachGraspConfig(),
                                  *_configs.getGrasp(),
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
  fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

  deactivateCcCntrts(_robot,armId);

  p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
  p3d_matrix4 tAtt;
  _configs.getAttachFrame(tAtt);
  if(tAtt[0][0] != 0 || tAtt[0][1] != 0 || tAtt[0][2] != 0 || tAtt[0][3] != 0){
      p3d_mat4Copy(tAtt, armData.getCcCntrt()->Tatt);
  }

  checkConfigForCartesianMode(qStart, NULL);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, qStart);
  }
  checkConfigForCartesianMode(approachGraspConfig, NULL);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfig);
  }
  checkConfigForCartesianMode(qGoal, NULL);
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
  desactivateTwoJointsFixCntrt(_robot, armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
  
  p3d_set_and_update_this_robot_conf(_robot, qStart);

  p3d_matrix4 tAtt;
  _configs.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    updateTatt = true;
  }
  if(!_configs.getGrasp()){
    _configs.setGrasp(new gpGrasp());
  }
  activateCcCntrts(_robot, armId, false);
  double confCost = -1;
  qGoal = _manipConf.getFreeHoldingConf(object, armId, *_configs.getGrasp(), tAtt, confCost, objGoto, placement);
  if(updateTatt){
    _configs.setAttachFrame(tAtt);
  }
  if (qGoal) {
    //Compute the approch grasp config for the placement grasp
    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    p3d_set_and_update_this_robot_conf(_robot, qGoal);
    configPt approachGraspConfigPlacement = _manipConf.getApproachGraspConf(object, armId, *(_configs.getGrasp()), qGoal, tAtt);
    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    p3d_set_and_update_this_robot_conf(_robot, qStart);
    armData.setCarriedObject((p3d_rob*) NULL);
    _robot->isCarryingObject = FALSE;
    if(approachGraspConfigPlacement){
      //Compute the approch grasp config for the start config.
      if(!_configs.getApproachGraspConfig()){
        p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
        p3d_set_and_update_this_robot_conf(_robot, qStart);
        approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_configs.getGrasp()), qStart, tAtt);
        p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
        p3d_set_and_update_this_robot_conf(_robot, qStart);
        armData.setCarriedObject((p3d_rob*) NULL);
        _robot->isCarryingObject = FALSE;
        if(!approachGraspConfig){
          status = MANIPULATION_TASK_NO_GRASP;
        }
      }else{
        approachGraspConfig = p3d_copy_config(_robot, _configs.getApproachGraspConfig());
      }
      _configs.setApproachGraspConfig(approachGraspConfigPlacement);
      _configs.setGraspConfig(qGoal);
    }else{
      status = MANIPULATION_TASK_NO_PLACE;
    }
  } else {
    status = MANIPULATION_TASK_NO_PLACE;
  }

  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armTakeToPlace(armId, qStart, approachGraspConfig, _configs.getApproachGraspConfig(), qGoal, object, support, placement, trajs);
  }
  else {
      cout << "Fail to findArmGraspsConfigs(armId,object,_configs)" << endl;
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
  fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

  deactivateCcCntrts(_robot,armId);

  p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
  p3d_matrix4 tAtt;
  _configs.getAttachFrame(tAtt);
  if(tAtt[0][0] != 0 || tAtt[0][1] != 0 || tAtt[0][2] != 0 || tAtt[0][3] != 0){
      p3d_mat4Copy(tAtt, armData.getCcCntrt()->Tatt);
  }

  checkConfigForCartesianMode(qStart, NULL);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, qStart);
  }
  checkConfigForCartesianMode(approachGraspConfig, NULL);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfig);
  }
  checkConfigForCartesianMode(approachGraspConfigPlacement, NULL);
  if(MPDEBUG){
      ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfigPlacement);
  }
  checkConfigForCartesianMode(qGoal, NULL);
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
    print_config(_robot, approachGraspConfigPlacement);
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

  deactivateCcCntrts(_robot, armId);
  p3d_set_and_update_this_robot_conf(_robot, qStart);

  p3d_matrix4 tAtt;
  _configs.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    updateTatt = true;
  }
  if(!_configs.getGrasp()){
    _configs.setGrasp(new gpGrasp());
  }
  activateCcCntrts(_robot, armId, false);
  double confCost = -1;
  configPt qGoal = _manipConf.getFreeHoldingConf(object, armId, *_configs.getGrasp(), tAtt, confCost, objGoto, placement);
  
  if(updateTatt){
    _configs.setAttachFrame(tAtt);
  }
  if (qGoal) {
    (*_robot->armManipulationData)[armId].setCarriedObject(object);
    _robot->isCarryingObject = TRUE;
    p3d_set_and_update_this_robot_conf(_robot, qGoal);
    configPt approachGraspConfig = _manipConf.getApproachGraspConf(object, armId, *(_configs.getGrasp()), qGoal, tAtt);
    (*_robot->armManipulationData)[armId].setCarriedObject(object);
    _robot->isCarryingObject = TRUE;
    p3d_set_and_update_this_robot_conf(_robot, qStart);
    (*_robot->armManipulationData)[armId].setCarriedObject((p3d_rob*) NULL);
    _robot->isCarryingObject = FALSE;
    if(approachGraspConfig){
      _configs.setApproachGraspConfig(approachGraspConfig);
      _configs.setGraspConfig(qGoal);
    }else{
      status = MANIPULATION_TASK_NO_PLACE;
    }
  } else {
    status = MANIPULATION_TASK_NO_PLACE;
  }

  if (status == MANIPULATION_TASK_OK)
  {
    //Compute the path between theses configurations
    status = armPlaceFromFree(armId, qStart, object, placement, _configs.getApproachGraspConfig(), qGoal,trajs);
  }
  else {
      cout << "Fail to findArmGraspsConfigs(armId,object,_configs)" << endl;
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
    _configs.getAttachFrame(tAtt);
    if(tAtt[0][0] != 0 || tAtt[0][1] != 0 || tAtt[0][2] != 0 || tAtt[0][3] != 0){
       p3d_mat4Copy(tAtt, armData.getCcCntrt()->Tatt);
    }
    
    checkConfigForCartesianMode(qStart, NULL);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, qStart);
    }
    checkConfigForCartesianMode(approachGraspConfig, NULL);
    if(MPDEBUG){
       ManipulationUtils::copyConfigToFORM(_robot, approachGraspConfig);
    }
    checkConfigForCartesianMode(depositConfig, NULL);
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
  
  deactivateCcCntrts(_robot, armId);
  desactivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(), armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
  
  p3d_set_and_update_this_robot_conf(_robot, qStart);

  p3d_matrix4 tAtt;
  _configs.getAttachFrame(tAtt);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    p3d_compute_Tatt(armData.getCcCntrt());
    _configs.setAttachFrame(armData.getCcCntrt()->Tatt);
    _configs.getAttachFrame(tAtt);
  }
  if(!_configs.getGrasp()){
    gpGrasp grasp;
    grasp.hand_type = armData.getHandProperties().type;
    grasp.config.resize(armData.getHandProperties().nb_dofs);
    grasp.openConfig.resize(armData.getHandProperties().nb_dofs);
    gpSet_grasp_configuration(_robot, grasp, qStart, armId);
    _configs.setGrasp(&grasp);
  }
//   activateCcCntrts(_robot, armId, false);
  
  configPt openGraspConfig = _manipConf.getOpenGraspConf(object, armId, *_configs.getGrasp(), qStart);
  if (openGraspConfig) {
    _configs.setOpenConfig(openGraspConfig);
  }else{
      status = MANIPULATION_TASK_NO_GRASP;
  }

  if(status == MANIPULATION_TASK_OK){
    configPt approachFreeConfig = _manipConf.getApproachFreeConf(object, armId, *(_configs.getGrasp()), qStart, tAtt);
    if(approachFreeConfig){
      _configs.setApproachFreeConfig(approachFreeConfig);
    }else{
        status = MANIPULATION_TASK_NO_GRASP;
    }
  }
  deactivateCcCntrts(_robot, armId);
  
  if (status == MANIPULATION_TASK_OK){
    //Compute the path between theses configurations
    status = armEscapeObject(armId, qStart, _configs.getOpenConfig(), _configs.getApproachFreeConfig(), object, trajs);
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

  if (MPDEBUG) {
      ManipulationUtils::copyConfigToFORM(_robot, qStart);
      ManipulationUtils::copyConfigToFORM(_robot, openGraspConfig);
      ManipulationUtils::copyConfigToFORM(_robot, approachFreeConfig);
  }
  setSafetyDistance(_robot, getSafetyDistanceValue());
  ManipulationUtils::unFixAllHands(_robot);
  if ((traj = computeTrajBetweenTwoConfigs(qStart, openGraspConfig, &status)) || (status == MANIPULATION_TASK_EQUAL_QSTART_QGOAL)){
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

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, char* placementName, gpGrasp& grasp, std::vector <p3d_traj*> &trajs) {
  
  if (!_robot) {
    printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
    return MANIPULATION_TASK_NOT_INITIALIZED;
  }
  
  configPt qi = p3d_copy_config(_robot, qStart), qf = p3d_copy_config(_robot, qGoal);
  p3d_rob* cur_robot = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
  p3d_rob* object = p3d_get_robot_by_name(objectName);
  if(!object && task != ARM_FREE && task != ARM_EXTRACT ){
    return MANIPULATION_TASK_UNKNOWN_OBJECT;
  }
  p3d_rob* support = p3d_get_robot_by_name(supportName);
  p3d_rob* placement = p3d_get_robot_by_name(placementName);
  p3d_sel_desc_id(P3D_ROBOT, _robot);
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  if (armId < 0 || armId >= (int)_robot->armManipulationData->size()) {
    status = MANIPULATION_TASK_INVALID_TASK;
  } else {
    ENV.setBool(Env::drawTraj, false);
    checkConfigForCartesianMode(qi, object);
    checkConfigForCartesianMode(qf, object);
//     showConfig_2(qi);
    ManipulationUtils::fixAllHands(_robot, qi, false);
    ManipulationUtils::fixAllHands(_robot, qf, false);
    p3d_set_and_update_this_robot_conf(_robot, qi);
    //Remove collision tolerence for the object
    if(object){
      p3d_set_collision_tolerance_inhibition(object, TRUE);
    }
    //Clear the graph
    cleanRoadmap();
    
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
          status = armPickTakeToFreePoint(armId, qi, objGoto, object, support, trajs);
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

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, char* placementName, std::vector <p3d_traj*> &trajs){
  gpGrasp grasp;
  return armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, placementName, trajs);
}

//! Computes a path for a specific task
//! This function is an interface for the softmotion
//! trajectory generation
#ifdef MULTILOCALPATH
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, char* placementName, gpGrasp& grasp, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs) {
    std::vector <p3d_traj*> trajs;
    p3d_traj* traj = NULL;
    MANIPULATION_TASK_MESSAGE returnMessage;

    if (!_robot) {
        printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }

    p3d_multiLocalPath_disable_all_groupToPlan(_robot);
    p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);

    if ((returnMessage = armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, placementName, grasp, trajs)) == MANIPULATION_TASK_OK) {
      //concatene
      if (concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
        smTrajs.clear();
//         for(unsigned i = 0; i < trajs.size(); i++){
        /* COMPUTE THE SOFTMOTION TRAJECTORY */
          MANPIPULATION_TRAJECTORY_CONF_STR conf;
          SM_TRAJ smTraj;
          computeSoftMotion(traj/*s.at(i)*/, conf, smTraj);
          confs.push_back(conf);
          smTrajs.push_back(smTraj);
//         }
      } else {
        returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
      }
    }
    return returnMessage;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, char* placementName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs) {
  gpGrasp grasp;
  return armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, placementName, grasp, confs, smTrajs);
}

#endif
