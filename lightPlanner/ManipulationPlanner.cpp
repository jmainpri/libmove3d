#include "ManipulationPlanner.hpp"

#include "lightPlanner.h"
#include "lightPlannerApi.h"

// WARNING : moky file not found
#ifdef DPG
#include "p3d_chanEnv_proto.h"
#endif

#ifdef CXX_PLANNER
#include "planner_cxx/plannerFunctions.hpp"
#if defined (USE_CXX_PLANNER)
// #include "planEnvironment.hpp"
#endif
#endif

#include "robotPos.h"

#include "Move3d-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"

#include <list>
#include <algorithm>

static bool MPDEBUG=true;

using namespace std;

/* ******************************* */
/* ******* (De)Constructor ******* */
/* ******************************* */
ManipulationPlanner::ManipulationPlanner(p3d_rob *robotPt) :
        _robot(robotPt),
        _configs(robotPt)
{
    _optimizeSteps = 100;
    _optimizeTime = 4.0; // 4 secondes
    _approachFreeOffset = 0.1; //0.1 meters
    _approachGraspOffset = 0.05; //0.1 meters


    // Warning : Not very efficient
    // Should only allocate this once
    _ErrorMap[MANIPULATION_TASK_OK] = "everything was fine";
    _ErrorMap[MANIPULATION_TASK_NOT_INITIALIZED] = "a variable was not properly initialized";
    _ErrorMap[MANIPULATION_TASK_NO_TRAJ_FOUND] = "failed to compute a trajectory for the desired task";
    _ErrorMap[MANIPULATION_TASK_INVALID_QSTART] = "";
    _ErrorMap[MANIPULATION_TASK_INVALID_QGOAL] = "";
    _ErrorMap[MANIPULATION_TASK_INVALID_TRAJ_ID] = "";
    _ErrorMap[MANIPULATION_TASK_INVALID_TASK] = "the desired task is undefined";
    _ErrorMap[MANIPULATION_TASK_UNKNOWN_OBJECT] = "there is no object with the specified name";
    _ErrorMap[MANIPULATION_TASK_NO_GRASP] = "no grasp or arm grasping configuration can be found for the current context (mobile base and obstacles configurations)";
    _ErrorMap[MANIPULATION_TASK_NO_PLACE] = "no placement or arm placement configuration can be found for the current context (mobile base and obstacles configurations, current grasp)";
    _ErrorMap[MANIPULATION_TASK_ERROR_UNKNOWN] = "something undefined was wrong";

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
void ManipulationPlanner::setDebugMode(bool value)
{
  MPDEBUG = value;
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
#ifdef XFORMS
#endif
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

void ManipulationPlanner::setApproachFreeOffset(double offset) {
    _approachFreeOffset = offset;
}
double ManipulationPlanner::getApproachFreeOffset(void) const {
    return _approachFreeOffset;
}

void ManipulationPlanner::setApproachGraspOffset(double offset) {
    _approachGraspOffset = offset;
}
double ManipulationPlanner::getApproachGraspOffset(void) const {
    return _approachGraspOffset;
}

/* ******************************* */
/* ******* Hands / Grasping ****** */
/* ******************************* */

void ArmManipulationData::fixHand(p3d_rob* robot, bool rest) {
    if (rest) {
        gpSet_hand_rest_configuration(robot, _handProp, this->getId());
    }
    gpFix_hand_configuration(robot, _handProp, this->getId());
    gpDeactivate_hand_selfcollisions(robot, this->getId());
}

void ArmManipulationData::unFixHand(p3d_rob* robot) {
    gpUnFix_hand_configuration(robot, _handProp, this->getId());
    gpActivate_hand_selfcollisions(robot, this->getId());
}

void ManipulationPlanner::fixAllHands(configPt q, bool rest) const {
    if (q != NULL) {
        p3d_set_and_update_this_robot_conf(_robot, q);
    }
    for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
        (*_robot->armManipulationData)[i].fixHand(_robot, rest);
    }
    if (q != NULL) {
        p3d_get_robot_config_into(_robot, &q);
    }
}

void ManipulationPlanner::unFixAllHands(void) {
    for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
        (*_robot->armManipulationData)[i].unFixHand(_robot);
    }
}

void ManipulationPlanner::unfixManipulationJoints(int armId) {
    for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
        unFixJoint(_robot, (*_robot->armManipulationData)[i].getManipulationJnt());
    }
}

void ManipulationPlanner::fixManipulationJoints(int armId, configPt q, p3d_rob* object) {
    p3d_matrix4 pos;
    if (object) {
        p3d_mat4Copy(object->joints[1]->abs_pos, pos);
    } else {
        p3d_mat4Copy(p3d_mat4IDENTITY, pos);
    }
    if (q) {
        p3d_set_and_update_this_robot_conf(_robot, q);
    }
    for (uint i = 0; i < (*_robot->armManipulationData).size(); i++) {
        if(!(*_robot->armManipulationData)[i].getCartesian()){
          fixJoint(_robot, (*_robot->armManipulationData)[i].getManipulationJnt(), pos);
        }
    }
    if (q) {
        p3d_get_robot_config_into(_robot, &q);
    }
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::findArmGraspsConfigs(int armId, p3d_rob* object, ManipulationData& configs) {

  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  
  (*_robot->armManipulationData)[armId].setCarriedObject(object);
  
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
    list<gpGrasp> graspList/*, tmp*/;
    
    graspList.clear();
    //Compute the grasp list for the given hand and object
    gpGet_grasp_list(object->name, armHandProp.type, graspList);
    
    if (graspList.size() == 0) {
      status = MANIPULATION_TASK_NO_GRASP;
      //cout << "ManipulationPlanner::graspList.size() == 0" << endl;
    } else {
      status = MANIPULATION_TASK_NO_GRASP;
      //cout << "ManipulationPlanner::computingConfig()" << endl;
      int counter = 0;
      bool validConf = false;
      for (list<gpGrasp>::iterator iter = graspList.begin(); iter != graspList.end(); iter++) {
        ManipulationData data(_robot);
        p3d_matrix4 tAtt;
        fixAllHands(NULL, true);
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
        }
        //         }
        
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

configPt ManipulationPlanner::getFreeHoldingConf( p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, std::vector<double> &objGoto ) const {

    configPt tmpConf = p3d_get_robot_config(_robot);
  
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    configPt q = p3d_get_robot_config(_robot);

    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
  
    // Set Manipulation joint and hand configuration
    p3d_matrix4 mat;
    p3d_mat4Copy( object->joints[1]->abs_pos , mat );

    mat[0][3] = objGoto[0];
    mat[1][3] = objGoto[1];
    mat[2][3] = objGoto[2];

//    int idManipIndexDof = (*_robot->armManipulationData)[armId].getManipulationJnt()->index_dof;
//
//    q[ idManipIndexDof + 0 ] = objGoto[0];
//    q[ idManipIndexDof + 1 ] = objGoto[1];
//    q[ idManipIndexDof + 2 ] = objGoto[2];
//    p3d_set_and_update_this_robot_conf(_robot, q);

    gpSet_grasp_configuration(_robot, grasp, q, armId);

    // Sample a configuration for the robot
    q = setRobotGraspPosWithoutBase(_robot, mat , tAtt, false , armId, true);
    deactivateCcCntrts(_robot, armId);

    // Reset robot to the initial robot configuration
    p3d_set_and_update_this_robot_conf(_robot, tmpConf);
    p3d_destroy_config(_robot, tmpConf);

    _robot->isCarryingObject = FALSE;
    
    if(q)
    {
      fixAllHands(NULL, true);
      return q;
    }
    else {
      cout << "No config found by setRobotGraspPosWithoutBase" << endl;
    }

  
  return NULL;
}


configPt ManipulationPlanner::getGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double* confCost) const {

    p3d_matrix4 handFrame;
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    gpHand_properties handProp = mData.getHandProperties();
    p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
    p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);

    // Check if a valid configuration exists
    // of the robot using this graspFrame
    configPt q = NULL;
    gpSet_grasp_configuration(_robot, grasp, armId);
    gpFix_hand_configuration(_robot, handProp, armId);

    // Compute Grasp configuration
    q = setRobotGraspPosWithoutBase(_robot, object->joints[1]->abs_pos, tAtt, false , armId, true);

    if (q) 
    {
        // If it exists, try to find better Rest Arm config and
        double restArmCost = setRobotArmsRest(_robot, object->joints[1]->abs_pos, armId, tAtt, _robot->openChainConf, q);
        double graspArmCost = computeRobotGraspArmCost(_robot, armId, grasp, q, _robot->openChainConf, object->joints[1]->abs_pos)/270;
        *confCost = (restArmCost + graspArmCost * 2) / 3;

        //printf("configuration Cost = %f\n", *confCost);

        deactivateCcCntrts(_robot, armId);
        gpSet_grasp_configuration(_robot, grasp, q, armId);
        return q;
    }
    *confCost = -1;
    return NULL;
}

configPt ManipulationPlanner::getOpenGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf) const 
{
    if (graspConf) 
    {
        ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
        configPt q = p3d_copy_config(_robot, graspConf);

        //Check the open configuration of the hand
        gpSet_grasp_open_configuration(_robot, grasp, q, armId);
      
        if (!p3d_is_collision_free(_robot,q)) 
        {
            // if the grasp open config is colliding, recompute it taking into account the environment:
            grasp.computeOpenConfig(_robot, object, true);
            gpSet_grasp_open_configuration(_robot, grasp, q, armId);
            if (p3d_is_collision_free(_robot,q)) {
                return q;
            }
        } else {
            return q;
        }
    }
    return NULL;
}

double ManipulationPlanner::distConfig( configPt q1, configPt q2, int group ) const
{
  double ljnt = 0.;
  int nbJoints = _robot->mlp->mlpJoints[group]->nbJoints;
  
  for (int i = 0; i < nbJoints; i++) 
  {
    int k = _robot->mlp->mlpJoints[group]->joints[i];
    p3d_jnt* jntPt = _robot->joints[k];
    
    for (int j = 0; j < jntPt->dof_equiv_nbr; j++) 
    {
      double dist = SQR(p3d_jnt_calc_dof_dist(jntPt, j, q1, q2));
      //printf(" dist[%d] = %f\n",jntPt->index_dof + j,dist);
      ljnt += dist;
    }
  }
  
  double l = sqrt(ljnt);
  
  return l;
}

configPt ManipulationPlanner::getApproachFreeConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const 
{
  if (graspConf) {
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    configPt q = p3d_copy_config(_robot, graspConf);
    gpHand_properties handProp = mData.getHandProperties();
    
    p3d_matrix4 objTmp;
    p3d_vector3 tAttY;
    p3d_mat4Copy(object->joints[1]->abs_pos, objTmp);
    if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_kuka_arm_ik")) {
      p3d_mat4ExtractColumnY(tAtt, tAttY);
    } else if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_lwr_arm_ik")) {
      p3d_mat4ExtractColumnZ(tAtt, tAttY);
    }
    objTmp[0][3] -= getApproachFreeOffset() * tAttY[0];
    objTmp[1][3] -= getApproachFreeOffset() * tAttY[1];
    objTmp[2][3] -= getApproachFreeOffset() * tAttY[2];
    //         q[mData.getManipulationJnt()->index_dof + 0] -= getApproachFreeOffset() * tAttY[0];
    //         q[mData.getManipulationJnt()->index_dof + 1] -= getApproachFreeOffset() * tAttY[1];
    //         q[mData.getManipulationJnt()->index_dof + 2] -= getApproachFreeOffset() * tAttY[2];
    gpUnFix_hand_configuration(_robot, handProp, armId);
    gpSet_grasp_open_configuration(_robot, grasp, q, armId);
    gpFix_hand_configuration(_robot, handProp, armId);
    
    
    std::pair<double,configPt> distToGraspQ;
    std::vector< std::pair<double,configPt> > allQ;
    
    for (unsigned int i=0; i<1000; i++) 
    {
      configPt qApproachFree = setRobotCloseToConfGraspApproachOrExtract(_robot, q, objTmp, tAtt, false, armId, true);
      
      if ( qApproachFree ) 
      {
        distToGraspQ.first  = distConfig( qApproachFree , graspConf , _UpBodyMLP );
        distToGraspQ.second = p3d_copy_config( _robot, qApproachFree );
        //std::cout << "Configuration cost : " << distToGraspQ.first << std::endl;
        allQ.push_back( distToGraspQ );
      }
      else {
        break;
      }
    }
    
    if (!allQ.empty()) 
    {
      std::sort(allQ.begin(),allQ.end());
      //std::cout << "Configuration cost : " << allQ[0].first << std::endl;
      //std::cout << "--------------------------------------" << std::endl;
      q = allQ[0].second;
      
      for (unsigned int i=1; i<allQ.size(); i++) {
        p3d_destroy_config( _robot, allQ[i].second );
      }
    }
    
    deactivateCcCntrts(_robot, armId);
    if (q) {
      fixAllHands(NULL, true);
      return q;
    }
  }
  return NULL;
}

configPt ManipulationPlanner::getApproachGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const{
  
  //  configPt tmpConf = p3d_get_robot_config(_robot);
  
  if(graspConf){
    ArmManipulationData mData = (*_robot->armManipulationData)[armId];
    configPt q = p3d_copy_config(_robot, graspConf);
    
    _robot->isCarryingObject = TRUE;
    
    // Set Manipulation joint and hand configuration
    q[(*_robot->armManipulationData)[armId].getManipulationJnt()->index_dof + 2] += getApproachGraspOffset(); //Z axis of the manipulation joint
    p3d_set_and_update_this_robot_conf(_robot, q);
    gpSet_grasp_configuration(_robot, grasp, q, armId);
    
    // Sample a configuration for the robot
    q = setRobotCloseToConfGraspApproachOrExtract(_robot, q, object->joints[1]->abs_pos, tAtt, false, armId, true);
    deactivateCcCntrts(_robot, armId);
    
    // Reset robot to the initial robot configuration
    p3d_set_and_update_this_robot_conf(_robot, graspConf);
    
    _robot->isCarryingObject = FALSE;
    
    if (q) {
      fixAllHands(NULL, true);
      return q;
    }
  }
  return NULL;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::getGraspOpenApproachExtractConfs(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, ManipulationData& configs) const {
    double confCost = -1;
  
    configPt q = getGraspConf(object, armId, grasp, tAtt, &confCost);
  
    if (q) {
      cout << "FOUND Grasp Config!!!!!" << endl;
      configs.setGraspConfig(q);
      configs.setGraspConfigCost(confCost);
      configs.setGrasp(new gpGrasp(grasp));
      configs.setAttachFrame(tAtt);
      p3d_destroy_config(_robot, q);
 
      configPt q = getOpenGraspConf(object, armId, grasp, configs.getGraspConfig());
      
      if (q) {
        cout << "FOUND Open Config!!!!!" << endl;
        configs.setOpenConfig(q);
        p3d_destroy_config(_robot, q);
    
        configPt q = getApproachFreeConf(object, armId, grasp, configs.getGraspConfig(), tAtt);
        
        if (q) {
          cout << "FOUND Approach Config!!!!!" << endl;
          configs.setApproachFreeConfig( configs.getOpenConfig() );
          configs.setApproachFreeConfig(q);
          p3d_destroy_config(_robot, q);
          
          configPt q = getApproachGraspConf(object, armId, grasp, configs.getGraspConfig(), tAtt);
          
          if (q) {
            cout << "FOUND Approach Grasp Config!!!!!" << endl;
            configs.setApproachGraspConfig(q);
            p3d_destroy_config(_robot, q);
            deactivateCcCntrts(_robot, armId);
            cout << "SUCCES ALL CONFIG FOUND" << endl;
            return MANIPULATION_TASK_OK; //success
          }
        }
      }
    }
    deactivateCcCntrts(_robot, armId);
    return MANIPULATION_TASK_NO_GRASP;
}

MANIPULATION_TASK_MESSAGE ManipulationPlanner::computeManipulationData(int armId,p3d_rob* object)
{
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  
  // Generate new manip configs
	ManipulationData manipConfigs(_robot);
	
	_configs = manipConfigs;
	
  for(unsigned int i=0; i<1; i++)
	{
		status = findArmGraspsConfigs( armId,object, _configs );
		
		if (status == MANIPULATION_TASK_OK) 
		{
			break;
		}
		else {
			cout << "ManipulationPlanner::status => " << _ErrorMap[status] << endl;
		}
	}
  return status;
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
        if (armData.getCartesian()) {
            /* Uptdate the Virual object for inverse kinematics */
            p3d_update_virtual_object_config_for_arm_ik_constraint(_robot, i, q);
            activateCcCntrts(_robot, i, false);
            unfixManipulationJoints(i);
            #ifdef MULTILOCALPATH
            if(_robot->lpl_type == P3D_MULTILOCALPATH_PLANNER){
              p3d_multiLocalPath_set_groupToPlan(_robot, armData.getCartesianGroup(), 1);
            }
            #endif
            armData.getManipulationJnt()->dist = _robot->joints[_robot->mlp->mlpJoints[armData.getHandGroup()]->joints[0]]->dist;
        } else {
            deactivateCcCntrts(_robot, i);
            fixManipulationJoints(i, q, object);
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
        
#ifdef MULTILOCALPATH
        if(_robot->lpl_type == P3D_MULTILOCALPATH_PLANNER){
          p3d_multiLocalPath_set_groupToPlan(_robot, armData.getCartesianGroup(), 1);
        }
#endif
    } else {
        deactivateCcCntrts(_robot, armId);
#ifdef MULTILOCALPATH
        if(_robot->lpl_type == P3D_MULTILOCALPATH_PLANNER){
          p3d_multiLocalPath_set_groupToPlan(_robot, armData.getCartesianGroup(), 0);
        }
#endif
        shootTheObjectInTheWorld(_robot, armData.getManipulationJnt());
    }
}
bool ManipulationPlanner::getArmCartesian(int armId) const {
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
MANIPULATION_TASK_MESSAGE ManipulationPlanner::concatTrajectories (std::vector<p3d_traj*>& trajs, p3d_traj** concatTraj) {
    if ( _robot==NULL ) {
        PrintInfo ( ( "concateneAllTrajectories: robot is NULL.\n" ) );
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }
    if ( trajs.empty() ) {
        PrintInfo ( ( "concateneAllTrajectories: the trajectory vector is empty.\n" ) );
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }
    *concatTraj = p3d_create_traj_by_copy(trajs[0]);
    for (int i = 1; i < (int)trajs.size(); i++) {
        p3d_concat_traj(*concatTraj, trajs[i]);
    }

    _robot->tcur = (*concatTraj);
    g3d_add_traj((char*)"Task", (*concatTraj)->num);

    return MANIPULATION_TASK_OK;
}

//! Compute an RRT with
//! @param smoothingSteps : number of smoothing steps
//! @param smootingTime : maximum smoothing Time
//! @param biDir : is the RRT bidirectional
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
    ENV.setDouble(Env::extensionStep, 6.0);


#if 0 &&( defined(USE_CXX_PLANNER) || defined(MOVE3D_CORE) )
    ENV.setBool(Env::drawGraph, false);
    ENV.setBool(Env::withSmoothing, true);
    ENV.setBool(Env::withShortCut, true);
    ENV.setBool(Env::withDeformation, false);
    ENV.setInt(Env::nbCostOptimize, smoothingSteps);
    ENV.setDouble(Env::timeOptimize, smootingTime);

  ChronoOn();
	
	if(_robot->GRAPH)
	{
		p3d_del_graph(_robot->GRAPH);
	}
	
  result = ext_p3d_run_rrt( _robot->GRAPH, fct_stop, fct_draw );

    if (_robot->GRAPH)
    {
        p3d_del_graph(_robot->GRAPH);
    }

    result = ext_p3d_run_rrt(_robot->GRAPH, fct_stop, fct_draw);

    ChronoPrint("");
    ChronoOff();
#else
    result = p3d_specific_search((char *)"");
    p3d_traj* traj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
    optimiseTrajectory(_robot, traj, smoothingSteps, smootingTime);
#endif
    if (!result) {
        printf("ArmGotoQ: could not find a path.\n");
        ManipulationUtils::printConstraintInfo(_robot);
        return 1;
    }
    return 0;
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

//! Computes a trajectory between two configurations
//! It first moves the qi and qf configuration to the ROBOT_POS and ROBOT_GOTO
//! This method calls the compute RRT method from this class
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

/* ******************************* */
/* ******* Re-planning *********** */
/* ******************************* */

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
    checkConfigForCartesianMode(NULL, NULL);
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
MANIPULATION_TASK_MESSAGE ManipulationPlanner::replanCollidingTraj(int currentLpId, std::vector <p3d_traj*> &trajs) {
    p3d_traj* traj = NULL;
    std::vector<double>  objStart, objGoto;
    XYZ_ENV->cur_robot = _robot;
    //initialize and get the current linear traj
    if (!traj) {
        if (_robot->nt < _robot->tcur->num - 2) {
            return MANIPULATION_TASK_INVALID_TRAJ_ID;
        } else {
            traj = _robot->t[_robot->tcur->num - 2];
        }
    }
    checkConfigForCartesianMode(NULL, NULL);
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
        return armPlanTask(ARM_FREE, 0, currentConfig, _robot->ROBOT_GOTO, objStart, objGoto, (char *) "",(char *) "", trajs);
    }
    p3d_traj* replanTraj = (p3d_traj*) p3d_get_desc_curid(P3D_TRAJ);
    if (optimized || j > 1) {
        optimiseTrajectory(_robot, replanTraj, _optimizeSteps, _optimizeTime);
    }
    trajs.push_back(replanTraj);
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
        if (concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
            /* COMPUTE THE SOFTMOTION TRAJECTORY */
            MANPIPULATION_TRAJECTORY_CONF_STR conf;
            SM_TRAJ smTraj;
            computeSoftMotion(traj, conf, smTraj);
            confs.push_back(conf);
            smTrajs.push_back(smTraj);
        } else {
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
    if (p3d_local_get_planner() != 9) {
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
/* ******* Manipulation Planning * */
/* ******************************* */

//! The arm Goto Pick method takes in an object and a configuration
//! and then computes a vector of trajectories
//! it calls the armPickGoto on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qStart : the configuration from which to grasp the object
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, std::vector <p3d_traj*> &trajs){
	
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  
	if ((*_robot->armManipulationData)[armId].getManipState() != handFree ) 
	{
		cout << "Warning::ManipulationPlanner::the robot is not in a hand free state" << endl;
		//return status;
	}
  
  status = computeManipulationData(armId,object);
	
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
    if ((*_robot->armManipulationData)[armId].getManipState() != handFree ){
        cout << "Warning::ManipulationPlanner::the robot is not in a hand free state" << endl;
        //return status;
    }

    return status;
}

//! Same as above but with more parameters
//! @param configPt graspConfig : the configuration to grasp the object
//! @param configPt openConfig : the configuration with the open hand
//! @param configPt approachFreeConfig : an aproaching configuration
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGoto(int armId, configPt qStart, p3d_rob* object, configPt graspConfig, configPt openConfig, configPt approachFreeConfig, std::vector <p3d_traj*> &trajs) {

    p3d_traj* traj = NULL;

    ArmManipulationData& armData = (*_robot->armManipulationData)[armId];

    fixAllHands(qStart, false);
    fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

    checkConfigForCartesianMode(qStart, object);
    ManipulationUtils::copyConfigToFORM(_robot, qStart);
    checkConfigForCartesianMode(approachFreeConfig, object);
    ManipulationUtils::copyConfigToFORM(_robot, approachFreeConfig);
    checkConfigForCartesianMode(openConfig, object);
    ManipulationUtils::copyConfigToFORM(_robot, openConfig);
    checkConfigForCartesianMode(graspConfig, object);
    ManipulationUtils::copyConfigToFORM(_robot, graspConfig);
    //fixJoint(_robot, armData.getManipulationJnt() , armData.getManipulationJnt()->abs_pos);

    if (!p3d_is_collision_free(_robot,qStart)){
        cout << "Warning::ManipulationPlanner::armPickGoto => qStart is not collision free" << endl;
    }

    if (!p3d_is_collision_free(_robot,approachFreeConfig)){
        cout << "ManipulationPlanner::armPickGoto => approachFreeConfig is not collision free" << endl;
    }
    // Compute to Approach config
    if ((traj = computeTrajBetweenTwoConfigs(qStart, approachFreeConfig))){
      trajs.push_back(traj);
      // Compute to Open config
      if ((traj = computeTrajBetweenTwoConfigs(approachFreeConfig, openConfig))){
        trajs.push_back(traj);
        (*_robot->armManipulationData)[armId].unFixHand(_robot);
        p3d_multiLocalPath_set_groupToPlan(_robot, (*_robot->armManipulationData)[armId].getHandGroup(), 1);
        // Compute to Grasp Config
        if ((traj = computeTrajBetweenTwoConfigs(openConfig, graspConfig))){
            trajs.push_back(traj);

            // This state should be set
            // After acknolegment by another module
            (*_robot->armManipulationData)[armId].getManipState() = holdingObjectInStablePose;

            return MANIPULATION_TASK_OK;
        }
      }
    }
    return MANIPULATION_TASK_NO_TRAJ_FOUND;
}

//! The Arm Pick Take To Free method takes in an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qGoal : the configuration to bring the object to
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFreePoint(int armId, std::vector<double> &objGoto , p3d_rob* object, std::vector <p3d_traj*> &trajs){
  
	MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
	
	if ((*_robot->armManipulationData)[armId].getManipState() != holdingObjectInStablePose) 
	{
		cout << "Warning::ManipulationPlanner:: the robot is not holding object in a stable pose" << endl;
		//return status;
	}
  
  p3d_matrix4 tAtt;
  _configs.getAttachFrame(tAtt);
    
  configPt qGoal = getFreeHoldingConf(object, armId, 
                                        *_configs.getGrasp(), 
                                        tAtt, 
                                        objGoto );
  if (qGoal) 
  {
    //cout << "Goal Config = " << endl;
    //showConfig_2(qGoal);

    if (status == MANIPULATION_TASK_OK){
        //Compute the path between theses configurations
        status = armPickTakeToFree(armId, qGoal, object,
                                   _configs.getGraspConfig(),
                                   _configs.getApproachGraspConfig(),
                                   *_configs.getGrasp(),
                                   trajs);
    }
  }
  else {
    status = MANIPULATION_TASK_NO_TRAJ_FOUND;
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
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qGoal, p3d_rob* object, std::vector <p3d_traj*> &trajs) {

    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;

    if ((*_robot->armManipulationData)[armId].getManipState() != holdingObjectInStablePose)
    {
        cout << "Warning::ManipulationPlanner:: the robot is not holding object in a stable pose" << endl;
        //return status;
    }

    if (status == MANIPULATION_TASK_OK) {
        //Compute the path between theses configurations
        status = armPickTakeToFree(armId, qGoal, object,
                                   _configs.getGraspConfig(),
                                   _configs.getApproachGraspConfig(),
                                   *_configs.getGrasp(),
                                   trajs);
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
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToFree(int armId, configPt qGoal,  p3d_rob* object, configPt qStart, configPt approachGraspConfig , gpGrasp &grasp , std::vector <p3d_traj*> &trajs) {

    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_NO_TRAJ_FOUND;

    p3d_traj* traj = NULL;

    ArmManipulationData& armData = (*_robot->armManipulationData)[armId];

    fixAllHands( qStart, false );
    fixJoint(_robot, _robot->baseJnt, _robot->baseJnt->abs_pos);

#ifdef MULTILOCALPATH
    if (_robot->lpl_type == P3D_MULTILOCALPATH_PLANNER )
    {
        p3d_multiLocalPath_set_groupToPlan(_robot, armData.getCartesianGroup(), 1);
    }
#endif

    deactivateCcCntrts(_robot,armId);

    p3d_set_object_to_carry_to_arm(_robot, armId, object->name );
    setAndActivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(),
                                    armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);
    //unfixManipulationJoints(armId);

    //cout << "qStart : " << endl;
    //showConfig_2(qStart);

    // set the approachGraspConfig
    // To be computed with the IK of the robot (mult matrix problem)
    p3d_set_and_update_this_robot_conf(_robot,approachGraspConfig);
    p3d_get_robot_config_into(_robot, &approachGraspConfig);
    //cout << "approachGraspConfig : " << endl;
    //showConfig_2(approachGraspConfig);

    gpHand_properties handProp = armData.getHandProperties();
    gpSet_grasp_configuration(_robot, grasp, qGoal, armId);
    //cout << "qGoal : " << endl;
    //showConfig_2(qGoal);
    //_configs.getAttachFrame(ct->Tatt);


    // Compute to Approach config
    if ((traj = computeTrajBetweenTwoConfigs(qStart, approachGraspConfig)))
    {
        trajs.push_back(traj);

        // Compute to Open config
        if ((traj = computeTrajBetweenTwoConfigs(approachGraspConfig, qGoal)))
        {
            trajs.push_back(traj);
            cout << "Manipulation : traj found" << endl;
            status =  MANIPULATION_TASK_OK;
        }
    }

// Wraning comment to see traj in Linear mode
//  _robot->isCarryingObject = false;
//  desactivateTwoJointsFixCntrt(_robot,armData.getManipulationJnt(),
//                               armData.getCcCntrt()->pasjnts[ armData.getCcCntrt()->npasjnts-1 ]);

    return status;
}

//! The Arm Pick Take To Place method takes an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
//
//! @param armId : which arm is used to grasp the object
//! @param qGoal : the configuration to bring the object to
//! @param object : pointer to the p3d_rob that represent the moving object
//! @param trajs : the vector of trajector optained
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickTakeToPlace(int armId, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs) {
    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    return status;
}
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlaceFromFree(int armId, configPt qStart, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs) {
    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    return status;
}

//! The Arm Pick And Place method takes in an object in a goal configuration
//! and then computes a vector of trajectories by calling
//! the armPickTakeToPlace on the manipulation data
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickAndPlace(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs) {
    MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
    return status;
}

//! The Arm Pick And Place method computes movtion to a grasping position
//! and takes the object to a free configuration
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPickGotoAndTakeToFree(int armId, configPt qStart, configPt qGoal, p3d_rob* object, p3d_rob* placement, std::vector <p3d_traj*> &trajs)
{
	MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
	
	ArmManipulationData& armData = (*_robot->armManipulationData)[armId];
	
	if (armData.getManipState() != handFree ) 
	{
		cout << "Manipulation planner : the robot is not in a hand free state" << endl;
		//return MANIPULATION_TASK_INVALID_QSTART;
	}
	
	// Generate new manip configs
	status = computeManipulationData(armId,object);
	
  if (status == MANIPULATION_TASK_OK)
	{
    // Compute the path between Start configuration
		// and the GraspConfig
    status = armPickGoto(armId, qStart, object, 
												 _configs.getGraspConfig(), 
												 _configs.getOpenConfig(), 
												 _configs.getApproachFreeConfig(), trajs);
  }
	else 
	{
		return MANIPULATION_TASK_NO_TRAJ_FOUND;
	}
	
	
	if (armData.getManipState() != holdingObjectInStablePose) 
	{
		cout << "Manipulation planner : the robot is not holding object in a stable pose" << endl;
		return MANIPULATION_TASK_INVALID_QSTART;
	}
	
	//Compute the path between theses configurations
	status = armPickTakeToFree(armId, qGoal, object,
														 _configs.getGraspConfig(),
														 _configs.getApproachGraspConfig(),
														 *_configs.getGrasp(),
														 trajs);
	
  return status;
}


/* ******************************* */
/* ******** Task Planning ******** */
/* ******************************* */

MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, std::vector <p3d_traj*> &trajs) {
  
  if (!_robot) {
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
  if (armId < 0 || armId >= (int)_robot->armManipulationData->size()) {
    status = MANIPULATION_TASK_INVALID_TASK;
  } else {
    ENV.setBool(Env::drawTraj, false);
    checkConfigForCartesianMode(qi, object);
    checkConfigForCartesianMode(qf, object);
    fixAllHands(qi, false);
    fixAllHands(qf, false);
    p3d_set_and_update_this_robot_conf(_robot, qi);
    switch (task) {
      case ARM_FREE: {
        printf("plan for ARM_FREE task\n");
        if (MPDEBUG) {
            ManipulationUtils::copyConfigToFORM(_robot, qi);
            ManipulationUtils::copyConfigToFORM(_robot, qf);
        }
        if ((traj = computeTrajBetweenTwoConfigs(qi, qf)) == NULL) {
            printf("ERROR armPlanTask(ARM_FREE) on traj\n");
            status = MANIPULATION_TASK_NO_TRAJ_FOUND;
            break;
        }
        trajs.push_back(traj);
        break;
      }
      case ARM_PICK_GOTO: {
        printf("plan for ARM_PICK_GOTO task\n");
        status = armPickGoto(armId, qi, object, trajs);
        break;
      }
      case ARM_PICK_TAKE_TO_FREE: {
        printf("plan for ARM_PICK_TAKE_TO_FREE task\n");
        status = armPickTakeToFree(armId, qf, object, trajs);
        break;
      }
      case ARM_PICK_TAKE_TO_FREE_POINT:{
        printf("plan for ARM_PICK_TAKE_TO_FREE_POINT task\n");
        status = armPickTakeToFreePoint(armId, objGoto, object, trajs);
        break;
      }
      case ARM_PICK_TAKE_TO_PLACE: {
        printf("plan for ARM_PICK_TAKE_TO_PLACE task\n");
        status = armPickTakeToPlace(armId, object, support, trajs);
        break;
      }
      case ARM_PLACE_FROM_FREE: {
        printf("plan for ARM_PLACE_FROM_FREE task\n");
        status = armPlaceFromFree(armId, qi, object, support, trajs);
        break;
      }
      case ARM_PICK_AND_PLACE: {
        printf("plan for ARM_PICK_AND_PLACE task\n");
        status = armPickAndPlace(armId, qi, qf, object, support, trajs);
        break;
      }
      case ARM_PICK_GOTO_AND_TAKE_TO_FREE: {
        printf("plan for ARM_PICK_GOTO_AND_TAKE_TO_FREE task\n");
        status = armPickGotoAndTakeToFree(armId, qi, qf, object, support, trajs);
        break;
      }
      default: {
        printf("%s: %d: ManipulationPlanner::armPlanTask(): wrong task.\n", __FILE__, __LINE__);
        status = MANIPULATION_TASK_INVALID_TASK;
        break;
      }
    }
    unfixManipulationJoints(armId);
    p3d_sel_desc_id(P3D_ROBOT,cur_robot);
    g3d_draw_allwin_active();
    if (status == MANIPULATION_TASK_OK) {
      printf("BioMove3D: armPlanTask OK\n");
    } else {
      printf("BioMove3D: armPlanTask Fail\n");
    }
  }
  return status;
}

//! Computes a path for a specific task
//! This function is an interface for the softmotion
//! trajectory generation
#ifdef MULTILOCALPATH
MANIPULATION_TASK_MESSAGE ManipulationPlanner::armPlanTask(MANIPULATION_TASK_TYPE_STR task, int armId, configPt qStart, configPt qGoal, std::vector<double> &objStart, std::vector<double> &objGoto, const char* objectName, const char* supportName, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs) {
    std::vector <p3d_traj*> trajs;
    p3d_traj* traj = NULL;
    MANIPULATION_TASK_MESSAGE returnMessage;

    if (!_robot) {
        printf("%s: %d: ManipulationPlanner::armPlanTask(): No robot initialized.\n", __FILE__, __LINE__);
        return MANIPULATION_TASK_NOT_INITIALIZED;
    }

    p3d_multiLocalPath_disable_all_groupToPlan(_robot);
    p3d_multiLocalPath_set_groupToPlan(_robot, _UpBodyMLP, 1);

    if ((returnMessage = armPlanTask(task, armId, qStart, qGoal, objStart, objGoto, objectName, supportName, trajs)) == MANIPULATION_TASK_OK) {
      //concatene
      if (concatTrajectories(trajs, &traj) == MANIPULATION_TASK_OK) {
        if(getArmCartesian(0)){ //TODO Softmotion smoothing
          return returnMessage; 
        }
        /* COMPUTE THE SOFTMOTION TRAJECTORY */
        MANPIPULATION_TRAJECTORY_CONF_STR conf;
        SM_TRAJ smTraj;
        computeSoftMotion(traj, conf, smTraj);
        confs.push_back(conf);
        smTrajs.push_back(smTraj);
      } else {
        returnMessage = MANIPULATION_TASK_NO_TRAJ_FOUND;
      }
    }

    return returnMessage;
}
#endif
