#include "ManipulationConfigs.hpp"
#include "robotPos.h"
#include "lightPlannerApi.h"
#include "Collision-pkg.h"


static bool MCDEBUG=false;
using namespace std;

ManipulationConfigs::ManipulationConfigs(p3d_rob* robot):_robot(robot){
  _optimizeRedundentSteps = 50;
  _approachFreeOffset = 0.10; //0.1 meters
  _approachGraspOffset = 0.10; //0.1 meters
}

ManipulationConfigs::~ManipulationConfigs(){
  //Nothing to do
}

void ManipulationConfigs::setDebugMode(bool value){
  MCDEBUG = value;
}

void ManipulationConfigs::setOptimizeRedundentSteps(int nbSteps){
  _optimizeRedundentSteps = nbSteps;
}
int ManipulationConfigs::getOptimizeRedundentSteps(void) const{
  return _optimizeRedundentSteps;
}

void ManipulationConfigs::setApproachFreeOffset(double offset) {
  _approachFreeOffset = offset;
}
double ManipulationConfigs::getApproachFreeOffset(void) const {
  return _approachFreeOffset;
}

void ManipulationConfigs::setApproachGraspOffset(double offset) {
    _approachGraspOffset = offset;
}
double ManipulationConfigs::getApproachGraspOffset(void) const {
  return _approachGraspOffset;
}


configPt ManipulationConfigs::getGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost) const {
  ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
  gpHand_properties handProp = mData.getHandProperties();
  // Check if a valid configuration exists
  // of the robot using this graspFrame
  gpSet_grasp_configuration(_robot, grasp, armId);
  gpFix_hand_configuration(_robot, handProp, armId);

  gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
  configPt qGrasp = setRobotGraspPosWithoutBase(_robot, object->joints[1]->abs_pos, tAtt, false, false, armId, true);
  if(qGrasp){
    confCost = optimizeRedundentJointConfigCost(_robot, mData.getCcCntrt()->argu_i[0], qGrasp, object->joints[1]->abs_pos, tAtt, grasp, armId, getOptimizeRedundentSteps());

    if( MCDEBUG )
    {
      std::cout << "--------------------------------------" << std::endl;
      std::cout << " Grasp Confort cost : " << confCost << std::endl;
      std::cout << "--------------------------------------" << std::endl;
    }

    gpSet_grasp_configuration(_robot, grasp, qGrasp, armId);
  }else {
    confCost = -1;
  }
  deactivateCcCntrts(_robot, armId);
  gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
  return qGrasp;
}

configPt ManipulationConfigs::getOpenGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf) const {
  if (graspConf) {
    configPt q = p3d_copy_config(_robot, graspConf);

    //Check the open configuration of the hand
    gpSet_grasp_open_configuration(_robot, grasp, q, armId);

    if (p3d_is_collision_free(_robot,q))
    {
      return q;

    }
    else {
        // if the grasp open config is colliding, recompute it taking into account the environment:
        grasp.computeOpenConfig(_robot, object, true);
        gpSet_grasp_open_configuration(_robot, grasp, q, armId);
        if (p3d_is_collision_free(_robot,q)) {
            return q;
      }
    }
  }
  return NULL;
}

configPt ManipulationConfigs::getApproachFreeConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const {

  if (graspConf)
  {
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    configPt q = p3d_copy_config(_robot, graspConf);
    gpHand_properties handProp = mData.getHandProperties();

    p3d_matrix4 objTmp;
    p3d_vector3 tAttY, tAttT;
    p3d_mat4Copy(object->joints[1]->abs_pos, objTmp);

    if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_kuka_arm_ik")) {
      p3d_mat4ExtractColumnY(tAtt, tAttY);
    } else if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_lwr_arm_ik")) {
      p3d_mat4ExtractColumnZ(tAtt, tAttY);
    }

    gpUnFix_hand_configuration(_robot, handProp, armId);
    gpSet_grasp_open_configuration(_robot, grasp, q, armId);
    gpFix_hand_configuration(_robot, handProp, armId);

    configPt qApproachFree = NULL;

    const unsigned int MaxDesc = 2 ;
    for (unsigned int i=0; i<MaxDesc; i++)
    {
      double alpha = 1 - (double)i/(double)MaxDesc;

      p3d_xformVect(objTmp, tAttY, tAttT);
      objTmp[0][3] -= alpha * getApproachFreeOffset() * tAttT[0];
      objTmp[1][3] -= alpha * getApproachFreeOffset() * tAttT[1];
      objTmp[2][3] -= alpha * getApproachFreeOffset() * tAttT[2];

      //    q[mData.getManipulationJnt()->index_dof + 0] -= getApproachFreeOffset() * tAttT[0];
      //    q[mData.getManipulationJnt()->index_dof + 1] -= getApproachFreeOffset() * tAttT[1];
      //    q[mData.getManipulationJnt()->index_dof + 2] -= getApproachFreeOffset() * tAttT[2];

      //    gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);

      qApproachFree = setRobotCloseToConfGraspApproachOrExtract(_robot, q, objTmp, tAtt, false, armId, true);
      if ( qApproachFree ){
        double dist = -1;
        if ((dist = optimizeRedundentJointConfigDist(_robot, mData.getCcCntrt()->argu_i[0], qApproachFree, object->joints[1]->abs_pos, tAtt, q, armId, getOptimizeRedundentSteps())) == -1){
          p3d_destroy_config(_robot, qApproachFree);
          qApproachFree = NULL;
        }
        break;
      }
    }

    p3d_destroy_config(_robot, q);
    q = NULL;

    deactivateCcCntrts(_robot, armId);
    return qApproachFree;
//    gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);

  }
  return NULL;
}

configPt ManipulationConfigs::getApproachGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const{

  if(graspConf){
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];

    gpHand_properties handProp = mData.getHandProperties();
    configPt q = p3d_copy_config(_robot, graspConf);

    if(object){
      mData.setCarriedObject(object);
      _robot->isCarryingObject = TRUE;
    }

    // Set Manipulation joint and hand configuration
    q[(*_robot->armManipulationData)[armId].getManipulationJnt()->index_dof + 2] += getApproachGraspOffset(); //Z axis of the manipulation joint
    p3d_set_and_update_this_robot_conf(_robot, q);
    if(grasp.ID != 0){
      gpSet_grasp_configuration(_robot, grasp, q, armId);
    }
    ManipulationUtils::fixAllHands(_robot, NULL, false);
    gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    // Sample a configuration for the robot
    configPt approachConfig = setRobotCloseToConfGraspApproachOrExtract(_robot, q, object->joints[1]->abs_pos, tAtt, false, armId, true);
    if ( approachConfig ){
      if (optimizeRedundentJointConfigDist(_robot, mData.getCcCntrt()->argu_i[0], approachConfig, object->joints[1]->abs_pos, tAtt, q, armId, getOptimizeRedundentSteps()) == -1){
        p3d_destroy_config(_robot, approachConfig);
        approachConfig = NULL;
      }
    }
    gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    deactivateCcCntrts(_robot, armId);
    p3d_destroy_config(_robot, q);
    // Reset robot to the initial robot configuration
    p3d_set_and_update_this_robot_conf(_robot, graspConf);

    _robot->isCarryingObject = FALSE;

    return approachConfig;
  }
  return NULL;
}

configPt ManipulationConfigs::getFreeHoldingConf( p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost, std::vector<double> &objGoto, p3d_rob* support) const {
  cout << " ManipulationConfigs::getFreeHoldingConf" << endl;

  configPt tmpConf = p3d_get_robot_config(_robot);
  int restore = false;
  p3d_matrix4 bakTatt, objPos;
  ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
  gpHand_properties handProp = mData.getHandProperties();
  configPt q = p3d_get_robot_config(_robot);
  if(tAtt[0][0] == 0 && tAtt[0][1] == 0 && tAtt[0][2] == 0 && tAtt[0][3] == 0){
    //NULL frame compute it.
    restore = true;
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, bakTatt);
    p3d_compute_Tatt(mData.getCcCntrt());
    p3d_mat4Copy(mData.getCcCntrt()->Tatt, tAtt);
  }

  if(object){
    mData.setCarriedObject(object);
    _robot->isCarryingObject = TRUE;
  }
  deactivateCcCntrts(_robot, armId);
  // Set Manipulation joint and hand configuration

  int idManipIndexDof = mData.getManipulationJnt()->index_dof;
  q[ idManipIndexDof + 0 ] = objGoto.at(0);
  q[ idManipIndexDof + 1 ] = objGoto.at(1);
  q[ idManipIndexDof + 2 ] = objGoto.at(2);

//Bit computation to get the rotation to sample.
//When the Bit == 1 implies the corresponding rotation has to be sampled
  int sampleObjectRotation = 7;
  if(objGoto.at(3)!=P3D_HUGE){
    q[ idManipIndexDof + 3 ] = objGoto.at(3);
    sampleObjectRotation ^= 1;
  }
  if(objGoto.at(4)!=P3D_HUGE){
    q[ idManipIndexDof + 4 ] = objGoto.at(4);
    sampleObjectRotation ^= 2;
  }
  if(objGoto.at(5)!=P3D_HUGE){
    q[ idManipIndexDof + 5 ] = objGoto.at(5);
    sampleObjectRotation ^= 4;
  }

  p3d_set_and_update_this_robot_conf(_robot, q);

  if(grasp.ID != 0){//if Tatt is valid thats mean that _config is up to date and the grasp isgp_grasp
    gpSet_grasp_configuration(_robot, grasp, q, armId);
  }
  ManipulationUtils::fixAllHands(_robot, NULL, false);
  if(object){
    gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    p3d_mat4Copy( object->joints[1]->abs_pos, objPos);
    if(support){
      p3d_col_deactivate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
    }
  }else{
    p3d_mat4Copy(mData.getManipulationJnt()->abs_pos , objPos);
  }
  q = setRobotGraspPosWithoutBase(_robot, objPos, tAtt, false, sampleObjectRotation , armId, true);
  if(q){
    optimizeRedundentJointConfigCost(_robot, mData.getCcCntrt()->argu_i[0], q, objPos, tAtt, grasp, armId, getOptimizeRedundentSteps());
  }
  if(object){
    gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    if(support){
      p3d_col_activate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
    }
  }
  deactivateCcCntrts(_robot, armId);
  
  // Reset robot to the initial robot configuration
  p3d_set_and_update_this_robot_conf(_robot, tmpConf);
  
  _robot->isCarryingObject = FALSE;
  mData.setCarriedObject((p3d_rob*)NULL);

  p3d_destroy_config(_robot, tmpConf);

  if(restore){
    p3d_mat4Copy(bakTatt, mData.getCcCntrt()->Tatt);
  }
  return q;
}

configPt ManipulationConfigs::getExtractConf(int armId, configPt currentConf, p3d_matrix4 tAtt) const{

  if(currentConf){
    ArmManipulationData mData = (*_robot->armManipulationData)[armId];
    gpHand_properties handProp = mData.getHandProperties();
    configPt q = p3d_copy_config(_robot, currentConf);

    // Set Manipulation joint and hand configuration
    ManipulationUtils::fixAllHands(_robot, NULL, false);

    // Sample a configuration for the robot
    configPt extractConfig = NULL;
    desactivateTwoJointsFixCntrt(_robot, mData.getManipulationJnt(), mData.getCcCntrt()->pasjnts[ mData.getCcCntrt()->npasjnts-1]);
    for(int i = 0; !extractConfig && i < 5; i++){
      q[(*_robot->armManipulationData)[armId].getManipulationJnt()->index_dof + 2] += getApproachGraspOffset(); //Z axis of the manipulation joint
      p3d_set_and_update_this_robot_conf(_robot, q);
      extractConfig = setRobotCloseToConfGraspApproachOrExtract(_robot, q, mData.getManipulationJnt()->abs_pos, tAtt, false, armId, true);
    }
    if ( extractConfig ){
      double dist = -1;
      if ((dist = optimizeRedundentJointConfigDist(_robot, mData.getCcCntrt()->argu_i[0], extractConfig,  mData.getManipulationJnt()->abs_pos, tAtt, currentConf, armId, getOptimizeRedundentSteps())) == -1){
        p3d_destroy_config(_robot, extractConfig);
        extractConfig = NULL;
      }
    }
    deactivateCcCntrts(_robot, armId);
    p3d_destroy_config(_robot, q);
    // Reset robot to the initial robot configuration
    p3d_set_and_update_this_robot_conf(_robot, currentConf);
    setAndActivateTwoJointsFixCntrt(_robot,mData.getManipulationJnt(), mData.getCcCntrt()->pasjnts[ mData.getCcCntrt()->npasjnts-1 ]);

    return extractConfig;
  }
  return NULL;
}
