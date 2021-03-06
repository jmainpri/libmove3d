/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include "ManipulationConfigs.hpp"

#include "Collision-pkg.h"
#include <g3d_window_proto.h>

#include "robotPos.h"
#include "lightPlannerApi.h"
#include "ManipulationArmData.hpp"

static bool MCDEBUG=false;
using namespace std;

//! Pointer to manipulation config
ManipulationConfigs* global_ManipulationConfigs = NULL;

//! get free holding config
configPt manipulation_get_free_holding_config()
{
  global_ManipulationConfigs->getFreeHoldingConf();
}

ManipulationConfigs::ManipulationConfigs(p3d_rob* robot):_robot(robot)
{
  // Basic parameters
  _optimizeRedundentSteps = 50;
  _approachFreeOffset = 0.10; //0.10 meters
  _approachGraspOffset = 0.10; //0.02 meters
  _safetyDistanceValue = 0.0;
  _useMobileBase = false;
  
  // Robot Pos options
  setMaxNumberOfTryForIK( 500 );
  setDebugConfAroundTheObject( false );
  
  // Data to be stored for FreeHoldingConf funct
  _IKData = new ManipIKConfigData();
  
  // Pointer to the class
  global_ManipulationConfigs = this;
}

ManipulationConfigs::~ManipulationConfigs()
{
  delete _IKData;
  global_ManipulationConfigs = NULL;
}

void ManipulationConfigs::setDebugMode(bool value) const
{
  MCDEBUG = value;
  setDebugConfAroundTheObject(value);
}

void ManipulationConfigs::setMobileBaseMode(bool value)
{
  _useMobileBase = value;
}

void ManipulationConfigs::setOptimizeRedundentSteps(int nbSteps)
{
  _optimizeRedundentSteps = nbSteps;
}

int ManipulationConfigs::getOptimizeRedundentSteps(void) const
{
  return _optimizeRedundentSteps;
}

void ManipulationConfigs::setApproachFreeOffset(double offset) 
{
  _approachFreeOffset = offset;
}

double ManipulationConfigs::getApproachFreeOffset(void) const 
{
  return _approachFreeOffset;
}

void ManipulationConfigs::setApproachGraspOffset(double offset) 
{
  _approachGraspOffset = offset;
}

double ManipulationConfigs::getApproachGraspOffset(void) const 
{
  return _approachGraspOffset;
}

void ManipulationConfigs::setSafetyDistanceValue(double value)
{
  _safetyDistanceValue = value;
}

double ManipulationConfigs::getSafetyDistanceValue(void) const
{
  return _safetyDistanceValue;
}

//! Generate the grasp configuration given the grasp the arm and the object.
//! @return the attach matrix computed given the grasp and Tatt2 from the p3d file
//! @return the configuration cost
//! @return the grasp configuration
configPt ManipulationConfigs::getGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost) const 
{
  ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
  gpHand_properties handProp = mData.getHandProperties();
  
  // Set grasp configuration and deactivate 
  gpSet_grasp_configuration(_robot, grasp, armId);
  gpFix_hand_configuration(_robot, handProp, armId);
  mData.deactivateManipulationCntrts(_robot);
  gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
  
  configPt qGrasp = NULL;
  
  // Check if a valid configuration of the robot exists
  // using this graspFrame
  if (!_useMobileBase) 
    qGrasp = sampleRobotGraspPosWithoutBase(_robot, object->joints[1]->abs_pos, tAtt, false, false, armId, true);
  else 
    qGrasp = sampleRobotGraspPosWithBase(_robot, object->joints[1]->abs_pos, tAtt, false, false, armId, true);
  
  if(qGrasp)
  {
    confCost = optimizeRedundentJointConfigCost(_robot, mData.getCcCntrt()->argu_i[0], qGrasp, 
                                                object->joints[1]->abs_pos, tAtt, grasp, armId, getOptimizeRedundentSteps());
    
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
  mData.deactivateManipulationCntrts(_robot);
  gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
  return qGrasp;
}

//! Generate the open configuration 
//! given the grasp configuration, the grasp, the arm and the object
//! the open config is the same as the grasp config with an open hand
configPt ManipulationConfigs::getOpenGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf) const 
{
  if (graspConf) 
  {
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    mData.deactivateManipulationCntrts(_robot);
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

//! Generate the open approach configuration given 
//! the grasp configuration, the grasp, the arm, the attach matrix and the object
configPt ManipulationConfigs::getApproachFreeConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const
{
  if (graspConf)
  {
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    configPt q = p3d_copy_config(_robot, graspConf);
    gpHand_properties handProp = mData.getHandProperties();
    mData.deactivateManipulationCntrts(_robot);
    
    p3d_matrix4 objPos, objTmp;
    p3d_vector3 tAttY, tAttT;
    p3d_mat4Copy(object->joints[1]->abs_pos, objPos);
    
    if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_kuka_arm_ik")) {
      p3d_mat4ExtractColumnY(tAtt, tAttY);
    } else if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_lwr_arm_ik")) {
      p3d_mat4ExtractColumnZ(tAtt, tAttY);
    } else if (!strcmp(mData.getCcCntrt()->namecntrt, "p3d_pr2_arm_ik")) {
      p3d_mat4ExtractColumnX(tAtt, tAttY);
    }
    
    gpUnFix_hand_configuration(_robot, handProp, armId);
    gpSet_grasp_open_configuration(_robot, grasp, q, armId);
    gpFix_hand_configuration(_robot, handProp, armId);
    
    configPt qApproachFree = NULL;
    
    const unsigned int MaxDesc = 2 ;
    for (unsigned int i=0; i<MaxDesc-1; i++)
    {
      double alpha = 1 - (double)i/(double)MaxDesc;
      
      p3d_mat4Copy(objPos, objTmp);
      p3d_xformVect(objTmp, tAttY, tAttT);
      objTmp[0][3] -= alpha * getApproachFreeOffset() * tAttT[0];
      objTmp[1][3] -= alpha * getApproachFreeOffset() * tAttT[1];
      objTmp[2][3] -= alpha * getApproachFreeOffset() * tAttT[2];
      // q[mData.getManipulationJnt()->index_dof + 0] -= getApproachFreeOffset() * tAttT[0];
      // q[mData.getManipulationJnt()->index_dof + 1] -= getApproachFreeOffset() * tAttT[1];
      // q[mData.getManipulationJnt()->index_dof + 2] -= getApproachFreeOffset() * tAttT[2];
      
      // gpDeactivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
      qApproachFree = sampleRobotCloseToConfGraspApproachOrExtract(_robot, q, objTmp, tAtt, false, armId, true);
      
      if ( qApproachFree ) 
      {
        double dist = -1;
        if ((dist = optimizeRedundentJointConfigDist(_robot, mData.getCcCntrt()->argu_i[0], qApproachFree, object->joints[1]->abs_pos, tAtt, q, armId, getOptimizeRedundentSteps())) == -1/* || dist == -2*/){
          //           ManipulationUtils::copyConfigToFORM(_robot, graspConf);
          //           ManipulationUtils::copyConfigToFORM(_robot, qApproachFree);
          p3d_destroy_config(_robot, qApproachFree);
          qApproachFree = NULL;
        }else{
          break;
        }
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

//! Generate the grasp approach configuration 
//! given the grasp configuration, the grasp, the arm, the attach matrix and the object
//! this configuration is away from the object with an open hand
configPt ManipulationConfigs::getApproachGraspConf(p3d_rob* object, int armId, gpGrasp& grasp, configPt graspConf, p3d_matrix4 tAtt) const{
  
  if(graspConf){
    ArmManipulationData& mData = (*_robot->armManipulationData)[armId];
    
    gpHand_properties handProp = mData.getHandProperties();
    configPt q = p3d_copy_config(_robot, graspConf);
    mData.deactivateManipulationCntrts(_robot);
    
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
    configPt approachConfig = sampleRobotCloseToConfGraspApproachOrExtract(_robot, q, object->joints[1]->abs_pos, tAtt, false, armId, true);
    if ( approachConfig ){
      double dist = -1;
      if ((dist = optimizeRedundentJointConfigDist(_robot, mData.getCcCntrt()->argu_i[0], approachConfig, object->joints[1]->abs_pos, tAtt, q, armId, getOptimizeRedundentSteps())) == -1/* || dist == -2*/){
        p3d_destroy_config(_robot, approachConfig);
        approachConfig = NULL;
      }
    }
    gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    mData.deactivateManipulationCntrts(_robot);
    p3d_destroy_config(_robot, q);
    // Reset robot to the initial robot configuration
    p3d_set_and_update_this_robot_conf(_robot, graspConf);
    
    _robot->isCarryingObject = FALSE;
    
    return approachConfig;
  }
  return NULL;
}

//! Generates a free configuration from a worspace point and a grasp
//! using the stored IK data
configPt ManipulationConfigs::getFreeHoldingConf() const 
{
  getFreeHoldingConf(_IKData->object, _IKData->armId, 
                     _IKData->grasp, _IKData->tAtt, 
                     _IKData->confCost, _IKData->objGoto, 
                     _IKData->support);
}

//! Generates a free configuration from a worspace point and a grasp
//! also store the IK data
configPt ManipulationConfigs::getFreeHoldingConf( p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt, double& confCost, std::vector<double> &objGoto, p3d_rob* support) const 
{
  cout << " ManipulationConfigs::getFreeHoldingConf" << endl;
  
  // Store the configData
  _IKData->object = object;
  _IKData->armId = armId;
  _IKData->grasp = grasp;
  _IKData->confCost = confCost;
  _IKData->objGoto = objGoto;
  _IKData->support = support;
  // Carefull mat4Copy : source -> destination
  p3d_mat4Copy( tAtt, _IKData->tAtt );
  
  configPt tmpConf = p3d_get_robot_config(_robot);
  configPt objectConf = NULL;
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
    objectConf = p3d_get_robot_config(object);
    mData.setCarriedObject(object);
    _robot->isCarryingObject = TRUE;
  }
  
  mData.deactivateManipulationCntrts(_robot);
  desactivateTwoJointsFixCntrt(_robot, 
                               mData.getManipulationJnt(),
                               mData.getCcCntrt()->pasjnts[mData.getCcCntrt()->npasjnts -1]);
  
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
  
  if (!_useMobileBase) 
    q = sampleRobotGraspPosWithoutBase(_robot, objPos, tAtt, false, sampleObjectRotation , armId, true);
  else 
    q = sampleRobotGraspPosWithBase(_robot, objPos, tAtt, false, sampleObjectRotation, armId, true);
  
  if(q){
    optimizeRedundentJointConfigCost(_robot, mData.getCcCntrt()->argu_i[0], q, objPos, tAtt, grasp, armId, getOptimizeRedundentSteps());
  }
  if(object){
    gpActivate_object_collisions(_robot, object->joints[1]->o, handProp, armId);
    if(support){
      p3d_col_activate_pair_of_objects(object->joints[1]->o, support->joints[1]->o);
    }
    p3d_set_and_update_this_robot_conf(object, objectConf);
    p3d_destroy_config(object, objectConf);
  }
  mData.deactivateManipulationCntrts(_robot);
  
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

//! Generate the extract configuration by moving the arm over Z axis 
//! until we have a collision free or passing 5 * offset
//! \param armId the arm to be used
//! \param currentConf the configuration 
//! \param pAtt the transformation between the object and the hand
configPt ManipulationConfigs::getExtractConf(int armId, configPt currentConf, p3d_matrix4 tAtt) const 
{
  if(currentConf){
    ArmManipulationData mData = (*_robot->armManipulationData)[armId];
    gpHand_properties handProp = mData.getHandProperties();
    configPt q = p3d_copy_config(_robot, currentConf);
    
    // Set Manipulation joint and hand configuration
    ManipulationUtils::fixAllHands(_robot, NULL, false);
    
    // Sample a configuration for the robot
    configPt extractConfig = NULL;
    mData.deactivateManipulationCntrts(_robot);
    for(int i = 0; !extractConfig && i < 5; i++){
      q[(*_robot->armManipulationData)[armId].getManipulationJnt()->index_dof + 2] += getApproachGraspOffset(); //Z axis of the manipulation joint
      p3d_set_and_update_this_robot_conf(_robot, q);
      extractConfig = sampleRobotCloseToConfGraspApproachOrExtract(_robot, q, mData.getManipulationJnt()->abs_pos, tAtt, false, armId, true);
    }
    if ( extractConfig ){
      double dist = -1;
      if ((dist = optimizeRedundentJointConfigDist(_robot, mData.getCcCntrt()->argu_i[0], extractConfig,  mData.getManipulationJnt()->abs_pos, tAtt, currentConf, armId, getOptimizeRedundentSteps())) == -1/* || dist == -2*/){
        p3d_destroy_config(_robot, extractConfig);
        extractConfig = NULL;
      }
    }
    mData.deactivateManipulationCntrts(_robot);
    p3d_destroy_config(_robot, q);
    // Reset robot to the initial robot configuration
    p3d_set_and_update_this_robot_conf(_robot, currentConf);
    setAndActivateTwoJointsFixCntrt(_robot,mData.getManipulationJnt(), mData.getCcCntrt()->pasjnts[ mData.getCcCntrt()->npasjnts-1 ]);
    
    return extractConfig;
  }
  return NULL;
}

//! Searches for configurations used in the PickGoto Motions
//! these configurations will be used to pick and place the object
//! \param object the object to be picked
//! \param armId the arm used
//! \param grasp the grasp chosen on the object
//! \param tAtt the attached matrix on the object
//! \return the configuration in the manipulationData
MANIPULATION_TASK_MESSAGE ManipulationConfigs::getGraspOpenApproachExtractConfs(p3d_rob* object, int armId, gpGrasp& grasp, p3d_matrix4 tAtt,  ManipulationData& configs) const 
{
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
  
  // Set hand frame
  p3d_matrix4 handFrame;
  p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, mData.getCcCntrt()->Tatt2, tAtt);
  
  q = getGraspConf(object, armId, grasp, tAtt, confCost);
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
    
    q = getOpenGraspConf(object, armId, grasp, configs.getGraspConfig());
    if (q) {
      if( debug_configs || MCDEBUG ) {
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
      q = getApproachFreeConf(object, armId, grasp, configs.getGraspConfig(), tAtt);
      setSafetyDistance(_robot, 0);
      if (q) {
        if( debug_configs || MCDEBUG ){
          cout << "FOUND Approach Free Config!!!!!" << endl;
        }
        //           configs.setApproachFreeConfig( configs.getOpenConfig() );
        configs.setApproachFreeConfig(q);
        p3d_destroy_config(_robot, q);
        q = NULL;
        
        setSafetyDistance(_robot, getSafetyDistanceValue());
        q = getApproachGraspConf(object, armId, grasp, configs.getGraspConfig(), tAtt);
        setSafetyDistance(_robot, 0);
        if (q) {
          if( debug_configs || MCDEBUG ){
            cout << "FOUND Approach Grasp Config!!!!!" << endl;
          }
          configs.setApproachGraspConfig(q);
          p3d_destroy_config(_robot, q);
          q = NULL;
          
          deactivateCcCntrts(_robot, armId);
          if( debug_configs || MCDEBUG ){
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

//! Searches for configurations used in the PlaceType Motions
//! these configurations will be used to 
MANIPULATION_TASK_MESSAGE ManipulationConfigs::getHoldingOpenApproachExtractConfs(p3d_rob* object, std::vector<double> &objGoto, p3d_rob* placement, int armId, gpGrasp& grasp, p3d_matrix4 tAtt,  ManipulationData& configs) const 
{
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  double confCost = -1;
  
  configPt qGoal = getFreeHoldingConf(object, armId, grasp, tAtt, confCost, objGoto, placement);
  
  // Retrieves the computed attached matrix
  p3d_matrix4 tAttTmp;
  configs.getAttachFrame(tAttTmp);
  if(tAttTmp[0][0] == 0 && tAttTmp[0][1] == 0 && tAttTmp[0][2] == 0 && tAttTmp[0][3] == 0){
    configs.setAttachFrame(tAtt);
  }
  
  if (qGoal)
  {
    // Compute the approch grasp config for the placement grasp
    configPt approachGraspConfigPlacement = getApproachGraspConf(object, armId, grasp, qGoal, tAtt);
    if(approachGraspConfigPlacement)
    {
      configPt openConfigPlacement = getOpenGraspConf(object, armId, grasp, qGoal);
      if(openConfigPlacement)
      {
        configPt approachFreeConfigPlacement = getApproachFreeConf(object, armId, grasp, qGoal, tAtt);
        // Compute the approch grasp config for the start config.
        if(approachFreeConfigPlacement)
        {
          configs.setApproachGraspConfig(approachGraspConfigPlacement);
          configs.setOpenConfig(openConfigPlacement);
          configs.setApproachFreeConfig(approachFreeConfigPlacement);
          configs.setGraspConfig(qGoal);
          configs.setGraspConfigCost(confCost);
          configs.setGrasp(&grasp);
          p3d_destroy_config(_robot, approachFreeConfigPlacement );
        }else{
          status = MANIPULATION_TASK_NO_PLACE;
        }
        p3d_destroy_config(_robot, openConfigPlacement );
      }else{
        status = MANIPULATION_TASK_NO_PLACE;
      }
      p3d_destroy_config(_robot, approachGraspConfigPlacement );
    }else{
      status = MANIPULATION_TASK_NO_PLACE;
    }
    p3d_destroy_config(_robot, qGoal );
  } else {
    status = MANIPULATION_TASK_NO_PLACE;
  }
  return status;
}

//! Main function that iterates through the grasp list
//! \param the arm Id
//! \param object the object to be grasped
//! \param configs the configurations 
MANIPULATION_TASK_MESSAGE ManipulationConfigs::findArmGraspsConfigs(int armId, p3d_rob* object, gpGrasp& grasp, ManipulationData& configs) const
{
  MANIPULATION_TASK_MESSAGE status = MANIPULATION_TASK_OK;
  
  if (grasp.ID != 0){ //a valid grasp is given
    ManipulationData data(_robot);
    p3d_matrix4 tAtt;
    ManipulationUtils::fixAllHands(_robot, NULL, true);
    status = getGraspOpenApproachExtractConfs(object, armId, grasp, tAtt, data);
    if(status == MANIPULATION_TASK_OK){
      configs = data;
      //             break;
      if(MCDEBUG){
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
      if (armId == -2) { //Compute closest arm
        if ((armId = getClosestWristToTheObject(_robot, object)) == -2) {
          printf("ERROR findArmGraspsConfigs on getClosestWristToTheObject");
          return MANIPULATION_TASK_NO_GRASP;
        }
      }
      gpHand_properties armHandProp = (*_robot->armManipulationData)[armId].getHandProperties();
      
      // Compute or reads the grasp list if available
      list<gpGrasp> graspList;
      graspList.clear();
      gpGet_grasp_list(object->name, armHandProp.type, graspList);
      
      status = MANIPULATION_TASK_NO_GRASP;
      
      if (graspList.size() != 0)  
      {
        int counter = 0;
        bool validConf = false;
        
        for (list<gpGrasp>::iterator iter = graspList.begin(); 
             iter != graspList.end(); iter++)
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
              cout << "Config Cost = " << data.getGraspConfigCost() << endl;
              if(data.getGraspConfigCost() < _robot->configCostThreshold){
                break;
              }
            }
            if(MCDEBUG){
              ManipulationUtils::copyConfigToFORM(_robot, data.getGraspConfig());
            }
          }else{
            data.clear();
          }
          counter++;
        }
        if (MCDEBUG) {
          printf("NbTest Before Config : %d\n", counter);
        }
        if (validConf) {
          status = MANIPULATION_TASK_OK;
        }
      }
    }
  }
  
  if (MCDEBUG && (status == MANIPULATION_TASK_OK) ) {
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
