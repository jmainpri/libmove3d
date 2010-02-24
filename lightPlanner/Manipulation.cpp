#include "Manipulation.h"
#include <iostream>
#include "lightPlanner.h"
#include "lightPlannerApi.h"
#include "robotPos.h"
#include "Collision-pkg.h"

using namespace std;

Manipulation::Manipulation(p3d_rob * robot){
  if(robot->nbCcCntrts != 2){
    cout << "Error: the number of arm  != 2" << endl;
    return;
  }
  _robot = robot;
  p3d_set_object_to_carry(_robot, (char*)GP_OBJECT_NAME_DEFAULT);
  for(int i = 0; i < robot->nbCcCntrts; i++){
    getHandGraspsMinMaxCosts(i, &_armMinMaxCost[i][0], &_armMinMaxCost[i][1]);
  }
  p3d_mat4Copy(p3d_mat4IDENTITY, _exchangeMat);
  p3d_col_stop();
  p3d_col_start(p3d_col_mode_kcd);
}

Manipulation::~Manipulation(){
  for(map<int, map<double, ManipulationData*, std::less<double> > >::iterator iter = _handsGraspsConfig.begin(); iter != _handsGraspsConfig.end(); iter++){
    for(map<double, ManipulationData* >::iterator it = iter->second.begin(); it != iter->second.end(); it++){
      delete(it->second);
    }
  }
}

int Manipulation::findAllArmsGraspsConfigs(p3d_matrix4 objectStartPos, p3d_matrix4 objectEndPos){
  int nbGraspConfigs = 0;
  if(_robot->nbCcCntrts != 2){
    cout << "Error: the number of arm  != 2" << endl;
    return 0;
  }
  int closestWrist = getClosestWristToTheObject(_robot);
//  switchBBActivationForGrasp();
  nbGraspConfigs = findAllSpecificArmGraspsConfigs(closestWrist, objectStartPos);
  nbGraspConfigs = findAllSpecificArmGraspsConfigs(1 - closestWrist, objectEndPos);
//  switchBBActivationForGrasp();
  for(unsigned int i = 0; i < _handsGraspsConfig.size(); i++){
    cout << _handsGraspsConfig[i].size() << endl;
  }
  return nbGraspConfigs;
}

int Manipulation::findAllSpecificArmGraspsConfigs(int armId, p3d_matrix4 objectPos){
  int nbGraspConfigs = 0;
  gpHand_properties handProp;
  switch (armId){
    case 0:{
      handProp.initialize(GP_SAHAND_RIGHT);
      gpHand_properties otherHand;
      otherHand.initialize(GP_SAHAND_LEFT);
      gpFix_hand_configuration(_robot, otherHand, 2);
      gpSet_hand_rest_configuration(_robot, otherHand, 2);
      break;
    }
    case 1:{
      handProp.initialize(GP_SAHAND_LEFT);
      gpHand_properties otherHand;
      otherHand.initialize(GP_SAHAND_RIGHT);
      gpFix_hand_configuration(_robot, otherHand, 1);
      gpSet_hand_rest_configuration(_robot, otherHand, 1);
      break;
    }
    default:{
      cout << "The arm id is not valid." << endl;
    }
  }
  gpDeactivate_hand_selfcollisions(_robot, 1);
  gpDeactivate_hand_selfcollisions(_robot, 2);
  list<gpGrasp> graspList;
  gpGet_grasp_list_SAHand(GP_OBJECT_NAME_DEFAULT, armId + 1, graspList);
  std::map<double, ManipulationData*, std::less<double> > configMap;
  
  //For each grasp, get the tAtt and check the collision
  for(list<gpGrasp>::iterator iter = graspList.begin(); iter != graspList.end(); iter++){
    ManipulationData* data = new ManipulationData(_robot);
    configPt graspConfig = data->getGraspConfig(), approachConfig = data->getApproachConfig();
    p3d_matrix4 tAtt;
    int nbTest = getCollisionFreeGraspAndApproach(objectPos, handProp, (*iter), armId + 1, tAtt, &graspConfig, &approachConfig);
    if(nbTest && nbTest != -1){
      data->setAttachFrame(tAtt);
      configPt openConfig = data->getOpenConfig();
      p3d_copy_config_into(_robot, graspConfig, &openConfig);
      gpSet_grasp_open_configuration(_robot, handProp, (*iter), openConfig, armId + 1);
      double cost = (1 - (((*iter).quality - _armMinMaxCost[armId][0]) / _armMinMaxCost[armId][1])) + nbTest / _maxColGrasps;
      data->setGrasp(new gpGrasp(*iter));
      configMap.insert(pair<double, ManipulationData*> (cost, data));
    }else if (nbTest == -1){
      delete(data);
      return 0;
    }else{
      delete(data);
    }
  }
  p3d_desactivateCntrt(_robot, _robot->ccCntrts[armId]);
  _handsGraspsConfig.insert(pair<int, map<double, ManipulationData*, std::less<double> > > (armId, configMap));
  return nbGraspConfigs;
}

int Manipulation::getCollisionFreeGraspAndApproach(p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig){
  p3d_matrix4 handFrame, fictive;
  p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, _robot->ccCntrts[whichArm - 1]->Tatt2, tAtt);
  fictive[0][0] = fictive[0][1] = fictive[0][2] = 0;
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpSet_grasp_configuration(_robot, handProp, grasp, whichArm);
  gpFix_hand_configuration(_robot, handProp, whichArm);
  if(whichArm == 1){
    q = setTwoArmsRobotGraspPosWithoutBase(_robot, objectPos, tAtt, fictive, whichArm - 1, false);
  }else if(whichArm == 2){
    q = setTwoArmsRobotGraspPosWithoutBase(_robot, objectPos, fictive, tAtt, whichArm - 1, false);
  }
  if(q){
    p3d_desactivateCntrt(_robot, _robot->ccCntrts[whichArm - 1]);
    gpSet_grasp_configuration(_robot, handProp, grasp, q, whichArm);
    p3d_copy_config_into(_robot, q, graspConfig);
    //Check the rest configuration of the hand
    gpSet_grasp_open_configuration(_robot, handProp, grasp, q, whichArm);
    p3d_set_and_update_this_robot_conf(_robot, q);
    g3d_draw_allwin_active();
    if(!p3d_col_test()){
      p3d_copy_config_into(_robot, q, approachConfig);
      return 1; //success
      //Check the approach configuration of the arm
//      tAtt[1][3] -= 0.1;
//      q = setTwoArmsRobotGraspPosWithoutBase(_robot, objectPos, fictive, tAtt, whichArm - 1, false);
//      tAtt[1][3] += 0.1;
//      p3d_desactivateCntrt(_robot, _robot->ccCntrts[whichArm - 1]);
//      if(q){
//        gpSet_grasp_open_configuration(_robot, handProp, grasp, q, whichArm);
//        p3d_copy_config_into(_robot, q, approachConfig);
//        return 1; //success
//      }
    }
  }
  return 0;
}

void Manipulation::getHandGraspsMinMaxCosts(int armId, double* minCost, double* maxCost){
  list<gpGrasp> graspList;
  gpGet_grasp_list_SAHand(GP_OBJECT_NAME_DEFAULT, armId + 1, graspList);
  list<gpGrasp>::iterator iter = graspList.begin();
  *maxCost = (*iter).quality;
  iter = graspList.end();
  iter--;
  *minCost = (*iter).quality;
}

void Manipulation::computeExchangeMat(configPt startConfig, configPt gotoConfig){
  if(_robot->nbCcCntrts == 2){
    p3d_matrix4 objectInit, objectEnd, arm0Init, arm0End, arm1Init, arm1End;
    p3d_set_and_update_this_robot_conf(_robot, startConfig);
    p3d_mat4Copy(_robot->curObjectJnt->abs_pos, objectInit);
    p3d_mat4Copy(_robot->ccCntrts[0]->pasjnts[_robot->ccCntrts[0]->npasjnts - 1]->abs_pos, arm0Init);
    p3d_mat4Copy(_robot->ccCntrts[1]->pasjnts[_robot->ccCntrts[1]->npasjnts - 1]->abs_pos, arm1Init);
    p3d_set_and_update_this_robot_conf(_robot, gotoConfig);
    p3d_mat4Copy(_robot->curObjectJnt->abs_pos, objectEnd);
    p3d_mat4Copy(_robot->ccCntrts[0]->pasjnts[_robot->ccCntrts[0]->npasjnts - 1]->abs_pos, arm0End);
    p3d_mat4Copy(_robot->ccCntrts[1]->pasjnts[_robot->ccCntrts[1]->npasjnts - 1]->abs_pos, arm1End);
    
    findBestExchangePosition2((p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), objectInit, objectEnd, arm0Init, arm0End, arm1Init, arm1End, _exchangeMat);
  }else {
    cout << "There is more or less than two closec Chain constraints" << endl;
  }
}

void Manipulation::computeDoubleGraspConfigList(){
  static list<gpDoubleGrasp> doubleGraspList;
  static list<gpDoubleGrasp>::iterator iter;
  p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_GOTO);
  if(!doubleGraspList.size()){
    p3d_col_stop();
    p3d_col_start(p3d_col_mode_pqp);
    gpDouble_grasp_generation((p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_RIGHT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_LEFT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), *(getGraspListFromMap(0)), *(getGraspListFromMap(1)), doubleGraspList);
    p3d_col_stop();
    p3d_col_start(p3d_col_mode_kcd);
    cout << doubleGraspList.size() << endl;
    iter = doubleGraspList.begin();
    for(list<gpDoubleGrasp>::iterator itDouble = doubleGraspList.begin(); itDouble != doubleGraspList.end(); itDouble++){
      
    }
  }
  if (iter == doubleGraspList.end()) {
    iter = doubleGraspList.begin();
  }
  gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_RIGHT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), (*iter).grasp1);
  gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_LEFT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), (*iter).grasp2);
  g3d_draw_allwin_active();
  iter++;
}

list<gpGrasp>* Manipulation::getGraspListFromMap(int armId){
  list<gpGrasp>* graspList = new list<gpGrasp>();
  for(map<double, ManipulationData* >::iterator it = _handsGraspsConfig[armId].begin(); it != _handsGraspsConfig[armId].end(); it++){
    gpGrasp* grasp = it->second->getGrasp();
    graspList->push_back(*grasp);
  }
  return graspList;
}
