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
  vector<gpHand_properties> handProp = InitHandProp(armId);
  
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
    double configCost = getCollisionFreeGraspAndApproach(objectPos, handProp[armId], (*iter), armId + 1, tAtt, &graspConfig, &approachConfig);
    if(configCost != -1){
      data->setAttachFrame(tAtt);
      configPt openConfig = data->getOpenConfig();
      p3d_copy_config_into(_robot, graspConfig, &openConfig);
      gpSet_grasp_open_configuration(_robot, handProp[armId], (*iter), openConfig, armId + 1);
      double cost = ((1 - (((*iter).quality - _armMinMaxCost[armId][0]) / _armMinMaxCost[armId][1])) + configCost) / 2;
      cout << cost << endl;
      (*iter).IKscore = configCost;
      data->setGrasp(new gpGrasp(*iter));
      data->setGraspConfigCost(configCost);
      configMap.insert(pair<double, ManipulationData*> (cost, data));
    }else{
      delete(data);
    }
  }
  p3d_desactivateCntrt(_robot, _robot->ccCntrts[armId]);
  _handsGraspsConfig.insert(pair<int, map<double, ManipulationData*, std::less<double> > > (armId, configMap));
  return nbGraspConfigs;
}

double Manipulation::getCollisionFreeGraspAndApproach(p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig){
  p3d_matrix4 handFrame, fictive;
  p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, _robot->ccCntrts[whichArm - 1]->Tatt2, tAtt);
  fictive[0][0] = fictive[0][1] = fictive[0][2] = 0;
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpSet_grasp_configuration(_robot, handProp, grasp, whichArm);
  gpFix_hand_configuration(_robot, handProp, whichArm);
  if(whichArm == 1){
    q = setTwoArmsRobotGraspPosWithoutBase(_robot, objectPos, tAtt, fictive, FALSE, whichArm - 1, true);
  }else if(whichArm == 2){
    q = setTwoArmsRobotGraspPosWithoutBase(_robot, objectPos, fictive, tAtt, FALSE, whichArm - 1, true);
  }
  if(q){
    double confCost = setRobotArmsRest(_robot, objectPos, whichArm - 1, tAtt, _robot->ROBOT_POS, q);
    //double confCost = computeRobotConfCostSpecificArm(_robot, _robot->ROBOT_POS, q, 1 - (whichArm - 1));
    p3d_desactivateCntrt(_robot, _robot->ccCntrts[whichArm - 1]);
    gpSet_grasp_configuration(_robot, handProp, grasp, q, whichArm);
    p3d_copy_config_into(_robot, q, graspConfig);
    //Check the rest configuration of the hand
    gpSet_grasp_open_configuration(_robot, handProp, grasp, q, whichArm);
    p3d_set_and_update_this_robot_conf(_robot, q);
    g3d_draw_allwin_active();
    if(!p3d_col_test()){
      p3d_copy_config_into(_robot, q, approachConfig);
      p3d_destroy_config(_robot, q);
      return confCost; //success
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
  p3d_destroy_config(_robot, q);
  return -1;
}

void Manipulation::drawSimpleGraspConfigs(){
  static map < int, map<double, ManipulationData*, std::less<double> > >::iterator armIter = _handsGraspsConfig.begin();
  static map < double, ManipulationData* >::iterator dataIter = (*armIter).second.begin();
  if (dataIter == (*armIter).second.end()) {
    armIter++;
    if (armIter == _handsGraspsConfig.end()) {
      armIter = _handsGraspsConfig.begin();
    }
    dataIter = (*armIter).second.begin();
  }
  cout << "Config cost (Grasp + config) : " <<  (*dataIter).first << endl;
  p3d_set_and_update_this_robot_conf(_robot, (*dataIter).second->getGraspConfig());
  g3d_draw_allwin_active();
  dataIter++;
}
void Manipulation::drawDoubleGraspConfigs(){
  static list <DoubleGraspData*>::iterator it = _handsDoubleGraspsConfigs.begin();
  if (it == _handsDoubleGraspsConfigs.end()) {
    it = _handsDoubleGraspsConfigs.begin();
  }
  p3d_set_and_update_this_robot_conf(_robot, (*it)->getConfig());
  g3d_draw_allwin_active();
  it++;
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
    cout << "There is more or less than two closed Chain constraints" << endl;
  }
}

void Manipulation::computeDoubleGraspConfigList(){
  static list<gpDoubleGrasp> doubleGraspList;
  static list<gpDoubleGrasp>::iterator iter;
  p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_GOTO);
  if(!doubleGraspList.size()){
    gpDouble_grasp_generation((p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_RIGHT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_LEFT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), *(getGraspListFromMap(0)), *(getGraspListFromMap(1)), doubleGraspList);
    cout << "graspList" << doubleGraspList.size() << endl;
    iter = doubleGraspList.begin();
    
    vector<gpHand_properties> handProp = InitHandProp(-1);
    for(list<gpDoubleGrasp>::iterator itDouble = doubleGraspList.begin(); itDouble != doubleGraspList.end(); itDouble++){
      DoubleGraspData* data = new DoubleGraspData(_robot);
      configPt dGraspConfig = data->getConfig();
      if(getCollisionFreeDoubleGraspAndApproach(_exchangeMat, handProp, (*itDouble), &dGraspConfig)){
        data->setDoubleGrasp((*itDouble));
        _handsDoubleGraspsConfigs.push_back(data);
        showConfig(dGraspConfig);
      }else{
        delete(data);
      }
    }
    cout << "Valid Grasps" << _handsDoubleGraspsConfigs.size() << endl;
  }
  if (iter == doubleGraspList.end()) {
    iter = doubleGraspList.begin();
  }
  gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_RIGHT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), (*iter).grasp1);
  gpSet_robot_hand_grasp_configuration((p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_LEFT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), (*iter).grasp2);
  g3d_draw_allwin_active();
  iter++;
}

int Manipulation::getCollisionFreeDoubleGraspAndApproach(p3d_matrix4 objectPos, vector<gpHand_properties> handProp, gpDoubleGrasp doubleGrasp, configPt* doubleGraspConfig){

  p3d_matrix4 handFrame, rTatt, lTatt;
  p3d_mat4Mult(doubleGrasp.grasp1.frame, handProp[0].Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, _robot->ccCntrts[0]->Tatt2, rTatt);
  p3d_mat4Mult(doubleGrasp.grasp2.frame, handProp[1].Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, _robot->ccCntrts[1]->Tatt2, lTatt);
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpSet_grasp_configuration(_robot, handProp[0], doubleGrasp.grasp1, 1);
  gpFix_hand_configuration(_robot, handProp[0], 1);
  gpSet_grasp_configuration(_robot, handProp[1], doubleGrasp.grasp2, 2);
  gpFix_hand_configuration(_robot, handProp[1], 2);
  q = setTwoArmsRobotGraspPosWithoutBase(_robot, _exchangeMat, rTatt, lTatt, TRUE, - 1, true);
  if(q){
    p3d_desactivateCntrt(_robot, _robot->ccCntrts[0]);
    p3d_desactivateCntrt(_robot, _robot->ccCntrts[1]);
    gpSet_grasp_configuration(_robot, handProp[0], doubleGrasp.grasp1, q, 1);
    gpSet_grasp_configuration(_robot, handProp[1], doubleGrasp.grasp2, q, 2);
    p3d_copy_config_into(_robot, q, doubleGraspConfig);
    p3d_destroy_config(_robot, q);
    return 1; //success
  }
  p3d_destroy_config(_robot, q);
  return 0;
}

vector<gpHand_properties> Manipulation::InitHandProp(int armId){
  vector<gpHand_properties> handProp;
  if(_robot->nbCcCntrts == 2){
    gpHand_properties prop1;
    prop1.initialize(GP_SAHAND_RIGHT);
    handProp.push_back(prop1);
    gpHand_properties prop2;
    prop2.initialize(GP_SAHAND_LEFT);
    handProp.push_back(prop2);
    
    switch (armId){
      case -1:{
        break;
      }
      case 0:{
        gpFix_hand_configuration(_robot, handProp[1], 2);
        gpSet_hand_rest_configuration(_robot, handProp[1], 2);
        break;
      }
      case 1:{
        gpFix_hand_configuration(_robot, handProp[0], 1);
        gpSet_hand_rest_configuration(_robot, handProp[0], 1);
        break;
      }
      default:{
        cout << "The arm id is not valid." << endl;
      }
    }
    gpDeactivate_hand_selfcollisions(_robot, 1);
    gpDeactivate_hand_selfcollisions(_robot, 2);
  }else {
    cout << "There is more or less than two closed Chain constraints" << endl;
  }
  return handProp;
}

list<gpGrasp>* Manipulation::getGraspListFromMap(int armId){
  list<gpGrasp>* graspList = new list<gpGrasp>();
  for(map<double, ManipulationData* >::iterator it = _handsGraspsConfig[armId].begin(); it != _handsGraspsConfig[armId].end(); it++){
    gpGrasp* grasp = it->second->getGrasp();
    graspList->push_back(*grasp);
  }
  return graspList;
}
