#include "Manipulation.h"
#include <iostream>
#include "lightPlanner.h"
#include "lightPlannerApi.h"
#include "robotPos.h"
#include "Collision-pkg.h"
#include "Util-pkg.h"
#include "p3d_chanEnv_proto.h"
using namespace std;

Manipulation::Manipulation(p3d_rob * robot){
  if(robot->nbCcCntrts != 2){
    cout << "Error: the number of arm  != 2" << endl;
    return;
  }
  _robot = robot;
  _offlineGraph = NULL;
  p3d_set_object_to_carry(_robot, (char*)GP_OBJECT_NAME_DEFAULT);
  p3d_mat4Copy(p3d_mat4IDENTITY, _exchangeMat);
}

Manipulation::~Manipulation(){
  for(map<int, map<int, ManipulationData*, std::less<int> > >::iterator iter = _handsGraspsConfig.begin(); iter != _handsGraspsConfig.end(); iter++){
    for(map<int, ManipulationData* >::iterator it = iter->second.begin(); it != iter->second.end(); it++){
      delete(it->second);
    }
  }
  if (_offlineGraph) {
    p3d_del_graph(_offlineGraph);
  }
}

void Manipulation::clear(){
  for(map<int, map<int, ManipulationData*, std::less<int> > >::iterator iter = _handsGraspsConfig.begin(); iter != _handsGraspsConfig.end(); iter++){
    for(map<int, ManipulationData* >::iterator it = iter->second.begin(); it != iter->second.end(); it++){
      delete(it->second);
    }
  }
  _handsGraspsConfig.clear();
  for(list <DoubleGraspData*>::iterator it = _handsDoubleGraspsConfigs.begin(); it != _handsDoubleGraspsConfigs.end(); it++){
    delete((*it));
  }
  _handsDoubleGraspsConfigs.clear();
}

void Manipulation::computeOfflineRoadmap(){
  deleteAllGraphs();
  p3d_matrix4 farObjectPos;
  p3d_mat4PosReverseOrder(farObjectPos, _robot->curObjectJnt->dof_data[0].vmax, _robot->curObjectJnt->dof_data[1].vmax, _robot->curObjectJnt->dof_data[2].vmax, 0, 0, 0);
  InitHandProp(0);
  InitHandProp(1);
  preComputeGotoObject(_robot, farObjectPos);
  vector<double> datas;
  datas.push_back(_robot->GRAPH->nnode);
  datas.push_back(_robot->GRAPH->time);
  _statDatas.push_back(datas);
} 

p3d_traj* Manipulation::computeRegraspTask(configPt startConfig, configPt gotoConfig, string offlineFile, int whichTest){
  vector<double> statDatas;
  double tu = 0, ts = 0;
  p3d_graph* loadedGraph = NULL;
  DoubleGraspData* dgData = NULL;
  gpDoubleGrasp doubleGrasp;
  gpGrasp firstGrasp, secondGrasp;
  ManipulationData* firstGraspData = NULL, *secondGraspData = NULL;
  gpHand_properties prop1, prop2;
  
  p3d_matrix4 objectStartPos, objectEndPos;
  deactivateCcCntrts(_robot, -1);
  for (int i  = 0; i < _robot->nbCcCntrts; i++) {
    desactivateTwoJointsFixCntrt(_robot, _robot->curObjectJnt, _robot->ccCntrts[i]->pasjnts[_robot->ccCntrts[i]->npasjnts - 1]);
  }
  //find Single Grasp configurations
  p3d_set_and_update_this_robot_conf(_robot, startConfig);
  p3d_mat4Copy(_robot->curObjectJnt->jnt_mat, objectStartPos);
  p3d_set_and_update_this_robot_conf(_robot, gotoConfig);
  p3d_mat4Copy(_robot->curObjectJnt->jnt_mat, objectEndPos);
  p3d_set_and_update_this_robot_conf(_robot, startConfig);
  //Find the closest arm to the start position of the object to start planning with
  int closestWrist = getClosestWristToTheObject(_robot);
  
  while (whichTest != 0 && _handsDoubleGraspsConfigs.size() == 0) {
    clear();
    computeRegraspTask(p3d_copy_config(_robot, startConfig), p3d_copy_config(_robot, gotoConfig), offlineFile, 0);
  }
  
  if (_handsDoubleGraspsConfigs.size() > 0) {
    p3d_copy_config_into(_robot, startConfig, &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    dgData = (*_handsDoubleGraspsConfigs.begin());
    doubleGrasp = dgData->getDoubleGrasp();
    //Get the datas corresponding to the double grasp
    if (closestWrist == 0) {
      firstGrasp = doubleGrasp.grasp1;
      secondGrasp = doubleGrasp.grasp2;
    }else {
      firstGrasp = doubleGrasp.grasp2;
      secondGrasp = doubleGrasp.grasp1;
    }
    firstGraspData = _handsGraspsConfig[closestWrist][firstGrasp.ID];
    secondGraspData = _handsGraspsConfig[1 - closestWrist][secondGrasp.ID];
    prop1.initialize(firstGraspData->getGrasp()->hand_type);
    prop2.initialize(secondGraspData->getGrasp()->hand_type);
    gpDeactivate_hand_selfcollisions(XYZ_ROBOT, 1);
    gpDeactivate_hand_selfcollisions(XYZ_ROBOT, 2);
    if (offlineFile.compare("")) {
      p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
      loadedGraph = XYZ_GRAPH;
      statDatas.push_back(_robot->GRAPH->nnode);
      statDatas.push_back(_robot->GRAPH->time);
    }
    InitHandProp(0);
    InitHandProp(1);
  }

  switch (whichTest) {
    case 0:{
      ChronoOn();
      findAllArmsGraspsConfigs(objectStartPos, objectEndPos);
      ChronoMicroTimes(&tu, &ts);
      ChronoPrint("Single Grasp configs: ");
      ChronoOff();
      statDatas.push_back(_handsGraspsConfig[0].size());
      statDatas.push_back(_handsGraspsConfig[1].size());
      statDatas.push_back(tu);
      //find Double Grasp configurations
      ChronoOn();
      computeExchangeMat(startConfig, gotoConfig);
      computeDoubleGraspConfigList();
      ChronoMicroTimes(&tu, &ts);
      ChronoPrint("Double Grasp configs: ");
      ChronoOff();
      statDatas.push_back(_handsDoubleGraspsConfigs.size());
      statDatas.push_back(tu);      
      break;
    }
    case 1:{
      //grasp the object
      cout << "Open pick config" << endl;
      deactivateCcCntrts(_robot, -1);
      p3d_set_and_update_this_robot_conf(_robot, startConfig);
      if (offlineFile.compare("")) {
        _robot->preComputedGraphs[1] = loadedGraph;
        p3d_jnt* jnts[1] = {_robot->curObjectJnt};
        correctGraphForNewFixedJoints(_robot->preComputedGraphs[1], startConfig, 1, jnts);
      }
      p3d_set_and_update_this_robot_conf(_robot, startConfig);
      p3d_traj* approachTraj = gotoObjectByConf(_robot, objectStartPos, firstGraspData->getApproachConfig());
      if (approachTraj) {
        if (offlineFile.compare("")) {
	  statDatas.push_back(_robot->GRAPH->nnode);
          statDatas.push_back(_robot->GRAPH->time);
          int nbTests = checkTraj(approachTraj, _robot->preComputedGraphs[1]);
	  statDatas.push_back(nbTests);
	  _robot->preComputedGraphs[1] = NULL;
        }
      }else {
        statDatas.push_back(-999);
        break;
      }
      statDatas.push_back(_robot->GRAPH->nnode);
      statDatas.push_back(_robot->GRAPH->time);
      
      //Close the hand
      cout << "Grasp pick config" << endl;
      p3d_copy_config_into(_robot, firstGraspData->getApproachConfig(), &(_robot->ROBOT_POS));
      gpActivate_hand_selfcollisions(_robot, closestWrist + 1);
      if(closestWrist == 0){
        gpUnFix_hand_configuration(_robot, prop1, 1);
      }else {
        gpUnFix_hand_configuration(_robot, prop2, 2);
      }
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      p3d_traj* graspTraj = gotoObjectByConf(_robot, objectStartPos, firstGraspData->getGraspConfig());
      if (graspTraj) {
        p3d_concat_traj(approachTraj, graspTraj);
      }else {
        statDatas.push_back(-888);
      }
      break;
    }
    case 2:{
      //Compute open approach double grasp traj
      cout << "open approach double grasp config" << endl;
      configPt doubleGraspConfigApproach = p3d_copy_config(_robot, dgData->getConfig());
      p3d_set_and_update_this_robot_conf(_robot, doubleGraspConfigApproach);
      gpDeactivate_hand_selfcollisions(_robot, closestWrist + 1);
      if(closestWrist == 0){
        gpFix_hand_configuration(_robot, prop1, 1);
        gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 2);
        gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp2, doubleGraspConfigApproach, 2);
      }else {
        gpFix_hand_configuration(_robot, prop2, 2);
        gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 1);
        gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp1, doubleGraspConfigApproach, 1);
      }
      //get the object exchange pos
      p3d_matrix4 objectExchangePos;
      p3d_set_and_update_this_robot_conf(_robot, doubleGraspConfigApproach);
      p3d_mat4Copy(_robot->curObjectJnt->jnt_mat, objectExchangePos);
      fixJoint(_robot, _robot->curObjectJnt, objectStartPos);
      p3d_copy_config_into(_robot, firstGraspData->getGraspConfig(), &(_robot->ROBOT_POS));
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      if (offlineFile.compare("")) {
        //p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
        _robot->preComputedGraphs[3] = loadedGraph;
        if(closestWrist == 0){
          correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 0, doubleGrasp.grasp1 , 2, doubleGrasp.grasp2, 1, 1);
        }else {
          correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 2, doubleGrasp.grasp1 , 0, doubleGrasp.grasp2, 1, 2);
        }
      }
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      p3d_traj *carryToExchangeApporach = carryObjectByConf(_robot, objectExchangePos, doubleGraspConfigApproach, closestWrist, false);
      if (carryToExchangeApporach) {
        if (offlineFile.compare("")) {
          statDatas.push_back(_robot->GRAPH->nnode);
          statDatas.push_back(_robot->GRAPH->time);
          int nbTests = checkTraj(carryToExchangeApporach, _robot->preComputedGraphs[3]);
          statDatas.push_back(nbTests);
          _robot->preComputedGraphs[3] = NULL;
        }
      }else {
        statDatas.push_back(-999);
        break;
      }
      statDatas.push_back(_robot->GRAPH->nnode);
      statDatas.push_back(_robot->GRAPH->time);
      
      //Compute double grasp traj
      cout << "Double Grasp config" << endl;
      if(closestWrist == 0){
        gpUnFix_hand_configuration(_robot, prop2, 2);
      }else {
        gpUnFix_hand_configuration(_robot, prop1, 1);
      }
      gpActivate_hand_selfcollisions(_robot, (1 - closestWrist) + 1);
      p3d_copy_config_into(_robot, doubleGraspConfigApproach, &(_robot->ROBOT_POS));
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      p3d_copy_config_into(_robot, dgData->getConfig(), &(_robot->ROBOT_GOTO));
      p3d_traj *carryToExchange = carryObjectByConf(_robot, objectExchangePos, dgData->getConfig(), closestWrist, false);
      if (carryToExchange) {
        p3d_concat_traj(carryToExchangeApporach, carryToExchange);
      }else {
        statDatas.push_back(-888);
      }
      break;
    }
    case 3:{
      //Compute open Deposit traj 
      cout << "Double Grasp open config deposit" << endl;
      configPt doubleGraspConfigDeposit = p3d_copy_config(_robot, dgData->getConfig());
      gpDeactivate_hand_selfcollisions(_robot, (1 - closestWrist) + 1);
      gpActivate_hand_selfcollisions(_robot, closestWrist + 1);
      p3d_set_and_update_this_robot_conf(_robot, doubleGraspConfigDeposit);
      if(closestWrist == 0){
        gpFix_hand_configuration(_robot, prop2, 2);
        gpUnFix_hand_configuration(_robot, prop1, 1);
        gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 1);
        gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp1, doubleGraspConfigDeposit, 1);
      }else {
        gpFix_hand_configuration(_robot, prop1, 1);
        gpUnFix_hand_configuration(_robot, prop2, 2);
        gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 2);
        gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp2, doubleGraspConfigDeposit, 2);
      }
      p3d_copy_config_into(_robot, dgData->getConfig(), &(_robot->ROBOT_POS));
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      p3d_traj *carryToOpenDeposit = carryObjectByConf(_robot, objectEndPos, doubleGraspConfigDeposit, 1 - closestWrist, false);
      if (!carryToOpenDeposit) {
        statDatas.push_back(-888);
        break;
      }
      //Compute deposit Traj
      cout << "Deposit traj" << endl;
      if(closestWrist == 0){
        gpFix_hand_configuration(_robot, prop1, 1);
      }else {
        gpFix_hand_configuration(_robot, prop2, 2);
      }
      gpDeactivate_hand_selfcollisions(_robot, closestWrist + 1);
      p3d_copy_config_into(_robot, doubleGraspConfigDeposit, &(_robot->ROBOT_POS));
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      if (offlineFile.compare("")) {
        //p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
        _robot->preComputedGraphs[3] = loadedGraph;
        if(closestWrist == 0){
          correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 2, doubleGrasp.grasp1 , 0, doubleGrasp.grasp2, 1, 2);
        }else {
          correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 0, doubleGrasp.grasp1 , 2, doubleGrasp.grasp2, 1, 1);
        }
      }
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      p3d_traj *carryToDeposit = carryObjectByConf(_robot, objectEndPos, secondGraspData->getGraspConfig(), 1 - closestWrist, false);
      fixJoint(_robot, _robot->curObjectJnt, objectStartPos);
      if (carryToDeposit) {
        if (offlineFile.compare("")) {
          statDatas.push_back(_robot->GRAPH->nnode);
          statDatas.push_back(_robot->GRAPH->time);
          int nbTests = checkTraj(carryToDeposit, _robot->preComputedGraphs[3]);
          statDatas.push_back(nbTests);
          _robot->preComputedGraphs[3] = NULL;
        }
        p3d_concat_traj(carryToOpenDeposit, carryToDeposit);
      }else {
        statDatas.push_back(-999);
      }
      statDatas.push_back(_robot->GRAPH->nnode);
      statDatas.push_back(_robot->GRAPH->time);
      break;
    }
    case 4:{
      //Compute deposit Traj
      cout << "End open traj" << endl;
      if(closestWrist == 0){
        gpUnFix_hand_configuration(_robot, prop2, 2);
      }else {
        gpUnFix_hand_configuration(_robot, prop1, 1);
      }
      p3d_copy_config_into(_robot, secondGraspData->getGraspConfig(), &(_robot->ROBOT_POS));
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      p3d_traj *endOpenTraj = gotoObjectByConf(_robot, objectEndPos, secondGraspData->getApproachConfig());
      if(!endOpenTraj){
        statDatas.push_back(-888);
        break;
      }
      //Goto goal position
      cout << "End traj" << endl;
      if(closestWrist == 0){
        gpFix_hand_configuration(_robot, prop2, 2);
      }else {
        gpFix_hand_configuration(_robot, prop1, 1);
      }
      gpDeactivate_hand_selfcollisions(_robot, 1);
      gpDeactivate_hand_selfcollisions(_robot, 2);
      p3d_copy_config_into(_robot, secondGraspData->getApproachConfig(), &(_robot->ROBOT_POS));
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      for (int i  = 0; i < _robot->nbCcCntrts; i++) {
        desactivateTwoJointsFixCntrt(_robot, _robot->curObjectJnt, _robot->ccCntrts[i]->pasjnts[_robot->ccCntrts[i]->npasjnts - 1]);
      }
      if (offlineFile.compare("")) {
        _robot->preComputedGraphs[1] = loadedGraph;
        p3d_jnt* jnts[1] = {_robot->curObjectJnt};
        correctGraphForNewFixedJoints(_robot->preComputedGraphs[1], gotoConfig, 1, jnts);
 //       _robot->preComputedGraphs[1] = NULL;
      }
      p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
      fixJoint(_robot, _robot->curObjectJnt, objectEndPos);
 //     p3d_copy_config_into(_robot, gotoConfig, &(_robot->ROBOT_GOTO));
 //     cout << secondGraspData->getGrasp()->ID << endl;
 //     XYZ_GRAPH = _robot->preComputedGraphs[1];
 //     _robot->GRAPH = _robot->preComputedGraphs[1];
 //     return NULL;
      p3d_traj *endTraj = gotoObjectByConf(_robot, objectEndPos, gotoConfig);
      fixJoint(_robot, _robot->curObjectJnt, objectEndPos);
      gpDeactivate_hand_selfcollisions(_robot, 1);
      gpDeactivate_hand_selfcollisions(_robot, 2);
      if (endTraj) {
        if (offlineFile.compare("")) {
          statDatas.push_back(_robot->GRAPH->nnode);
          statDatas.push_back(_robot->GRAPH->time);
          int nbTests = checkTraj(endTraj, _robot->preComputedGraphs[1]);
          statDatas.push_back(nbTests);
          _robot->preComputedGraphs[1] = NULL;
        }
        p3d_concat_traj(endOpenTraj, endTraj);
      }else {
	_robot->preComputedGraphs[1] = NULL;
        statDatas.push_back(-999);
      }
      statDatas.push_back(_robot->GRAPH->nnode);
      statDatas.push_back(_robot->GRAPH->time);
      break;
    }
    default:
      break;
  }
  _statDatas.push_back(statDatas);
  if (loadedGraph) {
    p3d_del_graph(loadedGraph);
  }
  p3d_destroy_config(_robot, startConfig);
  p3d_destroy_config(_robot, gotoConfig);
  for(unsigned int i = 0; i < statDatas.size(); i++){
    cout << statDatas[i] << ";";
  }
  cout << endl;
  return NULL;
}


p3d_traj* Manipulation::computeRegraspTask(configPt startConfig, configPt gotoConfig, string offlineFile){
  vector<double> statDatas;
  double tu = 0, ts = 0;
  p3d_graph* loadedGraph = NULL;
  
  p3d_matrix4 objectStartPos, objectEndPos;
  deactivateCcCntrts(_robot, -1);
  //find Single Grasp configurations
  p3d_set_and_update_this_robot_conf(_robot, startConfig);
  p3d_mat4Copy(_robot->curObjectJnt->jnt_mat, objectStartPos);
  p3d_set_and_update_this_robot_conf(_robot, gotoConfig);
  p3d_mat4Copy(_robot->curObjectJnt->jnt_mat, objectEndPos);
  p3d_set_and_update_this_robot_conf(_robot, startConfig);
  ChronoOn();
  findAllArmsGraspsConfigs(objectStartPos, objectEndPos);
  ChronoMicroTimes(&tu, &ts);
  ChronoPrint("Single Grasp configs: ");
  ChronoOff();
  statDatas.push_back(_handsGraspsConfig[0].size());
  statDatas.push_back(_handsGraspsConfig[1].size());
  statDatas.push_back(tu);
  //find Double Grasp configurations
  ChronoOn();
  computeExchangeMat(startConfig, gotoConfig);
  
  computeDoubleGraspConfigList();
  ChronoMicroTimes(&tu, &ts);
  ChronoPrint("Double Grasp configs: ");
  ChronoOff();
  statDatas.push_back(_handsDoubleGraspsConfigs.size());
  statDatas.push_back(tu);
  
  //Planning:
  //Find the closest arm to the start position of the object to start planning with
  p3d_set_and_update_this_robot_conf(_robot, startConfig);
  int closestWrist = getClosestWristToTheObject(_robot);
  //For each double grasp in the list
  for(list <DoubleGraspData*>::iterator doubleGraspIter = _handsDoubleGraspsConfigs.begin(); doubleGraspIter !=  _handsDoubleGraspsConfigs.end(); doubleGraspIter++){
    p3d_copy_config_into(_robot, startConfig, &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    gpDoubleGrasp doubleGrasp = (*doubleGraspIter)->getDoubleGrasp();
    //Get the datas corresponding to the double grasp
    gpGrasp firstGrasp, secondGrasp;
    ManipulationData* firstGraspData, *secondGraspData;
    if (closestWrist == 0) {
      firstGrasp = doubleGrasp.grasp1;
      secondGrasp = doubleGrasp.grasp2;
    }else {
      firstGrasp = doubleGrasp.grasp2;
      secondGrasp = doubleGrasp.grasp1;
    }
    firstGraspData = _handsGraspsConfig[closestWrist][firstGrasp.ID];
    secondGraspData = _handsGraspsConfig[1 - closestWrist][secondGrasp.ID];
    
    gpHand_properties prop1;
    prop1.initialize(firstGraspData->getGrasp()->hand_type);
    gpHand_properties prop2;
    prop2.initialize(secondGraspData->getGrasp()->hand_type);
    gpDeactivate_hand_selfcollisions(XYZ_ROBOT, 1);
    gpDeactivate_hand_selfcollisions(XYZ_ROBOT, 2);
    
    //grasp the object
    cout << "Open pick config" << endl;
    deactivateCcCntrts(_robot, -1);
    p3d_set_and_update_this_robot_conf(_robot, startConfig);
    InitHandProp(0);
    InitHandProp(1);
    if (offlineFile.compare("")) {
      p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
      loadedGraph = XYZ_GRAPH;
      _robot->preComputedGraphs[1] = loadedGraph;
      p3d_jnt* jnts[1] = {_robot->curObjectJnt};
      correctGraphForNewFixedJoints(_robot->preComputedGraphs[1], startConfig, 1, jnts);
    }
    p3d_set_and_update_this_robot_conf(_robot, startConfig);
    p3d_traj* approachTraj = gotoObjectByConf(_robot, objectStartPos, firstGraspData->getApproachConfig());
    if (!approachTraj) {
      statDatas.push_back(-999);
      continue;
    }
    if (offlineFile.compare("")) {
      checkTraj(approachTraj, _robot->preComputedGraphs[1]);
      _robot->preComputedGraphs[1] = NULL;
    }
    statDatas.push_back(_robot->GRAPH->nnode);
    statDatas.push_back(_robot->GRAPH->time);
    
    //Close the hand
    cout << "Grasp pick config" << endl;
    p3d_copy_config_into(_robot, firstGraspData->getApproachConfig(), &(_robot->ROBOT_POS));
    gpActivate_hand_selfcollisions(_robot, closestWrist + 1);
    if(closestWrist == 0){
      gpUnFix_hand_configuration(_robot, prop1, 1);
    }else {
      gpUnFix_hand_configuration(_robot, prop2, 2);
    }
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    p3d_traj* graspTraj = gotoObjectByConf(_robot, objectStartPos, firstGraspData->getGraspConfig());
    if (graspTraj) {
      p3d_concat_traj(approachTraj, graspTraj);
    }else {
      continue;
    }
    //Compute open approach double grasp traj
    cout << "open approach double grasp config" << endl;
    configPt doubleGraspConfigApproach = p3d_copy_config(_robot, (*doubleGraspIter)->getConfig());
    p3d_set_and_update_this_robot_conf(_robot, doubleGraspConfigApproach);
    gpDeactivate_hand_selfcollisions(_robot, closestWrist + 1);
    if(closestWrist == 0){
      gpFix_hand_configuration(_robot, prop1, 1);
      gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 2);
      gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp2, doubleGraspConfigApproach, 2);
    }else {
      gpFix_hand_configuration(_robot, prop2, 2);
      gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 1);
      gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp1, doubleGraspConfigApproach, 1);
    }
    //get the object exchange pos
    p3d_matrix4 objectExchangePos;
    p3d_set_and_update_this_robot_conf(_robot, doubleGraspConfigApproach);
    p3d_mat4Copy(_robot->curObjectJnt->jnt_mat, objectExchangePos);
    fixJoint(_robot, _robot->curObjectJnt, objectStartPos);
    p3d_copy_config_into(_robot, firstGraspData->getGraspConfig(), &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    if (offlineFile.compare("")) {
      //p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
      _robot->preComputedGraphs[3] = loadedGraph;
      if(closestWrist == 0){
        correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 0, doubleGrasp.grasp1 , 2, doubleGrasp.grasp2, 1, 1);
      }else {
        correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 2, doubleGrasp.grasp1 , 0, doubleGrasp.grasp2, 1, 2);
      }
    }
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    p3d_traj *carryToExchangeApporach = carryObjectByConf(_robot, objectExchangePos, doubleGraspConfigApproach, closestWrist, false);
    if (carryToExchangeApporach) {
      if (offlineFile.compare("")) {
        checkTraj(carryToExchangeApporach, _robot->preComputedGraphs[3]);
        _robot->preComputedGraphs[3] = NULL;
      }
      p3d_concat_traj(approachTraj, carryToExchangeApporach);
    }else {
      statDatas.push_back(-999);
      continue;
    }
    statDatas.push_back(_robot->GRAPH->nnode);
    statDatas.push_back(_robot->GRAPH->time);
    
    //Compute double grasp traj
    cout << "Double Grasp config" << endl;
    if(closestWrist == 0){
      gpUnFix_hand_configuration(_robot, prop2, 2);
    }else {
      gpUnFix_hand_configuration(_robot, prop1, 1);
    }
    gpActivate_hand_selfcollisions(_robot, (1 - closestWrist) + 1);
    p3d_copy_config_into(_robot, doubleGraspConfigApproach, &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    p3d_copy_config_into(_robot, (*doubleGraspIter)->getConfig(), &(_robot->ROBOT_GOTO));
    p3d_traj *carryToExchange = carryObjectByConf(_robot, objectExchangePos, (*doubleGraspIter)->getConfig(), closestWrist, false);
    if (carryToExchange) {
      p3d_concat_traj(approachTraj, carryToExchange);
    }else {
      continue;
    }
    //Compute open Deposit traj 
    cout << "Double Grasp open config deposit" << endl;
    configPt doubleGraspConfigDeposit = p3d_copy_config(_robot, (*doubleGraspIter)->getConfig());
    gpDeactivate_hand_selfcollisions(_robot, (1 - closestWrist) + 1);
    gpActivate_hand_selfcollisions(_robot, closestWrist + 1);
    p3d_set_and_update_this_robot_conf(_robot, doubleGraspConfigDeposit);
    if(closestWrist == 0){
      gpFix_hand_configuration(_robot, prop2, 2);
      gpUnFix_hand_configuration(_robot, prop1, 1);
      gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 1);
      gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp1, doubleGraspConfigDeposit, 1);
    }else {
      gpFix_hand_configuration(_robot, prop1, 1);
      gpUnFix_hand_configuration(_robot, prop2, 2);
      gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 2);
      gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp2, doubleGraspConfigDeposit, 2);
    }
    p3d_copy_config_into(_robot, (*doubleGraspIter)->getConfig(), &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    p3d_traj *carryToOpenDeposit = carryObjectByConf(_robot, objectEndPos, doubleGraspConfigDeposit, 1 - closestWrist, false);
    if (carryToOpenDeposit) {      
      p3d_concat_traj(approachTraj, carryToOpenDeposit);
    }else {
      continue;
    }
    
    //Compute deposit Traj
    cout << "Deposit traj" << endl;
    if(closestWrist == 0){
      gpFix_hand_configuration(_robot, prop1, 1);
    }else {
      gpFix_hand_configuration(_robot, prop2, 2);
    }
    gpDeactivate_hand_selfcollisions(_robot, closestWrist + 1);
    p3d_copy_config_into(_robot, doubleGraspConfigDeposit, &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    if (offlineFile.compare("")) {
      //p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
      _robot->preComputedGraphs[3] = loadedGraph;
      if(closestWrist == 0){
        correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 2, doubleGrasp.grasp1 , 0, doubleGrasp.grasp2, 1, 2);
      }else {
        correctGraphForHandsAndObject(_robot, _robot->preComputedGraphs[3], 0, doubleGrasp.grasp1 , 2, doubleGrasp.grasp2, 1, 1);
      }
    }
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    p3d_traj *carryToDeposit = carryObjectByConf(_robot, objectEndPos, secondGraspData->getGraspConfig(), 1 - closestWrist, false);
    if (carryToDeposit) {
      if (offlineFile.compare("")) {
        checkTraj(carryToDeposit, _robot->preComputedGraphs[3]);
        _robot->preComputedGraphs[3] = NULL;
      }
      p3d_concat_traj(approachTraj, carryToDeposit);
    }else {
      statDatas.push_back(-999);
      continue;
    }
    statDatas.push_back(_robot->GRAPH->nnode);
    statDatas.push_back(_robot->GRAPH->time);
    
    //Compute deposit Traj
    cout << "End open traj" << endl;
    if(closestWrist == 0){
      gpUnFix_hand_configuration(_robot, prop2, 2);
    }else {
      gpUnFix_hand_configuration(_robot, prop1, 1);
    }
    p3d_copy_config_into(_robot, secondGraspData->getGraspConfig(), &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    p3d_traj *endOpenTraj = gotoObjectByConf(_robot, objectEndPos, secondGraspData->getApproachConfig());
    if (endOpenTraj) {
      p3d_concat_traj(approachTraj, endOpenTraj);
    }else {
      continue;
    }
    //Goto goal position
    cout << "End traj" << endl;
    if(closestWrist == 0){
      gpFix_hand_configuration(_robot, prop2, 2);
    }else {
      gpFix_hand_configuration(_robot, prop1, 1);
    }
    p3d_copy_config_into(_robot, secondGraspData->getApproachConfig(), &(_robot->ROBOT_POS));
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    for (int i  = 0; i < _robot->nbCcCntrts; i++) {
      desactivateTwoJointsFixCntrt(_robot, _robot->curObjectJnt, _robot->ccCntrts[i]->pasjnts[_robot->ccCntrts[i]->npasjnts - 1]);
    }
    if (offlineFile.compare("")) {
      //p3d_readGraph(offlineFile.c_str(), DEFAULTGRAPH);
      _robot->preComputedGraphs[1] = loadedGraph;
      p3d_jnt* jnts[1] = {_robot->curObjectJnt};
      correctGraphForNewFixedJoints(_robot->preComputedGraphs[1], startConfig, 1, jnts);
    }
    p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_POS);
    p3d_traj *endTraj = gotoObjectByConf(_robot, objectEndPos, gotoConfig);
    fixJoint(_robot, _robot->curObjectJnt, objectStartPos);
    if (endTraj) {
      statDatas.push_back(_robot->GRAPH->nnode);
      statDatas.push_back(_robot->GRAPH->time);
      if (offlineFile.compare("")) {
        checkTraj(endTraj, _robot->preComputedGraphs[1]);
        _robot->preComputedGraphs[1] = NULL;
        if (loadedGraph) {
          p3d_del_graph(loadedGraph);
        }
      }
      p3d_destroy_config(_robot, startConfig);
      p3d_destroy_config(_robot, gotoConfig);
      p3d_concat_traj(approachTraj, endTraj);
      cout << statDatas.size() << endl;
      _statDatas.push_back(statDatas);
      return approachTraj;
    }else {
      statDatas.push_back(-999);
      continue;
    }
  }
  if (loadedGraph) {
    p3d_del_graph(loadedGraph);
  }
  p3d_destroy_config(_robot, startConfig);
  p3d_destroy_config(_robot, gotoConfig);
  for(unsigned int i = 0; i < statDatas.size(); i++){
    cout << statDatas[i] << ";";
  }
  cout << endl;
  _statDatas.push_back(statDatas);
  return NULL;
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
  std::map<int, ManipulationData*, std::less<int> > configMap;
  
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
      gpSet_grasp_open_configuration(_robot, (*iter), openConfig, armId + 1);
      (*iter).IKscore = 1 - configCost;
      data->setGrasp(new gpGrasp(*iter));
      data->setGraspConfigCost(configCost);
      configMap.insert(pair<int, ManipulationData*> ((*iter).ID, data));
    }else{
      delete(data);
    }
  }
  p3d_desactivateCntrt(_robot, _robot->ccCntrts[armId]);
  _handsGraspsConfig.insert(pair<int, map<int, ManipulationData*, less<int> > > (armId, configMap));
  return nbGraspConfigs;
}

double Manipulation::getCollisionFreeGraspAndApproach(p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig){
  p3d_matrix4 handFrame, fictive;
  p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, _robot->ccCntrts[whichArm - 1]->Tatt2, tAtt);
  fictive[0][0] = fictive[0][1] = fictive[0][2] = 0;
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpSet_grasp_configuration(_robot, grasp, whichArm);
  gpFix_hand_configuration(_robot, handProp, whichArm);
  if(whichArm == 1){
    q = setTwoArmsRobotGraspPosWithoutBase(_robot, objectPos, tAtt, fictive, FALSE, whichArm - 1, true);
  }else if(whichArm == 2){
    q = setTwoArmsRobotGraspPosWithoutBase(_robot, objectPos, fictive, tAtt, FALSE, whichArm - 1, true);
  }
  if(q){
    double restArmCost = setRobotArmsRest(_robot, objectPos, whichArm - 1, tAtt, _robot->ROBOT_POS, q);
    double graspArmCost = getRobotGraspArmCost(grasp, q);
    double confCost = (restArmCost + graspArmCost) / 2;
    p3d_desactivateCntrt(_robot, _robot->ccCntrts[whichArm - 1]);
    gpSet_grasp_configuration(_robot, grasp, q, whichArm);
    p3d_copy_config_into(_robot, q, graspConfig);
    //Check the rest configuration of the hand
    gpSet_grasp_open_configuration(_robot, grasp, q, whichArm);
    p3d_set_and_update_this_robot_conf(_robot, q);
    g3d_draw_allwin_active();
    if(!p3d_col_test()){
      p3d_copy_config_into(_robot, q, approachConfig);
      p3d_destroy_config(_robot, q);
      return confCost; //success
    }
  }
  p3d_destroy_config(_robot, q);
  return -1;
}

void Manipulation::drawSimpleGraspConfigs(){
  static map < int, map<int, ManipulationData*, std::less<int> > >::iterator armIter = _handsGraspsConfig.begin();
  static map < int, ManipulationData* >::iterator dataIter = (*armIter).second.begin();
  if (dataIter == (*armIter).second.end()) {
    armIter++;
    if (armIter == _handsGraspsConfig.end()) {
      armIter = _handsGraspsConfig.begin();
    }
    dataIter = (*armIter).second.begin();
  }
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

void Manipulation::printStatDatas(){
  for(unsigned int i = 0; i < _statDatas.size(); i++){
    for(unsigned int j = 0; j < _statDatas[i].size(); j++){
      cout << _statDatas[i][j] << ";";
    }
    cout << endl;
  }
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
  list<gpDoubleGrasp> doubleGraspList;
  //p3d_set_and_update_this_robot_conf(_robot, _robot->ROBOT_GOTO);
  gpDouble_grasp_generation((p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_RIGHT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*) GP_SAHAND_LEFT_ROBOT_NAME), (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), *(getGraspListFromMap(0)), *(getGraspListFromMap(1)), doubleGraspList);
  
  vector<gpHand_properties> handProp = InitHandProp(-1);
  for(list<gpDoubleGrasp>::iterator itDouble = doubleGraspList.begin(); itDouble != doubleGraspList.end(); itDouble++){
    DoubleGraspData* data = new DoubleGraspData(_robot);
    configPt dGraspConfig = data->getConfig();
    if(getCollisionFreeDoubleGraspAndApproach(_exchangeMat, handProp, (*itDouble), &dGraspConfig)){
      data->setDoubleGrasp((*itDouble));
      _handsDoubleGraspsConfigs.push_back(data);
      //showConfig(dGraspConfig);
    }else{
      delete(data);
    }
  }
  cout << "Valid Grasps" << _handsDoubleGraspsConfigs.size() << endl;
}

double Manipulation::getRobotGraspArmCost(gpGrasp grasp, configPt q){
  p3d_set_and_update_this_robot_conf(_robot, q);
  p3d_vector3 graspDir, robotObjectDir, base, object;
  grasp.direction(graspDir);
  p3d_vectNormalize(graspDir, graspDir);
  p3d_jnt* baseJnt = _robot->baseJnt;
  if (baseJnt || (baseJnt && baseJnt->num == 0)) {
    baseJnt = _robot->joints[1];
  }
  p3d_mat4ExtractTrans(baseJnt->abs_pos, base);
  p3d_mat4ExtractTrans(_robot->curObjectJnt->abs_pos, base);
  robotObjectDir[0] = object[0] - base[0];
  robotObjectDir[1] = object[1] - base[1];
  robotObjectDir[2] = object[2] - base[2];
  p3d_vectNormalize(robotObjectDir, robotObjectDir);
  return 1 - ((1 + p3d_vectDotProd(graspDir, robotObjectDir)) / 2);
}

int Manipulation::getCollisionFreeDoubleGraspAndApproach(p3d_matrix4 objectPos, vector<gpHand_properties> handProp, gpDoubleGrasp doubleGrasp, configPt* doubleGraspConfig){

  p3d_matrix4 handFrame, rTatt, lTatt, exchangeMat, torsoPos;
  p3d_mat4Copy(_exchangeMat, exchangeMat);
  for(int i = 0; i < _robot->njoints; i++){
    if (_robot->joints[i]->type == P3D_ROTATE) {
      p3d_mat4Copy(_robot->joints[i]->abs_pos, torsoPos);
      break;
    }
  }
  doubleGrasp.computeBestObjectOrientation(torsoPos, exchangeMat);
  p3d_mat4Mult(doubleGrasp.grasp1.frame, handProp[0].Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, _robot->ccCntrts[0]->Tatt2, rTatt);
  p3d_mat4Mult(doubleGrasp.grasp2.frame, handProp[1].Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, _robot->ccCntrts[1]->Tatt2, lTatt);
  //Check if there is a valid configuration of the robot using this graspFrame
  configPt q = NULL;
  gpSet_grasp_configuration(_robot, doubleGrasp.grasp1, 1);
  gpFix_hand_configuration(_robot, handProp[0], 1);
  gpSet_grasp_configuration(_robot, doubleGrasp.grasp2, 2);
  gpFix_hand_configuration(_robot, handProp[1], 2);
  q = setTwoArmsRobotGraspPosWithoutBase(_robot, exchangeMat, rTatt, lTatt, TRUE, -1, true);
  if(q){
    //Test the collisions with open hands
    configPt doubleGraspOpen = p3d_copy_config(_robot, q);
    gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 1);
    gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp1, doubleGraspOpen, 1);
    p3d_set_and_update_this_robot_conf(_robot, doubleGraspOpen);
    if (p3d_col_test()) {
      p3d_destroy_config(_robot, q);
      p3d_destroy_config(_robot, doubleGraspOpen);
      g3d_draw_allwin_active();
      return 0;
    }
    //test the other hand
    p3d_copy_config_into(_robot, q, &doubleGraspOpen);
    gpCompute_grasp_open_config(_robot, doubleGrasp, (p3d_rob*)p3d_get_robot_by_name((char*)GP_OBJECT_NAME_DEFAULT), 2);
    gpSet_grasp_open_configuration (_robot, doubleGrasp.grasp2, doubleGraspOpen, 2);
    p3d_set_and_update_this_robot_conf(_robot, doubleGraspOpen);
    if (p3d_col_test()) {
      p3d_destroy_config(_robot, q);
      p3d_destroy_config(_robot, doubleGraspOpen);
      g3d_draw_allwin_active();
      return 0;
    }
    p3d_destroy_config(_robot, doubleGraspOpen);
    p3d_desactivateCntrt(_robot, _robot->ccCntrts[0]);
    p3d_desactivateCntrt(_robot, _robot->ccCntrts[1]);
    gpSet_grasp_configuration(_robot, doubleGrasp.grasp1, q, 1);
    gpSet_grasp_configuration(_robot, doubleGrasp.grasp2, q, 2);
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

int Manipulation::checkTraj(p3d_traj * traj, p3d_graph* graph){
  _robot->tcur = traj;
  int j = 0, returnValue = 0, optimized = traj->isOptimized;
  if(optimized){
    p3dAddTrajToGraph(_robot, graph, traj);
  }
  do{
    printf("Test %d\n", j);
    j++;
    returnValue = checkCollisionsOnPathAndReplan(_robot, _robot->tcur, graph, optimized);//replanForCollidingPath(_robot, _robot->tcur, graph, curConfig, _robot->tcur->courbePt, optimized);
  }while(returnValue != 1 && returnValue != 0 && returnValue != -2 && j < 10);
  if (optimized && j > 1){
    optimiseTrajectory(200,10);
  }
  traj = _robot->tcur;
  return j;
}

list<gpGrasp>* Manipulation::getGraspListFromMap(int armId){
  list<gpGrasp>* graspList = new list<gpGrasp>();
  for(map<int, ManipulationData* >::iterator it = _handsGraspsConfig[armId].begin(); it != _handsGraspsConfig[armId].end(); it++){
    gpGrasp* grasp = it->second->getGrasp();
    graspList->push_back(*grasp);
  }
  return graspList;
}
