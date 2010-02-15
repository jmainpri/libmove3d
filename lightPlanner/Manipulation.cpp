#include "Manipulation.h"
#include <iostream>
#include "lightPlanner.h"

using namespace std;

Manipulation::Manipulation(p3d_rob * robot){
  _robot = robot;
}

Manipulation::~Manipulation(){
  for(unsigned int i; i < _handsGraspsConfig.size(); i++){
    for(map<gpGrasp, configPt >::iterator it = _handsGraspsConfig[i].begin(); it != _handsGraspsConfig[i].end(); it++){
      p3d_destroy_config(_robot, it->second);
    }
  }
}

int Manipulation::findAllArmsGraspsConfigs(){
  int nbGraspConfigs = 0;
  for(int i = 0; i < _robot->nbCcCntrts; i++){
    nbGraspConfigs += findAllSpecificArmGraspsConfigs(i);
  }
  return nbGraspConfigs;
}

int Manipulation::findAllSpecificArmGraspsConfigs(int armId){
  int nbGraspConfigs = 0;
  gpHand_type handType;
  switch (armId){
    case 0:{
      handType = GP_SAHAND_RIGHT;
    }
    case 1:{
      handType = GP_SAHAND_LEFT;
      break;
    }
    default:{
      cout << "The arm id is not valid." << endl;
    }
  }
  std::list<gpGrasp> graspList;
  gpGet_grasp_list_SAHand(GP_OBJECT_NAME_DEFAULT, armId + 1, graspList);
  gpHand_properties handProp;
  handProp.initialize(handType);
  //Activate the corresponding constraint
  p3d_activateCntrt(_robot, _robot->ccCntrts[armId]);
  //For each grasp, get the tAtt and check the collision
  for(std::list<gpGrasp>::iterator iter = graspList.begin(); iter != graspList.end(); iter++){
    nbGraspConfigs += findGraspConfig(armId, (*iter), false);
  }
  return nbGraspConfigs;
}

int Manipulation::findGraspConfig(int armId, gpGrasp grasp, bool activateCntrt){
  if(activateCntrt){
    //Activate the corresponding constraint
    p3d_activateCntrt(_robot, _robot->ccCntrts[armId]);
  }
  
  return 0;
}
