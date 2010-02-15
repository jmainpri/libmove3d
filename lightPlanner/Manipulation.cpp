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
  p3d_col_stop();
  p3d_col_start(p3d_col_mode_kcd);
}

Manipulation::~Manipulation(){
  for(map<double, ManipulationData* >::iterator it = _handsGraspsConfig.begin(); it != _handsGraspsConfig.end(); it++){
    delete(it->second);
  }
}

int Manipulation::findAllArmsGraspsConfigs(p3d_matrix4 objectStartPos, p3d_matrix4 objectEndPos){
  int nbGraspConfigs = 0;
  if(_robot->nbCcCntrts != 2){
    cout << "Error: the number of arm  != 2" << endl;
    return 0;
  }
  int closestWrist = getClosestWristToTheObject(_robot);
  switchBBActivationForGrasp();
  nbGraspConfigs = findAllSpecificArmGraspsConfigs(closestWrist, objectStartPos);
  nbGraspConfigs = findAllSpecificArmGraspsConfigs(1 - closestWrist, objectEndPos);
  switchBBActivationForGrasp();
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
  
  
  //Activate the corresponding constraint
  p3d_activateCntrt(_robot, _robot->ccCntrts[armId]);
  //For each grasp, get the tAtt and check the collision
  for(list<gpGrasp>::iterator iter = graspList.begin(); iter != graspList.end(); iter++){
    ManipulationData* data = new ManipulationData(_robot);
    configPt graspConfig = data->getGraspConfig(), approachConfig = data->getApproachConfig();
    p3d_matrix4 tAtt;
    int nbTest = getCollisionFreeGraspAndApproach(_robot, objectPos, handProp, (*iter), armId + 1, tAtt, &graspConfig, &approachConfig);
    if(!nbTest){
      data->setAttachFrame(tAtt);
      configPt openConfig = data->getOpenConfig();
      p3d_copy_config_into(_robot, graspConfig, &openConfig);
      gpSet_grasp_open_configuration(_robot, handProp, (*iter), openConfig, armId + 1);
      double cost = (1 - (((*iter).quality - _armMinMaxCost[armId][0]) / _armMinMaxCost[armId][1])) + nbTest / _maxColGrasps;
      _handsGraspsConfig.insert(pair<double, ManipulationData*> (cost, data));
    }
  }
  p3d_desactivateCntrt(_robot, _robot->ccCntrts[armId]);
  return nbGraspConfigs;
}

int Manipulation::getCollisionFreeGraspAndApproach(p3d_rob* robot, p3d_matrix4 objectPos, gpHand_properties handProp, gpGrasp grasp, int whichArm, p3d_matrix4 tAtt, configPt* graspConfig, configPt* approachConfig){
  p3d_matrix4 handFrame, fictive;
  p3d_mat4Mult(grasp.frame, handProp.Tgrasp_frame_hand, handFrame);
  p3d_mat4Mult(handFrame, robot->ccCntrts[whichArm -1]->Tatt2, tAtt);
  fictive[0][0] = fictive[0][1] = fictive[0][2] = 0;
  //Check if there is a valid configuration of the robot using this graspFrame
  int maxColGrasps = 0;
  configPt q = NULL;
  gpSet_grasp_configuration(robot, handProp, grasp, whichArm);
  gpFix_hand_configuration(robot, handProp, whichArm);
  do{
    if(q){
      p3d_destroy_config(robot, q);
    }
    if(whichArm == 1){
      q = setTwoArmsRobotGraspPosWithoutBase(robot, objectPos, tAtt, fictive, whichArm - 1, false);
    }else if(whichArm == 2){
      q = setTwoArmsRobotGraspPosWithoutBase(robot, objectPos, fictive, tAtt, whichArm - 1, false);
    }
    maxColGrasps ++;
    if(q){
      p3d_set_and_update_this_robot_conf(robot, q);
      g3d_draw_allwin_active();
      gpSet_grasp_configuration(robot, handProp, grasp, whichArm);
      *graspConfig = p3d_get_robot_config(robot);
      //Check the rest configuration of the hand
      gpSet_grasp_open_configuration(robot, handProp, grasp, whichArm);
      g3d_draw_allwin_active();
      if(!p3d_col_test()){
        //Check the approach configuration of the arm
        tAtt[1][3] -= 0.3;
        q = setTwoArmsRobotGraspPosWithoutBase(robot, objectPos, fictive, tAtt, whichArm - 1, false);
        if(q){
          p3d_set_and_update_this_robot_conf(robot, q);
          gpSet_grasp_open_configuration(robot, handProp, grasp, whichArm);
          *approachConfig = p3d_get_robot_config(robot);
          return maxColGrasps; //success
        }
      }
    }
  }while(maxColGrasps <= _maxColGrasps);

  return 1;
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
