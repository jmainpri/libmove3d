#include "ManipulationUtils.hpp"

#include "ManipulationArmData.hpp"

#include "lightPlannerApi.h"

#include "move3d-headless.h"

/* Message gestion */

void ManipulationUtils::undefinedRobotMessage() {
  printf("The robot has not been defined in ManipulationPlanner. Recreate an instance of ManipulationPlanner.\n");
}

void ManipulationUtils::undefinedObjectMessage() {
  printf("The object has not been defined in ManipulationPlanner. Set it with setObjectToManipulate().\n");
}

void ManipulationUtils::undefinedSupportMessage() {
  printf("The support has not been defined in ManipulationPlanner. Set it with setSupport().\n");
}

void ManipulationUtils::undefinedCameraMessage() {
  printf("The joint of the pan/tilt unit has not been defined in ManipulationPlanner. Set it with setCameraJnt().\n");
}

void ManipulationUtils::printManipulationMessage(MANIPULATION_TASK_MESSAGE message) {
  std::cout << "ManipulationPlanner::status => ";
  switch(message){
    case MANIPULATION_TASK_OK:{
      std::cout << "everything was fine" << std::endl;
      break;
    }
    case MANIPULATION_TASK_NOT_INITIALIZED:{
    std::cout << "a variable was not properly initialized" << std::endl;
      break;
    }
    case MANIPULATION_TASK_NO_TRAJ_FOUND:{
    std::cout << "failed to compute a trajectory for the desired task" << std::endl;
      break;
    }
    case MANIPULATION_TASK_INVALID_QSTART:{
    std::cout << "Check the constraint and the collision for the Start Configuration" << std::endl;
      break;
    }
    case MANIPULATION_TASK_INVALID_QGOAL:{
    std::cout << "Check the constraint and the collision for the Goal Configuration" << std::endl;
      break;
    }
    case MANIPULATION_TASK_INVALID_TRAJ_ID:{
    std::cout << "An invalid trajectory ID is specified : Trajectory not found" << std::endl;
      break;
    }
    case MANIPULATION_TASK_INVALID_TASK:{
    std::cout << "the desired task is undefined" << std::endl;
      break;
    }
    case MANIPULATION_TASK_UNKNOWN_OBJECT:{
    std::cout << "there is no object with the specified name" << std::endl;
      break;
    }
    case MANIPULATION_TASK_NO_GRASP:{
    std::cout << "no grasp or arm grasping configuration can be found for the current context (mobile base and obstacles configurations)" << std::endl;
      break;
    }
    case MANIPULATION_TASK_NO_PLACE:{
    std::cout << "no placement or arm placement configuration can be found for the current context (mobile base and obstacles configurations, current grasp)" << std::endl;
      break;
    }
    case MANIPULATION_TASK_ERROR_UNKNOWN:{
    std::cout << "something undefined was wrong" << std::endl;
      break;
    }
    case MANIPULATION_TASK_EQUAL_QSTART_QGOAL:{
      std::cout << "The start and goal configuration are the same." << std::endl;
      break;
    }
    default:{
      std::cout << "Unknown Error." << std::endl;
      break;
    }
  }
}

//! Prints some info about a robot's contraints
int ManipulationUtils::printConstraintInfo(p3d_rob* robot) {
  int i = 0;
  if (robot == NULL) {
    printf("%s: %d: ManipulationPlanner::printConstraintInfo().\n", __FILE__, __LINE__);
    ManipulationUtils::undefinedRobotMessage();
    return 1;
  }

  printf("constraint info: \n");
  printf("robot \"%s\" has %d constraints: \n", robot->name, robot->cntrt_manager->ncntrts);
  for (i = 0; i < robot->cntrt_manager->ncntrts; i++) {
    printf("%s, active= %d\n", robot->cntrt_manager->cntrts[i]->namecntrt, robot->cntrt_manager->cntrts[i]->active);
  }
  return 0;
}

/* UI gestion */
int ManipulationUtils::forbidWindowEvents() {
  g3d_win *win = NULL;
  win = g3d_get_cur_win();
  if (win == NULL) {
    return 1;
  }
  win->vs.eventsEnabled = 0;
  return 0;
}

int ManipulationUtils::allowWindowEvents() {
  g3d_win *win = NULL;
  win = g3d_get_cur_win();
  if (win == NULL) {
    return 1;
  }
  win->vs.eventsEnabled = 1;
  return 0;
}

int ManipulationUtils::copyConfigToFORM(p3d_rob* robot, configPt q) {
  if (robot == NULL)  {
    printf("%s: %d: ManipulationPlanner::copyConfigTrajToFORM().\n", __FILE__, __LINE__);
    ManipulationUtils::undefinedRobotMessage();
    return 1;
  }
  char name[128];
  
  sprintf(name, "configTraj_%d", robot->nconf);
  p3d_set_new_robot_config(robot, name, q, robot->ikSol, robot->confcur);
  robot->confcur = robot->conf[0];
#ifdef WITH_XFORMS
  FORMrobot_update(p3d_get_desc_curnum(P3D_ROBOT));
#else
 g3d_add_config_to_ui(name,robot,q);
#endif

  p3d_set_and_update_this_robot_conf_multisol(robot, robot->confcur->q, NULL, 0, robot->ikSol);
  return 0;
}

bool ManipulationUtils::isValidVector(std::vector<double> objectPos){
  for(unsigned int i = 0; i < objectPos.size(); i++){
    if (objectPos[i] != P3D_HUGE){
      return true;
    }
  }
  return false;
}

void ManipulationUtils::fixAllHands(p3d_rob* robot, configPt q, bool rest) {
    if (q != NULL) {
        p3d_set_and_update_this_robot_conf(robot, q);
    }
    for (uint i = 0; i < (*robot->armManipulationData).size(); i++) {
        (*robot->armManipulationData)[i].fixHand(robot, rest);
    }
    if (q != NULL) {
        p3d_get_robot_config_into(robot, &q);
    }
}

void ManipulationUtils::unFixAllHands(p3d_rob* robot) {
    for (uint i = 0; i < (*robot->armManipulationData).size(); i++) {
        (*robot->armManipulationData)[i].unFixHand(robot);
    }
}

void ManipulationUtils::unfixManipulationJoints(p3d_rob* robot, int armId) {
    for (uint i = 0; i < (*robot->armManipulationData).size(); i++) {
        unFixJoint(robot, (*robot->armManipulationData)[i].getManipulationJnt());
    }
}

void ManipulationUtils::fixManipulationJoints(p3d_rob* robot, int armId, configPt q, p3d_rob* object) {
    p3d_matrix4 pos;
    if (object) {
        p3d_mat4Copy(object->joints[1]->abs_pos, pos);
    } else {
        p3d_mat4Copy(p3d_mat4IDENTITY, pos);
    }
    if (q) {
        p3d_set_and_update_this_robot_conf(robot, q);
    }
    for (uint i = 0; i < (*robot->armManipulationData).size(); i++) {
        if(!(*robot->armManipulationData)[i].getCartesian()){
          fixJoint(robot, (*robot->armManipulationData)[i].getManipulationJnt(), pos);
        }
    }
    if (q) {
        p3d_get_robot_config_into(robot, &q);
    }
}
