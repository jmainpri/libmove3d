#include "ManipulationUtils.hpp"


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

void ManipulationUtils::printManipulationError(MANIPULATION_TASK_MESSAGE message) {
  switch (message) {
    case MANIPULATION_TASK_OK:
      printf("MANIPULATION_TASK_OK\n");
      break;
    case MANIPULATION_TASK_NOT_INITIALIZED:
      printf("MANIPULATION_TASK_NOT_INITIALIZED\n");
      break;
    case MANIPULATION_TASK_NO_TRAJ_FOUND:
      printf("MANIPULATION_TASK_NO_TRAJ_FOUND\n");
      break;
    case MANIPULATION_TASK_INVALID_QSTART:
      printf("MANIPULATION_TASK_INVALID_QSTART \n");
      break;
    case MANIPULATION_TASK_INVALID_QGOAL:
      printf("MANIPULATION_TASK_INVALID_QGOAL\n");
      break;
    case MANIPULATION_TASK_INVALID_TRAJ_ID:
      printf("MANIPULATION_TASK_INVALID_TRAJ_ID\n");
      break;
    case MANIPULATION_TASK_INVALID_TASK:
      printf("MANIPULATION_TASK_INVALID_TASK\n");
      break;
    case MANIPULATION_TASK_UNKNOWN_OBJECT:
      printf("MANIPULATION_TASK_UNKNOWN_OBJECT\n");
      break;
    case MANIPULATION_TASK_NO_GRASP:
      printf("MANIPULATION_TASK_NO_GRASP\n");
      break;
    case MANIPULATION_TASK_NO_PLACE:
      printf("MANIPULATION_TASK_NO_PLACE\n");
      break;
    case MANIPULATION_TASK_ERROR_UNKNOWN:
      printf("MANIPULATION_TASK_ERROR_UNKNOWN\n");
      break;
  }
}


void ManipulationUtils::printManipulationMessage(MANIPULATION_TASK_MESSAGE message) {
  switch (message) {
    case MANIPULATION_TASK_OK:
      printf("MANIPULATION_TASK_OK\n");
      break;
    case MANIPULATION_TASK_NOT_INITIALIZED:
      printf("MANIPULATION_TASK_NOT_INITIALIZED\n");
      break;
    case MANIPULATION_TASK_NO_TRAJ_FOUND:
      printf("MANIPULATION_TASK_NO_TRAJ_FOUND\n");
      break;
    case MANIPULATION_TASK_INVALID_QSTART:
      printf("MANIPULATION_TASK_INVALID_QSTART\n");
      break;
    case MANIPULATION_TASK_INVALID_QGOAL:
      printf("MANIPULATION_TASK_INVALID_QGOAL\n");
      break;
    case MANIPULATION_TASK_INVALID_TRAJ_ID:
      printf("MANIPULATION_TASK_INVALID_TRAJ_ID\n");
      break;
    case MANIPULATION_TASK_INVALID_TASK:
      printf("MANIPULATION_TASK_INVALID_TASK\n");
      break;
    case MANIPULATION_TASK_UNKNOWN_OBJECT:
      printf("MANIPULATION_TASK_UNKNOWN_OBJECT\n");
      break;
    case MANIPULATION_TASK_NO_GRASP:
      printf("MANIPULATION_TASK_NO_GRASP\n");
      break;
    case MANIPULATION_TASK_NO_PLACE:
      printf("MANIPULATION_TASK_NO_PLACE\n");
      break;
    case MANIPULATION_TASK_ERROR_UNKNOWN:
      printf("MANIPULATION_TASK_ERROR_UNKNOWN\n");
      break;
  }
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
