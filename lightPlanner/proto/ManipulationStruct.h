#ifndef __MANIPULATIONSTRUCT_H__
#define __MANIPULATIONSTRUCT_H__

#include <vector>

//! @ingroup manipulation 
//! The different manipulation tasks that can be planned with Jido:
typedef enum MANIPULATION_TASK_TYPE_STR {
  ARM_FREE = 1, /*!< move the arm from a free configuration (in the air) to another free configuration */
  ARM_PICK_GOTO = 2,  /*!< move the arm from a free configuration to a grasping configuration of the object placed on a support */
  ARM_PICK_TAKE_TO_FREE = 3,  /*!< move the arm from a grasping configuration (of the object placed on a support) to a free configuration */
  ARM_PICK_TAKE_TO_PLACE = 4,  /*!< move the arm from a grasping configuration to a configuration with the same grasp but a different object placement */
  ARM_PLACE_FROM_FREE = 5  /*!< move the arm from a free configuration to a placement configuration */
} MANIPULATION_TASK_TYPE_STR;



//! @ingroup manipulation 
//! The different messages that can be returned by the armPlanTask function:
typedef enum MANIPULATION_TASK_MESSAGE {
  MANIPULATION_TASK_OK, /*!< everything was fine */
  MANIPULATION_TASK_NOT_INITIALIZED,  /*!< a variable was not properly initialized */
  MANIPULATION_TASK_NO_TRAJ_FOUND,  /*!< failed to compute a trajectory for the desired task */
  MANIPULATION_TASK_INVALID_QSTART,
  MANIPULATION_TASK_INVALID_QGOAL,
  MANIPULATION_TASK_INVALID_TRAJ_ID,
  MANIPULATION_TASK_INVALID_TASK,  /*!< the desired task is undefined */
  MANIPULATION_TASK_UNKNOWN_OBJECT, /*!< there is no object with the specified name */
  MANIPULATION_TASK_NO_GRASP, /*!< no grasp or arm grasping configuration can be found for the current context (mobile base and obstacles configurations) */
  MANIPULATION_TASK_NO_PLACE,  /*!< no placement or arm placement configuration can be found for the current context (mobile base and obstacles configurations, current grasp) */
  MANIPULATION_TASK_ERROR_UNKNOWN /*!< something undefined was wrong */
} MANIPULATION_TASK_MESSAGE;

typedef struct MANIPULATION_SEGMENT_AXIS_DATA_STR {
 double ic_a;
 double ic_v;
 double ic_x;
 double jerk;
} MANIPULATION_SEGMENT_AXIS_DATA_STR;

typedef struct MANIPULATION_SEGMENT_STR {
  double time;
  int lp;
  std::vector<MANIPULATION_SEGMENT_AXIS_DATA_STR> data;
} MANIPULATION_SEGMENT_STR;

typedef struct MANPIPULATION_TRAJECTORY_STR {
  std::vector<MANIPULATION_SEGMENT_STR> seg;
}MANPIPULATION_TRAJECTORY_STR;

#endif
