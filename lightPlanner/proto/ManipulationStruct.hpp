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
#ifndef __ManipulationStruct_H__
#define __ManipulationStruct_H__

#include <vector>

//! @ingroup manipulation 
//! The different manipulation tasks that can be planned with Jido:
typedef enum MANIPULATION_TASK_TYPE_STR {
  ARM_FREE = 1, /*!< move the arm from a free configuration (in the air) to another free configuration */
  ARM_PICK_GOTO = 2,  /*!< move the arm from a free configuration to a grasping configuration of the object placed on a support */
  ARM_TAKE_TO_FREE = 3,  /*!< move the arm from a grasping configuration (of the object placed on a support) to a free configuration */
  ARM_TAKE_TO_PLACE = 4,  /*!< move the arm from a grasping configuration to a configuration with the same grasp but a different object placement */
  ARM_PLACE_FROM_FREE = 5, /*!< move the arm from a free configuration to a placement configuration */
  ARM_EXTRACT = 6, /*!< move the arm over Z axis to escape from collision */
  ARM_ESCAPE_OBJECT = 7 /*!< move the arm to escape from a placed object */
//   ARM_PICK_AND_PLACE = 6, /*!< move the arm from a free configuration to a grasping configuration of the object placed on a support then to a placement configuration */
// 	ARM_PICK_GOTO_AND_TAKE_TO_FREE = 7,
//   ARM_PICK_TAKE_TO_FREE_POINT = 8
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
  MANIPULATION_TASK_ERROR_UNKNOWN, /*!< something undefined was wrong */
  MANIPULATION_TASK_EQUAL_QSTART_QGOAL /*!< The start and goal configurations are identical ! */  
} MANIPULATION_TASK_MESSAGE;

//! @ingroup manipulation 
//! Defines the arm mainpulation-state
typedef enum MANIPULATION_ARM_STATE
{
	handFree,
	holdingObjectInStablePose,
	holdingObjectInFlyingPose
	
} MANIPULATION_ARM_STATE;


typedef std::pair < std::vector<int>, std::vector < std::vector <double> > > MANPIPULATION_TRAJECTORY_CONF_STR;
//   std::vector<int> &lp;
//   std::vector < std::vector <double> > &positions;
// }MANPIPULATION_TRAJECTORY_CONF_STR;

#endif
