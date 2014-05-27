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
#ifndef __MANIPULATIONDYNAMIC_HPP__
#define __MANIPULATIONDYNAMIC_HPP__

class  ManipulationDynamic : public ManipulationPlanner 
{
  public:
  /* ******************************* */
  /* ******* (Con)Destructor ******* */
  /* ******************************* */
  ManipulationDynamic(p3d_rob* robot);
  virtual ~ManipulationDynamic();
  /* ******************************* */
  /* *********** Methods *********** */
  /* ******************************* */
    /** \brief Check if the current path is in collision or not. Start from the begining of the trajectory
    \return 1 in case of collision, 0 otherwise*/
    int checkCollisionOnTraj();
    /** \brief Check if the current path is in collision or not. Start from the given local path id
    \return 1 in case of collision, 0 otherwise*/
    int checkCollisionOnTraj(int currentLpId);
    /** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
    \return MANIPULATION_TASK_OK for success */
    MANIPULATION_TASK_MESSAGE replanCollidingTraj(int currentLpId, std::vector <p3d_traj*> &trajs);
    /** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
    \return MANIPULATION_TASK_OK for success */
    MANIPULATION_TASK_MESSAGE replanCollidingTraj(int currentLpId, std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> &confs, std::vector <SM_TRAJ> &smTrajs);
    
  private:
    /*!< pointer to the robot */
    p3d_rob * _robot;
};
#endif