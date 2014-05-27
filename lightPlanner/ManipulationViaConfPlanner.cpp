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
#include "ManipulationViaConfPlanner.hpp"
#include "ManipulationPlanner.hpp"

using namespace std;

//constructor
ManipulationViaConfPlanner::ManipulationViaConfPlanner(p3d_rob * robot):ManipulationPlanner(robot)
{
  _robot = robot;
#ifdef DEBUG_STATUS
  cout << "Manipulation planner robot is : " << _robot->name << endl;
#endif
}
//destructor
ManipulationViaConfPlanner::~ManipulationViaConfPlanner()
{

}

MANIPULATION_TASK_MESSAGE ManipulationViaConfPlanner::planTrajFromConfigArrayInRobotTheForm(std::vector<SM_TRAJ> &smTrajs)
{
  vector<p3d_traj*> trajs;
  vector<p3d_traj*> viaPointTrajs;
  p3d_traj* traj;

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <double>  objStart, objGoto;
  gpGrasp grasp;
  SM_TRAJ smTraj;

  MANIPULATION_TASK_MESSAGE res;
  p3d_multiLocalPath_disable_all_groupToPlan(_robot);
  p3d_multiLocalPath_set_groupToPlan(_robot, getUpBodyMLP(), 1);

  viaPointTrajs.clear();

  for(int i=(_robot->nconf-1) ; i>0; i--) {

    trajs.clear();
//    res = armPlanTask(ARM_FREE,0, _robot->conf[i]->q, _robot->conf[i-1]->q, objStart, objGoto,  (char*)"", (char*)"", (char*)"",  trajs);
    clock_t test = clock();
    res = planNavigation(_robot->conf[i]->q, _robot->conf[i-1]->q, true, trajs);
    cout << "planNavigation take : " << ((double)clock() - test) / CLOCKS_PER_SEC << " s" <<endl;
    if(res!=MANIPULATION_TASK_OK) {
     printf("ERROR plan armFree on path %d\n",i);
     return res;
    }
    viaPointTrajs.push_back(trajs.at(0));
  }

  if (concatTrajectories(viaPointTrajs, &traj) == MANIPULATION_TASK_OK) {
    smTrajs.clear();
    MANPIPULATION_TRAJECTORY_CONF_STR conf;
    /* COMPUTE THE SOFTMOTION TRAJECTORY */
    computeSoftMotion(traj, conf, smTraj);
    confs.push_back(conf);
    smTrajs.push_back(smTraj);
  }
  else
  {
      return MANIPULATION_TASK_NO_TRAJ_FOUND;
  }
  return MANIPULATION_TASK_OK;
}
