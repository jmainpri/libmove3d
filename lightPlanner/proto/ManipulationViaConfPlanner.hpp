#ifndef __MANIPULATIONVIACONFPLANNER_HPP__
#define __MANIPULATIONVIACONFPLANNER_HPP__

#include "P3d-pkg.h"
#include "Localpath-pkg.h"

#include "ManipulationPlanner.hpp"

class  ManipulationViaConfPlanner: public ManipulationPlanner {
  public:
  /* ******************************* */
  /* ******* (Con)Destructor ******* */
  /* ******************************* */
  ManipulationViaConfPlanner(p3d_rob * robot);
  virtual ~ManipulationViaConfPlanner();
  /* ******************************* */
  /* *********** Methods *********** */
  /* ******************************* */
  /** Plans a path to go from the currently defined ROBOT_POS config to the currently defined ROBOT_GOTO config for the arm only.
    \return MANIPULATION_TASK_OK for success */
  MANIPULATION_TASK_MESSAGE planTrajFromConfigArrayInRobotTheForm(std::vector<SM_TRAJ> &smTrajs);


   private:
    /*!< pointer to the robot */
    p3d_rob * _robot;
};
#endif