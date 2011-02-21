#ifndef __MANIPULATIONDYNAMIC_HPP__
#define __MANIPULATIONDYNAMIC_HPP__

class  ManipulationDynamic: public ManipulationPlanner {
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