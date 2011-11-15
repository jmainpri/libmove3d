#include "ManipulationViaConfPlanner.hpp"
#include "ManipulationPlanner.hpp"

using namespace std;

//constructor
ManipulationViaConfPlanner::ManipulationViaConfPlanner(p3d_rob * robot):ManipulationPlanner(robot)
{
  _robot = robot;
  cout << "Manipulation planner robot is : " << _robot->name << endl;
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
    res = planNavigation(_robot->conf[i]->q, _robot->conf[i-1]->q, true, trajs);
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
  return MANIPULATION_TASK_OK;
}
