// author: Romain Iehl <riehl@laas.fr>

#include "ml_mechanical_functions.hpp"
#include "robot.hpp"
#include "Planner-pkg.h"
#include "Collision-pkg.h"

std::vector<p3d_jnt*> getCollidingPassiveJoints(Robot* R, Configuration& conf)
{
  std::vector<p3d_jnt*> joints;
  p3d_obj* objs[2];
  
  R->setAndUpdateWithoutConstraints(conf);
  if(p3d_col_test() <= 0)
  {
    std::cout << "No collision detected" << std::endl;
    return(joints);
  }
  
  // NOTE: KCD only retuns the first collision pair !!!
  // NOTE: ONLY THE PASSIVE JOINT INVOLVED IN THE COLLISION IS RETURNED
  //       BUT ALL THE PARENT JOINTS SHOULD BE ALSO CONSIDERED ???
  /*
    p3d_col_get_report(0,&polys[0],&polys[1]);
    for(uint i(0); i < 2; i++)
    if(polys[i]->p3d_objPt->jnt != NULL)
    if(!p3d_jnt_get_is_active_for_planner(polys[i]->p3d_objPt->jnt))
    joints.push_back(polys[i]->p3d_objPt->jnt);
  */
  p3d_col_get_report_obj(&objs[0], &objs[1]);
  for(uint i(0); i < 2; i++)
    if(objs[i]->jnt != NULL)
      if(!p3d_jnt_get_is_active_for_planner(objs[i]->jnt))
	joints.push_back(objs[i]->jnt);

  return(joints);
}

bool getCurrentInvalidConf(Robot* R, Configuration& q)
{
  return(p3d_get_current_q_inv(R->getRobotStruct(), q.getConfigStruct()));
}

void resetCurrentInvalidConf(Robot* R)
{
  p3d_reset_current_q_inv(R->getRobotStruct());
}
