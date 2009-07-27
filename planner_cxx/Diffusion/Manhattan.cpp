/*
 * Manhattan.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "RRT.hpp"

using namespace std;

int RRT::selectNewJntInList(p3d_rob *robotPt, std::vector<p3d_jnt*>& joints,
			   std::vector<p3d_jnt*>& oldJoints, std::vector<p3d_jnt*>& newJoints)
{
  for(uint i(0); i < joints.size(); i++)
  {
    bool found(false);
    for(uint j(0); j < oldJoints.size(); j++)
    {
      if(oldJoints[j] == joints[i])
      {
	found = true;
	break;
      }
    }
    if(!found)
    {
      newJoints.push_back(joints[i]);
      oldJoints.push_back(joints[i]);
    }
  }
  return(newJoints.size() > 0);
}

int RRT::getCollidingPassiveJntList(p3d_rob *robotPt, configPt qinv,
			       std::vector<p3d_jnt*>& joints)
{
  p3d_poly* polys[2];

  // with BIO module
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
//    p3d_jnt** passiveJoints;
//    int nJoints;
//    bio_get_list_of_passive_joints_involved_in_collision(
//      robotPt, qinv, &nJoints, &passiveJoints);
//    for(int i(0); i < nJoints; i++)
//      joints.push_back(passiveJoints[i]);
//    MY_FREE(passiveJoints, p3d_jnt*, nJoints);
  }
  // without BIO module
  else
  {
    p3d_set_and_update_this_robot_conf_without_cntrt(robotPt,qinv);
    if(p3d_col_test() <= 0)
    {
      cout << "No collision detected" << endl;
      return(false);
    }

    // NOTE: KCD only retuns the first collision pair !!!
    // NOTE: ONLY THE PASSIVE JOINT INVOLVED IN THE COLLISION IS RETURNED
    //       BUT ALL THE PARENT JOINTS SHOULD BE ALSO CONSIDERED ???
    p3d_col_get_report(0,&polys[0],&polys[1]);
    for(uint i(0); i < 2; i++)
      if(polys[i]->p3d_objPt->jnt != NULL)
	if(!p3d_jnt_get_is_active_for_planner(polys[i]->p3d_objPt->jnt))
	  joints.push_back(polys[i]->p3d_objPt->jnt);
  }
  return(joints.size() > 0);
}

void RRT::shoot_jnt_list_and_copy_into_conf(p3d_rob *robotPt, configPt qrand,
				       std::vector<p3d_jnt*>& joints)
{
  double perturb = 0.1; // NOTE: THIS SHOULD BE A PARAMETER

  // NOTE : the random shoot should be centered at q_inv !!!
  //        (doesn't matter in the case of "circular" joints)

  for(uint i(0); i < joints.size(); i++)
  {
    p3d_jnt* joint(joints[i]);
    for(int j(0); j < joint->dof_equiv_nbr; j++)
    {
      double vmin, vmax;
      double val, rval;
      p3d_jnt_get_dof_rand_bounds(joint, j, &vmin, &vmax);
      int k(joint->index_dof + j);
      if(!p3d_jnt_is_dof_circular(joint, j))
	val = p3d_random(vmin,vmax);
      else
      {
	double midrange = (vmax-vmin) / 2.0;
	// perturbation factor
	midrange *= perturb;
	rval = p3d_random(-midrange,midrange);
	val = qrand[k] + rval;
	val = MAX(MIN(val, vmax), vmin);
      }
      qrand[k] = val;
    }
  }
}
