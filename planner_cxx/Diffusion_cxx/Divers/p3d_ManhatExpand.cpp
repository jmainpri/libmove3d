#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Bio-pkg.h"
#include "Collision-pkg.h"

#include <iostream>

using namespace std;

/**
 * PASS_EXT_ONLY_WHEN_ACTIVE_EXTENSION
 * This flag is TRUE if we allow the expansion
 * of the passive parameters only when the expension of
 * the active ones succeeded
*/
int PASS_EXT_WHEN_ACT_FAILED = FALSE;

/**
 * MANHAT_EXPANSION_RATIO
 * Ratio giving the amount of Manhattan
 * expansion against classical expansions when
 * the flag IS_MANHATTAN_EXPANSION is TRUE
 */
double MANHAT_EXPANSION_RATIO = 1.;

/**
 * Maximal number of try to expand
 * the passive nodes during a Manhattan
 * like expansion.
 */
int MAX_PASSIVE_EXTEND = 10;

/**
 * Get the maximal number of try to expand
 * the passive nodes during a Manhattan
 * like expansion.
 * @return: The maximal number of try to expand
 * the passive nodes
 */
int p3d_GetMaxPassiveExpand(void) {
  return MAX_PASSIVE_EXTEND;
}

/**
 * Set the maximal number of try to expand
 * the passive nodes during a Manhattan
 * like expansion.
 * @param[In] MaxPassiveExpand: Maximal number of try to expand
 * the passive nodes
 */
void p3d_SetMaxPassiveExpand(int MaxPassiveExpand) {
  MAX_PASSIVE_EXTEND = MaxPassiveExpand;
}

/**
 * p3d_GetIsPasExtWhenAct
 * Function get if we allow or not the expansion
 * of the passive parameters only when the expension of
 * the active ones succeeded
 * return:TRUE if  we allow
 * the expansion of the passive parameters only when
 * the expension of the active ones succeeded
 */
int p3d_GetIsPasExtWhenAct(void) {
  return PASS_EXT_WHEN_ACT_FAILED;
}

/**
 * p3d_GetIsPasExtWhenAct
 * Function get if we allow or not the expansion
 * of the passive parameters only when the expension of
 * the active ones succeeded
 * param[In] IsPasExtWhenAct: TRUE if  we allow
 * the expansion of the passive parameters only when
 * the expension of the active ones succeeded
 */
void p3d_SetIsPasExtWhenAct(int IsPasExtWhenAct) {
  PASS_EXT_WHEN_ACT_FAILED = IsPasExtWhenAct;
}

/**
 * p3d_GetManhattanRatio
 * Get the ratio giving the amount of Manhattan
 * expansion against classical expansions when
 * the flag IS_MANHATTAN_EXPANSION is TRUE
 */
double p3d_GetManhattanRatio(void) {
  return MANHAT_EXPANSION_RATIO;
}

/**
 * p3d_SetManhattanRatio
 * Set the ratio giving the amount of Manhattan
 * expansion against classical expansions when
 * the flag IS_MANHATTAN_EXPANSION is TRUE
 */
void p3d_SetManhattanRatio(double ManhatExpanRatio) {
  MANHAT_EXPANSION_RATIO = ManhatExpanRatio;
}


/**
 * p3d_ExpanBlockedByColl
 * Todo
 */
int p3d_ExpanBlockedByColl(p3d_rob *robotPt, configPt *qinvPt) {
  // with BIO module
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    if(!bio_get_current_q_inv(robotPt,qinvPt)) {
      return 0;
    }
    return 1;
  }
  // without BIO module
  if(!p3d_get_current_q_inv(robotPt,qinvPt)) {
    return 0;
  }
  return 1;
}

/**
* p3d_perturb_and_check_passive_params_involved_in_collision
* Todo. function no more used
*/
int p3d_perturb_and_check_passive_params_involved_in_collision(p3d_rob *robotPt,
							       configPt qinv)
{
  int processOK = FALSE;

  // with BIO module
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    // next function possibly changes q_inv
    processOK = bio_perturb_and_check_passive_params_involved_in_collision(robotPt,
									   qinv);
  }
  // without BIO module
  else {
    printf("WARNING: p3d_perturb_and_check_passive_params_involved_in_collision\
 : only works with BIO module\n");
  }

  return(processOK);
}

int selectNewJntInList(p3d_rob *robotPt, std::vector<p3d_jnt*>& joints,
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

/**
 * p3d_GetCollidingtPassiveJntList
 * Todo
 */
int getCollidingPassiveJntList(p3d_rob *robotPt, configPt qinv,
			       std::vector<p3d_jnt*>& joints)
{
  p3d_poly* polys[2];

  // with BIO module
  if(p3d_col_get_mode() == p3d_col_mode_bio) {
    p3d_jnt** passiveJoints;
    int nJoints;
    bio_get_list_of_passive_joints_involved_in_collision(
      robotPt, qinv, &nJoints, &passiveJoints);
    for(int i(0); i < nJoints; i++)
      joints.push_back(passiveJoints[i]);
    MY_FREE(passiveJoints, p3d_jnt*, nJoints);
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

void shoot_jnt_list_and_copy_into_conf(p3d_rob *robotPt, configPt qrand,
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
