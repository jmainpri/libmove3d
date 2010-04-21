// author: Romain Iehl <riehl@laas.fr>
#include "passive_shoot_functions.hpp"

#include "configuration.hpp"
#include "P3d-pkg.h"

void perturbCircularJoints_ShootOthers(Configuration& qrand,
				       std::vector<p3d_jnt*>& joints)
{
  assert(joints.size() > 0);
  // NOTE: THIS SHOULD BE A PARAMETER
  //double perturb = 0.1;
  double perturb = 0.5;

  // NOTE : the random shoot should be centered at q_inv !!!
  // (doesn't matter in the case of "circular" joints)

  for(uint i(0); i < joints.size(); i++)
  {
    p3d_jnt* joint(joints[i]);
    for(int j(0); j < joint->dof_equiv_nbr; j++)
    {
      double vmin(0), vmax(0);
      double val(0);
      p3d_jnt_get_dof_rand_bounds(joint, j, &vmin, &vmax);
      assert(vmax - vmin > 0);
      int k(joint->index_dof + j);
      if(!p3d_jnt_is_dof_circular(joint, j))
	val = p3d_random(vmin,vmax);
      else
      {
	val = p3d_random(MAX(vmin, qrand.getConfigStruct()[k] - perturb),
			  MIN(vmax, qrand.getConfigStruct()[k] + perturb));
      }
      qrand.getConfigStruct()[k] = val;
    }
  }
}
