// author: Romain Iehl <riehl@laas.fr>
#include "passive_joints_zone.hpp"

#include <vector>
#include <limits>

#include "robot.hpp"
#include "P3d-pkg.h"
#include "Collision-pkg.h"

template<class Container>
void pushBackIfValid(Container& l, p3d_jnt* j)
{
  if (j->o == NULL) {
    std::cout << "Joint " << j->name << " has no associated object." << std::endl;
  }
  else {
    l.push_back(j);
  }
}

PassiveJointsZone::PassiveJointsZone(Robot* R, std::vector<int> objs, double step) :
  mR(R),
  mStep(step)
{
  // Separate the joints/objects into the active ones and passive ones.
  // <= seems necessary, due to crappy old unused joint[0]
  for(int i(0); i <= R->getRobotStruct()->njoints; i++)
  {
    p3d_jnt* jntPt = R->getRobotStruct()->joints[i];
    for(int j=0; j<jntPt->dof_equiv_nbr; j++)
    {
      if(p3d_jnt_get_dof_is_user(jntPt, j))
      {
	if(p3d_jnt_get_dof_is_active_for_planner(jntPt,j))
	{
	  if(std::find(mActiveSet.begin(), mActiveSet.end(), jntPt) == mActiveSet.end())
	  {
	    std::list<p3d_jnt*> todo;
	    pushBackIfValid(todo, jntPt);
	    while(todo.size() > 0)
	    {
	      for(int k(0); k < todo.front()->n_next_jnt; k++)
	      {
		pushBackIfValid(todo, todo.front()->next_jnt[k]);
	      }
	      mActiveSet.push_back(todo.front());
	      todo.pop_front();
	    }
	  }
	}
	else if(std::find(mActiveSet.begin(), mActiveSet.end(), jntPt) == mActiveSet.end() &&
		std::find(mPassiveSet.begin(), mPassiveSet.end(), jntPt) == mPassiveSet.end())
	{
	  pushBackIfValid(mPassiveSet, jntPt);
	}
      }
    }
  }
}

double PassiveJointsZone::cost(configPt q)
{
  Configuration _q(mR, q);
  mR->setAndUpdate(_q);
  double minDist(std::numeric_limits<double>::max());
  for(unsigned i(0); i < mActiveSet.size(); i++) {
    for(unsigned j(0); j < mPassiveSet.size(); j++) {
      minDist = std::min(minDist, p3d_distanceObjToObj(mActiveSet[i]->o, mPassiveSet[j]->o));
    }
    minDist = std::min(minDist, p3d_distanceObjToEnv(mActiveSet[i]->o));
  }
  // std::cout << "cost: " << 1/minDist << std::endl;
  return(5 * 1/(minDist / mStep));
  //return(minDist / step());
}

std::vector<p3d_jnt*> PassiveJointsZone::joints(Configuration& conf)
{
  mR->setAndUpdate(conf);
  std::list<p3d_jnt*> remainingPassiveJoints(mPassiveSet.begin(), mPassiveSet.end());
  std::vector<p3d_jnt*> neighbourJoints;
  for(unsigned i(0); i < mActiveSet.size(); i++)
  {
    for(std::list<p3d_jnt*>::iterator it(remainingPassiveJoints.begin());
	it != remainingPassiveJoints.end();)
    {
      bool tooClose =
	p3d_col_get_mode() == p3d_col_mode_kcd ?
	p3d_distanceObjToObj(mActiveSet[i]->o, (*it)->o) < p3d_get_env_dmax() * 5 :
	pqp_tolerance(mActiveSet[i]->o, (*it)->o, p3d_get_env_dmax() * 5);
      
      if(tooClose)
      {
	neighbourJoints.push_back(*it);
	// erase the joint at this position,
	// but only after increment it for the next iteration.
	std::list<p3d_jnt*>::iterator eraseThis = it;
	it++;
	remainingPassiveJoints.erase(eraseThis);
      }
      else {
	it++;
      }
    }
  }
  return(neighbourJoints);
}
