// author: Romain Iehl <riehl@laas.fr>

#include "ml_bio_functions.hpp"
#include "robot.hpp"
#include "Planner-pkg.h"
#include "Bio-pkg.h"
#include "Collision-pkg.h"

LigandAtoms::LigandAtoms(Robot* R) :
  mR(R),
  mTarget(3, 0)
{
  p3d_rob* robotPt(mR->getRobotStruct());
  // Assume that we have a ligand if this is > 0
  if(robotPt->num_subrobot_ligand > 0)
  {
    // go through all objects, and get their p3d_poly pointers if
    // there is .LIG. in the object's name.
    for(int i(0); i < robotPt->no; i++)
    {
      std::string objName(robotPt->o[i]->name);
      if(objName.find(".LIG.") != std::string::npos)
      {
	for(int j(0); j < robotPt->o[i]->np; j++)
	{
	  mLigandAtoms.push_back(robotPt->o[i]->pol[j]);
	}
      }
    }
  }
  this->setTarget(this->geometricalCenter());
}

std::vector<double> LigandAtoms::geometricalCenter() const
{
  std::vector<double> center(3, 0);
  for(unsigned i(0); i < mLigandAtoms.size(); i++)
  {
    for(unsigned j(0); j < 3; j++)
    {
      center[j] += mLigandAtoms[i]->poly->pos[j][3];
    }
  }
  if(mLigandAtoms.size() > 0)
  {
    for(unsigned i(0); i < 3; i++)
    {
      center[i] /= mLigandAtoms.size();
    }
  }

  return(center);
}

void LigandAtoms::setTarget(std::vector<double> target)
{
  for(unsigned i(0); i < 3; i++)
  {
    mTarget[i] = target[i];
  }
}

double LigandAtoms::distanceToTarget() const
{
  std::vector<double> center(this->geometricalCenter());
  double x(center[0]-mTarget[0]);
  double y(center[1]-mTarget[1]);
  double z(center[2]-mTarget[2]);
  return(sqrt(x*x + y*y + z*z));
}

std::vector<p3d_jnt*> bioGetCollidingPassiveJoints(Robot* R, Configuration& conf)
{
  std::vector<p3d_jnt*> joints;
  if(p3d_col_get_mode() == p3d_col_mode_bio)
  {
    p3d_jnt** passiveJoints = NULL;
    int nJoints = 0;
    bio_get_list_of_passive_joints_involved_in_collision(
      R->getRobotStruct(), conf.getConfigStruct(), &nJoints, &passiveJoints);
    for(int i(0); i < nJoints; i++)
    {
      joints.push_back(passiveJoints[i]);
    }
    MY_FREE(passiveJoints, p3d_jnt*, nJoints);
  }
  return(joints);
}

bool bioGetCurrentInvalidConf(Robot* R, Configuration& q)
{
  return(bio_get_current_q_inv(R->getRobotStruct(), q.getConfigStruct()));
}

void bioResetCurrentInvalidConf(Robot* R)
{
  bio_reset_current_q_inv(R->getRobotStruct());
}
