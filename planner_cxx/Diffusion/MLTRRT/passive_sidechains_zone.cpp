// author: Romain Iehl <riehl@laas.fr>
#include "passive_sidechains_zone.hpp"

#include "Bio-pkg.h"

#define _DEBUG_MSG(format, args...)					\
  {									\
    if(mDebug)								\
    {									\
      char* format2 = new char[strlen(format) + (8 + 1)*sizeof(char)];	\
      strcpy(format2, "%i - ");						\
      strcat(format2, format);						\
      printf(format2, __LINE__ , ##args);				\
      delete format2;							\
    }									\
  }


PassiveSidechainsZone::PassiveSidechainsZone(p3d_rob* robotPt) :
  mR(robotPt),
  mDebug(false)
{
  if(getenv("SCH_DEBUG"))
  {
    mDebug = true;
  }
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

  for(int cur_jnt_number(0); cur_jnt_number < robotPt->njoints; cur_jnt_number++)
  {
    p3d_jnt* cur_jnt = robotPt->joints[cur_jnt_number];
    if(cur_jnt->bio_jnt_type == BIO_GAMMA_JNT &&
       cur_jnt->dof_data[0].vmin != cur_jnt->dof_data[0].vmax &&
       p3d_jnt_get_dof_is_user(cur_jnt, 0))
    {
      // Add all polyhedras of the joint (ie atoms) to the mAtoms vector.
      for(int i(0); i < cur_jnt->o->np; i++)
      {
	mAtoms.push_back(cur_jnt->o->pol[i]);
	// and also to the poly->AA mapping.
	mPolyToAA[cur_jnt->o->pol[i]] = cur_jnt->bio_AAnumber;
      }
      // if this is a new AA, add its first joint to the mFirstJoints vector
      // and to the mAAToJoint mapping.
      if(mAAToJoint.find(cur_jnt->bio_AAnumber) == mAAToJoint.end())
      {
	mFirstJoints.push_back(cur_jnt);
	mAAToJoint[cur_jnt->bio_AAnumber] = cur_jnt;
      }
    }
  }

  _DEBUG_MSG("Initialized PassiveSidechainsZone:\ntotal ligand atoms: %i\ntotal sidechains: %i\n", mLigandAtoms.size(), mFirstJoints.size());
}

unsigned PassiveSidechainsZone::count()
{
  return(mFirstJoints.size());
}

const std::vector<p3d_jnt*>& PassiveSidechainsZone::firstJoints()
{
  return(mFirstJoints);
}

double PassiveSidechainsZone::minDist()
{
  double min(10e9);
  for(unsigned i(0); i < mLigandAtoms.size(); i++)
  {
    for(unsigned j(0); j < mAtoms.size(); j++)
    {
      min = std::min(min, this->poly_polyDistance(mLigandAtoms[i], mAtoms[j]));
    }
  }
  return(min);
}

std::vector<p3d_jnt*> PassiveSidechainsZone::joints(Configuration& conf)
{
  std::set<p3d_jnt*> joints;
  mR.setAndUpdate(conf);
  for(unsigned i(0); i < mLigandAtoms.size(); i++)
  {
    for(unsigned j(0); j < mAtoms.size(); j++)
    {
      double dist = this->poly_polyDistance(mLigandAtoms[i], mAtoms[j]);
      if(dist < 1.0)
      {
	p3d_jnt* joint = mAtoms[j]->p3d_objPt->jnt;
	if(joints.find(joint) == joints.end())
	{
	  _DEBUG_MSG("adding new joint %s, because %s and %s are %f angstroms apart.\n", joint->name, mAtoms[j]->p3d_objPt->name, mLigandAtoms[i]->p3d_objPt->name, dist);
	  joints.insert(mAtoms[j]->p3d_objPt->jnt);
	}
      }
    }
  }
  //std::cout << "close joints size : " << joints.size() << std::endl;
  return(std::vector<p3d_jnt*>(joints.begin(), joints.end()));
}
