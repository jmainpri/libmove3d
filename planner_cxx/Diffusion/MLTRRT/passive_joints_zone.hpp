// author: Romain Iehl <riehl@laas.fr>

#ifndef PASSIVE_JOINTS_ZONE_HPP_INCLUDED
#define PASSIVE_JOINTS_ZONE_HPP_INCLUDED

#include <vector>
#include "passive_zone.hpp"

class Robot;
typedef double* configPt;
class Configuration;
struct jnt;
typedef struct jnt p3d_jnt;

//------------------------------------------------------------------------------
// Passive zone functions
//------------------------------------------------------------------------------
// Get neighbouring bodies of a given set of bodies,
// according to some metric / maximum distance etc.
class PassiveJointsZone : public PassiveZone
{
public:
  PassiveJointsZone(Robot* R, std::vector<int> objs, double step);
  
  double cost(configPt q);
  std::vector<p3d_jnt*> joints(Configuration& conf);

protected:
  Robot* mR;
  std::vector<p3d_jnt*> mActiveSet;
  std::vector<p3d_jnt*> mPassiveSet;
  double mStep;
};


#endif
