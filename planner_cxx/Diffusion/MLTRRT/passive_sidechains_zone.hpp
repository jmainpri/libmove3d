// author: Romain Iehl <riehl@laas.fr>

#ifndef PASSIVE_SIDECHAINS_ZONE_HPP_INCLUDED
#define PASSIVE_SIDECHAINS_ZONE_HPP_INCLUDED

#include <algorithm>
#include <vector>
#include <map>

#include "passive_zone.hpp"
#include "Collision-pkg.h"
#include "robot.hpp"

class PassiveSidechainsZone : public PassiveZone
{
public:
  PassiveSidechainsZone(p3d_rob* robotPt);

  unsigned count();

  const std::vector<p3d_jnt*>& firstJoints();

  inline double poly_polyDistance(p3d_poly* p1, p3d_poly* p2)
  {
    double x(p1->poly->pos[0][3] - p2->poly->pos[0][3]);
    double y(p1->poly->pos[1][3] - p2->poly->pos[1][3]);
    double z(p1->poly->pos[2][3] - p2->poly->pos[2][3]);
    return(std::max(0.0, sqrt(x*x + y*y + z*z) - p1->r - p2->r));
  }

  double minDist();

  std::vector<p3d_jnt*> joints(Configuration& conf);

protected:
  Robot mR;
  std::vector<p3d_jnt*> mFirstJoints;
  std::map<int, p3d_jnt*> mAAToJoint;
  std::vector<p3d_poly*> mAtoms;
  std::vector<p3d_poly*> mLigandAtoms;
  std::map<p3d_poly*, int> mPolyToAA;
  bool mDebug;
};


#endif
