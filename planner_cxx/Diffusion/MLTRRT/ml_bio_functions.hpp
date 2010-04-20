// author: Romain Iehl <riehl@laas.fr>

#ifndef ML_BIO_FUNCTIONS_HPP_INCLUDED
#define ML_BIO_FUNCTIONS_HPP_INCLUDED

#include <tr1/memory>
#include <vector>
#include "configuration.hpp"

class Robot;
struct p3d_poly;

class LigandAtoms
{
public:
  LigandAtoms(Robot* R);

  std::vector<double> geometricalCenter() const;
  void setTarget(std::vector<double> target);
  double distanceToTarget() const;

protected:
  Robot* mR;
  std::vector<p3d_poly*> mLigandAtoms;
  std::vector<double> mTarget;
};

std::vector<p3d_jnt*> bioGetCollidingPassiveJoints(Robot* R, Configuration& conf);

bool bioGetCurrentInvalidConf(Robot* R, Configuration& q);

void bioResetCurrentInvalidConf(Robot* R);

#endif
