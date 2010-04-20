// author: Romain Iehl <riehl@laas.fr>

#ifndef ML_MECHANICAL_FUNCTIONS_HPP_INCLUDED
#define ML_MECHANICAL_FUNCTIONS_HPP_INCLUDED

#include <vector>
#include "configuration.hpp"

class Robot;

std::vector<p3d_jnt*> getCollidingPassiveJoints(Robot* R, Configuration& conf);

bool getCurrentInvalidConf(Robot* R, Configuration& q);

void resetCurrentInvalidConf(Robot* R);

#endif
