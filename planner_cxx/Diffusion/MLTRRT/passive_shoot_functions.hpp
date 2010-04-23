// author: Romain Iehl <riehl@laas.fr>

#ifndef PASSIVE_SHOOT_FUNCTIONS_HPP
#define PASSIVE_SHOOT_FUNCTIONS_HPP

#include <vector>

class Configuration;
struct jnt;

void perturbCircularJoints_ShootOthers(Configuration& qrand,
				       std::vector<struct jnt*>& joints);

#endif
