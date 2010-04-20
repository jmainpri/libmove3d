// author: Romain Iehl <riehl@laas.fr>

#ifndef PASSIVE_ZONE_HPP_INCLUDED
#define PASSIVE_ZONE_HPP_INCLUDED

#include <vector>

class Configuration;
struct jnt;
typedef struct jnt p3d_jnt;

class PassiveZone
{
public:
  virtual ~PassiveZone() {}
  virtual std::vector<p3d_jnt*> joints(Configuration& conf) = 0;
};

#endif
