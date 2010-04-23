// authors :
//   Romain Iehl <riehl@laas.fr>
//   Ron Alterovitz

#ifndef BALL_ENERGY_HPP
#define BALL_ENERGY_HPP

struct env;
struct BallEnergyData;
class Configuration;

class BallEnergy
{
public:
  BallEnergy(struct env* env);

  void update(Configuration& conf);
  
  double computeEnergy(Configuration& conf);

  double computeEnergy();

protected:
  BallEnergyData* d;
};

#endif
