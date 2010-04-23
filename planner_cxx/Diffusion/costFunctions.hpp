#ifndef COST_FUNCTIONS_HPP
#define COST_FUNCTIONS_HPP

class Configuration;
struct rob;

class costType
{
public:
  enum t
    {
      costMap2D,
      flexibleSidechainsAvoidance,
      passiveAndObstaclesAvoidance,
      passiveAndObstaclesProximity,
      BALLmmff94
    };
};

void initCostFunctions(struct rob* robot);

void setCostFunction(costType::t type);

double computeCost(Configuration& conf);

#endif
