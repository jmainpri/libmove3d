#ifndef COST_SPACE_HPP_INCLUDED
#define COST_SPACE_HPP_INCLUDED

#include <boost/function.hpp>
#include <map>
#include <string>

#include "ConfigSpace/configuration.hpp"
#include "ConfigSpace/localpath.hpp"
#include "Roadmap/graph.hpp"

/*!
 * Delta step cost method enum
 */
enum CostSpaceDeltaStepMethod 
{
	cs_mechanical_work,
	cs_integral,
	cs_visibility,
	cs_average,
	cs_config_cost_and_dist,
	cs_boltzman_cost
};

/*!
 * Class thats holding the CostSpace
 */
class CostSpace
{
public:
  CostSpace();
  
  // Compute the cost of the configuration conf.
  double cost(Configuration& conf);
	
  // Compute the cost of
  double cost(LocalPath& path);
  
  // Select the cost function with the given name in the map
  void setCost(std::string name);

  // Register a new cost function.
  void addCost(std::string name, 
	       boost::function<double(Configuration&)> f);
	
  // Delete a cost function
  void deleteCost(std::string name);
	
  // Get All Cost Functions
  std::vector<std::string> getAllCost();
	
  // Initializes the Cost space motion planning problem
  void initMotionPlanning(Graph* graph, Node* start, Node* goal);
	
  // Set DeltaStepCost
  void setDeltaStepMethod(CostSpaceDeltaStepMethod method) { m_deltaMethod = method; }
  
protected:
  boost::function<double(Configuration&)> mSelectedCost;
  std::map<std::string, boost::function<double(Configuration&)> > mFunctions;
	
private:
  // Compute the delta step cost
  double deltaStepCost(double cost1, double cost2, double length);
	
  // Delta
  enum CostSpaceDeltaStepMethod m_deltaMethod;
	
};

extern CostSpace* global_costSpace;

double computeIntersectionWithGround(Configuration& conf);
double computeBasicCost(Configuration& conf);

#endif
