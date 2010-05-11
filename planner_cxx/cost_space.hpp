#ifndef COST_SPACE_HPP_INCLUDED
#define COST_SPACE_HPP_INCLUDED

#include <boost/function.hpp>
#include <map>
#include <string>

#include "configuration.hpp"

class CostSpace
{
public:
  CostSpace();
  
  // Compute the cost of the configuration conf.
  double cost(Configuration& conf);
  
  // Select the cost function with the given name among the registered cost functions.
  void setCost(std::string name);

  // Register a new cost function.
  void addCost(std::string name, 
	       boost::function<double(Configuration&)> f);
  
protected:
  boost::function<double(Configuration&)> mSelectedCost;
  std::map<std::string, boost::function<double(Configuration&)> > mFunctions;
};

extern CostSpace* global_costSpace;

#endif
