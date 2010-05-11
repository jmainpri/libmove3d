#include "cost_space.hpp"
#include <iostream>

using std::string;

//------------------------------------------------------------------------------
CostSpace::CostSpace()
{
}
//------------------------------------------------------------------------------
double CostSpace::cost(Configuration& conf)
{
  if(!mSelectedCost.empty())
  {
    return mSelectedCost(conf);
  }
  else
  {
    std::cout << "Warning : CostSpace::cost(Configuration& conf) called, but \
      the cost function has not been set." << std::endl;
    return(1.0);
  }
}
//------------------------------------------------------------------------------
void CostSpace::setCost(string name)
{
  if(mFunctions.find(name) != mFunctions.end())
  {
    mSelectedCost = mFunctions[name];
  }
  else
  {
    std::cout << "Warning : in CostSpace::setCost(string name), could not find a cost function named " << name  << std::endl;
  }
}
//------------------------------------------------------------------------------
void CostSpace::addCost(string name, 
	     boost::function<double(Configuration&)> f)
{
  if(mFunctions.find(name) == mFunctions.end())
  {
    mFunctions[name] = f;
  }
  else
  {
    std::cout << "Warning : in CostSpace::addCost, replacing the cost function named " << name << "by another." << std::endl;
    mFunctions[name] = f;
  }
}
//------------------------------------------------------------------------------
CostSpace* global_costSpace(NULL);
//------------------------------------------------------------------------------
