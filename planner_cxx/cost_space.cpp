#include "cost_space.hpp"
#include <iostream>

#include "planningAPI.hpp"

#include "P3d-pkg.h"
#include "GroundHeight-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

//using std::string;
//------------------------------------------------------------------------------
CostSpace* global_costSpace(NULL);

//------------------------------------------------------------------------------
CostSpace::CostSpace() : m_deltaMethod(integral)
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
void CostSpace::deleteCost(string name)
{
	if(mFunctions.find(name) != mFunctions.end())
	{
		std::map< string, boost::function<double(Configuration&)> >::iterator it;
		it=mFunctions.find(name);
		mFunctions.erase (it);
	}
	else 
	{
		std::cout << "Warning : in CostSpace::deleteCost, the cost function " << name << " does not exist." << std::endl;
	}
}
//------------------------------------------------------------------------------
std::vector<string> CostSpace::getAllCost()
{
	std::vector<string> functions;
	std::map< string, boost::function<double(Configuration&)> >::iterator it;
	
	for ( it=mFunctions.begin() ; it != mFunctions.end(); it++ )
		functions.push_back((*it).first);
	
	return functions;
}

//------------------------------------------------------------------------------
/**
 * ComputeDeltaStepCost
 * Compute the cost of a portion of path */
double CostSpace::deltaStepCost(double cost1, double cost2, double length)
{
    double epsilon = 0.002;
    //double alpha;
    double kb = 0.00831, temp = 310.15;
	
    //length *= ENV.getDouble(Env::KlengthWeight);
	double powerOnIntegral = ENV.getDouble(Env::KlengthWeight);
	
    if ( ENV.getBool(Env::isCostSpace) )
    {
        switch (m_deltaMethod)
        {
			case mechanical_work:
				double cost;
				if (cost2 > cost1)
				{
					cost = length * epsilon + cost2 - cost1;
				}
				else
				{
					cost = epsilon * length;
				}
				return cost;
				
			case integral:
			case visibility:
				
				return pow(((cost1 + cost2)/2),powerOnIntegral)*length;
				
				//    case MECHANICAL_WORK:
				//
				//      if(cost2 > cost1)
				//      {
				//    	  return length*(epsilon+ cost2 - cost1);
				//      }
				//      else
				//      {
				//		  return epsilon*length;
				//      }
				
			case average:
				return (cost1 + cost2) / 2.;
				
				//			case config_cost_and_dist:
				//				alpha = p3d_GetAlphaValue();
				//				return alpha * (cost1 + cost2) / 2. + (1. - alpha) * length;
				
			case boltzman_cost:
				
				if (cost2 > cost1)
					return 1;
				return		1/exp(ENV.getInt(Env::maxCostOptimFailures)*(cost2-cost1)/(kb*temp));
				
			default:
				std::cout << "Warning: " << __func__ <<  std::endl;
        }
    }
    //no cost function
    return length;
}
//----------------------------------------------------------------------
double CostSpace::cost(LocalPath& path)
{
	if (!ENV.getBool(Env::isCostSpace))
	{
		return path.getParamMax();
	}
	
	double Cost = 0;
	
	double currentCost, prevCost;
	Eigen::Vector3d taskPos;
	Eigen::Vector3d prevTaskPos(0, 0, 0);
	
	double currentParam = 0;
	
	double DeltaStep = path.getResolution();
	double CostDistStep = DeltaStep;
	
	//                cout << "DeltaStep  = "  << DeltaStep << endl;
	unsigned int nStep = path.getParamMax() / DeltaStep;
	
//	cout << "nStep = " << nStep <<  endl;
	
	shared_ptr<Configuration> confPtr;
	prevCost = path.getBegin()->cost();
//	cout << "prevCost = " << prevCost << endl;
	
#ifdef LIGHT_PLANNER
	// If the value of Env::HRIPlannerWS changes while executing this
	// function, it could lead to the use of the incorrectly initialized 
	// prevTaskPos variable, and this triggers a compiler warning.
	// So, save the value of Env::HRIPlannerWS in a local variable.
	const bool isHRIPlannerWS = ENV.getBool(Env::HRIPlannerWS);
	//const bool isHRIPlannerWS = false;
	if(isHRIPlannerWS)
	{
		prevTaskPos = path.getBegin()->getTaskPos();
	}
#endif
	// Case of task space
	vector<double> Pos;
	
	//                cout << "nStep =" << nStep << endl;
	for (unsigned int i = 0; i < nStep; i++)
	{
		currentParam += DeltaStep;
		
		confPtr = path.configAtParam(currentParam);
		currentCost = cost(*confPtr);
		//cout << "CurrentCost = " << currentCost << endl;
#ifdef LIGHT_PLANNER		
		// Case of task space
		if(isHRIPlannerWS)
		{
			taskPos = confPtr->getTaskPos();
			CostDistStep = ( taskPos - prevTaskPos ).norm();
			prevTaskPos = taskPos;
		}
#endif		
		Cost += deltaStepCost(prevCost, currentCost, CostDistStep);
		
		prevCost = currentCost;
	}
	
//	cout << "Path Cost = " << Cost << endl;
	
	return Cost;
}
//----------------------------------------------------------------------
extern void* GroundCostObj;

double computeIntersectionWithGround(Configuration& conf)
{
	double cost(0);
	if(GroundCostObj)
	{
		GHintersectionVerticalLineWithGround(GroundCostObj, conf.getConfigStruct()[6],
											 conf.getConfigStruct()[7], &cost);
	}
	return(cost);
}
//----------------------------------------------------------------------
void CostSpace::initMotionPlanning(Graph* graph, Node* start, Node* goal)
{
	start->getNodeStruct()->temp = ENV.getDouble(Env::initialTemperature);
	start->getNodeStruct()->comp->temperature = ENV.getDouble(Env::initialTemperature);
	start->getNodeStruct()->nbFailedTemp = 0;
	
	p3d_SetGlobalNumberOfFail(0);
	
	//  GlobalNbDown = 0;
	//  Ns->NbDown = 0;
	p3d_SetNodeCost(graph->getGraphStruct(),
					start->getNodeStruct(), 
					start->getConfiguration()->cost());
	
	p3d_SetCostThreshold(start->getNodeStruct()->cost);
	
	p3d_SetInitCostThreshold(start->getNodeStruct()->cost);
	
	if ( ENV.getBool(Env::expandToGoal) && (goal != NULL))
	{
		goal->getNodeStruct()->temp					= ENV.getDouble(Env::initialTemperature);
		goal->getNodeStruct()->comp->temperature	= ENV.getDouble(Env::initialTemperature);
		start->getNodeStruct()->temp				= ENV.getDouble(Env::initialTemperature);
		goal->getNodeStruct()->nbFailedTemp = 0;
		//    Ng->NbDown = 0;
		p3d_SetNodeCost(graph->getGraphStruct(), 
						goal->getNodeStruct(), 
						goal->getConfiguration()->cost());
		
		p3d_SetCostThreshold(MAX(start->getNodeStruct()->cost, 
								  goal->getNodeStruct()->cost));
		
		//        p3d_SetCostThreshold(MAX(
		//								p3d_GetNodeCost(this->getStart()->getNodeStruct()), 
		//								p3d_GetNodeCost(this->getGoal()->getNodeStruct()) ));
		
		p3d_SetAverQsQgCost(
							( graph->getGraphStruct()->search_start->cost
							 +graph->getGraphStruct()->search_goal->cost) / 2.);
	}
	else
	{
		p3d_SetCostThreshold(start->getNodeStruct()->cost);
		p3d_SetInitCostThreshold(start->getNodeStruct()->cost );
		p3d_SetAverQsQgCost(graph->getGraphStruct()->search_start->cost);
	}
}
