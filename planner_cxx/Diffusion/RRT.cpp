//
// C++ Implementation: rrt
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "RRT.hpp"

using namespace std;
using namespace tr1;

RRT::RRT(Robot* R, Graph* G) :
        TreePlanner(R,G)
{
	_nbConscutiveFailures = 0;
}

RRT::~RRT()
{
}

/**
 * Checks out the Stop condition
 */
bool RRT::checkStopConditions()
{
	if(TreePlanner::checkStopConditions())
	{
		return true;
	}

 return false;

}

/**
 * Checks out the preconditions
 */
bool RRT::preConditions()
{
	cout << "Entering RRT preCondition" << endl;
	if(TreePlanner::preConditions())
	{
		cout << "RRT.preCondition()" << endl;
		if (ENV.getBool(Env::expandToGoal))
		{
			if(trajFound())
			{
				cout << "Start And Goal in same component" << endl;
				return true;
			}

			LocalPath direct(_Start->getConfiguration(), _Goal->getConfiguration());
			if (direct.getValid())
			{
				connectNodeToCompco(_Start,_Goal);

				cout << "Direct connection" << endl;
				return true;
			}
		}
		return true;
	}
	else
	{
		return false;
	}
}

/**
 * Initializes an RRT Planner
 */
int  RRT::init()
{

	int added = TreePlanner::init();

	_expan = new TreeExpansionMethod(_Graph);

	setInit(true);

	return added;
}

/**
 * Three phases One Step Expansion
 *  - Direction
 *  - Node
 *  - Process
 *
 *  @param fromComp the component which is expanded
 *  @param toComp the goal component
 */
int RRT::expandOneStep(Node* fromComp, Node* toComp)
{
	// ML-RRT expansion case
/*	if (ENV.getBool(Env::isManhattan) && !(this->manhattanSamplePassive()))
	{
		return passiveExpandOneStep(fromComp, toComp);
	}
	// Standard expansion case
	else
	{*/
		Node* directionNode(NULL);
		Node* expansionNode(NULL);
		shared_ptr<Configuration> directionConfig;

		// get direction
		directionConfig = _expan->getExpansionDirection(fromComp, toComp, true,
				directionNode);

		// get node for expansion
		expansionNode = _expan->getExpansionNode(fromComp, directionConfig,
				ENV.getInt(Env::DistConfigChoice));

		// expansion
		return _expan->expandProcess(expansionNode, directionConfig, directionNode,
				ENV.getExpansionMethod());
//	}
}

