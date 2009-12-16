/*
 * TreePlanner.cpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#include "TreePlanner.hpp"

using namespace std;

/**
 * Constructor
 */
TreePlanner::TreePlanner(Robot* R, Graph* G) :
        Planner(R,G),
	_nbConscutiveFailures(0)
{

}

/**
 * Destructor
 */
TreePlanner::~TreePlanner()
{

}

int TreePlanner::init()
{
	int ADDED = 0;
	Planner::init();
	_nbConscutiveFailures = 0;

	ADDED += Planner::setStart(_Robot->getInitialPosition());
	ADDED += Planner::setGoal(_Robot->getGoTo());

	_Graph->setStart(_Start);
	_Graph->setGoal(_Goal);

	return ADDED;
}
/**
 * Checks out that the plannification
 * problem fits such requirement
 */
bool TreePlanner::preConditions()
{
	cout << "Entering preCondition" << endl;

	if (ENV.getBool(Env::isCostSpace) && (ENV.getExpansionMethod()
			== Env::Connect))
	{
		cout
				<< "Warning: Connect expansion strategy \
		is usually unadapted for cost spaces\n"
				<< endl;
	}

	if ((ENV.getBool(Env::biDir) || ENV.getBool(Env::expandToGoal))
			&& (*_Start->getConfiguration() == *_Goal->getConfiguration()) )
	{
		cout << "Tree Expansion failed: root nodes are the same" << endl;
		return false;
	}

	if (_Start->getConfiguration()->IsInCollision())
	{
		cout << "Start in collision" << endl;
		return false;
	}

	if (ENV.getBool(Env::expandToGoal)
			&& _Goal->getConfiguration()->IsInCollision())
	{
		cout << "Goal in collision" << endl;
		return false;
	}

	cout << "Tree Planner precondition: OK" << endl;
	return true;
}

/**
 * Checks out if the plannification
 * problem has reach its goals
 */
bool TreePlanner::checkStopConditions()
{
	if (ENV.getBool(Env::expandToGoal) && trajFound())
	{
		cout << "Success: the start and goal components are connected." << endl;
		return (true);
	}
	if (/*ENV.getBool(Env::ligandExitTrajectory)*/false)
	{
		double d(_Start->getConfiguration()->dist(
				*_Graph->getLastnode()->getConfiguration()));
		if (d > 12.0)
		{
			ENV.setBool(Env::expandToGoal, true);
			_Goal = _Graph->getLastnode();
			_Graph->getGraphStruct()->search_goal = _Goal->getNodeStruct();
			_Goal->getNodeStruct()->rankFromRoot = 1;
			_Goal->getNodeStruct()->type = ISOLATED;
			_Robot->getGoTo() = _Goal->getConfiguration()->copy();
			cout << "Success: distance from start is " << d << endl;
			return (true);
		}
	}

	if (_Start->maximumNumberNodes())
	{
		cout
				<< "Failure: the maximum number of nodes in the start component is reached."
				<< endl;
		return (true);
	}

	if (ENV.getBool(Env::biDir))
	{
		if (_Goal->maximumNumberNodes())
		{
			cout
					<< "Failure: the maximum number of nodes in the goal component is reached."
					<< endl;
			return (true);
		}
	}

	if (_Graph->getNbNode() >= ENV.getInt(Env::maxNodeCompco))
	{
		cout << "Failure: the maximum number of nodes is reached." << endl;
		return (true);
	}

	if (_nbConscutiveFailures > ENV.getInt(Env::NbTry))
	{
		cout
				<< "Failure: the maximum number of consecutive failures to expand a component is reached."
				<< endl;
		return (true);
	}

	if (!(*_stop_func)())
		p3d_SetStopValue(true);
	if (p3d_GetStopValue())
	{
		cout << "RRT expansion cancelled." << endl;
		return (true);
	}
	return (false);
}

/**
 * Tries to connect one node from
 * one component to the other
 */
bool TreePlanner::connectNodeToCompco(Node* N, Node* CompNode)
{
	return p3d_ConnectNodeToComp(N->getGraph()->getGraphStruct(),
				N->getNodeStruct(), CompNode->getCompcoStruct());
}

/**
 * Main Function of the Tree Planner,
 * Bi-Directionality is handled here
 */
uint TreePlanner::run()
{
//	cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << endl;
	if(!preConditions())
	{
		return 0;
	}

	cout << "pre cond passed" << endl;

	int NbCurCreatedNodes = 0;
	int NbTotCreatedNodes = 0;

	Node* fromNode = _Start;
	Node* toNode = _Goal;

	while (!checkStopConditions())
	{
                ENV.setInt(Env::progress,(int)(_Graph->getNbNode()/ENV.getInt(Env::maxNodeCompco)));
//                cout << "progress = " << ENV.getInt(Env::progress) << endl;
//                cout << (int)(_Graph->getNbNode()/ENV.getInt(Env::maxNodeCompco)) << endl;
//		cout << "ENV.getInt(Env::maxNodeCompco) = " << ENV.getInt(Env::maxNodeCompco) << endl;
		// Do not expand in the case of a balanced bidirectional expansion,
		// if the components are unbalanced.
		if (!(ENV.getBool(Env::biDir) && ENV.getBool(Env::expandBalanced)
				&& (fromNode->getCompcoStruct()->nnode
						> toNode->getCompcoStruct()->nnode + 2)))
		{
			// expand one way
			// one time (Main function of Tree like planners
			NbCurCreatedNodes = expandOneStep(fromNode,toNode);

			if (NbCurCreatedNodes > 0)
			{
                                if(ENV.getBool(Env::drawGraph))
				{
					(*_draw_func)();
				}

				NbTotCreatedNodes += NbCurCreatedNodes;
				_nbConscutiveFailures = 0;

				if (ENV.getBool(Env::expandToGoal))
				{
					// If it expands towards a goal
					// Tries to link with local method
					if (connectNodeToCompco(_Graph->getLastnode(), toNode))
					{
//						cout << "nb Comp : " << _Graph->getGraphStruct()->ncomp<< endl;
						cout << "connected" << endl;
//                                                return (NbTotCreatedNodes);
					}
				}
			}
			else
			{
				_nbConscutiveFailures++;
			}
		}
		if (ENV.getBool(Env::biDir))
		{
			swap(fromNode, toNode);
		}
	}
	if (ENV.getBool(Env::drawGraph))
	{
		(*_draw_func)();
	}
	return (NbTotCreatedNodes);
}
