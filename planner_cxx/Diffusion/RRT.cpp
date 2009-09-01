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

RRT::RRT(WorkSpace* WS) :
	Planner(WS)
{
	_nbConscutiveFailures = 0;
}

RRT::~RRT()
{
}

int RRT::init()
{

	int ADDED = 0;
	Planner::init();
	_nbConscutiveFailures = 0;
	ADDED += Planner::setStart(_Robot->getInitialPosition());
	ADDED += Planner::setGoal(_Robot->getGoTo());
	_stop_func = fct_stop;
	_draw_func = fct_draw;
	_expan = new TreeExpansionMethod(_Graph);
	_Init = true;

	if (ENV.getBool(Env::isCostSpace) == true)
	{
		p3d_InitSpaceCostParam(_Graph->getGraphStruct(),
				_Start->getNodeStruct(), _Goal->getNodeStruct());
	}

	return ADDED;
}

bool RRT::preConditions()
{
	if (ENV.getBool(Env::isCostSpace) && (ENV.getExpansionMethod()
			== Env::Connect))
	{
		cout
				<< "Warning: Connect expansion strategy \
		is usually unadapted for cost spaces\n"
				<< endl;
	}

	if ((ENV.getBool(Env::biDir) || ENV.getBool(Env::expandToGoal))
			&& _Start->getConfiguration()->equal(*_Goal->getConfiguration()))
	{
		cout << "Tree Expansion failed: root nodes are the same" << endl;
		return false;
	}

	if (ENV.getBool(Env::expandToGoal))
	{
		connectNodeToCompco(_Start, _Goal);
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

	if (ENV.getBool(Env::expandToGoal))
	{
		LocalPath direct(_Start->getConfiguration(), _Goal->getConfiguration());
		if (direct.getValid())
		{
			cout << "Direct connection" << endl;
			return false;
		}
	}
}

bool RRT::checkStopConditions()
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

bool RRT::connectNodeToCompco(Node* N, Node* CompNode)
{
	if (ENV.getBool(Env::isCostSpace))
	{
		return costConnectNodeToComp(N, CompNode);
	}
	else
	{
		return p3d_ConnectNodeToComp(N->getGraph()->getGraphStruct(),
				N->getNodeStruct(), CompNode->getCompcoStruct());
	}
}

int RRT::expandOneStep(Node* fromComp, Node* toComp)
{
	// ML-RRT expansion case
	if (ENV.getBool(Env::isManhattan) && !(this->manhattanSamplePassive()))
	{
		return passiveExpandOneStep(fromComp, toComp);
	}
	// Standard expansion case
	else
	{
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
		return _expan->expandProcess(expansionNode, directionConfig,
				directionNode, ENV.getExpansionMethod());
	}
}

uint RRT::run()
{

	int NbCurCreatedNodes = 0;
	int NbTotCreatedNodes = 0;

	Node* fromNode = _Start;
	Node* toNode = _Goal;

	while (!checkStopConditions())
	{
		// Do not expand in the case of a balanced bidirectional expansion,
		// if the components are unbalanced.
		if (!(ENV.getBool(Env::biDir) && ENV.getBool(Env::expandBalanced)
				&& (fromNode->getCompcoStruct()->nnode
						> toNode->getCompcoStruct()->nnode + 2)))
		{
			// expand one way
			// one time (Main function of RRT
			NbCurCreatedNodes = expandOneStep(fromNode, toNode);

			if (NbCurCreatedNodes > 0)
			{
				if (ENV.getBool(Env::drawGraph))
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
						cout << "nb Comp : " << _Graph->getGraphStruct()->ncomp
								<< endl;
						cout << "connected" << endl;
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

