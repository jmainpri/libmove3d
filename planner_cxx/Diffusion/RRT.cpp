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

	return ADDED;
}


bool RRT::checkStopConditions()
{
	if (ENV.getBool(Env::expandToGoal) && trajFound())
	{
		cout << "Success: the start and goal components are connected." << endl;
		return (true);
	}
	if (/*ENV.getBool(Env::ligandExitTrajectory)*/0)
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

shared_ptr<Configuration> RRT::diffuseOneConf(
		shared_ptr<Configuration> qCurrent)
{
	shared_ptr<LocalPath> path = shared_ptr<LocalPath> (new LocalPath(qCurrent,
			_Robot->shoot()));

	return path->configAtParam(min(path->length(), _expan->step()));
}

Node*
RRT::connectNode(Node* currentNode, LocalPath& path, double pathDelta,
                Node* directionNode, double currentCost, int& nbCreatedNodes)
{

        if ((pathDelta == 1. && directionNode))
        {
                _Graph->MergeComp(currentNode, directionNode, path.length());
                return (directionNode);
        }
        else
        {
                Node* newNode = _Graph->insertNode(path.getEnd(), currentNode,
                                currentCost, _expan->step());
                nbCreatedNodes++;
                return (newNode);
        }
}

int RRT::expandProcess(Node* expansionNode,
                shared_ptr<Configuration> directionConfig,
                Node* directionNode,
                Env::expansionMethod method)
{
        bool extensionSucceeded(false);
        bool failed(false);
        int nbCreatedNodes(0);
        Node fromNode(*expansionNode);
        Node* extensionNode(NULL);
        shared_ptr<LocalPath> directionLocalpath;
        double positionAlongDirection(0.);
        shared_ptr<LocalPath> extensionLocalpath;
        double extensionCost(0.);
        bool firstIteration(true);

        // Perform extension toward directionConfig
        // Additional nodes creation in the nExtend case, but without checking for expansion control
        while (firstIteration || (method == Env::nExtend && !failed
                        && positionAlongDirection < 1.))
        {
                directionLocalpath = shared_ptr<LocalPath> (new LocalPath(
                                fromNode.getConfiguration(), directionConfig));
                extensionSucceeded = _expan->nextStep(*directionLocalpath, directionNode,
                                positionAlongDirection, extensionLocalpath, method);
                failed |= !extensionSucceeded;

                // Transition test for cost spaces, increase temperature in case of failure
                if (!failed && ENV.getBool(Env::isCostSpace))
                {
                        extensionCost = extensionLocalpath->getEnd()->cost();
                }
                // Expansion Control
                if (firstIteration && !failed)
                {
                        if (ENV.getBool(Env::expandControl) && !_expan->expandControl(
                                        *directionLocalpath, positionAlongDirection, *expansionNode))
                                failed = true;
                }
                // Add node to graph if everything succeeded
                if (!failed)
                {
                        extensionNode = connectNode(&fromNode, *extensionLocalpath,
                                        positionAlongDirection, directionNode, extensionCost,
                                        nbCreatedNodes);
                }
                if (firstIteration && failed)
                	_expan->expansionFailed(*expansionNode);

                if (!failed)
                {
					fromNode = *extensionNode;
				}
				firstIteration = false;

        }

        if (ENV.getBool(Env::isCostSpace) && ENV.getInt(Env::CostMethodChoice)
                        == MAXIMAL_THRESHOLD)
        {
                p3d_updateCostThreshold();
        }

        return nbCreatedNodes;
}

int RRT::expandOneStep(Node* fromComp, Node* toComp)
{
	// ML-RRT expansion case
	if (ENV.getBool(Env::isManhattan) && !(this->manhattanSamplePassive()))
	{
		return passiveExpandOneStep(fromComp,toComp);
	}
	// Standard expansion case
	else
	{
	    Node* directionNode(NULL);
	    Node* expansionNode(NULL);
	    shared_ptr<Configuration> directionConfig;

		// get direction
		directionConfig = fromComp->getExpansionDirection(toComp, true,
				directionNode);
		// get node for expansion toward direction
		expansionNode = fromComp->getExpansionNode(directionConfig, ENV.getInt(
				Env::DistConfigChoice));
		// expansion
		return expandProcess(expansionNode, directionConfig,
				directionNode, ENV.getExpansionMethod());
	}
}

uint RRT::run()
{
	//   p3d_InitRun(_Graph->get_graph(), _Start->get_node(),
	// 	      _Goal ? _Goal->get_node() : NULL);

	if (ENV.getBool(Env::isCostSpace) && (ENV.getExpansionMethod()
			== Env::Connect))
	{
		cout
				<< "Warning: Connect expansion strategy \
		is usually unadapted for cost spaces\n"
				<< endl;
		return (0);
	}

	if ((ENV.getBool(Env::biDir) || ENV.getBool(Env::expandToGoal))
			&& _Start->getConfiguration()->equal(*_Goal->getConfiguration()))
	{
		cout << "Tree Expansion failed: root nodes are the same" << endl;
		return (0);
	}

	int NbCurCreatedNodes = 0;
	int NbTotCreatedNodes = 0;
	Node* fromNode = _Start;
	Node* toNode = _Goal;

	if (ENV.getBool(Env::expandToGoal))
		_Start->connectNodeToCompco(_Goal, _expan->step());

	while (!checkStopConditions())
	{
		// Do not expand in the case of a balanced bidirectional expansion,
		// if the components are unbalanced.
		if (!(ENV.getBool(Env::biDir) && ENV.getBool(Env::expandBalanced)
				&& (fromNode->getCompcoStruct()->nnode
						> toNode->getCompcoStruct()->nnode + 2)))
		{
			NbCurCreatedNodes = (ENV.getBool(Env::biDir) ?
					expandOneStep(fromNode, toNode) :
						expandOneStep(_Start, _Goal));
			if (NbCurCreatedNodes != 0)
			{
				if (ENV.getBool(Env::drawGraph))
				{
					(*_draw_func)();
				}
				NbTotCreatedNodes += NbCurCreatedNodes;
				_nbConscutiveFailures = 0;

				if (ENV.getBool(Env::expandToGoal))
				{
					if ((ENV.getBool(Env::biDir) ? toNode : _Goal)->connectNodeToCompco(
									_Graph->getLastnode(), _expan->step()))
					{
						//						printf("connected\n");
					}
				}
			}
			else
			{
				_nbConscutiveFailures++;
			}
		}
		if (ENV.getBool(Env::biDir))
			swap(fromNode, toNode);
	}
	if (ENV.getBool(Env::drawGraph))
	{
		(*_draw_func)();
	}
	return (NbTotCreatedNodes);
}

