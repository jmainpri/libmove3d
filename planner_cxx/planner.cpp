//
// C++ Implementation: planner
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "planner.hpp"

using namespace std;
using namespace tr1;

Planner::Planner(WorkSpace* WS)
{
	_Start = NULL;
	_Goal = NULL;
	_Robot = NULL;
	_Graph = NULL;
	_WorkSpace = WS;
	_Init = false;
	_stop_func = fct_stop;
	_draw_func = fct_draw;
}

Planner::~Planner()
{
}

WorkSpace* Planner::getWorkspace()
{
	return _WorkSpace;
}

bool Planner::trajFound()
{
	return (_Goal ? _Start->inSameComponent(_Goal) : false);
}

Robot* Planner::getActivRobot()
{
	return _WorkSpace->getActivEnvironnement()->getActivRobot();
}

void Planner::setRobot(Robot* R)
{
	_Robot = R;
}

Graph* Planner::getActivGraph()
{
	return _Graph;
}

void Planner::setGraph(Graph* G)
{
	_Graph = G;
}

Node* Planner::getStart()
{
	return _Start;
}

Node* Planner::getGoal()
{
	return _Goal;
}

bool Planner::getInit()
{
	return _Init;
}

int Planner::init()
{
	Robot* R = getActivRobot();

	Graph* G = R->getActivGraph();

	if (G == NULL)
	{
		G = R->newGraph();
		R->getRobotStruct()->GRAPH = G->getGraphStruct();
	}
	this->setGraph(G);
	this->setRobot(R);

	_stop_func = fct_stop;
	_draw_func = fct_draw;

	return 0;
}

bool Planner::setStart(shared_ptr<Configuration> Cs)
{
	bool b = false;
	if (!_Init)
	{
		Node* N = _Graph->searchConf(Cs);
		if (N == NULL)
		{
			_Start = new Node(_Graph, Cs);
			_Graph->insertNode(_Start);
			_Graph->linkNode(_Start);
			b = true;
		}
		else
		{
			_Start = N;
		}
	}

	if (_Init && (*_Start->getConfiguration() != *Cs))
	{
		_Start = new Node(_Graph, Cs);
		_Graph->insertNode(_Start);
		_Graph->linkNode(_Start);
		b = true;
	}

	_Start->setInStart();

	_Graph->getGraphStruct()->search_start = _Start->getNodeStruct();
	return b;
}

bool Planner::setGoal(shared_ptr<Configuration> Cg)
{
	bool b = false;
	if (ENV.getBool(Env::expandToGoal))
	{
		if (!_Init)
		{
			Node* N = _Graph->searchConf(Cg);
			if (N == NULL)
			{
				_Goal = new Node(_Graph, Cg);
				_Graph->insertNode(_Goal);
				// Warning
//				_Graph->linkNode(_Goal);
				b = true;
			}
			else
			{
				_Goal = N;
			}
		}

		if (_Init && (_Goal == NULL || (*_Goal->getConfiguration() != *Cg)))
		{
			_Goal = new Node(_Graph, Cg);
			_Graph->insertNode(_Goal);
//			_Graph->linkNode(_Goal);
			b = true;
		}
		_Graph->getGraphStruct()->search_goal = _Goal->getNodeStruct();
		_Goal->setInGoal();
	}
	else
	{
		_Goal = NULL;
	}

	return b;
}

void Planner::setInit(bool b)
{
	_Init = b;
}

