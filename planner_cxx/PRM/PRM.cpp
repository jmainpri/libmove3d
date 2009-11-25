//
// C++ Implementation: prm
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "PRM.hpp"

using namespace std;
using namespace tr1;

PRM::PRM(Robot* R, Graph* G) :
        Planner(R,G)
{
	_nbConscutiveFailures = 0;
}

PRM::~PRM()
{
}

int PRM::init()
{
	int ADDED = 0;

	Planner::init();
	_nbConscutiveFailures = 0;
	ADDED += Planner::setStart(_Robot->getInitialPosition());
	ADDED += Planner::setGoal(_Robot->getGoTo());
	_Init = true;

	return ADDED;
}

bool PRM::checkStopConditions(int(*fct_stop)(void))
{
	if (ENV.getBool(Env::expandToGoal) && trajFound())
	{
		cout << "Success: the start and goal components are connected." << endl;
		return (true);
	}

	if (_nbConscutiveFailures > ENV.getInt(Env::NbTry))
	{
		cout
				<< "Failure: the maximum number of consecutive failures is reached."
				<< endl;
		p3d_SetStopValue(true);
		return (true);
	}

	if (_Graph->getGraphStruct()->nnode >= ENV.getInt(Env::maxNodeCompco))
	{
		cout << "Stop: the maximum number of nodes in the graph is reached."
				<< endl;
		return (true);
	}

	if (fct_stop)
	{
		if (!(*fct_stop)())
		{
			PrintInfo(("basic PRM building canceled\n"));
			return true;
		}
	}

	return false;
}

/*fonction principale de l'algorithme PRM*/
uint PRM::expand(p3d_graph* Graph_Pt, int(*fct_stop)(void), void(*fct_draw)(
		void))
{
	if (ENV.getBool(Env::expandToGoal) && _Start->getConfiguration()->equal(
			*_Goal->getConfiguration()))
	{
		cout << "graph creation failed: start and goal are the same" << endl;
		return (0);
	}

	int nbAddedNode = 0;

	while (!checkStopConditions(*fct_stop))
	{
		shared_ptr<Configuration> newConf = _Robot->shoot();

		if ( newConf->setConstraints() && (!newConf->IsInCollision()) )
		{
			Node* N = new Node(_Graph,newConf);
			_Graph->insertNode(N);
			_Graph->linkNode(N);

			_nbConscutiveFailures = 0;
			nbAddedNode++;
			if (ENV.getBool(Env::drawGraph))
			{
				*Graph_Pt = *(_Graph->getGraphStruct());
				(*fct_draw)();
			}
		}
		else
		{
			_nbConscutiveFailures++;
		}
	}
	*Graph_Pt = *(_Graph->getGraphStruct());
	return nbAddedNode;
}

