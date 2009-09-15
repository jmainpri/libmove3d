/*
 * RRT-Transition.cpp
 *
 *  Created on: Jul 27, 2009
 *      Author: jmainpri
 */

#include "Transition-RRT.hpp"
#include "../Expansion/TransitionExpansion.hpp"

using namespace std;
using namespace tr1;

TransitionRRT::TransitionRRT(WorkSpace* WS) :
	RRT(WS)
{

}

TransitionRRT::~TransitionRRT()
{

}

int TransitionRRT::init()
{
	int added = TreePlanner::init();

	_expan = new TransitionExpansion(this->getActivGraph());

	p3d_InitSpaceCostParam(this->getActivGraph()->getGraphStruct(),
					this->getStart()->getNodeStruct(),
					this->getGoal()->getNodeStruct());

	return added;
}

/**
 * costConnectNodeToComp
 * Try to connect a node to a given component
 * taking into account the fact that the space
 * is a cost space
 * @return: TRUE if the node and the componant have
 * been connected.
 */
bool TransitionRRT::connectNodeToCompco(Node* node, Node* compNode)
{
	int SavedIsMaxDis = FALSE;
	Node* node2(NULL);

	switch(p3d_GetNodeCompStrategy()) {
	case K_NEAREST_NODE_COMP:
		/*Connect randomly to one of the k nearest
      nodes of the componant */
		/*todo*/
		break;
	case NEAREST_NODE_COMP:
	default:
		SavedIsMaxDis =  p3d_GetIsMaxDistNeighbor();
		p3d_SetIsMaxDistNeighbor(FALSE);

		node2 = _Graph->nearestWeightNeighbour(compNode,
				node->getConfiguration(),
				false,
				p3d_GetDistConfigChoice());

		p3d_SetIsMaxDistNeighbor(SavedIsMaxDis);

		LocalPath path(node->getConfiguration(),node2->getConfiguration());

		if( path.getValid() ){

			if(path.length() <= _expan->step())
			{
				int nbCreatedNodes=0;

				_expan->addNode(node,path,1.0,node2,node2->getConfiguration()->cost(),nbCreatedNodes);
				cout << "Path Valid Connected" << endl;
				return true;
			}

			if( _expan->expandToGoal(
					node,
					node2->getConfiguration()))
			{
				cout << "attempting connect " << node->getConfiguration()->cost() << " to " << node2->getConfiguration()->cost() << endl;
				if(_expan->expandProcess(node, node2->getConfiguration(), node2, Env::nExtend) >= 1 )
					return true;
			}
		}
		return false;
	}
	return false;
}
