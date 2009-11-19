/*
 * TreeExpansionMethod.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */

#include "TreeExpansionMethod.hpp"

using namespace std;
using namespace tr1;

TreeExpansionMethod::TreeExpansionMethod() :
	BaseExpansionMethod()
{
}

TreeExpansionMethod::TreeExpansionMethod(Graph* ptrGraph) :
	BaseExpansionMethod(ptrGraph)
{
}

TreeExpansionMethod::~TreeExpansionMethod()
{
}

bool TreeExpansionMethod::expandToGoal(Node* expansionNode,
		std::tr1::shared_ptr<Configuration> directionConfig)
{
	return false;
}

int TreeExpansionMethod::expandProcess(Node* expansionNode, shared_ptr<
		Configuration> directionConfig, Node* directionNode,
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

		extensionSucceeded = this->nextStep(*directionLocalpath, directionNode,
				positionAlongDirection, extensionLocalpath, method);

		failed |= !extensionSucceeded;

//                if(failed)
//                {
//                    cout << " Path not valid" << endl;
//                }

		// Expansion Control
		if (firstIteration && !failed)
		{
			if (ENV.getBool(Env::expandControl)
					&& !this->expandControl(*directionLocalpath,
							positionAlongDirection, *expansionNode))
			{
				failed = true;
			}
		}

		// Add node to graph if everything succeeded
		if (!failed)
		{
			extensionNode = addNode(&fromNode, *extensionLocalpath,
					positionAlongDirection, directionNode,
					nbCreatedNodes);
		}
		if (firstIteration && failed)
		{
			this->expansionFailed(*expansionNode);
		}

		if (!failed)
		{
			fromNode = *extensionNode;
		}
		firstIteration = false;

	}
	directionNode = extensionNode;
	return nbCreatedNodes;
}

Node* TreeExpansionMethod::getExpansionNode(Node* compNode, shared_ptr<
		Configuration> direction, int distance)
{

	if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
	{
		return mGraph->getNode(compNode->getCompcoStruct()->dist_nodes->N);
	}
	else
	{
		return selectExpansionNode(compNode, direction, distance);
	}
}

Node* TreeExpansionMethod::selectExpansionNode(Node* compNode, shared_ptr<
		Configuration> direction, int distance)
{
	int KNearest = -1;
	int NearestPercent;

	switch (distance)
	{

	case NEAREST_EXP_NODE_METH:
		/* Choose the nearest node of the componant*/
		return (mGraph->nearestWeightNeighbour(compNode, direction,
				p3d_GetIsWeightedChoice(), distance));

	case K_NEAREST_EXP_NODE_METH:
		/* Select randomly among the K nearest nodes of a componant */
		NearestPercent = kNearestPercent;
		KNearest
				= MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
		// TODO : fix
		//   ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ,
		// KNearest);

		return (mGraph->nearestWeightNeighbour(compNode, direction,
				p3d_GetIsWeightedChoice(), distance));

	case BEST_SCORE_EXP_METH:
		/* Select the node which has the best score: weight*dist */
		return (mGraph->nearestWeightNeighbour(compNode, direction,
				p3d_GetIsWeightedChoice(), distance));

	case K_BEST_SCORE_EXP_METH:
		NearestPercent = kNearestPercent;
		KNearest
				= MAX(1,(int)((NearestPercent*(compNode->getCompcoStruct()->nnode))/100.));
		// TODO : fix
		// ExpansionNodePt = KNearestWeightNeighbor(mG, compNode->mN->comp, direction->mQ, KNearest);
		return (mGraph->nearestWeightNeighbour(compNode, direction,
				p3d_GetIsWeightedChoice(), distance));

	case RANDOM_IN_SHELL_METH:
		/* Select randomly among all the nodes inside a given portion of shell */
		return (mGraph->getNode(hrm_selected_pb_node(mGraph->getGraphStruct(),
				direction->getConfigStruct(), compNode->getNodeStruct()->comp)));

	case RANDOM_NODE_METH:
		return (mGraph->getNode(p3d_RandomNodeFromComp(
				compNode->getCompcoStruct())));

	default:
		/* By default return the nearest node of the componant */
		return (mGraph->nearestWeightNeighbour(compNode, direction,
				p3d_GetIsWeightedChoice(), distance));
	}
}

shared_ptr<Configuration> TreeExpansionMethod::getExpansionDirection(
		Node* fromComp, Node* toComp, bool samplePassive, Node*& directionNode)
{

	if (p3d_GetCostMethodChoice() == MONTE_CARLO_SEARCH)
	{

		shared_ptr<Configuration> q = mGraph->getRobot()->shootDir(
				samplePassive);

		p3d_addConfig(mGraph->getRobot()->getRobotStruct(),
				q->getConfigStruct(),
				fromComp->getCompcoStruct()->dist_nodes->N->q,
				q->getConfigStruct());

		return (q);

	}
	else
	{
		shared_ptr<Configuration> q = selectExpansionDirection(fromComp,
				toComp, samplePassive, directionNode);

		return (q);
	}
}

shared_ptr<Configuration> TreeExpansionMethod::selectExpansionDirection(
		Node* expandComp, Node* goalComp, bool samplePassive,
		Node*& directionNode)
{

	shared_ptr<Configuration> q;
	int savedRlg;

	if (IsDirSampleWithRlg)
	{
		// Save the previous Rlg setting to shoot without Rlg
		savedRlg = p3d_get_RLG();
		p3d_set_RLG(false);
	}

	// Selection in the entire CSpace and
	// biased to the Comp of the goal configuration
	if (IsGoalBias && p3d_random(0., 1.) <= GoalBias)
	{
		// select randomly a node in the goal component as direction of expansion
		directionNode = mGraph->randomNodeFromComp(goalComp);
		q = directionNode->getConfiguration();
	}
	else
	{
		switch (ExpansionDirectionMethod)
		{
		case SUBREGION_CS_EXP:
			// Selection in a subregion of the CSpace
			// (typically close to the current tree)
			// and  biased to the goal configuration
			q = shared_ptr<Configuration> (
					new Configuration(mGraph->getRobot()));

			p3d_shoot_inside_box(mGraph->getRobot()->getRobotStruct(),
			/*expandComp->getConfiguration()->getConfigStruct(),*/
			q->getConfigStruct(), expandComp->getCompcoStruct()->box_env_small,
					(int) samplePassive);
			break;

		case GLOBAL_CS_EXP:
		default:
			// Selection in the entire CSpace
			q = mGraph->getRobot()->shoot(samplePassive);
		}
	}
	if (!IsDirSampleWithRlg)
	{
		//Restore the previous Rlg setting
		p3d_set_RLG(savedRlg);
	}
	return (q);
}

void TreeExpansionMethod::printAllNodes(vector<Node*>& nodes)
{
	vector<Node*>::iterator it = nodes.begin();

	// Get member of the map
	cout << " printAllNodes ------------------------" << endl;
	cout << " nb of nodes " << nodes.size() << endl;
//	for (; it != nodes.end(); it++)
//	{
	for(int i=0;i<nodes.size();i++){
//		Node* ptrNode = *it;
		cout << "Number of neigh = " << nodes[i]->getNumberOfNeighbors();
		cout << " Cost = " << nodes[i]->getSelectCost() << endl;
//		ptrNode->print();
	}

}

/**
 * EST
 */
Node* TreeExpansionMethod::getExpansionNode(vector<Node*>& nodes)
{
	// Get id biased to high values
	double x = (double) (pow(p3d_random(0,1),3));
	unsigned int size = nodes.size();
	unsigned int id = (unsigned int) (x * (double) (size-1));
//	cout << "id = " <<  id << endl;
//	printAllNodes(nodes);

	int	MaxFail = 5;

	for(;;)
	{
	unsigned int i = 0;
	vector<Node*>::iterator it = nodes.begin();
		// Get member of the map
		for (; it != nodes.end(); it++, i++)
		{
	//		if (i == id)
	//		{
				Node* thisNode(*it);

				if( thisNode->getNbExpandFailed() < MaxFail )
				{
//					cout << "id = " << i << endl;
					return thisNode;
				}

	//			if( thisNode->getNumberOfNeighbors() > 1 )
	//			{
	//				if( nodes.size() > 10)
	//				{
	//					nodes.erase(it);
	//				}
	//				return getExpansionNode(nodes);

	//				nodes.erase(it);
	//				return thisNode;
	//			}
	//			cout << "Size = " << nodes.size() << endl;

	//			printAllNodes(nodes);
	//		}
		}
		MaxFail++;
	}


	cout << "Problem not finding node" << endl;
//	Node* expandNode;
	return nodes[id];

}

/**
 * EST
 */
shared_ptr<Configuration> TreeExpansionMethod::getExpansionDirection(
		Node* expansionNode, Node* toComp)
{
	shared_ptr<Configuration> q;

	q = mGraph->getRobot()->shoot(true);
	return q;

}
