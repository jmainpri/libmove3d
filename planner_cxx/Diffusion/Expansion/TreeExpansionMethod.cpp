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
	BaseExpansionMethod() {
}

TreeExpansionMethod::TreeExpansionMethod(Graph* ptrGraph) :
	BaseExpansionMethod(ptrGraph) {
}

TreeExpansionMethod::~TreeExpansionMethod(){
}

Node* TreeExpansionMethod::addNode(Node* currentNode, LocalPath& path, double pathDelta,
		Node* directionNode, double currentCost, int& nbCreatedNodes)
{

	if ((pathDelta == 1. && directionNode))
	{
		cout << "MergeComp" << endl;
		mGraph->MergeComp(currentNode, directionNode, path.length());
		return (directionNode);
	}
	else
	{
		Node* newNode = mGraph->insertNode(path.getEnd(), currentNode,
				currentCost, step());

		nbCreatedNodes++;
		return (newNode);
	}
}

int TreeExpansionMethod::expandProcess(Node* expansionNode,
		shared_ptr<Configuration> directionConfig, Node* directionNode,
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

		extensionSucceeded = this->nextStep(*directionLocalpath,
				directionNode, positionAlongDirection, extensionLocalpath,
				method);

		failed |= !extensionSucceeded;

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

		// Transition test for cost spaces, increase temperature in case of failure
		if (!failed && ENV.getBool(Env::isCostSpace))
		{
			/*extensionCost = extensionLocalpath->getEnd()->cost();

			if (!costTestSucceeded(&fromNode, extensionLocalpath->getEnd(),
					extensionCost))
			{
				adjustTemperature(false, &fromNode);
				failed = true;

				int nbCostFail = ENV.getInt(Env::nbCostTransFailed);
				nbCostFail++;
				ENV.setInt(Env::nbCostTransFailed, nbCostFail);

				if (ENV.getBool(Env::printCostFail))
					cout << "nbCostFail = " << nbCostFail << endl;
			}

			if (!failed && (extensionCost
					> expansionNode->getConfiguration()->cost()))
			{
				adjustTemperature(true, &fromNode);
			}*/
		}

		// Add node to graph if everything succeeded
		if (!failed)
		{
			extensionNode = addNode(&fromNode, *extensionLocalpath,
					positionAlongDirection, directionNode, extensionCost,
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

	if (ENV.getBool(Env::isCostSpace) && ENV.getInt(Env::CostMethodChoice)
			== MAXIMAL_THRESHOLD)
	{
		p3d_updateCostThreshold();
	}

	return nbCreatedNodes;
}
