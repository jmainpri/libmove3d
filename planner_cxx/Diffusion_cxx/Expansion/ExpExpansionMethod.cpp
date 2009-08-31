/*
 * ExpansiveExpansionMethod.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */

#include "ExpExpansionMethod.hpp"
#include "P3d-pkg.h"
#include "../../proto/p3d_sample_proto.h"

#include <iostream>
#include <algorithm>
#include <iterator>
#include <vector>

using namespace std;

ExpansiveExpansionMethod::ExpansiveExpansionMethod() :
	BaseExpansionMethod(),
	typeOfBiasing(squared),
	NbDof(0) {


}

ExpansiveExpansionMethod::ExpansiveExpansionMethod(Graph* ptrGraph) :
	BaseExpansionMethod(ptrGraph){

	mRobot = ptrGraph->getRobot();
	NbDof = mRobot->getNbOfUserDof();
	NbDirections = static_cast<int>(pow(2,NbDof));
	vector< vector<bool> > nCube;

	nCube = generateAllDiagonals();
	generateDirectionMap(nCube);
}

ExpansiveExpansionMethod::~ExpansiveExpansionMethod(){

}



Node* ExpansiveExpansionMethod::getExpansionNode(Node* compNode){

	//cout << "ExpansiveExpansionMethod::getExpansionNode Not Implemented!!" <<endl;

	Node* newNode;

	for(int i=0;i<1000;i++)
	{
		newNode = mGraph->randomNodeFromComp(compNode);

		if(!newNode->isClosed())
		{
			return selectExpansionNode(newNode);
		}
	}

	cout << "Error----------------------------" << endl;
	cout << "getExpansionNode-----------------" << endl;

	return mGraph->randomNodeFromComp(compNode);

}

Node* ExpansiveExpansionMethod::selectExpansionNode(Node* compNode){

//	double x = 1-getBiasedParam(squared);
//
//	uint id = (uint)(x*(sortedNodes.size()-1));
//
//	return sortedNodes[id];

	return compNode;
}

void ExpansiveExpansionMethod::addNewNode(Node* nodeToAdd){

	double newNodeCost = nodeToAdd->getCost();

	vector<Node*>::iterator it;

	for (it=sortedNodes.begin(); it!=sortedNodes.end(); ++it){

		if( newNodeCost < (*it)->getCost() ){
			sortedNodes.insert(it,nodeToAdd);
			break;
		}
	}


}

double ExpansiveExpansionMethod::getBiasedParam(Biasing type){

	double randomFlat = p3d_random(0,1);

	if(type==squared){
		return pow(randomFlat,2);
	}
	if(type==cubic){
		return pow(randomFlat,3);
	}
	if(type==exponential){
		return (exp(randomFlat-1) - exp(-1) ) / ( 1 - exp(-1) );
	}
}
