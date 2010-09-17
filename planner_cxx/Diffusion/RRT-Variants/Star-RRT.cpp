/*
 *  Star-RRT.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 31/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "Star-RRT.hpp"


#include "Threshold-RRT.hpp"
#include "Expansion/StarExpansion.hpp"

#include "API/Roadmap/node.hpp"
#include "API/Roadmap/graph.hpp"

#include <iostream>

#include "Planner-pkg.h"

using namespace std;

/** 
 * Constructor from a WorkSpace object
 * @param WS the WorkSpace
 */
StarRRT::StarRRT(Robot* R, Graph* G) : RRT(R,G)
{
	cout << "StarRRT::StarRRT(R,G)" << endl;
}

/** 
 * Destructor
 */
StarRRT::~StarRRT()
{
	
}

/**
 * Initialzation of the plannificator
 * @return the number of node added during the init phase
 */
int StarRRT::init()
{
	int added = TreePlanner::init();
	_expan = new StarExpansion(_Graph);
	setInit(true);
	return added;
}

/**
 *
 */
bool StarRRT::connectionToTheOtherCompco(Node* toNode)
{
	// TODO connection to other compco
	return false;
}

