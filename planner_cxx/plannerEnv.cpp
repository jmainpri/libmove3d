/*
 *  PlannerEnv.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 21/09/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "plannerEnv.hpp"

#include <iostream>

using namespace std;

// A new container is created for each module
// First declaire the maps of praramters
// Then fill in the maps that associate the enum to the Qt container
// When Qt is disabled this just acts as a normal container

// Definition of the parameter container
Parameters<
PlanParam::boolParameter,
PlanParam::intParameter,
PlanParam::doubleParameter,
PlanParam::stringParameter,
PlanParam::vectorParameter>* PlanEnv = NULL;

// @brief Function that inizializes the 
// Parameter container
void initPlannerParameters()
{
	// Create 5 maps for all types
	std::map<PlanParam::boolParameter,		boolContainer*>				myBoolMap;
	std::map<PlanParam::intParameter,			intContainer*>				myIntMap;
	std::map<PlanParam::doubleParameter,	doubleContainer*>			myDoubleMap;
	std::map<PlanParam::stringParameter,	stringContainer*>			myStringMap;
	std::map<PlanParam::vectorParameter,	vectorContainer*>			myVectorMap;
	
	// Fill the 5 maps
	myBoolMap.insert(std::make_pair(PlanParam::isMaxDisNeigh,			new boolContainer(false)));
	myBoolMap.insert(std::make_pair(PlanParam::isWeightedChoice,	new boolContainer(false)));
	
	myIntMap.insert(std::make_pair(PlanParam::tata,				new intContainer(5)));
	
	myDoubleMap.insert(std::make_pair(PlanParam::eleven,	new doubleContainer(11.0)));

#ifdef QT_LIBRARY
	myStringMap.insert(std::make_pair(PlanParam::titi,		new stringContainer("titi")));
#endif
	
	std::vector<double> tutu;
	tutu.push_back( 1 ); tutu.push_back( 8 );
	
	myVectorMap.insert(std::make_pair(PlanParam::tutu,			new vectorContainer(tutu)));

	// Make the new parameter container
	PlanEnv =  new Parameters<
	PlanParam::boolParameter,
	PlanParam::intParameter,
	PlanParam::doubleParameter,
	PlanParam::stringParameter,
	PlanParam::vectorParameter>(
															myBoolMap,
															myIntMap,
															myDoubleMap,
															myStringMap,
															myVectorMap);
}
