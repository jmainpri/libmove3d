/*
 *  PlannerEnv.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 21/09/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "PlannerEnv.hpp"

#include <iostream>

using namespace std;

// A new container is created for each module
// First declaire the maps of praramters
// Then fill in the maps that associate the enum to the Qt container
// When Qt is disables this just acts as a cormal container

// Definition of the parameter container
Parameters<
PlannerParameters::boolParameter,
PlannerParameters::intParameter,
PlannerParameters::doubleParameter,
PlannerParameters::stringParameter,
PlannerParameters::vectorParameter>* PlannerEnv = NULL;

// @brief Function that inizializes the 
// Parameter container
void initPlannerParameters()
{
	// Create 5 maps for all types
	std::map<PlannerParameters::boolParameter,		boolContainer*>				myBoolMap;
	std::map<PlannerParameters::intParameter,			intContainer*>				myIntMap;
	std::map<PlannerParameters::doubleParameter,	doubleContainer*>			myDoubleMap;
	std::map<PlannerParameters::stringParameter,	stringContainer*>			myStringMap;
	std::map<PlannerParameters::vectorParameter,	vectorContainer*>			myVectorMap;
	
	// Fill the 5 maps
	myBoolMap.insert(std::make_pair(PlannerParameters::one,			new boolContainer(false)));
	myBoolMap.insert(std::make_pair(PlannerParameters::two,			new boolContainer(true)));
	myBoolMap.insert(std::make_pair(PlannerParameters::three,		new boolContainer(false)));
	myBoolMap.insert(std::make_pair(PlannerParameters::four,		new boolContainer(true)));

	myIntMap.insert(std::make_pair(PlannerParameters::five,				new intContainer(5)));
	myIntMap.insert(std::make_pair(PlannerParameters::six,				new intContainer(6)));
	myIntMap.insert(std::make_pair(PlannerParameters::seven,			new intContainer(7)));
	myIntMap.insert(std::make_pair(PlannerParameters::eight,			new intContainer(8)));
	
	myDoubleMap.insert(std::make_pair(PlannerParameters::nine,			new doubleContainer(9.0)));
	myDoubleMap.insert(std::make_pair(PlannerParameters::ten,				new doubleContainer(10.0)));
	myDoubleMap.insert(std::make_pair(PlannerParameters::eleven,		new doubleContainer(11.0)));
	myDoubleMap.insert(std::make_pair(PlannerParameters::twelve,		new doubleContainer(12.0)));

	myStringMap.insert(std::make_pair(PlannerParameters::thirteen,	new stringContainer("thirteen")));
	myStringMap.insert(std::make_pair(PlannerParameters::fifteen,		new stringContainer("fifteen")));
	myStringMap.insert(std::make_pair(PlannerParameters::sixteen,		new stringContainer("sixteen")));
	myStringMap.insert(std::make_pair(PlannerParameters::seventeen,	new stringContainer("seventeen")));
	
	
	std::vector<double> a,b;
	a.push_back( 1 ); a.push_back( 8 );
	b.push_back( 1 ); b.push_back( 9 );
	
	myVectorMap.insert(std::make_pair(PlannerParameters::eighteen,			new vectorContainer(a)));
	myVectorMap.insert(std::make_pair(PlannerParameters::nineteen,			new vectorContainer(b)));

	// Make the new parameter container
	PlannerEnv =  new Parameters<
	PlannerParameters::boolParameter,
	PlannerParameters::intParameter,
	PlannerParameters::doubleParameter,
	PlannerParameters::stringParameter,
	PlannerParameters::vectorParameter>(
															myBoolMap,
															myIntMap,
															myDoubleMap,
															myStringMap,
															myVectorMap);
	

	// Print all parameters
	cout << "Init Test Env : " << endl;
	cout << PlannerEnv->getBool(PlannerParameters::one) << endl;
	cout << PlannerEnv->getBool(PlannerParameters::two) << endl;
	cout << PlannerEnv->getBool(PlannerParameters::three) << endl;
	cout << PlannerEnv->getBool(PlannerParameters::four) << endl;
	
	cout << PlannerEnv->getInt(PlannerParameters::five) << endl;
	cout << PlannerEnv->getInt(PlannerParameters::six) << endl;
	cout << PlannerEnv->getInt(PlannerParameters::seven) << endl;
	cout << PlannerEnv->getInt(PlannerParameters::eight) << endl;
	
	cout << PlannerEnv->getDouble(PlannerParameters::nine) << endl;
	cout << PlannerEnv->getDouble(PlannerParameters::ten) << endl;
	cout << PlannerEnv->getDouble(PlannerParameters::eleven) << endl;
	cout << PlannerEnv->getDouble(PlannerParameters::twelve) << endl;
	
	cout << PlannerEnv->getString(PlannerParameters::thirteen).toStdString() << endl;
	cout << PlannerEnv->getString(PlannerParameters::fifteen).toStdString() << endl;
	cout << PlannerEnv->getString(PlannerParameters::sixteen).toStdString() << endl;
	cout << PlannerEnv->getString(PlannerParameters::seventeen).toStdString() << endl;
//	
//	cout << PlannerEnv.getVector(PlannerParameters::eighteen) << endl;
//	cout << PlannerEnv.getVector(PlannerParameters::nineteen) << endl;
}
