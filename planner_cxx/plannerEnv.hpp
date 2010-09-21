/*
 *  PlannerEnv.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 21/09/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "p3d/ParametersEnv.hpp"

namespace  PlannerParameters 
{
	enum boolParameter 
	{
		one,
		two,
		three,
		four
	};
	
	enum intParameter 
	{
		five,
		six,
		seven,
		eight
	};
	
	enum doubleParameter 
	{
		nine,
		ten,
		eleven,
		twelve
	};
	
	enum stringParameter 
	{
		thirteen,
		fifteen,
		sixteen,
		seventeen
	};
	
	enum vectorParameter 
	{
		eighteen,
		nineteen,
	};
	
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<
PlannerParameters::boolParameter,
PlannerParameters::intParameter,
PlannerParameters::doubleParameter,
PlannerParameters::stringParameter,
PlannerParameters::vectorParameter>* PlannerEnv;

// Functions that initializes the planner
// Parameters
void initPlannerParameters();