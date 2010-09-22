/*
 *  PlannerEnv.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 21/09/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#ifndef PLANNER_ENV_HPP
#define PLANNER_ENV_HPP

#include "p3d/ParametersEnv.hpp"

namespace  PlanParam
{
	enum boolParameter 
	{
		isMaxDisNeigh,
		isWeightedChoice,
		stopPlanner
	};
	
	enum intParameter 
	{
		tata
	};
	
	enum doubleParameter 
	{
		eleven,
		
	};
	
	enum stringParameter 
	{
		titi
	};
	
	enum vectorParameter 
	{
		tutu
	};
	
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<
PlanParam::boolParameter,
PlanParam::intParameter,
PlanParam::doubleParameter,
PlanParam::stringParameter,
PlanParam::vectorParameter>* PlanEnv;

// Functions that initializes the planner
// Parameters
void initPlannerParameters();

#endif
