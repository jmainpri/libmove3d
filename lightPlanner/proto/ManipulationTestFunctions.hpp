/*
 *  ManipulationTestFunctions.hpp
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 05/11/10.
 *  Copyright 2010 Laas/CNRS. All rights reserved.
 *
 */

#ifndef MANIPULATION_TEST_FUNCTIONS_HPP
#define MANIPULATION_TEST_FUNCTIONS_HPP

//#include "device.h"
//#include "p3d.h"

#include "ManipulationPlanner.hpp"

#include <string>

class ManipulationTestFunctions 
{
	
public:
	ManipulationTestFunctions();
	ManipulationTestFunctions(std::string RobotNameContains);
	virtual ~ManipulationTestFunctions();
	
	void runTest(int i);
	
private:
	void initManipulationGenom();
	
	bool testArmFree();
	bool testArmPickGoto();
	bool testArmPickToFree();
	
	p3d_rob* m_Robot;
	
	configPt m_qInit;
	configPt m_qGoal;
	
	std::string m_OBJECT_NAME;
	
	ManipulationPlanner* m_manipulation;
};

#endif