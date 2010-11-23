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
	
	bool runTest(int i);
  
  void initManipulationGenom();
  
  //! Generic function to test the genom requests
	bool manipTest(MANIPULATION_TASK_TYPE_STR type);
	
private:
	
	p3d_rob* m_Robot;
	
	configPt m_qInit;
	configPt m_qGoal;
	
	std::string m_OBJECT_NAME;
  
    std::vector<double> m_objStart, m_objGoto;
	
	ManipulationPlanner* m_manipulation;
};

#endif