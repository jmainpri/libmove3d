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

  void setDebugMode(bool value);
  
  void setInitConfiguration(configPt q);
  void setGoalConfiguration(configPt q);
  
  void setObject(std::string name);
  void resetObject();
  
  void setPlacement(std::string name);
  void resetPlacement();
  
  void setSupport(std::string name);
  void resetSupport();

  void setToPoint(std::vector<double> point);
  void resetToPoint();
  
  void addTraj(std::string name, p3d_traj* traj);
  void addConf(std::string name, configPt q);
  
  std::vector< std::pair<std::string, p3d_traj*> > getTrajVector();
  std::vector< std::pair<std::string, configPt> > getConfVector();
  
  //! Creates a manipulation planner
  //! if it doesnot exists, if it does eares it
  void initManipulationGenom();

  //! Getters
  inline ManipulationPlanner* getManipulationPlanner(){return m_manipulation;}
  
  //! Draws the workspace
  void drawEvalutedWorkspace();
  
  //! Save evaluated workspace to file
  void saveToFileEvalutedWorkspace();
  
  bool readWorkspaceFromFile(std::string fileName);
  
  //! Generic function to test the genom requests
  bool manipTest(MANIPULATION_TASK_TYPE_STR type);
  
  //! Returns the manipulation planner
  ManipulationPlanner* getManipPlanner() { return m_manipulation; }
  
  
private:
  bool saveObjectAndRobotState(configPt q,std::string ObjectName);
  bool restoreObjectAndRobotState();

  //! evaluates the number of grasp succes over 
  //! a series of gradomly selected targets
  bool manipTestGraspingWithDifferentObjectOrientations(bool rotate_only_around_z, double& successRate);
  
  //! Compute workspace of with partiular 
  //! manipulation planner parameters
  bool evaluateWorkspace();
  
  //! Compute the entire trajectories for a set of
  //! given entries
  bool computeTrajectories();
	
	p3d_rob* m_Robot;
	
	configPt m_qInit;
	configPt m_qGoal;
  
  bool m_ToFreePoint;
	
	std::string m_OBJECT_NAME;
  std::string m_SUPPORT_NAME;
  std::string m_PLACEMENT_NAME;
  
  std::vector<double> m_objStart, m_objGoto;
  
  std::vector< std::pair<std::string,p3d_traj*> > m_trajvector;
  std::vector< std::pair<std::string,configPt> > m_confvector;
  std::vector< std::pair<std::string,configPt> > m_worldState;
  
  unsigned int m_nbOrientations;
  
  std::vector< std::pair<double, std::vector<double> > > m_workspacePoints;
	
	ManipulationPlanner* m_manipulation;
};

#endif
