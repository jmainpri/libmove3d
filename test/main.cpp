//
//  main.cpp
//  libmove3d
//
//  Created by Jim Mainprice on 24/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.


// To use the test you must have a symbolic
// link to the move3d-assets ManipulationPlanner

#include <iostream>

#ifdef LIGHT_PLANNER
#include "lightPlanner/proto/ManipulationTestFunctions.hpp"
#endif

#include "P3d-pkg.h"
#include "Collision-pkg.h"

using namespace std;
extern void init_all_draw_functions_dummy();

int main(int argc, char *argv[])
{
  string home, filename;
  init_all_draw_functions_dummy();
  
  home =  getenv("HOME_MOVE3D");
  if ( "" == home )
  {
    cout << "Error : HOME_MOVE3D not define" << endl;
    return 0;
  }
  
  // Set collision checker and bounding boxes
  p3d_col_set_mode(p3d_col_mode_pqp);
  p3d_BB_set_mode_close();
  
  // Read environement (meshes and kinematics)
  filename = "/test/gsPr2.p3d"; 
  if (!p3d_read_desc((home+filename).c_str())) 
  {
    cout << "Error : give a p3d filename as argument" << endl;
    return 0;
  }
  
  // Read scenarion (initial and final positions)
  p3d_col_set_mode(p3d_col_mode_pqp);
  p3d_col_start(p3d_col_mode_pqp);
  
  filename = "/test/SCENARIO/ManipulationPr2Constrainted.sce";
  p3d_rw_scenario_init_name();
  if (!p3d_read_scenario((home+filename).c_str())) 
  {
    cout << "Error : give a p3d filename as argument" << endl;
    return 0;
  }
  
  // Set the robots to initial Pos if defined
  for(int i=0; i<XYZ_ENV->nr; i++){
    if(!p3d_isNullConfig(XYZ_ENV->robot[i], XYZ_ENV->robot[i]->ROBOT_POS)){
      p3d_set_and_update_this_robot_conf(XYZ_ENV->robot[i], XYZ_ENV->robot[i]->ROBOT_POS);
    }
  }

#ifdef LIGHT_PLANNER  
  ManipulationTestFunctions tests;
  if(!tests.runTest(2))
  {
    cout << "ManipulationTestFunctions::Fail" << endl;
    return 0;
  }
#endif

  return 0;
}
