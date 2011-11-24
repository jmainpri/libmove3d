//
//  main.cpp
//  libmove3d
//
//  Created by Jim Mainprice on 24/11/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.

#include <iostream>
#include "lightPlanner/proto/ManipulationTestFunctions.hpp"

using namespace std;

int main(int argc, char *argv[])
{
  string home( getenv("HOME_MOVE3D") );
  
  if ( "" == home )
  {
    cout << "Error : HOME_MOVE3D not define" << endl;
    return 0;
  }
  
  string filename("/test/Pr2.p3d"); 
  
  if (!p3d_read_desc((home+filename).c_str()) 
  {
    cout << "Error : give a p3d filename as argument" << endl;
    return 0;
  }

  ManipulationTestFunctions tests;
  
  if(!tests.runTest(argc))
  {
    cout << "ManipulationTestFunctions::Fail" << endl;
    return 0;
  }
}
