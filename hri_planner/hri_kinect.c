/*
 *  hri_kinect.cpp
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 06/01/11.
 *  Copyright 2011 __MyCompanyName__. All rights reserved.
 *
 */

#include "Hri_planner-pkg.h"
#include "P3d-pkg.h"

// right arm 1,2,3
//  left arm 4,5,6
void hri_set_human_config_from_kinect( HRI_AGENTS * agents , configPt q , kinectData& humKin )
{
  p3d_rob * robot = NULL;
//  p3d_jnt* joint = NULL;
//  int index_dof;
  
  if (!q) {
    printf("Error : hri_set_human_config_from_kinect\n");
  }
  
  // gets the first human robot
  for(int i=0; i<agents->all_agents_no; i++) 
  {
    if( agents->all_agents[i]->is_human )
    {
      robot = agents->all_agents[i]->robotPt;
      break;
    }
  }
  
  //return FALSE;
}

configPt hri_get_configuration_from_kinect_data( p3d_rob* robot, kinectData& data )
{
  // Set pelvis
  p3d_jnt* joint = p3d_get_robot_jnt_by_name(robot, (char*) "Pelvis");
  int index_dof = joint->index_dof;
  
  configPt q = p3d_get_robot_config(robot);
  
  q[index_dof+0] = data.TORSO[0];
  q[index_dof+1] = data.TORSO[1];
  q[index_dof+2] = data.TORSO[2];
  
  p3d_vector3 sum,midP;
  
  p3d_vectAdd( data.HIP_LEFT, data.HIP_RIGHT , sum);
  p3d_vectScale( midP , sum , 0.5 );
  
  
  double dist = sqrt( pow(data.HIP_LEFT[0]-midP[0],2) + pow(data.HIP_LEFT[1]-midP[1],2) );
  q[index_dof+5] = asin( ( data.HIP_LEFT[1] - midP[1] ) / dist  );
  
  return q;
}