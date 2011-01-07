/*
 *  hri_kinect.cpp
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 06/01/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#include "Hri_planner-pkg.h"
#include "P3d-pkg.h"

//! human kinect information
bool        m_data_exists = false;
kinectData  m_humKin;

//! set the human position form the kinect information
//! right arm 1,2,3
//! left arm 4,5,6
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

//! Computes a condfiguration
//! from a set of points 
configPt hri_get_configuration_from_kinect_data( p3d_rob* robot, kinectData& data )
{
  // Set pelvis
  p3d_jnt* joint = p3d_get_robot_jnt_by_name(robot, (char*) "Pelvis");
  int index_dof = joint->index_dof;
  
  configPt q = p3d_get_robot_config(robot);
  
  q[index_dof+0] = data.TORSO[0];
  q[index_dof+1] = data.TORSO[1];
  q[index_dof+2] = data.TORSO[2]; // Hack 1 meter
  
  // calcul de l'orientation du torso
  p3d_vector3 sum,midP;
  p3d_vectAdd( data.SHOULDER_LEFT, data.SHOULDER_RIGHT , sum );
  p3d_vectScale(  sum , midP , 0.5 );
  
  q[index_dof+5] = atan2( data.SHOULDER_LEFT[1]-midP[1] , data.SHOULDER_LEFT[0]-midP[0]  );
  q[index_dof+5] += -M_PI/2; // Hack + 3 Pi / 2


  // Set and update robot to 
  // new position
  p3d_set_and_update_this_robot_conf( robot, q );

  p3d_vector3 pos;
  joint = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderX");
  p3d_jnt_get_cur_vect_point( joint , pos );

  index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderY")->index_dof; 
  q[index_dof+1] = p3d_vectDistance( data.ELBOW_RIGHT , pos ) - 0.2066 ;

  printf(" distance = %f \n", q[index_dof+1] );
  printf(" x = %f , y = %f , z = %f \n" , data.SHOULDER_RIGHT[0] , data.SHOULDER_RIGHT[1] , data.SHOULDER_RIGHT[2] );
  printf(" x = %f , y = %f , z = %f \n" , pos[0] , pos[1] , pos[2] );

  // calcul de la direction pour le bras droit
  p3d_vector3 sub,dir;
  p3d_vectSub( data.ELBOW_RIGHT , pos , sub );
  p3d_vectNormalize( sub , dir );

  double c2 = sqrt( pow( -dir[0] , 2 ) + pow( dir[2] , 2 ) );

  // selon x
  double alpha1 = atan2( dir[2]/c2 , -dir[1]/c2 );

  // selon z
  double alpha2 = atan2( -dir[0] , c2 );

  //index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderX")->index_dof;
  //q[index_dof] = alpha1;

  index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderZ")->index_dof;
  q[index_dof] = alpha2;

  p3d_vectSub( data.HAND_RIGHT , data.ELBOW_RIGHT , sub ); 
  p3d_vectNormalize( sub , dir ); 

  double s4 = sqrt( pow( dir[0] , 2 ) + pow( dir[2] , 2 ) ); 

  // selon y  
  double alpha3 = atan2( -dir[2]/s4, -dir[0]/s4 ); 

  // coude 
  double alpha4 = atan2( s4 , dir[1] ); 

//   index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rShoulderY")->index_dof; 
//   q[index_dof] = alpha3; 

//   index_dof = p3d_get_robot_jnt_by_name(robot, (char*) "rElbowZ")->index_dof; 
//   q[index_dof] = alpha4; 

//   printf("Human new condifuration\n"); 
//   printf("-----------------------------------\n"); 
  return q;
}

void hri_store_kinect_model( kinectData& data )
{
  m_data_exists = true;
  m_humKin = data;
}

//!
//! 
void hri_draw_kinect_points()
{
  if (!m_data_exists) {
    return;
  }
  
  double r = 0.10;
  g3d_drawSphere(m_humKin.HEAD[0],m_humKin.HEAD[1],m_humKin.HEAD[2],r);
  g3d_drawSphere(m_humKin.NECK[0],m_humKin.NECK[1],m_humKin.NECK[2],r);
  
  g3d_drawSphere(m_humKin.TORSO[0],
                 m_humKin.TORSO[1],
                 m_humKin.TORSO[2],r);
  
  g3d_drawSphere(m_humKin.SHOULDER_RIGHT[0],
                 m_humKin.SHOULDER_RIGHT[1],
                 m_humKin.SHOULDER_RIGHT[2],r);
  
  g3d_drawSphere(m_humKin.SHOULDER_LEFT[0],
                 m_humKin.SHOULDER_LEFT[1],
                 m_humKin.SHOULDER_LEFT[2],r);
  
  g3d_drawSphere(m_humKin.SHOULDER_RIGHT[0],
                 m_humKin.SHOULDER_RIGHT[1],
                 m_humKin.SHOULDER_RIGHT[2],r);

  g3d_drawSphere(m_humKin.ELBOW_RIGHT[0],
                 m_humKin.ELBOW_RIGHT[1],
                 m_humKin.ELBOW_RIGHT[2],r);
  
  g3d_drawSphere(m_humKin.ELBOW_LEFT[0],
                 m_humKin.ELBOW_LEFT[1],
                 m_humKin.ELBOW_LEFT[2],r);
  
  g3d_drawSphere(m_humKin.HIP_RIGHT[0],
                 m_humKin.HIP_RIGHT[1],
                 m_humKin.HIP_RIGHT[2],r);
  
  g3d_drawSphere(m_humKin.HIP_LEFT[0],
                 m_humKin.HIP_LEFT[1],
                 m_humKin.HIP_LEFT[2],r);
  
  g3d_drawSphere(m_humKin.HAND_RIGHT[0],
                 m_humKin.HAND_RIGHT[1],
                 m_humKin.HAND_RIGHT[2],r);
  
  g3d_drawSphere(m_humKin.HAND_LEFT[0],
                 m_humKin.HAND_LEFT[1],
                 m_humKin.HAND_LEFT[2],r);
}
