/*
 *  hri_kinect.h
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 06/01/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef _HRI_KINECT_H
#define _HRI_KINECT_H

#include "hri_agent.h"

// Only HEAD, NECK, TORSO, 
// SHOULDERS, ELBOW, 
// HAND, HIP, KNEE and FOOT 
struct kinectData 
{
  p3d_vector3 HEAD, NECK, TORSO;
  p3d_vector3 SHOULDER_RIGHT;
  p3d_vector3 SHOULDER_LEFT;
  p3d_vector3 ELBOW_RIGHT;
  p3d_vector3 ELBOW_LEFT;
  p3d_vector3 HIP_LEFT;
  p3d_vector3 HIP_RIGHT;
  p3d_vector3 HAND_RIGHT;
  p3d_vector3 HAND_LEFT;
  p3d_vector3 KNEE_RIGHT;
  p3d_vector3 KNEE_LEFT;
  p3d_vector3 FOOT_RIGHT;
  p3d_vector3 FOOT_LEFT;
};

struct kinectAgent 
{
  int kinectId;
  kinectData data;
  HRI_AGENT* agent;
};

#define KINECT_MAX_NUM_HUMANS 16

struct kinectAgents 
{
  int num;
  kinectAgent humans[KINECT_MAX_NUM_HUMANS];
};

configPt hri_get_configuration_from_kinect_data( p3d_rob* robot, kinectData& data );
void hri_store_kinect_model( kinectData& data );
void hri_draw_kinect_points();

#endif
