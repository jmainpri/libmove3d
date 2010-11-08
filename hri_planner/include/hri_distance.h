/*
 *  hri_distance.h
 *  Move3D-core
 *
 *  Created by Jim Mainprice on 05/11/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef HRI_DISTANCE_H
#define HRI_DISTANCE_H

#include "hri_agent.h"

double hri_robot_min_distance(HRI_AGENTS* agents);
double hri_distance_cost( HRI_AGENTS* agents, double& distance );

#endif