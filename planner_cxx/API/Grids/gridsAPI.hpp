/*
 *  gridsAPI.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 06/05/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef GRID_API_H
#define GRID_API_H

#include <Eigen/Core>

#include "Grids/BaseGrid.hpp"
#include "Grids/TwoDGrid.hpp"
#include "Grids/ThreeDGrid.h"
#include "Grids/ThreeDPoints.h"

extern API::BaseGrid* API_activeGrid;
extern Eigen::Vector3d global_DrawnSphere;

#endif
