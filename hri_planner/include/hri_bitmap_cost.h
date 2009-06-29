/*
 * hri_bitmap_cost.h
 *
 *  Created on: Jun 29, 2009
 *      Author: kruset
 */

#ifndef HRI_BITMAP_COST_H_
#define HRI_BITMAP_COST_H_
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "math.h"

static double getPathCost(hri_bitmapset* btset, hri_bitmap* oldpath_bitmap, hri_bitmap* new_values_bitmap, hri_bitmap_cell* robot_position);


#endif /* HRI_BITMAP_COST_H_ */
