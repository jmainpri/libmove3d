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
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "math.h"
#include "hri_bitmap_util.h"

double getPathCost(hri_bitmapset* btset, hri_bitmap* oldpath_bitmap, hri_bitmap_cell* robot_position);
double hri_bt_A_CalculateCellG(hri_bitmap_cell* current_cell, hri_bitmap_cell* fromcell );
int CalculateCellValue(hri_bitmapset * btset, hri_bitmap * bitmap,  hri_bitmap_cell* cell, hri_bitmap_cell* fromcell );

#endif /* HRI_BITMAP_COST_H_ */
