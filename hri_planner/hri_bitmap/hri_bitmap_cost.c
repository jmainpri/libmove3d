#include "../include/hri_bitmap_cost.h"
/*
 * hri_bitmap_cost.c
 *
 * Helper functions dealing with costs of robot motion
 *
 */

/* similar to M_SQRT2 in math.h*/
#ifndef M_SQRT3
#define M_SQRT3 1.732050807568877294
#endif

/**
 * calculates the g cost of A*, the cost of a the path up to this cell,
 * based on the costs to its parent cell in the path
 */
double hri_bt_A_CalculateCellG(hri_bitmap_cell* current_cell, hri_bitmap_cell* fromcell ) {
  if (fromcell->g < 0 || current_cell->val < 0){
    return -1;
  }
  double result = fromcell->g + current_cell->val;

  int manhattan_distance = ABS(current_cell->x - fromcell->x) + ABS(current_cell->y - fromcell->y) + ABS(current_cell->z - fromcell->z);

  if(manhattan_distance == 1) {
    result += 1; // normal grid step
  } else if(manhattan_distance == 2) {
    result += M_SQRT2; // 2d diagonal step
  } else if(manhattan_distance == 3) {
    result += M_SQRT3; // 3d diagonal step
  }
  if (isHardEdge(current_cell, fromcell)) {
    result += BT_PATH_HARD_EDGE_COST;
  }
  return result;
}

/*********************ASTAR**************************************/
/*!
 * \brief Calculate the cost of a cell when reached from a different cell
 * sets cell-> val unless for collision in soft obstacle
 * \param cell the cell
 *
 * \return FALSE in case of a collision
 */
/****************************************************************/
int CalculateCellValue(hri_bitmapset * btset, hri_bitmap * bitmap,  hri_bitmap_cell* cell, hri_bitmap_cell* fromcell )
{
  configPt qc,q_o;
  double saved[3];

  if(btset->manip == BT_MANIP_REACH) {
    qc = p3d_get_robot_config(btset->robot); /* ALLOC */
    // for REACH type path finding, calculate collision
    q_o = p3d_get_robot_config(btset->object); /* ALLOC */
    saved[0] = q_o[6]; saved[1] = q_o[7]; saved[2] = q_o[8];

    q_o[6] = cell->x*btset->pace+btset->realx;
    q_o[7] = cell->y*btset->pace+btset->realy;
    q_o[8] = cell->z*btset->pace+btset->realz;

    p3d_set_and_update_this_robot_conf(btset->object,q_o);

    if(!hri_compute_R6IK(btset->robot,btset->object,qc)){
      btset->bitmap[BT_3D_OBSTACLES]->data[cell->x][cell->y][cell->z].val = -2;
      cell->val = -2;
      p3d_destroy_config(btset->robot, qc); /* FREE */
    }
    else{
      cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z);
      cell->q = qc;
    }


    q_o[6] = saved[0];  q_o[7] = saved[1];  q_o[8] = saved[2];
    p3d_set_and_update_this_robot_conf(btset->object,q_o);
    p3d_destroy_config(btset->object, q_o); /* FREE */

    if(cell->val < 0)
      return FALSE;
    else
      return TRUE;

  } else if (btset->manip == BT_MANIP_NAVIGATION) {

    // for navigation type, consider whether we are in hard, soft or no obstacle zone
    if (btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].val == BT_OBST_SURE_COLLISION) { /* hard obstacle */
      cell->val = -2;
      return FALSE;
    } else if(btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].val > 0 ){ /* soft obstacles */
      qc = p3d_get_robot_config(btset->robot); /* ALLOC */
      qc[6]  = cell->x*btset->pace+btset->realx;
      qc[7]  = cell->y*btset->pace+btset->realy;
      qc[11] = atan2(cell->y-fromcell->y,cell->x-fromcell->x);
      // moved the robot config qc to current cell position and angle
      p3d_set_and_update_this_robot_conf(btset->robot, qc); // move the robot to cell
      p3d_destroy_config(btset->robot, qc); /*  FREE */
      if( p3d_col_test_robot_statics(btset->robot, FALSE)) { // check whether robot collides
        // collision happens
//        fromcellno = get_direction(fromcell, cell);
//        // in the obstacle bitmap, set collision in from direction to true
//        btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].obstacle[fromcellno] = TRUE; /* collision when u move from fromcell to cell */
        cell->val = -1; // required for recalculating costs of old path
        return FALSE;
      }
    }
    // no obstacle near, or no collision
    cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z);

    if(cell->val < 0) {
      return FALSE;
    }
    if(cell->val < BT_NAVIG_THRESHOLD) {
      // too little to matter for safetyand comfort, but can still make the robot change ways
      cell->val = 0;
    }
    return TRUE;

  } else if (btset->manip == BT_MANIP_MANIPULATION) {
    cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z);

    if(cell->val < 0)
      return FALSE;

    return TRUE;
  }

  // should never happen
  PrintError(("Bug: not implemented bitmap->manip type %i", btset->manip));
  return FALSE;
}


/**
 * updates parents g, then updates this cells g with changed values in bitmap,
 * up to path start or robot position, returns TRUE if robot position reached (robot on path)
 *
 */
static int updateCellGRecursiveTo(hri_bitmapset* btset, hri_bitmap* oldpath_bitmap, hri_bitmap_cell* current, hri_bitmap_cell* robot_position)
{
  int robot_on_path;
  if (current == NULL) return FALSE; // should never happen
  if (current->parent != NULL && current != robot_position) {
    robot_on_path = updateCellGRecursiveTo(btset, oldpath_bitmap, current->parent, robot_position);
    if (robot_on_path) { // else don't bother
      if (CalculateCellValue(btset, oldpath_bitmap, current, current->parent)) {
        current->g = hri_bt_A_CalculateCellG(current, current->parent);
      } else {
        current->g = -1;
      }
    }
  } else {
    if (current == robot_position) {
      robot_on_path = TRUE;
      current->g = 0;
    } else {
      robot_on_path = FALSE;
    }
  }
  return robot_on_path;
}

/**
 * returns the remaining costs of the given path in the bitmapset, after a robot has potentially moved,
 * particularly useful after other objects have moved as well.
 * returns 0 in case of error (or if costs are 0, which should not happen unless start = goal)
 * or -1 in case of collision.
 */
double getPathCost(hri_bitmapset* btset, hri_bitmap* oldpath_bitmap, hri_bitmap_cell* robot_position)
{
  double result = 0;
  int robot_position_on_path; // flag to indicate whether robot position is on path

  if (oldpath_bitmap->search_goal == NULL || oldpath_bitmap->search_start == NULL) {
    PrintError(("start or goal == NULL"));
    return FALSE;
  }

  robot_position_on_path = updateCellGRecursiveTo(btset, oldpath_bitmap, oldpath_bitmap->search_goal, robot_position);
  if (robot_position_on_path == FALSE) {
    result = -3;
  } else {
    result = oldpath_bitmap->search_goal->g;
  }
  return result;
}
