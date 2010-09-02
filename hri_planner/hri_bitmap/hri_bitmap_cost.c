#include "../include/hri_bitmap_cost.h"
/*
 * hri_bitmap_cost.c
 *
 * Helper functions dealing with costs of robot motion
 *
 */


/**
 * calculates the g cost of A*, the cost of a the path up to this cell,
 * based on the costs to its parent cell in the path
 *
 * dimensions refers to the dimensions of the bitmap
 */
double hri_bt_A_CalculateCellG(hri_bitmapset * btset, hri_bitmap_cell* current_cell, hri_bitmap_cell* fromcell, double distance ) {
  if (fromcell->g < 0 || current_cell->val < 0){
    return -1;
  }
  double result = fromcell->g + current_cell->val + (distance * btset->parameters->path_length_weight);


  return result;
}

/*********************ASTAR**************************************/
/*!
 * \brief Calculate the cost of a cell when reached from a different cell
 * sets cell-> val unless for collision in soft obstacle
 * \param cell the cell
 *
 * \return FALSE and sets cell value to a negative number in case of a collision
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

    /* if(!hri_compute_R6IK(btset->robot,btset->object,qc)){ */
/*       btset->bitmap[BT_3D_OBSTACLES]->data[cell->x][cell->y][cell->z].val = -2; */
/*       cell->val = -2; */
/*       p3d_destroy_config(btset->robot, qc); /\* FREE *\/ */
/*     } */
/*     else{ */
/*       cell->val = bitmap->calculate_cell_value(btset,cell->x,cell->y,cell->z); */
/*       cell->q = qc; */
/*     } */

    q_o[6] = saved[0];  q_o[7] = saved[1];  q_o[8] = saved[2];
    p3d_set_and_update_this_robot_conf(btset->object,q_o);
    p3d_destroy_config(btset->object, q_o); /* FREE */

  } else if (btset->manip == BT_MANIP_MANIPULATION) {
     cell->val = bitmap->calculate_cell_value(btset, cell->x, cell->y, cell->z);
  } else { // treating all other values of btset->manip as Navigation

    // for navigation type, consider whether we are in hard, soft or no obstacle zone
    if (btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].val == BT_OBST_SURE_COLLISION) { /* hard obstacle */
      cell->val = -2;
    } else if((btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].val > 0 ) &&
        (localPathCollides (btset, cell, fromcell)) ) {
      // collision happens
      //        fromcellno = get_direction(fromcell, cell);
      //        // in the obstacle bitmap, set collision in from direction to true
      //        btset->bitmap[BT_OBSTACLES]->data[cell->x][cell->y][cell->z].obstacle[fromcellno] = TRUE; /* collision when u move from fromcell to cell */
      cell->val = -1; // required for recalculating costs of old path
    } else {
      // no obstacle near, or no collision
      cell->val = bitmap->calculate_cell_value(btset, cell->x, cell->y, cell->z);

      if(cell->val > 0) {
        if ( cell->val < BT_NAVIG_THRESHOLD) {
          // too little to matter for safety and comfort, but can still make the robot change ways
          cell->val = 0;
        }
      }
    }
  }

  return cell->val >= 0; // non-negative means no collision
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
        current->g = hri_bt_A_CalculateCellG(btset, current, current->parent, getCellDistance(current, current->parent));
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

/**
 * returns the nth cell after start, by going backwards from end
 */
static hri_bitmap_cell* hri_bt_nth_from_start(hri_bitmap_cell* path_start, hri_bitmap_cell* path_end, int n) {
  // make 2 cells run from end to start in distance n
  hri_bitmap_cell* runner1 = path_end;
  hri_bitmap_cell* runner2 = path_end;
  int delay = n;
  while (runner1 != NULL) {
    runner1 = runner1->parent;
    if (delay > 0) {
      delay--;
    } else {
      runner2 = runner2->parent;
    }
  }
  if (delay == 0) {
    return runner2;
  } else {
    return NULL;
  }
}

/**
 * returns 1 if path starts are different enough, 0 if not, or -1
 * if not applicable
 */
static int hri_bt_differentStart(hri_bitmap_cell* path1_start, hri_bitmap_cell* path1_end, hri_bitmap_cell* path2_start, hri_bitmap_cell* path2_end)
{
  hri_bitmap_cell* path1_strip_end;
  hri_bitmap_cell* path2_strip_end;
  int path_start_difference = 0;
  if (path1_start == NULL || path1_end == NULL || path2_start == NULL || path2_end == NULL) {
    return -1;
  }
  path1_strip_end = hri_bt_nth_from_start(path1_start, path1_end, 20); // magic number
  path2_strip_end = hri_bt_nth_from_start(path2_start, path2_end, 20);
  if (path1_strip_end == NULL || path2_strip_end == NULL){
    return -1;
  }

  while (path1_strip_end) {
    // addpath manhattan distance to sum
    path_start_difference += ABS(path1_strip_end->x - path2_strip_end->x) +
    ABS(path1_strip_end->y - path2_strip_end->y) +
    ABS(path1_strip_end->z - path2_strip_end->z);
    path1_strip_end = path1_strip_end-> parent;
    path2_strip_end = path2_strip_end-> parent;
  }
  if (path_start_difference > 0) { //  another magic number
    return TRUE;
  }
  return FALSE;
}

/**
 * returns 0 if old path should not be kept, else the costs of oldpath in new bitmap
 */
int hri_bt_keep_old_path(hri_bitmapset* bitmapset, hri_bitmap* bitmap_oldpath, hri_bitmap* bitmap_newpath, double newcosts, hri_bitmap_cell* new_search_start)
{
  double oldcost;
  hri_bitmap_cell* old_search_start = hri_bt_get_cell(bitmap_oldpath, new_search_start->x, new_search_start->y, new_search_start->z);
  int useOldPath = FALSE;
  if (hri_bt_differentStart(new_search_start, bitmap_newpath->search_goal, old_search_start, bitmap_oldpath->search_goal) == TRUE) {
       // calculate costs of staying on path as it is
       oldcost = getPathCost(bitmapset, bitmap_oldpath, old_search_start);
       if (oldcost > 0) {
//         printf("%f  %f  %f\n", oldcost, newcosts, ((oldcost - newcosts) / oldcost) * 100);
         /* result < oldcost should never be the case, unless path was updated without calling this function */
         if (oldcost > newcosts) {
           if (((oldcost - newcosts) / oldcost) * 100 < bitmapset->parameters->path_reuse_threshold ) {
             useOldPath = TRUE;
           }
         } else { // oldcost == result trivial case when nothing relevant has changed
           if (oldcost < newcosts) {
             // old path is better than new one, A Star not optimal, BUG?
             printf("BUG: Old path has better costs than new path.\n");
           }
         }
       } // endif oldpath has no collision
     } // endif newpath starts differently
  return useOldPath;
}
