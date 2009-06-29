#include "../include/hri_bitmap_cost.h"
/*
 * hri_bitmap_cost.c
 *
 * Helper functions dealing with costs of robot motion
 *
 */

extern int CalculateCellValue(hri_bitmapset * btset, hri_bitmap * bitmap,  hri_bitmap_cell* cell, hri_bitmap_cell* fromcell);
extern double hri_bt_A_CalculateCellG(hri_bitmap_cell* current_cell, hri_bitmap_cell* fromcell);



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
static double getPathCost(hri_bitmapset* btset, hri_bitmap* oldpath_bitmap, hri_bitmap* new_values_bitmap, hri_bitmap_cell* robot_position)
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
