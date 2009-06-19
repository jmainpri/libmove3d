#include "hri_bitmap_util.h"
/**
 * hri_bitmap_util.c
 * used for small independent functions related to bitmap handling
 */

/*
 * used to determine whether xyz coordinates are on a given bitmap
 *
 */
bool on_map(int x, int y, int z, hri_bitmap* bitmap) {
  if( (bitmap->nx - 1 < x) || 
      (bitmap->ny - 1 < y) || 
      (bitmap->nz - 1 < z) ||
      (0 > x) ||  
      (0 > y) ||  
      (0 > z)) {
    return FALSE;
  }
  return TRUE;
}

/** 
 * returns the direction the satellite cell is with respect to the center cell
 */
int get_direction(hri_bitmap_cell *satellite_cell, hri_bitmap_cell *center_cell) {
  int xdiff, ydiff;
  xdiff = satellite_cell->x - center_cell->x;
  ydiff = satellite_cell->y - center_cell->y;
  if(xdiff==-1) {
    if (ydiff==-1) return BT_DIRECTION_NORTH;
    if (ydiff== 0) return BT_DIRECTION_NORTHEAST;
    if (ydiff== 1) return BT_DIRECTION_EAST;
  }
  if(xdiff==0) {
    if(ydiff==-1) return BT_DIRECTION_SOUTHEAST;
    if(ydiff== 1) return BT_DIRECTION_SOUTH;
  }
  if(xdiff==1) {
    if(ydiff==-1) return BT_DIRECTION_SOUTHWEST;
    if(ydiff== 0) return BT_DIRECTION_WEST;
    if(ydiff== 1) return BT_DIRECTION_NORTHWEST;
  }
}