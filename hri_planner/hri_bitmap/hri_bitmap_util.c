#include "hri_bitmap_util.h"
/**
 * hri_bitmap_util.c
 * used for small independent functions related to bitmap handling
 */

/****************************************************************/
/*!
 * \brief get the bitmap of given type
 *
 * \param type type of the bitmap
 *
 * \return the bitmap
 */
/****************************************************************/
hri_bitmap* hri_bt_get_bitmap(int type, hri_bitmapset* bitmapset) {
  int i;

  if (bitmapset == NULL) {
    return NULL;
  }
  return bitmapset->bitmap[type];
}

/*
 * used to determine whether xyz coordinates are on a given bitmap
 *
 */
int on_map(int x, int y, int z, hri_bitmap* bitmap) {
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
    if (ydiff==-1) return BT_DIRECTION_NORTHEAST;
    if (ydiff== 0) return BT_DIRECTION_EAST;
    if (ydiff== 1) return BT_DIRECTION_SOUTHEAST;
  }
  if(xdiff==0) {
    if(ydiff==-1) return BT_DIRECTION_NORTH;
    if(ydiff== 1) return BT_DIRECTION_SOUTH;
  }
  if(xdiff==1) {
    if(ydiff==-1) return BT_DIRECTION_NORTHWEST;
    if(ydiff== 0) return BT_DIRECTION_WEST;
    if(ydiff== 1) return BT_DIRECTION_SOUTHWEST;
  }
  PrintError(("Bug: Invalid entries causing xdiff, ydiff = %i,%i", xdiff, ydiff));
  return -1;
}


void get_neighbor(hri_bitmap * bitmap, bitmap_cell * current, int direction, bitmap_cell *neighbor) {
  int x = current->x, y = current->y;
  switch (direction) {
  case BT_DIRECTION_NORTH:
    y -=1;
    break;
  case BT_DIRECTION_NORTHEAST:
    x -=1;
    y -=1;
    break;
  case BT_DIRECTION_EAST:
    x -=1;
    break;
  case BT_DIRECTION_SOUTHEAST:
    x -=1;
    y +=1;
    break;
  case BT_DIRECTION_SOUTH:
    y +=1;
    break;
  case BT_DIRECTION_SOUTHWEST:
    x +=1;
    y +=1;
    break;
  case BT_DIRECTION_WEST:
    x +=1;
    break;
  case BT_DIRECTION_NORTHWEST:
    x +=1;
    y -=1;
    break;
  }
  
  if (on_map(x, y, 0, bitmap)) {
    neighbor->x = x;
    neighbor->y = y;
    neighbor->closed = hri_bt_get_cell(bitmap, x, y, 0)->closed;
    neighbor->parent = hri_bt_get_cell(bitmap, x, y, 0)->parent;
  }
}

