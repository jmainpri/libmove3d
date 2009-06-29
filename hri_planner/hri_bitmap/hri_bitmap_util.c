#include "../include/hri_bitmap_util.h"
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

/**
 * returns true if the direction between the last cell and the middle cell is 90deg or more
 * then the direction between mittle_cell and its parent
 */
int isHardEdge(hri_bitmap_cell *last_cell, hri_bitmap_cell *middle_cell) {
  if (last_cell == NULL ||
      middle_cell == NULL ||
      middle_cell->parent == NULL) {
    return FALSE;
  }
  int direction1 = get_direction(last_cell, middle_cell);
  int direction2 = get_direction(middle_cell, middle_cell->parent);
  // both are ints between 0 and 7
  int difference = ABS(direction2 - direction1);
  // difference is between 0 and 7: 1, and 7 are 45 degree changes
  if (ABS(difference) > 1 && ABS(difference) < 7 ) {
    return TRUE;
  } else {
    return FALSE;
  }
}

/**
 * returns the neighboring cell in the 2d direction
 */
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




/**
 * copies bitmap values from one bitmap to the other
 * also copies id (which is not used anywhere currently)
 */
void hri_bt_copy_bitmap_values(hri_bitmap* bitmap_source, hri_bitmap* bitmap_target)
{
  int i,j,k;
  hri_bitmap_cell* current;
  hri_bitmap_cell* currentClone;
  //  newbtset->realx = btset->realx;
  //newbtset->realy = btset->realy;
  //newbtset->realz = btset->realz;

  bitmap_target->active = bitmap_source->active;
  bitmap_target->nx = bitmap_source->nx;
  bitmap_target->ny = bitmap_source->ny;
  bitmap_target->nz = bitmap_source->nz;
  bitmap_target->type = bitmap_source->type;
  bitmap_target->id = bitmap_source->id;
  //newbtset->pace = btset->pace;


  bitmap_target->searched = bitmap_source->searched;
  bitmap_target->calculate_cell_value = bitmap_source->calculate_cell_value;

  // copy cells
  for(i=0; i<bitmap_source->nx; i++) {
    for(j=0; j<bitmap_source->ny; j++) {
      for(k=0; k<bitmap_source->nz; k++) {
        bitmap_target->data[i][j][k].val = bitmap_source->data[i][j][k].val;
        bitmap_target->data[i][j][k].h = bitmap_source->data[i][j][k].h;
        bitmap_target->data[i][j][k].g =  bitmap_source->data[i][j][k].g;
        bitmap_target->data[i][j][k].parent = bitmap_source->data[i][j][k].parent;
        bitmap_target->data[i][j][k].closed =  bitmap_source->data[i][j][k].closed;
        bitmap_target->data[i][j][k].open   = bitmap_source->data[i][j][k].open;
        bitmap_target->data[i][j][k].x = bitmap_source->data[i][j][k].x;
        bitmap_target->data[i][j][k].y = bitmap_source->data[i][j][k].y;
        bitmap_target->data[i][j][k].z = bitmap_source->data[i][j][k].z;
        bitmap_target->data[i][j][k].locked = bitmap_source->data[i][j][k].locked;
      }
    }
  }
  // set pointers to cell clones
   if (bitmap_source->search_start != NULL) {
     bitmap_target->search_start =  hri_bt_get_cell(bitmap_target, bitmap_source->search_start->x, bitmap_source->search_start->y, bitmap_source->search_start->z);
   } else {
     bitmap_target->search_start = NULL;
   }
   if (bitmap_source->search_goal != NULL) {
     bitmap_target->search_goal = hri_bt_get_cell(bitmap_target, bitmap_source->search_goal->x, bitmap_source->search_goal->y, bitmap_source->search_goal->z);
   } else {
     bitmap_target->search_goal = NULL;
   }
   if (bitmap_source->current_search_node != NULL) {
     bitmap_target->current_search_node = hri_bt_get_cell(bitmap_target, bitmap_source->current_search_node->x, bitmap_source->current_search_node->y, bitmap_source->current_search_node->z);
   } else {
     bitmap_target->current_search_node = NULL;
   }
   // set all parent pointers in path to clones
   current = bitmap_source->search_goal;
   currentClone = bitmap_target->search_goal;
   while(current != bitmap_source->search_start && current != NULL) {
     currentClone->parent = hri_bt_get_cell(bitmap_target, current->parent->x, current->parent->y, current->parent->z);
     current = current->parent;
     currentClone = currentClone->parent;
   }
}

/****************************************************************/
/*!
 * \brief copy a bitmap
 *
 * \param bitmap the bitmap to be copied
 *
 * \return the new bitmap
 */
/****************************************************************/
hri_bitmap* hri_bt_create_copy(hri_bitmap* bitmap)
{
  hri_bitmap* newbitmap;
  int i,j;


  if(bitmap == NULL)
    return NULL;

  newbitmap = MY_ALLOC(hri_bitmap,1);

  // ALLOC cell space
  newbitmap->data = MY_ALLOC(hri_bitmap_cell**,bitmap->nx);
  for(i=0; i<bitmap->nx; i++) {
    newbitmap->data[i] = MY_ALLOC(hri_bitmap_cell*,bitmap->ny);
    for(j=0; j<bitmap->ny; j++) {
      newbitmap->data[i][j] = MY_ALLOC(hri_bitmap_cell,bitmap->nz);
    }
  }

  hri_bt_copy_bitmap_values(bitmap, newbitmap);

  return newbitmap;
}


/****************************************************************/
/*!
 * \brief Creates an empty bitmapset
 *
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
hri_bitmapset*  hri_bt_create_empty_bitmapset()
{
  int i;
  hri_bitmapset* bitmapset = MY_ALLOC(hri_bitmapset,1);

  if( bitmapset==NULL)
    return FALSE;

  bitmapset->bitmap = MY_ALLOC(hri_bitmap*,BTS_SIZE);
  if(bitmapset->bitmap==NULL)
    return FALSE;

  for(i=0; i<BTS_SIZE; i++){
    bitmapset->bitmap[i] = NULL;
  }

  bitmapset->pathexist = FALSE;

  return bitmapset;
}



/****************************************************************/
/*!
 * \brief Creates a bitmap structure with initialized cell structures
 *
 * \param x     xdimension of bitmap
 * \param y     ydimension
 * \param z     zdimension
 * \param pace  real distance equivalent of the dist between 2 cells
 * !

 * \return NULL in case of a problem
 */
/****************************************************************/
hri_bitmap*  hri_bt_create_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int))
{
  hri_bitmap* bitmap = hri_bt_create_empty_bitmap(x, y, z, pace, type, fct);

  if(bitmap == NULL)
    return NULL;

  // create all cells
  hri_bt_create_data(bitmap);


  return bitmap;
}

/****************************************************************/
/*!
 * \brief Creates necessary data field for an empty bitmap
 *
 * \param bitmap    empty bitmap
 *
 * \return NULL in case of a problem
 */
/****************************************************************/
int hri_bt_create_data(hri_bitmap* bitmap)
{
  int x,y;
  if(bitmap==NULL)
    return FALSE;
  if(bitmap->data != NULL){
    PrintError(("Memory leak bug, allocating bitmap data twice"));
  } else {
    bitmap->data = MY_ALLOC(hri_bitmap_cell**,bitmap->nx);
    for(x=0; x<bitmap->nx; x++) {
      bitmap->data[x] = MY_ALLOC(hri_bitmap_cell*,bitmap->ny);
      for(y=0; y<bitmap->ny; y++) {
        bitmap->data[x][y] = MY_ALLOC(hri_bitmap_cell,bitmap->nz);
      }
    }
  }
  hri_bt_reset_bitmap_data(bitmap);
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Creates an empty bitmap
 *
 * \param x     xdimension of bitmap
 * \param y     ydimension
 * \param z     zdimension
 * \param pace  real distance equivalent of the dist between 2 cells
 * \param type  type of the bitmap
 *
 * \return NULL in case of a problem
 */
/****************************************************************/
hri_bitmap*  hri_bt_create_empty_bitmap(int x, int y, int z, double pace, int type, double (*fct)(hri_bitmapset*,int, int, int))
{
  hri_bitmap* bitmap = MY_ALLOC(hri_bitmap,1);

  if(x < 1 || y < 1 || z < 1){
    PrintWarning(("NHP - Be careful, you're creating a bitmap with x<1 or y<1"));
    return NULL;
  }

  bitmap->active = FALSE; /* the activation flag allows the bitmap be visible on screen */

  //bitmap->realx = env->box.x1;
  //bitmap->realy = env->box.y1;
  //bitmap->realz = env->box.z1;

  bitmap->nx = x;
  bitmap->ny = y;
  bitmap->nz = z;
  //btset->pace = pace;
  bitmap->search_start = NULL;
  bitmap->search_goal = NULL;
  bitmap->current_search_node = NULL;
  bitmap->searched = FALSE;
  bitmap->type = type;
  bitmap->data = NULL;

  bitmap->calculate_cell_value = fct;

  return bitmap;
}

int hri_bt_change_bitmap_position(hri_bitmapset * btset, double x, double y, double z)
{
  if(btset == NULL)
    return FALSE;

  btset->realx = x;
  btset->realy = y;
  btset->realz = z;

  return TRUE;
}


/****************************************************************/
/*!
 * \brief Destroys a bitmap structure
 *
 * \param bitmap  the bitmap
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_bitmap(hri_bitmap* bitmap)
{
  if(bitmap==NULL)
    return TRUE;

  if(bitmap->data != NULL)
    hri_bt_destroy_bitmap_data(bitmap);

  MY_FREE(bitmap,hri_bitmap,1);
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a bitmap data
 *
 * \param bitmap  the bitmap
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_bitmap_data(hri_bitmap* bitmap)
{
  int x,y;

  for(x=0;x<bitmap->nx; x++) {
    for(y=0;y<bitmap->ny; y++)
      MY_FREE(bitmap->data[x][y],hri_bitmap_cell,bitmap->nz);
    MY_FREE(bitmap->data[x],hri_bitmap_cell*,bitmap->ny);
  }
  MY_FREE(bitmap->data,hri_bitmap_cell**,bitmap->nx);

  return TRUE;
}

/**
 * Calculates whether all path cell's xyz of two paths are equal
 */
int hri_bt_equalPath(hri_bitmap* bitmap1, hri_bitmap* bitmap2) {
  hri_bitmap_cell* current1 = bitmap1->search_goal;
  hri_bitmap_cell* current2 = bitmap2->search_goal;
  while (current1 != NULL && current2 != NULL) {
    if (current1->x != current2->x ||
        current1->y != current2->y ||
        current1->z != current2->z ) {
      break;
    }
    current1 = current1->parent;
    current2 = current2->parent;
  }
  // both should be NULL now if path are equal
  if (current1 != NULL || current2 != NULL) {
    return FALSE;
  }
  return TRUE;
}


/****************************************************************/
/*!
 * \brief get the cell of given coordinates
 *
 * \param bitmap the bitmap
 * \param x      x coord
 * \param y      y coord
 * \param z      z coord
 *
 * \return NULL in case of a problem
 */
/****************************************************************/
hri_bitmap_cell* hri_bt_get_cell(hri_bitmap* bitmap, int x, int y, int z)
{
  if(bitmap==NULL)
    return NULL;

  if(x<0 || x>bitmap->nx-1)
    return NULL;
  if(y<0 || y>bitmap->ny-1)
    return NULL;
  if(z<0 || z>bitmap->nz-1)
    return NULL;

  return &bitmap->data[x][y][z];
}



/**
 * returns the bitmap cell closest to x,y,z doubles, prefers positions on PATH
 * within grid cell distance BT_PATH_OLDPATH_FINDCELL_TOLERANCE
 */
hri_bitmap_cell* hri_bt_getCellOnPath(hri_bitmap* bitmap, double x, double y, double z) {
  hri_bitmap_cell* current;
  hri_bitmap_cell* candidate = NULL;
  double best_Distance = BT_PATH_OLDPATH_FINDCELL_TOLERANCE + 1, temp_distance;

  if (bitmap->type != BT_PATH || bitmap->search_goal == NULL) {
    candidate = hri_bt_get_cell(bitmap, (int) (x + 0.5), (int) (y + 0.5), (int) (z + 0.5)); //  + 0.5 causes rounding
  } else { // oldpath exist, check path cells first
    current = bitmap->search_goal;
    while (current != NULL ) {
      // search for the best fit on path, the cell with minimal distance
      temp_distance = DISTANCE3D(current->x, current->y, current->z, x, y, z);
      if ( temp_distance < BT_PATH_OLDPATH_FINDCELL_TOLERANCE && temp_distance < best_Distance) {
        candidate = current;
        best_Distance = temp_distance;
      }
      current = current->parent;
    }
    if (candidate == NULL) {
      candidate = hri_bt_get_cell(bitmap, (int) (x + 0.5), (int) (y + 0.5), (int) (z + 0.5)); //  + 0.5 causes rounding
    }
  }

  return candidate;
}
