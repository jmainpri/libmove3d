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
  bitmapset->manip = BT_MANIP_NAVIGATION;
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


/****************************************************************/
/*!
 * \brief get the cell closest to given real coordinates
 *
 * \param bitmap the bitmap
 * \param x      x coord
 * \param y      y coord
 * \param z      z coord
 *
 * \return NULL in case of a problem
 */
/****************************************************************/
hri_bitmap_cell* hri_bt_get_closest_cell(hri_bitmapset* bitmapset, hri_bitmap* bitmap, double x, double y, double z)
{
  // round by adding 0.5
  return hri_bt_get_cell(bitmap,
      (int)(((x- bitmapset->realx) / bitmapset->pace) + 0.5),
      (int)(((y- bitmapset->realx) / bitmapset->pace) + 0.5),
      (int)(((z- bitmapset->realx) / bitmapset->pace) + 0.5));
}

/**
 * returns the angle difference between two angles as value between -PI and PI,
 * meaning the smaller of the left and right side angles
 */
double getAngleDeviation(double angle1, double angle2) {
  double angle_deviation = angle1 - angle2;
  // get the angle deviation between -PI and PI
  if (angle_deviation < -M_PI) {
    angle_deviation = M_2PI + angle_deviation;
  } else if (angle_deviation > M_PI) {
    angle_deviation = M_2PI - angle_deviation;
  }
  return angle_deviation;
}


/**
 * returns the bitmap cell closest to x,y,z doubles of real world
 */
hri_bitmap_cell* hri_bt_getCellOnPath(hri_bitmapset * btset, hri_bitmap* bitmap, double x, double y, double z) {
  hri_bitmap_cell* current;
  hri_bitmap_cell* candidate = NULL;
  hri_bitmap_cell* candidate_next = NULL;

  double best_Distance = btset->parameters->path_reuse_cell_startcell_tolerance + 1, temp_distance;

  if (bitmap->type != BT_PATH || bitmap->search_goal == NULL) {
    candidate = hri_bt_get_closest_cell(btset, bitmap, x, y, z); //  + 0.5 causes rounding
  } else { // oldpath exist, check path cells first
    current = bitmap->search_goal;
    while (current != NULL ) {
      // search for the best fit on path, the cell with minimal distance
      temp_distance = DISTANCE3D(current->x, current->y, current->z, x, y, z);
      if ( temp_distance < btset->parameters->path_reuse_cell_startcell_tolerance && temp_distance < best_Distance) {
        candidate_next = candidate;
        candidate = current;
        best_Distance = temp_distance;
      }
      current = current->parent;
    }
    if (candidate == NULL) {
      candidate = hri_bt_get_closest_cell(btset, bitmap, x, y, z); //  + 0.5 causes rounding
    } else {
      // prefer ot the closest cell, but the next cell on path after tha, to avoid the robot going back
      if (candidate_next != NULL) {
        return candidate_next;
      }
    }
  }

  return candidate;
}


/**
 * Checks for 3d collision for robot movement from one cell to another, return FALSE if none, TRUE if collision
 * assuming for now a robot that in each waypoint stops, turns, moves in sequence, and takes the shorter rotation angle
 */
int localPathCollides (hri_bitmapset * btset, hri_bitmap_cell* cell, hri_bitmap_cell* fromcell )
{
  const int PI_STEPS = 32; // the higher, the more angles will be tested in rotation (32 seems to work with B21)

  configPt qorigin;
  double target_angle;
  double original_angle;
  int result = FALSE;
  int i, j;

  qorigin = p3d_get_robot_config(btset->robot); /*ALLOC */
  target_angle = atan2(cell->y - fromcell->y, cell->x - fromcell->x);

  // very quick static test whether target position is in collision
  qorigin[ROBOTq_X] = cell->x * btset->pace + btset->realx;
  qorigin[ROBOTq_Y] = cell->y * btset->pace + btset->realy;
  qorigin[ROBOTq_RZ] = target_angle;
  // moved the robot config qc to current cell position and angle
  p3d_set_and_update_this_robot_conf(btset->robot, qorigin); // move the robot to cell
  if( p3d_col_test_robot_statics(btset->robot, FALSE)) { // check whether robot collides
    result = TRUE;
  } else {
    // test for collisions with non-static objects
    for (i = 0; i<btset->human_no; i++){
      if (!btset->human[i]->exists || btset->human[i]->transparent)
        continue;
      if (p3d_col_test_robot_other(btset->robot, btset->human[i]->HumanPt, FALSE)) {
        result = TRUE;
        break;
      }
    }
  }

  if (result == FALSE) {
    if (fromcell->parent != NULL) {// set the angle the robot would be in parent cell
      original_angle = atan2(fromcell->y - fromcell->parent->y, fromcell->x - fromcell->parent->x);
    } else {
      original_angle = qorigin[ROBOTq_RZ];
    }

    /**** test if initial rotation collides, if rotation is required ****/
    if (ABS(target_angle - original_angle) > 0.1) { // allow for rounding error

      // instead of real localpath test for rotation, make a few in between tests at 22,5 degree angles

      // we assume that the difference between the angles is not 180 degrees (would make no sense for a-star)
      // angles are values between - pi and + pi
      int steps = (int) ((target_angle - original_angle) / (M_PI / PI_STEPS));
      if (steps < -PI_STEPS) {
        steps = steps + PI_STEPS * 2;
      } else if (steps > PI_STEPS) {
        steps = steps - PI_STEPS * 2;
      }
      // steps is the number of times M_PI / PI_STEPS has to be added to original angle to reach target angle, between -7 and 7

      /*** check for each step whether robot with that rotation collides with objects **/

      // prepare robot configuration in origin spot
      qorigin[ROBOTq_X] = fromcell->x * btset->pace + btset->realx;
      qorigin[ROBOTq_Y] = fromcell->y * btset->pace + btset->realy;

      steps > 0? steps-- : steps++; // do not recalculate original position static collision
//      printf ("%f to %f = %i steps  \n", original_angle, target_angle, steps);
      for (i = steps; i != 0; steps > 0? i-- : i++) {
//        printf ("step %i = %f \n", i , qorigin[ROBOTq_RZ]);
        qorigin[ROBOTq_RZ] = original_angle + (i * (M_PI / PI_STEPS));
        // moved the robot config qc to current cell position and angle
        p3d_set_and_update_this_robot_conf(btset->robot, qorigin);
        if( p3d_col_test_robot_statics(btset->robot, FALSE)) { // check whether robot collides
          result = TRUE;
          break;
        }
        if (result == FALSE) {
          // test for collisions with non-static objects
          for (j = 0; j<btset->human_no; j++){
            if ( !btset->human[j]->exists ||
                btset->human[j]->transparent )
              continue;
            if (p3d_col_test_robot_other(btset->robot, btset->human[j]->HumanPt, FALSE)) {
              result = TRUE;
              break;
            }
          }
        }
      }
    }
  } // end if target test passed

  p3d_destroy_config(btset->robot, qorigin); /* FREE */

  return result;
}

/**
 * computes the euclidean distance
 */
double getCellDistance (hri_bitmap_cell* cell1, hri_bitmap_cell* cell2 )
{
  return DISTANCE3D(cell1->x, cell1->y, cell1->z, cell2->x, cell2->y, cell2->z);
}


/**
 * Checks for 3d collision for robot movement from one cell to next, return FALSE if none, TRUE if collision
 * assuming for now a robot that in each waypoint stops, turns, moves in sequence, and takes the shorter rotation angle
 *
 * this function is MUCH more costy to perform than the stepwise static check
 */
int localPathCollidesFullCheck (hri_bitmapset * btset, hri_bitmap_cell* cell, hri_bitmap_cell* fromcell )
{
  configPt qorigin, qtarget;
  double target_angle;
  double original_angle;
  int result = FALSE;
  p3d_localpath *path=NULL;
  double temp_env_dmax;
  int ntest;

  qorigin = p3d_get_robot_config(btset->robot); /*ALLOC */
  target_angle = atan2(cell->y - fromcell->y, cell->x - fromcell->x);


  // very quick static test whether target position is in collision
  qorigin[ROBOTq_X] = cell->x * btset->pace + btset->realx;
  qorigin[ROBOTq_Y] = cell->y * btset->pace + btset->realy;
  qorigin[ROBOTq_RZ] = target_angle;
  // moved the robot config qc to current cell position and angle
  p3d_set_and_update_this_robot_conf(btset->robot, qorigin); // move the robot to cell
  if( p3d_col_test_robot_statics(btset->robot, FALSE)) { // check whether robot collides
    result = TRUE;
  }

  if (result == FALSE) {
    qtarget = p3d_get_robot_config(btset->robot); /*ALLOC */

    temp_env_dmax = p3d_get_env_dmax();
    p3d_set_env_dmax(0);

    if (fromcell->parent != NULL) {// set the angle the robot would be in parent cell
      original_angle = atan2(fromcell->y - fromcell->parent->y, fromcell->x - fromcell->parent->x);
    } else {
      original_angle = qorigin[ROBOTq_RZ];
    }


    /**** test if initial rotation collides, if rotation is required ****/
    if (ABS(target_angle - original_angle) > 0.1) { // allow for rounding error

      // create robot position in origin cell with target rotation
      qorigin[ROBOTq_X] = fromcell->x * btset->pace + btset->realx;
      qorigin[ROBOTq_Y] = fromcell->y * btset->pace + btset->realy;
      qorigin[ROBOTq_RZ] = target_angle;

      // create robot position at origin cell with original rotation
      qtarget[ROBOTq_X] = fromcell->x * btset->pace + btset->realx;
      qtarget[ROBOTq_Y] = fromcell->y * btset->pace + btset->realy;
      qtarget[ROBOTq_RZ] = original_angle;

      path = p3d_local_planner(btset->robot, qtarget, qorigin);

      if (path == NULL ||  p3d_unvalid_localpath_test(btset->robot, path, &ntest)){
        result = TRUE;
      }
    }

// commented out because much too costy, and no existing testcase
//    /**** test movement forward collides ****/
//    if (result == FALSE) { // if no rotation or rotation was ok
//      // create robot position at target cell
//      qorigin[ROBOTq_X] = cell->x * btset->pace + btset->realx;
//      qorigin[ROBOTq_Y] = cell->y * btset->pace + btset->realy;
//      qorigin[ROBOTq_RZ] = target_angle;
//
//      // create robot position at origin cell
//      qtarget[ROBOTq_X] = fromcell->x * btset->pace + btset->realx;
//      qtarget[ROBOTq_Y] = fromcell->y * btset->pace + btset->realy;
//      qtarget[ROBOTq_RZ] = target_angle;
//
//      path = p3d_local_planner(btset->robot, qtarget, qorigin);
//
//      if (path == NULL ||  p3d_unvalid_localpath_test(btset->robot, path, &ntest)){
//        result = TRUE;
//      }
//
//    } // end if rotation test passed

    /** clean up **/

    p3d_destroy_config(btset->robot, qtarget); /* FREE */

    p3d_set_env_dmax(temp_env_dmax);
    destroy_list_localpath(btset->robot, path);
  } // end if static test passed

  p3d_destroy_config(btset->robot, qorigin); /* FREE */

  return result;
}

/**
 * returns an approximation of the radius of the bounding circle (2D) of a robot,
 * meaning the distance between its rotational center and the farthest point.
 */
double getRotationBoundingCircleRadius(p3d_rob *robot)
{
  configPt robotq;
  double original_rz;

  double rotation_radius = 0;

  if (robot != NULL) {
    robotq = p3d_get_robot_config(robot); /** ALLOC **/
    original_rz = robotq[ROBOTq_RZ];
    robotq[ROBOTq_RZ] = 0;
    /* turn the robot rz to zero to get its zero position bounding box
     * it is not minimal, but that way, we at least get the same expand rate
     * for any turning angle of the robot
     */
    p3d_set_and_update_this_robot_conf(robot, robotq);
    // calculate the distance between the robot turning point
    //(robotq[ROBOTq_X], robotq[ROBOTq_Y] assuming it is in the middle of the BB) and the bounding box corners
    // choose between comparing to min or max coordinates
    rotation_radius =
      MAX(DISTANCE2D(robot->BB.xmax, robot->BB.ymax, robotq[ROBOTq_X], robotq[ROBOTq_Y]),
          DISTANCE2D(robot->BB.xmin, robot->BB.ymin, robotq[ROBOTq_X], robotq[ROBOTq_Y]));
    // restore orignal robot rotation
    robotq[ROBOTq_RZ] = original_rz;
    p3d_set_and_update_this_robot_conf(robot, robotq);

    p3d_destroy_config(robot, robotq); /** FREE **/
  }
  return rotation_radius;
}
