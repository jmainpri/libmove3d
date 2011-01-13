#include "../include/hri_bitmap_cost.h"
/*
 * hri_bitmap_cost.c
 *
 * Helper functions dealing with costs of robot motion
 *
 */


/**
 * calculates the g cost of A*, the cost of a the path up to this cell,
 * considering it is reached from a certain parent cell.
 * This may return different values depending on from where we try to reach this cell.
 *
 * dimensions refers to the dimensions of the bitmap
 */
double hri_bt_A_CalculateCellG(hri_bitmapset * btset, hri_bitmap_cell* current_cell, hri_bitmap_cell* fromcell, double distance ) {
  // fromcell is closed, meaning its cost from search start are known to be minimal.
  if (fromcell->g < 0 || current_cell->val < 0){
    return -1;
  }
  double result = fromcell->g + current_cell->val + (distance * btset->parameters->path_length_weight);


  return result;
}

/****************************************************************/
/*!
 * \brief A* search: heuristic function h()
 * the purpose of this function is to slighly change the weights of cells
 * depending on the distance to the target
 *
 * \param bitmap the bitmap
 * \param x_s    x coord of current cell
 * \param x_s    y coord of current cell
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
double hri_bt_dist_heuristic(hri_bitmapset * btset, hri_bitmap* bitmap, int x_s, int y_s, int z_s)
{
  /**
   * Note that a heuristic must be GUARANTEED to return a value <= the real cost of the path, else A* becomes incorrect
   * However, the closer the value is to the realcost (the higher it is) the faster A* will find a solution.
   *
   * The calculation therefore depends on hri_bt_A_CalculateCellG
   */
  int x_f = bitmap->search_goal->x,
  y_f = bitmap->search_goal->y,
  z_f = bitmap->search_goal->z;

  // Taking the euclid distance and multiply it with distance costs
  // A better approximation could be to use some kind of Manhattan-like distance
  // or to use Voronoi-like paths to consider walls and obstacles
  return btset->parameters->path_length_weight * sqrt((double) SQR(x_f-x_s)+SQR(y_f-y_s)+SQR(z_f-z_s));

  /*
   double cost = 0;
   double h_2ddiag, h_2dmanh, h_diag;
   double D3 = M_SQRT3, D2 = M_SQRT2, D=1.;

   // if start = goal
   if(DISTANCE3D(x_s, y_s, z_s, x_f, y_f, z_f) == 0) {
   return 0;
   }

   // add minimal 3d manhattan distance times sqrt(3) to costs
   h_diag = MIN( MIN(ABS(x_f-x_s), ABS(y_f-y_s)) , ABS(z_f-z_s) );
   cost += h_diag * D3;

   if( MIN(ABS(x_f-x_s), ABS(y_f-y_s)) >  ABS(z_f-z_s)) {
   // if xy min manhattan distance < z distance
   h_2ddiag = MIN(ABS(x_f-x_s)-h_diag, ABS(y_f-y_s)-h_diag);
   h_2dmanh = ABS(x_f-x_s)-h_diag + ABS(y_f-y_s)-h_diag;
   cost+= D2 * h_2ddiag + D * (h_2dmanh - 2*h_2ddiag);
   } else {
   if( ABS(x_f-x_s) > ABS(y_f-y_s)){
   h_2ddiag = MIN(ABS(x_f-x_s)-h_diag, ABS(z_f-z_s)-h_diag);
   h_2dmanh = ABS(x_f-x_s)-h_diag + ABS(z_f-z_s)-h_diag;
   cost+= D2 * h_2ddiag + D * (h_2dmanh - 2*h_2ddiag);
   }
   else{
   h_2ddiag = MIN(ABS(z_f-z_s)-h_diag, ABS(y_f-y_s)-h_diag);
   h_2dmanh = ABS(z_f-z_s)-h_diag + ABS(y_f-y_s)-h_diag;
   cost+= D2 * h_2ddiag + D * (h_2dmanh - 2*h_2ddiag);
   }
   }
   cost+=cost;
   //cost*=(1+0.01);

   return cost;
   */
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

/****************************************************************/
/*!
 * \brief Calculate safety cost of the given coordinate
 *
 * \param x x coordinate
 * \param y y coordinate
 *
 * \return the cost
 */
/****************************************************************/
double hri_bt_calc_dist_value(hri_bitmapset * btset, int x, int y, int z)
{
  int i;
  double radius,height;
  double val = 0, sigmoid = 0, quot = 0, res =0;
  double realx, realy;
  double humanx, humany;
  double distance;

  if(btset==NULL){
    PrintError(("btset is null, cant get distance value\n"));
    return -1;
  }

  for(i=0; i<btset->human_no; i++){
    if(!btset->human[i]->exists)
      continue;
    height = btset->human[i]->state[btset->human[i]->actual_state].dheight;
    radius = btset->human[i]->state[btset->human[i]->actual_state].dradius;

    realx = (x*btset->pace)+btset->realx;
    realy = (y*btset->pace)+btset->realy;
    humanx = btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v;
    humany = btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v;

    distance = DISTANCE2D(realx,realy,humanx,humany);

    if(distance > radius) {
      val = 0;
    } else {
      // sigmoid function up to radius
      sigmoid = cos(distance / radius * M_PI_2) + 0;
      quot = 1 / (0.6 + distance );
      val = pow(height * (sigmoid * quot), 3);
    }
    if(res < val) {
      res = val;
    }
  }

  return res;

}




/****************************************************************/
/*!
 * \brief Calculate velocity cost of the given coordinate
 *
 * \param x x coordinate
 * \param y y coordinate
 *
 * \return the cost
 */
/****************************************************************/
double hri_bt_calc_vel_value(hri_bitmapset * btset,int x, int y, int z)
{



  return 0;
}



/****************************************************************/
/*!
 * \brief Calculate visibility cost of the given coordinate
 *
 * \param x x coordinate
 * \param y y coordinate
 *
 * \return the cost
 */
/****************************************************************/
double hri_bt_calc_vis_value(hri_bitmapset * btset, int x, int y, int z)
{
  int i;
  double radius,height,stretch_back;
  double val = 0,res =0;
  double realx, realy;
  double angle,angle_deviation,deltax,deltay, orient,distance_cosine;
  double humanx, humany;
  double distance;

  if(btset==NULL){
    PrintError(("btset is null, cant get visibility value\n"));
    return -1;
  }

  for (i=0; i<btset->human_no; i++) {
    if(!btset->human[i]->exists)
      continue;
    height = btset->human[i]->state[btset->human[i]->actual_state].vheight;
    stretch_back = btset->human[i]->state[btset->human[i]->actual_state].vback;
    radius = btset->human[i]->state[btset->human[i]->actual_state].vradius;


    realx = (x*btset->pace)+btset->realx;
    realy = (y*btset->pace)+btset->realy;
    humanx = btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v;
    humany = btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v;
    orient = btset->human[i]->HumanPt->joints[HUMANj_NECK_PAN]->dof_data->v + btset->human[i]->HumanPt->joints[HUMANj_BODY]->dof_data[5].v;

    distance = DISTANCE2D(realx,realy,humanx,humany);

    if(distance > radius ) {
      val = 0;
    } else {

      deltax = realx-humanx;
      deltay = realy-humany;
      angle = atan2(deltay, deltax);
      // orient goes from -PI to PI, angle goes from - PI to PI
      // get the absolute angle deviation between 0 and PI
      angle_deviation = ABS(getAngleDeviation(orient, angle));

      if (btset->human[i]->actual_state != BT_MOVING) {
        if ((angle_deviation < M_PI_4 )) {
            // leave open area in front of human
            val =0;
        } else {
            // cosine function is 0 at borders of radius
            distance_cosine = pow(cos((distance * M_PI_2) / radius), 2); // value between 0 and 1 depending on distance and radius

            // use stretch to increase / decrease weight more on more backward angles
            val = distance_cosine * (height + (angle_deviation - M_PI_4) * stretch_back);
        }
      } else {
        // if human is moving, visibility does not matter, however we predict
        // space in front of the human, assuming that's the direction the human is moving to
        if (angle_deviation < M_PI_4 ) {
            // predictive costs
            distance_cosine = pow(cos((distance * M_PI_2) / radius), 2); // value between 0 and 1 depending on distance and radius

            // use stretch to increase / decrease weight more on more backward angles
            val = distance_cosine * (height + (M_PI - angle_deviation) * stretch_back);
        } else {
            val = 0;
        }
      }
    }
    if(res < val) {
      res = val;
    }
  }

  return res;
}

/****************************************************************/
/*!
 * \brief Calculate the combined cost of the given coordinate
 *
 * \param x x coordinate
 * \param y y coordinate
 *
 * \returns in case of a sure collision, else the cost combining hidden zones, visibility and distance regardless of collisions
 */
/****************************************************************/
double hri_bt_calc_combined_value(hri_bitmapset * btset, int x, int y, int z)
{
  double dist, vis, hz;
  int result;

  if(btset==NULL || btset->bitmap==NULL){
    PrintError(("Try to calculate an unexisting bitmap\n"));
    return -2;
  }

  if (btset->bitmap[BT_OBSTACLES]->data[x][y][z].val == BT_OBST_SURE_COLLISION) {
    return -2;
  }


  // if( btset->bitmap[BT_OBSTACLES]!= NULL &&  btset->bitmap[BT_OBSTACLES]->data != NULL)
  //   if(btset->bitmap[BT_OBSTACLES]->data[x][y][z].val < 0)
  //     return -1;      COMMENT TO CHECK OBSTACLES OUTSIDE OF THIS FUNCTION ACCORDING TO WHERE WE CAME FROM


  /* res = p3d_col_test_robot(btset->robot,TRUE); */

  /*   if(res){ */
  /*     printf("Robot in colision!!!\n"); */
  /*     return 0; */
  /*   } */

  hz =  btset->bitmap[BT_HIDZONES]->calculate_cell_value(btset,x,y,z);
  // hri_bt_calc_hz_value(x,y,z,0,NULL);

  // TK 2011: Why?
//  if(hz > -1)
//    return hz;

  vis  =  btset->bitmap[BT_VISIBILITY]->calculate_cell_value(btset,x,y,z);
  // hri_bt_calc_vis_value(x,y,z,0,NULL);
  dist =  btset->bitmap[BT_DISTANCE]->calculate_cell_value(btset,x,y,z);
  // dist = hri_bt_calc_dist_VALUE(x,y,z,0,NULL);
  //printf("Values: %f %f\n",dist,vis);
  if(btset->combine_type == BT_COMBINE_SUM) {
    result = dist + vis + hz;
  } else if(btset->combine_type == BT_COMBINE_MAX) {
    result = MAX(dist, vis);
    result = MAX(result, hz);
  } else {
    PrintError(("Can't combine bitmaps\n"));
    result = 0;
  }

  // experimental feature to avoid robot moving in the middle of corridor, walk on left or right lane instead
  // usually turned off, so val != BT_OBST_SURE_CORRIDOR_MARK unless feature turned on
  if (btset->bitmap[BT_OBSTACLES]->data[x][y][z].val == BT_OBST_SURE_CORRIDOR_MARK) {
    result += btset->parameters->corridor_Costs;
  }

  if(result > 0 && result < BT_NAVIG_THRESHOLD) {
    // too little to matter for safety and comfort, but can still make the robot change ways,
    //best to make robot ignore those to have a straight path
    result = 0;
  }

  // add costs around objects for feeling of object and robot safety
  if(btset->bitmap[BT_OBSTACLES]->data[x][y][z].val > 0) {
    result +=btset->bitmap[BT_OBSTACLES]->data[x][y][z].val;
  }

  return result;
}
