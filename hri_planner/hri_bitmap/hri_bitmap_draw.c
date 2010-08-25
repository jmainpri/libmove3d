#include "../include/hri_bitmap_draw.h"

#ifndef MAX
#define MAX(a,b)  ( (a) > (b) ? (a) : (b) )
#endif

#ifndef ABS_CEIL
/* returns ceiling for positive values, and floor for negative values. ABS_CEIL(-3.2)= -4, ABS_CEIL(3,2)= 4*/
#define ABS_CEIL(value) ((value) < 0) ? floor(value): ceil(value)
#endif

#ifndef ABS_FLOOR
/* returns floor for positive values, and ceil for negative values. ABS_CEIL(-3.2)= -3, ABS_CEIL(3,2)= 3*/
#define ABS_FLOOR(value) ((value) < 0) ? ceil(value): floor(value)
#endif

/****************************************************************/
/*!
 * \brief Put one obstacle to the bitmap, obs value is -1
 *
 * \param bitmap  a bitmap
 * \param obj     obstacle object
 * \param expand  the tolerance buffer around the object
 * \param value   the value this objects boundigbox should give to bitmap cells
 * \param manip   TRUE if we care about 3D space
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_insert_obs(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_obj* obj, p3d_env* env, double expand, double value, int manip)
{
//  int objxmin, objxmax, objymin, objymax, objzmin, objzmax;

  if(bitmap == NULL){
    PrintError(("NHP - Cannot create obstacles bitmap=NULL\n"));
    return FALSE;
  }

  if(obj == NULL){
    PrintError(("object bitmap insertion problem\n"));
    return FALSE;
  }

  // check if object + expand is on bitmap, ignore z for navigation
  if(obj->BB.xmax + expand < btset->realx || obj->BB.xmin - expand > bitmap->nx * btset->pace + btset->realx ||
      obj->BB.ymax + expand  < btset->realy || obj->BB.ymin - expand > bitmap->ny * btset->pace + btset->realy ||
      ((obj->BB.zmax + expand  < btset->realz || obj->BB.zmin - expand > bitmap->nz * btset->pace + btset->realz) && manip)) {
    /*PrintError(("object out of range\n"));*/
    return FALSE;
  }


  hri_bt_fill_bitmap_zone(btset ,bitmap,
      obj->BB.xmin, obj->BB.xmax,
      obj->BB.ymin, obj->BB.ymax,
      obj->BB.zmin, obj->BB.zmax,
      expand, value, manip);

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Put one obstacle to the bitmap, obs value is -1
 *
 * \param G       a graph
 * \param bitmap  a bitmap
 * \param obj     obstacle object
 * \param robot  to whom the obstacle belongs
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_insert_obsrobot(hri_bitmapset * btset, hri_bitmap* bitmap, p3d_rob* obj, p3d_env* env, double expand, double value, int manip)
{


  if(bitmap == NULL){
    PrintError(("NHP - Cannot create obstacles bitmap=NULL\n"));
    return FALSE;
  }

  if(obj == NULL){
    PrintError(("object bitmap insertion problem\n"));
    return FALSE;
  }


  /* printf("Object limits %f %f, %f %f\n",obj->BB.xmin,obj->BB.xmax,obj->BB.ymin,obj->BB.ymax ); */

  // check if object + expand is on bitmap, ignore z for navigation
  if(obj->BB.xmax + expand < btset->realx || obj->BB.xmin - expand > bitmap->nx * btset->pace + btset->realx ||
      obj->BB.ymax + expand < btset->realy || obj->BB.ymin - expand > bitmap->ny * btset->pace + btset->realy ||
      ((obj->BB.zmax + expand < btset->realz || obj->BB.zmin - expand > bitmap->nz * btset->pace + btset->realz) && manip)) {
    /*PrintError(("object out of range\n"));*/
    return FALSE;
  }

  hri_bt_fill_bitmap_zone(btset ,bitmap,
       obj->BB.xmin, obj->BB.xmax,
       obj->BB.ymin, obj->BB.ymax,
       obj->BB.zmin, obj->BB.zmax,
       expand, value, manip);

  return TRUE;
}



/****************************************************************/
/*!
 * \brief Set a value for a given capped rectangle on bitmap around a real rectangle
 *
 * ............
 * .o/------\o.    # = inner rectangle
 * ./++++++++\.    +/\ = zone around rectangle with expand distance minimum
 * .|++####++|.    o  = corners not covered
 * .|++####++|.
 * .\++++++++/.
 * .o\------/o.
 * ............
 *
 * \param bitmap    a bitmap
 * \param objxmin   the minimum x real coordinate of rectancle
 * \param objxmax
 * \param objymin
 * \param objymax
 * \param objzmin
 * \param objzmax
 * \param val       the value to be set
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int  hri_bt_fill_bitmap_zone(hri_bitmapset * btset, hri_bitmap* bitmap, double xmin, double xmax, double ymin,
    double ymax, double zmin, double zmax, double expand, int val, int manip)
{
  int x,y,z;
  // outer coordinates
  int objxmin, objxmax, objymin, objymax, objzmin, objzmax;
  // inner coordinates for fast calculation
  int inner_xmin, inner_xmax, inner_ymin, inner_ymax, inner_zmin, inner_zmax;
  int is_inside_x, is_inside_y, is_inside_z;
  // double coordinates for checking grid points in edges
  double refx, refy, refz, cellx, celly, cellz;
  double new_val, distance;

  //  calculate the cell coordinates for the capped obstacle bounding box
  objxmin = ABS_FLOOR((xmin - btset->realx - expand) / btset->pace);
  objxmax = ABS_CEIL((xmax - btset->realx + expand) / btset->pace);
  objymin = ABS_FLOOR((ymin - btset->realy - expand) / btset->pace);
  objymax = ABS_CEIL((ymax - btset->realy + expand) / btset->pace);
  if (manip) {
    objzmin = ABS_FLOOR((zmin - btset->realz - expand) / btset->pace);
    objzmax = ABS_CEIL((zmax - btset->realz + expand) / btset->pace);
  } else { // for navigation, we only care about 2d space (for practical reasons)
    objzmin = 0;
    objzmax = 0;
  }

  //  calculate the cell coordinates for the inner real bounding box
  inner_xmin = ABS_FLOOR((xmin - btset->realx ) / btset->pace);
  inner_xmax = ABS_CEIL((xmax - btset->realx ) / btset->pace);
  inner_ymin = ABS_FLOOR((ymin - btset->realy ) / btset->pace);
  inner_ymax = ABS_CEIL((ymax - btset->realy ) / btset->pace);
  if (manip) {
    inner_zmin = ABS_FLOOR((zmin - btset->realz) / btset->pace);
    inner_zmax = ABS_CEIL((zmax - btset->realz) / btset->pace);
  } else { // for navigation, we only care about 2d space (for practical reasons)
    inner_zmin = 0;
    inner_zmax = 0;
  }

  //  printf("Obstacle %s placed at %i,%i,%i to %i,%i,%i with expand %f\n", obj->name, objxmin, objymin, objzmin, objxmax, objymax, objzmax, expand);


  if(bitmap == NULL) {
    return FALSE;
  }

  if(objxmin<0)
    objxmin = 0;
  if(objymin<0)
    objymin = 0;
  if(objzmin<0)
    objzmin = 0;
  if(objxmax>bitmap->nx-1)
    objxmax = bitmap->nx-1;
  if(objymax>bitmap->ny-1)
    objymax = bitmap->ny-1;
  if(objzmax>bitmap->nz-1)
    objzmax = bitmap->nz-1;

  for(x=objxmin; x<objxmax+1; x++) {
    for(y=objymin; y<objymax+1; y++) {
      for(z=objzmin; z<objzmax+1; z++) {
        if (bitmap->data[x][y][z].val == BT_OBST_SURE_COLLISION) {
          continue;
        }
        // calculate the real coordinates of grid cell
        cellx = btset->realx + (x * btset->pace);
        celly = btset->realy + (y * btset->pace);
        cellz = btset->realz + (z * btset->pace);
        is_inside_x = FALSE;
        is_inside_y = FALSE;
        is_inside_z = FALSE;
        if (x <= inner_xmin) {
          refx = xmin;
        } else if (x >= inner_xmax) {
          refx = xmax;
        } else {
          is_inside_x = TRUE;
          refx = cellx;
        }
        if (y <= inner_ymin) {
          refy = ymin;
        } else if (y >= inner_ymax) {
          refy = ymax;
        } else {
          is_inside_y = TRUE;
          refy = celly;
        }
        if (!manip && z <= inner_zmin) {
          refz = zmin;
        } else if (!manip && x >= inner_xmax) {
          refz = zmax;
        } else {
          is_inside_z = TRUE;
          refz = cellz;
        }

        if (!manip) {
          distance = DISTANCE2D(cellx, celly, refx, refy);
        } else {
          distance = DISTANCE3D(cellx, celly, cellz, refx, refy, refz);
        }

        // need to check whether we are outside caps
        if ( (!is_inside_x) &&
            (!is_inside_y) &&
            (!manip || !is_inside_z)) {
          // we are in the cap zone, check distance of grid coordinate to inner rectangle edge
          if (distance > expand) {
            //for rounded corners, in the cap zone we cnsider distance to closest edge
            continue;
          }
        }

        // In border regions around center BB, for the object potential 3d collisions, add extra cost to the bitmap
        if ( val == BT_OBST_POTENTIAL_OBJECT_COLLISION &&
            ((!is_inside_x) ||
            (!is_inside_y) ||
            (!is_inside_z)) ) {

          new_val = pow((cos(distance / expand * M_PI_2 )) * btset->parameters->soft_collision_distance_weight, 2) + btset->parameters->soft_collision_base_cost;
        } else if (val == BT_OBST_MARK_CORRIDOR) {
          // mark cells with 0 with potential mark, and cells with potential mark with sure mark
          if (bitmap->data[x][y][z].val == BT_OBST_POTENTIAL_CORRIDOR_MARK) {
            bitmap->data[x][y][z].val = BT_OBST_SURE_CORRIDOR_MARK;
          } else if(bitmap->data[x][y][z].val == 0 ) {
            bitmap->data[x][y][z].val = BT_OBST_POTENTIAL_CORRIDOR_MARK;
          }

          // ignore all other cells
          continue;

        } else {
          new_val = val;
        }

//        printf("%f\n", bitmap->data[x][y][z].val);
        // since we use this for obstacles, do not override a smaller value with a higher one
        if (new_val == BT_OBST_SURE_COLLISION || bitmap->data[x][y][z].val == 0 || bitmap->data[x][y][z].val < new_val) {
          bitmap->data[x][y][z].val = new_val;
        }
      }
    }
  }

  return TRUE;
}

/**
 * removs all potential corridor marks, add costs for all sure corridor marks
 */
void hri_bt_clearCorridorMarks(hri_bitmapset * btset, hri_bitmap* bitmap)
{
  int x,y,z;
  for(x=0; x < bitmap->nx; x++) {
    for(y=0; y < bitmap->ny; y++) {
      for(z=0; z < bitmap->nz; z++) {
        if (bitmap->data[x][y][z].val == BT_OBST_POTENTIAL_CORRIDOR_MARK) {
          bitmap->data[x][y][z].val = 0;
        }
//        else if (bitmap->data[x][y][z].val == BT_OBST_SURE_CORRIDOR_MARK) {
//          bitmap->data[x][y][z].val = btset->parameters->corridor_Costs;
//        }
      }
    }
  }
}


/****************************************************************/
/*!
 * \brief Shows a presentation of a found path on bitmap
 *
 * \param bitmap    a bitmap
 * \param color
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
void hri_bt_show_path(hri_bitmapset * btset, hri_bitmap* bitmap)
{
  hri_bitmap_cell* current;
  int i,j,color;

  if(bitmap == NULL)
    return;

  if(bitmap->searched) {

    // vertical lines on start and goal
    g3d_drawOneLine(
        bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0,
        bitmap->search_start->x*btset->pace+btset->realx, bitmap->search_start->y*btset->pace+btset->realy, 0.5, Red, NULL);
    g3d_drawOneLine(
        bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0,
        bitmap->search_goal->x*btset->pace+btset->realx, bitmap->search_goal->y*btset->pace+btset->realy, 0.5, Red, NULL);

    // the path itself
    current = bitmap->search_goal;
    if (bitmap->nz == 1) {
      while(current != bitmap->search_start && current != NULL) {
              g3d_drawOneLine( current->x*btset->pace+btset->realx,
                  current->y*btset->pace+btset->realy,
                  0.1,
                  current->parent->x*btset->pace+btset->realx,
                  current->parent->y*btset->pace+btset->realy,
                  0.1,
                  Green, NULL);
              current = current->parent;
            }
    } else {

      while(current != bitmap->search_start && current != NULL){
        g3d_drawOneLine( current->x*btset->pace+btset->realx,
            current->y*btset->pace+btset->realy,
            current->z*btset->pace+btset->realz,
            current->parent->x*btset->pace+btset->realx,
            current->parent->y*btset->pace+btset->realy,
            current->parent->z*btset->pace+btset->realz,
            Green, NULL);
        current = current->parent;
      }
    }

    // all open nodes
    for(i=0; i<bitmap->nx; i++){
      for(j=0; j<bitmap->ny; j++){
        if(bitmap->data[i][j][0].open == 1)
          color = Blue;
        if(bitmap->data[i][j][0].closed == 1)
          color = Red;
        if(bitmap->data[i][j][0].parent != NULL) {
          g3d_drawOneLine(
              i*btset->pace+btset->realx, j*btset->pace+btset->realy, 0,
              bitmap->data[i][j][0].parent->x*btset->pace+btset->realx,bitmap->data[i][j][0].parent->y*btset->pace+btset->realy, 0, color, NULL);
        }
      }
    }
  }

}

// unknown obsolete code
//void hri_bt_show_cone(hri_bitmapset * btset, hri_bitmap* bitmap, int h, int r)
//{
//  int x,y,z;
//
//  for(x=0; x<bitmap->nx; x++){
//    for(y=0; y<bitmap->ny; y++){
//      for(z=0; z<50; z++){
//  if(sqrt(x*x+y*y)-z == 0)
//    g3d_drawOneLine(x*btset->pace,y*btset->pace,1-z*0.01,
//        x*btset->pace,y*btset->pace,1,1,NULL);
//      }
//    }
//  }
//}
