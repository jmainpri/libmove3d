#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"
#include "Localpath-pkg.h"
#include "Collision-pkg.h"
#include "Graphic-pkg.h"
#include "Hri_planner-pkg.h"
#include "math.h"
#include "include/hri_bitmap_util.h"
#include "include/hri_bitmap_draw.h"
#include "include/hri_bitmap_cost.h"
#include "include/hri_bitmap_bin_heap.h"

#ifndef MAX
#define MAX(a,b)  ( (a) > (b) ? (a) : (b) )
#endif

#define HUMAN 111
#define CENTER 112


hri_bitmapset* BTSET = NULL;
hri_bitmapset * ACBTSET = NULL;
pp3d_graph BTGRAPH = NULL;

int PLACEMENT;
int PLCMT_TYPE;

static int insert2table(double value, int cx, int cy, int cz, double * Table,	int * x, int * y, int * z, int l);

static int is_in_fow(double xh, double yh, double xt, double yt, double orient, double fowangle);

/* similar to M_SQRT2 in math.h*/
#ifndef M_SQRT3
#define M_SQRT3 1.732050807568877294
#endif

#ifndef M_SQRT5
#define M_SQRT5 2.236067977499789696
#endif

/****************************************************************/
/*!
 * \brief Destroys a bitmapset structure
 *
 * \param bitmapset  the bitmapset
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_bitmapset(hri_bitmapset* bitmapset)
{
  int i;

  if(bitmapset->bitmap != NULL){
    for(i=0; i<bitmapset->n; i++){
      hri_bt_destroy_bitmap(bitmapset->bitmap[i]);
    }
    MY_FREE(bitmapset->bitmap,hri_bitmap*,BT_BITMAP_NO);
  }

  for(i=0; i<bitmapset->human_no; i++){
    if(bitmapset->human[i] != NULL)
      hri_bt_destroy_human(bitmapset->human[i]);
  }
  MY_FREE(bitmapset->human,hri_human*,bitmapset->human_no);
  MY_FREE(bitmapset->parameters, hri_astar_parameters, 1);

  if(bitmapset->path)
    hri_bt_destroy_path(bitmapset);

  MY_FREE(bitmapset,hri_bitmapset,1);

  bitmapset = NULL;

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a human :-)
 *
 * \param human  the human
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_human(hri_human* human)
{
  int i;

  for(i=0; i<BT_STATE_NO; i++){
    hri_bt_destroy_state(human->state[i]);
  }
  MY_FREE(human->state,hri_human_state,BT_STATE_NO);
  MY_FREE(human,hri_human,1);

  return TRUE;
}

/****************************************************************/
/*!
 * \brief Destroys a human state
 *
 * \param state  the state
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_destroy_state(hri_human_state state)
{
  /* it's empty for the moment */

  return TRUE;

}

/****************************************************************/
/*!
 * \brief Marks all bitmaps of the given type in the set as activeand recalculates all weigths
 *
 * \param type  the bitmap
 *
 * \return TRUE if successful
 */
/****************************************************************/
int hri_bt_activate(int type, hri_bitmapset* bitmapset)
{

  if(bitmapset==NULL || bitmapset->bitmap==NULL)
    return FALSE;

  hri_bitmap *bitmap = hri_bt_get_bitmap(type, bitmapset);

  if(bitmap==NULL)
    return FALSE;

  if(bitmap->data == NULL) {
    hri_bt_create_data(bitmap);
  }
  if (type== BT_COMBINED) { // need to initialize obstacles bitmap to activate combined.
    if(hri_bt_get_bitmap(BT_OBSTACLES, bitmapset) == NULL) {
      hri_bt_create_data(hri_bt_get_bitmap(BT_OBSTACLES, bitmapset));
    }
  }
  if(type != BT_PATH ) { // oath bitmap is filled on findPath
    if ( !hri_bt_fill_bitmap(bitmapset, type)) {
      PrintWarning(("NHP - Try to fill an unvalid typed bitmap: %i", type));
      return FALSE;
    }
  }
  bitmap->active = TRUE;
  return TRUE;
}

/****************************************************************/
/*!
 * \brief Fill the bitmap with right parameters as defined by the bitmaps own calculate function
 *
 * \param type  type of the bitmap
 *
 * \return TRUE if successful
 */
/****************************************************************/
int hri_bt_fill_bitmap(hri_bitmapset * btset, int type)
{
  int x,y,z;

  // check whether type is legal
  if (type > (btset->n-1)){
    return FALSE;
  }

  // TK human can change position and comeinto existence
  if(type == BT_OBSTACLES){
    if(btset->bitmap[BT_OBSTACLES] != NULL) {
      hri_bt_create_obstacles(btset);
      return TRUE;
    } else {
      return FALSE;
    }
  }

  if(type == BT_COMBINED){
    hri_bt_create_obstacles(btset);
  }

  if(type == BT_PATH){
    PrintWarning(("NHP - Trying to fill a BT_PATH bitmap\n"));
    return TRUE;
  }

  for(x=0; x<btset->bitmap[type]->nx; x++){
    for(y=0; y<btset->bitmap[type]->ny; y++){
      for(z=0; z<btset->bitmap[type]->nz; z++){
        btset->bitmap[type]->data[x][y][z].val =
        btset->bitmap[type]->calculate_cell_value(btset, x, y, z);
      }
    }
  }

  return TRUE;
}


/****************************************************************/
/*!
 * \brief Put all obstacles to the bitmaps, obs value is -1
 *
 * \param G          a graph
 * \param bitmapset  bitmaps
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_create_obstacles( hri_bitmapset* btset )
{
  int i, j, discard_movable_object, is_human;
  p3d_env* env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  /* expand rates: expands obstacles on the grid such that robot positions around the obstacle become
   * unavailable if they make the robot and the obstacle collide. Will be transformed to grid distance
   * at last possible moment to reduce rounding errors.
   */
  double safe_expand_rate, minimum_expand_rate;


  if(btset == NULL)
    return FALSE;

  // set all cells to 0 first
  hri_bt_reset_bitmap_data(btset->bitmap[BT_OBSTACLES]);

  safe_expand_rate = getRotationBoundingCircleRadius(btset->robot);


  // defined in Move3d/include/Hri_planner-pkg.h
#ifdef HRI_JIDO
  minimum_expand_rate = 0.40 - 1 * btset->pace;  /* THIS IS FOR JIDO  - NEEDS TO BE DONE PROPERLY*/
#else
  minimum_expand_rate = 0.20; // guessed for arbitrary robots
#endif

  // creates wide blue perimeter around walls
  for(i=0; i<env->no ; i++) {
    // base collides
    hri_bt_insert_obs(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, minimum_expand_rate, BT_OBST_SURE_COLLISION, 0);
    // potential 3d collision
    hri_bt_insert_obs(btset,btset->bitmap[BT_OBSTACLES], env->o[i], env, safe_expand_rate, BT_OBST_POTENTIAL_OBJECT_COLLISION, 0);
  }

  if (btset->parameters->use_corridors == TRUE) {
    for(i=0; i<env->no ; i++) {
      // prepare for corridor weigths
      hri_bt_insert_obs(btset, btset->bitmap[BT_OBSTACLES], env->o[i], env, safe_expand_rate * 1.5, BT_OBST_MARK_CORRIDOR, 0);
    }
    hri_bt_clearCorridorMarks(btset, btset->bitmap[BT_OBSTACLES]);
  }

  //  creates red perimeter around objects
  for(i=0; i<env->nr; i++) {
    // for all movable objects that are not the robot, (strcmp works the other way round)
    discard_movable_object = FALSE;
    is_human = FALSE;
    if( !strcasestr(env->robot[i]->name,"ROBOT") && !strcasestr(env->robot[i]->name,"VISBALL")) {

      // check robot is not non-existing human
      for(j=0; j<btset->human_no; j++){
        // check whether robot is this human (strcmp works the other way round)
        if (!strcasecmp(env->robot[i]->name, btset->human[j]->HumanPt->name)) {
          is_human = TRUE;
          // only care if human exist
          if(btset->human[j]->exists == FALSE) {
            discard_movable_object = TRUE;
          } else if(btset->human[j]->transparent) {
            /* for existing humans,
             * if human moves, assume the space he occupies may become free, so no obstacle
             * (needs careful controller)
             */
            discard_movable_object = TRUE;
          }
          break;
        }
      }

      if (discard_movable_object)
        continue;

      hri_bt_insert_obsrobot(btset, btset->bitmap[BT_OBSTACLES], env->robot[i], env, minimum_expand_rate, BT_OBST_SURE_COLLISION, 0);
      if (strcasestr(env->robot[i]->name, "TABLE")) {
          hri_bt_insert_obsrobot(btset, btset->bitmap[BT_OBSTACLES], env->robot[i], env, safe_expand_rate, BT_OBST_POTENTIAL_OBJECT_COLLISION, 0);
      } else if (is_human ) {
        // not drawing soft collisions for other objects, as they are not checked later anyways
        hri_bt_insert_obsrobot(btset, btset->bitmap[BT_OBSTACLES], env->robot[i], env, safe_expand_rate, BT_OBST_POTENTIAL_HUMAN_COLLISION, 0);
      }
      /* printf("Obstacles updated for %s\n",env->robot[i]->name); */
    }
  }


  return TRUE;
}



/* REVISION */
/****************************************************************/
/*!
 * \brief Shows a presentation a bitmap
 *
 * \param bitmap    a bitmap
 *
 * \return FALSE in case of a problem
 */
/***************************************************************/
void  hri_bt_show_bitmap(hri_bitmapset * btset, hri_bitmap* bitmap)
{
  int i,j;
  //  double colorvector[4];
  //  colorvector[0] = 1;       //red
  //  colorvector[1] = 0;       //green
  //  colorvector[2] = 0;       //blue
  //  colorvector[3] = 0.75;    //transparency

  // drawline default parameters
  double scale, length, base, value;
  int color;


  if( bitmap->type == BT_PATH){
    hri_bt_show_path(btset,bitmap);
    return;
  }



  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      color = Blue;
      scale = 0.01;
      length = 0.1;
      base = bitmap->data[i][j][0].val;
      value = bitmap->data[i][j][0].val;

      // Adapt parameters
      switch(bitmap->type){
        case BT_DISTANCE:
          if ( bitmap->data[i][j][0].val >0) {
            color = Green;
          } else {
            continue;
          }
          break;
        case BT_VISIBILITY:
          if ( bitmap->data[i][j][0].val >0) {
            color = Green;
          } else {
            continue;
          }
          break;
        case BT_HIDZONES:
          if(bitmap->data[i][j][0].val <= 0)
            continue; // don't draw
          break;
        case BT_OBSTACLES:
          if(bitmap->data[i][j][0].val == 0)
            continue; // don't draw
          if(bitmap->data[i][j][0].val == BT_OBST_SURE_CORRIDOR_MARK) {
            base = 0;
            value = 0;
            length = - 0.1;
            color = Green; // don't draw
          } else if(bitmap->data[i][j][0].val == BT_OBST_SURE_COLLISION) {
            base = 0;
            value = 0;
            length = - 0.1;
            color = Red;
          }
          break;
        case BT_COMBINED:
          if(bitmap->data[i][j][0].val == -2)
            color = Red;
          if(bitmap->data[i][j][0].val == -1)
            continue; // don't draw
          break;
        case BT_VELOCITY:
          if(bitmap->data[i][j][0].val <= -1)
            continue;// don't draw
          break;
      }


      g3d_drawOneLine(i*btset->pace+btset->realx, j*btset->pace+btset->realy, base * scale,
                      i*btset->pace+btset->realx, j*btset->pace+btset->realy, value * scale + length, color, NULL);


      //				TK: Old obsolete code
      //					case BT_VELOCITY:
      //						if(bitmap->data[i][j][0].val != -1)
      //							g3d_drawOneLine(i*btset->pace,j*btset->pace, bitmap->data[i][j][0].val, i*btset->pace,j*btset->pace,bitmap->data[i][j][0].val+50, 1, NULL);
      //						break;
      //
      //
    }
  }
}

/* REVISION*/
int hri_bt_save_bitmap(hri_bitmapset* btset,hri_bitmap * B)
{
  FILE* f = NULL;
  int i, j, k;

  f = fopen("gridsample.txt","w");
  fprintf(f,"#grid sample: the first two int is the length and height followed by \\n \n");
  fprintf(f,"#grid sample: negative values are obstacles, no permitted to cross over \n");
  //fprintf(f,"%d %d\n ",(int)B->nx,(int)B->ny);
  for(i=0; i<B->nx; i++){
    for(j=0; j<B->ny; j++){
      for(k=0; k<B->nz; k++){
        fprintf(f,"%f %f %f %f\n ",i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz,B->data[i][j][k].val);
      }
      fprintf(f,"\n");
    }
    fprintf(f,"\n");
  }
  fclose(f);

  return 0;
}


/* REVISION*/
/****************************************************************/
/*!
 * \brief Shows a presentation all bitmaps
 *
 * \param bitmapset   an array of bitmap
 * \param step        the diffenrence of height between bitmaps
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
void  hri_bt_show_bitmapset(hri_bitmapset* bitmapset)
{
  int i;

  for(i=0; i<BT_BITMAP_NO; i++){
    hri_bt_show_bitmap(bitmapset,bitmapset->bitmap[i]);
  }

}


/**
 * Sets default pathfinding weights and flags
 */
void hri_bt_init_btset_parameters(hri_bitmapset* bitmapset)
{
  bitmapset->parameters = MY_ALLOC(hri_astar_parameters, 1);
  bitmapset->parameters->path_length_weight = 40;
  bitmapset->parameters->soft_collision_distance_weight = 8;
  bitmapset->parameters->soft_collision_base_cost = 15;
  bitmapset->parameters->start_cell_tolerance = 2;
  bitmapset->parameters->goal_cell_tolerance = 8; // high tolerance for flawed grasp planning

  /** distance beyond which we do not care about moving humans
   * anymore, as by the time the robot gets there, the human will be
   * somewhere else. If negative, moving humans will always be considered.
   * if zero, moving humans will never be considered. Default is -1 **/
  bitmapset->parameters->moving_human_deprecation = -1;

  /** reluctance is an experimental feature to prefer previously planned. Default is FALSE.
   * paths to new ones, that does not seem to change the robot behavior much (maybe in special cases?).
   * Also might be buggy as result path start is not request start */
  bitmapset->parameters->use_changepath_reluctance = FALSE;
  bitmapset->parameters->path_reuse_cell_startcell_tolerance = 3;
  bitmapset->parameters->path_reuse_threshold = 30;
  /** corridors is an experimental feature to increase costs in the middle of corridors, benefit was not proved yet. Default is FALSE*/
  bitmapset->parameters->use_corridors = FALSE;
  bitmapset->parameters->corridor_Costs = 50;
  /** directional calculations are an experimental feature, default is FALSE */
  bitmapset->parameters->directional_cost = FALSE;
  /** Prevents sharp turns If the angle in the path is greater than this number, the edge won't be allowed.
   * This is useful in particular when using directional costs.
   * When value is > Pi, e.g. 4, all angles are allowed.
   * Experimental feature, therefore Default = 4  */
  bitmapset->parameters->angle2d_minimum = 4; //M_PI_2 - 0.1;
  /** used if directional_cost == TRUE
   * the angle at which a human is considered "behind" the robot and thus not relevant for costs
   * default is M_PI_2 = 90 degrees**/
  bitmapset->parameters->directional_freePassAngle = M_PI_2;
  /** used if directional_cost == TRUE
     * the inverse angle at which a moving human in front of the robot is considered "harmless",
     * as he does not move towards the robot, but will go elsewhere
     * 0 means the human goes in the opposite direction (conflict), Pi means he is going in same direction (nice)
     * default is M_PI_4 = 45 degrees **/
  bitmapset->parameters->directional_noConflictHeading = M_PI_4;
}



/**
 * Creates an empty bitmapset for the current p3d env, identifies and counts robot, humans, bottle and visball
 */
hri_bitmapset* hri_bt_create_bitmaps()
{
  int i, hnumber=0;

  hri_bitmapset* bitmapset = hri_bt_create_bitmapsworobots();
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  for(i=0; i<env->nr; i++) {
    if( strcasestr(env->robot[i]->name,"human"))
      hnumber++;
  }

  bitmapset->human = MY_ALLOC(hri_human*, hnumber);

  hnumber = 0;
  for(i=0; i<env->nr; i++) {
    if( strcasestr(env->robot[i]->name,"ROBOT") ) {
      bitmapset->robot = env->robot[i];
    } else if( strcasestr(env->robot[i]->name,"human")) {
      bitmapset->human[hnumber] = hri_bt_create_human(env->robot[i]);
      hnumber++;
    } else if( strcasestr(env->robot[i]->name,"VISBALL") ) {
      bitmapset->visball = env->robot[i];
    } else if( strcasestr(env->robot[i]->name,"BOTTLE") ) {
      bitmapset->object = env->robot[i];
    }
  }

  if(hnumber == 0)
    PrintWarning(("NHP - No humans in the environment"));
  if(bitmapset->visball == NULL)
    PrintWarning(("NHP - No visibility ball present, check the p3d file"));

  bitmapset->human_no = hnumber;

  return bitmapset;
}


/**
 * Creates an empty bitmapset without reading robot information from the p3d environment
 */
hri_bitmapset* hri_bt_create_bitmapsworobots()
{
  hri_bitmapset* bitmapset = MY_ALLOC(hri_bitmapset,1);

  bitmapset->human = NULL;
  bitmapset->human_no = 0;
  bitmapset->visball = NULL;
  bitmapset->robot = NULL;
  bitmapset->actual_human = 0;
  bitmapset->bitmap = NULL;
  bitmapset->manip = BT_MANIP_NAVIGATION;
  bitmapset->BT_target_available = FALSE;

  hri_bt_init_btset_parameters(bitmapset);

  return bitmapset;
}

/****************************************************************/
/*!
 * \brief Initializes a bitmapset structure with empty bitmaps
 *
 * \param x     xdimension of bitmaps in cells, must be >= 1
 * \param y     ydimension >= 1
 * \param z     zdimension >= 1
 * \param pace  real distance equivalent of the dist between 2 cells
 *
 * \return NULL in case of a problem
 */
/****************************************************************/
int hri_bt_init_bitmaps(hri_bitmapset * bitmapset, int x, int y, int z, double pace)
{
  p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);

  if(x < 1 || y < 1 || z < 1){
    PrintWarning(("NHP - Creating empty bitmaps with %i<1 or %i<1 or %i<1", x, y, z));
    return FALSE;
  }

  bitmapset->realx = env->box.x1;
  bitmapset->realy = env->box.y1;
  bitmapset->realz = env->box.z1;
  bitmapset->pace = pace;

  bitmapset->bitmap = MY_ALLOC(hri_bitmap*,BT_BITMAP_NO);
  bitmapset->max_size = BT_BITMAP_NO;

  bitmapset->bitmap[BT_DISTANCE]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_DISTANCE, hri_bt_calc_dist_value);
  bitmapset->bitmap[BT_VISIBILITY] = hri_bt_create_empty_bitmap(x,y,z,pace,BT_VISIBILITY,hri_bt_calc_vis_value);
  bitmapset->bitmap[BT_HIDZONES]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_HIDZONES,hri_bt_calc_hz_value);
  bitmapset->bitmap[BT_OBSTACLES]  = hri_bt_create_bitmap(x,y,z,pace,BT_OBSTACLES,NULL);
  bitmapset->bitmap[BT_VELOCITY]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_VELOCITY,hri_bt_calc_vel_value);
  bitmapset->bitmap[BT_COMBINED]   = hri_bt_create_empty_bitmap(x,y,z,pace,BT_COMBINED,hri_bt_calc_combined_value);
  bitmapset->bitmap[BT_PATH]       = hri_bt_create_bitmap(x,y,z,pace,BT_PATH,hri_bt_calc_combined_value);

  bitmapset->n = 7;

  bitmapset->path = NULL;
  bitmapset->pathexist = FALSE;
  bitmapset->combine_type = BT_COMBINE_SUM; /* default value */
  bitmapset->changed = FALSE;



  return TRUE;

}

/**
 * creates default human base from a robot in the environment
 * adds default sitting and standing states
 */
hri_human* hri_bt_create_human(p3d_rob * robot)
{
  hri_human * human = MY_ALLOC(hri_human,1);

  human->HumanPt = robot;
  human->state = MY_ALLOC(hri_human_state, BT_STATE_NO);
  human->states_no = BT_STATE_NO;
  human->exists = FALSE; /* HUMAN EXIST */
  human->transparent = FALSE; /* HRI_PLANNER_GUI may move through human */
  human->id = -1;

  // human aware navigation costs

  strcpy(human->state[BT_SITTING].name,"SITTING");
  human->state[BT_SITTING].dheight = 8;
  human->state[BT_SITTING].dradius = 1.3;
  human->state[BT_SITTING].vheight = 50;
  human->state[BT_SITTING].vback = 50;
  human->state[BT_SITTING].vradius = 2.4;
  human->state[BT_SITTING].hradius = 0.0;

  strcpy(human->state[BT_STANDING_TRANSPARENT].name,"STANDING TRANS");
  human->state[BT_STANDING_TRANSPARENT].dheight = 8;
  human->state[BT_STANDING_TRANSPARENT].dradius = 1.3;
  human->state[BT_STANDING_TRANSPARENT].vheight = 40;
  human->state[BT_STANDING_TRANSPARENT].vback = 50;
  human->state[BT_STANDING_TRANSPARENT].vradius = 2;
  human->state[BT_STANDING_TRANSPARENT].hradius = 0.0;

  strcpy(human->state[BT_STANDING].name,"STANDING");
  human->state[BT_STANDING].dheight = 8;
  human->state[BT_STANDING].dradius = 1.3;
  human->state[BT_STANDING].vheight = 40;
  human->state[BT_STANDING].vback = 50;
  human->state[BT_STANDING].vradius = 2;
  human->state[BT_STANDING].hradius = 0.0;

  strcpy(human->state[BT_MOVING].name,"MOVING");
  human->state[BT_MOVING].dheight = 8;
  human->state[BT_MOVING].dradius = 1.5;
  human->state[BT_MOVING].vheight = 40;
  human->state[BT_MOVING].vback = 40;
  human->state[BT_MOVING].vradius = 1.6;
  human->state[BT_MOVING].hradius = 0.0;


  // joint states for ACHILLE and BATMAN models of human
  if (strcasestr(robot->name,"achile")) {
  	// ACHILE joints
  	human->state[BT_SITTING].c1 =  DTOR(-90);// left hip
  	human->state[BT_SITTING].c2 =  DTOR( 90);// left knee
  	human->state[BT_SITTING].c3 =  DTOR(-90);// right hip
  	human->state[BT_SITTING].c4 =  DTOR( 90);// right knee
  	human->state[BT_SITTING].c7 =  0.60;// torso height

  	human->state[BT_STANDING_TRANSPARENT].c1 = 0;// left hip
  	human->state[BT_STANDING_TRANSPARENT].c2 = 0;// left leg
  	human->state[BT_STANDING_TRANSPARENT].c3 = 0;// right hip
  	human->state[BT_STANDING_TRANSPARENT].c4 = 0;// right leg
  	human->state[BT_STANDING_TRANSPARENT].c7 = 1;// torso height

  	human->state[BT_STANDING].c1 = 0;// left hip
  	human->state[BT_STANDING].c2 = 0;// left leg
  	human->state[BT_STANDING].c3 = 0;// right hip
  	human->state[BT_STANDING].c4 = 0;// right leg
  	human->state[BT_STANDING].c7 = 1;// torso height

  	human->state[BT_MOVING].c1 =  0;// left hip
  	human->state[BT_MOVING].c2 =  DTOR(-5);// left leg
  	human->state[BT_MOVING].c3 =  0;// right hip
  	human->state[BT_MOVING].c4 =  DTOR( 20);// right leg
  	human->state[BT_MOVING].c5 = DTOR(-10);// left knee
  	human->state[BT_MOVING].c6 = DTOR(-10);// right knee
  	human->state[BT_MOVING].c7 = 1;// torso height
  } else {
  	if (strcasestr(robot->name,"superman") == NULL) {
  		PrintWarning(("Human model robots should be named starting with ACHILE or SUPERMAN"));
  	}
  	// BATMAN
  	human->state[BT_SITTING].c1 =  DTOR(-14.08);// left hip
  	human->state[BT_SITTING].c2 =  DTOR( 90);// left leg
  	human->state[BT_SITTING].c3 =  DTOR( 14.08);// right hip
  	human->state[BT_SITTING].c4 =  DTOR( 90);// right leg
  	human->state[BT_SITTING].c5 =  DTOR(-90);// left knee
  	human->state[BT_SITTING].c6 =  DTOR(-90);// right knee
  	human->state[BT_SITTING].c7 =  0.46;// torso height

  	human->state[BT_STANDING_TRANSPARENT].c1 = 0;// left hip
  	human->state[BT_STANDING_TRANSPARENT].c2 = 0;// left leg
  	human->state[BT_STANDING_TRANSPARENT].c3 = 0;// right hip
  	human->state[BT_STANDING_TRANSPARENT].c4 = 0;// right leg
  	human->state[BT_STANDING_TRANSPARENT].c5 = 0;// left knee
  	human->state[BT_STANDING_TRANSPARENT].c6 = 0;// right knee
  	human->state[BT_STANDING_TRANSPARENT].c7 = 0.85;// torso height

  	human->state[BT_STANDING].c1 = 0;// left hip
  	human->state[BT_STANDING].c2 = 0;// left leg
  	human->state[BT_STANDING].c3 = 0;// right hip
  	human->state[BT_STANDING].c4 = 0;// right leg
  	human->state[BT_STANDING].c5 = 0;// left knee
  	human->state[BT_STANDING].c6 = 0;// right knee
  	human->state[BT_STANDING].c7 = 0.85;// torso height

  	human->state[BT_MOVING].c1 =  0;// left hip
  	human->state[BT_MOVING].c2 =  DTOR(-5);// left leg
  	human->state[BT_MOVING].c3 =  0;// right hip
  	human->state[BT_MOVING].c4 =  DTOR( 20);// right leg
  	human->state[BT_MOVING].c5 = DTOR(-10);// left knee
  	human->state[BT_MOVING].c6 = DTOR(-10);// right knee
  	human->state[BT_MOVING].c7 = 0.85;// torso height
  }

  // default state, even for ghosts
  human->actual_state = BT_STANDING;

  return human;

}

/* REVISION */
/****************************************************************/
/*!
 * \brief create the graph regarding to the path found on bitmap
 * used for display of robot motion in Move3d
 * \param G      the graph
 * \param bitmap the bitmap
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_bitmap_to_GRAPH(hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap)
{
  configPt q;
  p3d_node *prev_node;
  double dist = 0;
  int done = FALSE;
  p3d_node *NewNode = NULL;
  double prev_orient;

  if(bitmap->searched) {
    bitmap->current_search_node = bitmap->search_goal;
  } else {
    return FALSE;
  }
  /* c'est ici qu'on doit gerer le bras */

  prev_orient = atan2(G->last_node->N->q[ROBOTq_Y]-bitmap->current_search_node->y,
                      G->last_node->N->q[ROBOTq_X]-bitmap->current_search_node->x);

  prev_node =  G->last_node->N;

  if (G->rob->nb_dof < ROBOTq_RZ) {
    PrintError(("Incompatible Robot DOFs."));
    return FALSE;
  }
  if (G->rob->nb_dof < ROBOTq_PAN) {
    PrintWarning(("Bad robot used for copying config"));
  }
  // need to alloc as graph uses alloced data. Start with goal config of arms, as we go backwards in path
  q = p3d_copy_config(G->rob, G->search_goal->q); /* ALLOC */

  while(!done){
    if(bitmap->current_search_node == bitmap->search_start) {
      done = TRUE;
    }
    /*  if( (bitmap->current_search_node != bitmap->search_goal) &&  */
    /* 	(bitmap->current_search_node != bitmap->search_start)){ */

    q[ROBOTq_X] = bitmap->current_search_node->x * btset->pace + btset->realx;
    q[ROBOTq_Y] = bitmap->current_search_node->y * btset->pace + btset->realy;

    /*  q[ROBOTq_RZ] = (prev_orient + atan2((bitmap->current_search_node->y-bitmap->current_search_node->parent->y), */
    /* 					(bitmap->current_search_node->x-bitmap->current_search_node->parent->x)))/2; */
    if(bitmap->current_search_node != bitmap->search_start) {
      q[ROBOTq_RZ] = atan2((bitmap->current_search_node->parent->y-bitmap->current_search_node->y),
                           (bitmap->current_search_node->parent->x-bitmap->current_search_node->x))+M_PI;
    } else{
      q[ROBOTq_RZ] = G->search_start->q[ROBOTq_RZ];
    }

    // create path of configuration where robot always looks ahead
    if (G->rob->nb_dof >= ROBOTq_PAN) {
      q[ROBOTq_PAN] = 0;
    }

    if(!p3d_equal_config(G->rob, q, G->search_start->q)){
      NewNode = p3d_APInode_make(G, q);
      NewNode->type = LINKING;
      p3d_insert_node(G, NewNode);
      dist = p3d_APInode_dist(G,prev_node,NewNode);
      p3d_create_edges(G,prev_node,NewNode,dist);
      p3d_add_node_compco(NewNode, prev_node->comp, TRUE);
      prev_node = NewNode;
    }

    q = p3d_copy_config(G->rob, G->search_start->q);  /* ALLOC */
    bitmap->current_search_node = bitmap->current_search_node->parent;

  } // end while
    // destroy the last q as it was never used
  p3d_destroy_config(G->rob,q);
  dist = p3d_APInode_dist(G,prev_node,G->search_start);
  p3d_create_edges(G,prev_node ,G->search_start ,dist);
  p3d_merge_comp(G, G->search_start->comp, &(prev_node->comp));

  return TRUE;

}

/****************************************************************/
/*!
 * \brief resets all the path related item in bitmaps
 *
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
void  hri_bt_reset_path(hri_bitmapset * btset)
{
  hri_bitmap* bitmap;

  bitmap = btset->bitmap[BT_PATH];

  if(bitmap == NULL){
    PrintError(("Try to erase a path from non path bitmap\n"));
    return;
  }

  bitmap->search_start = NULL;
  bitmap->search_goal = NULL;
  bitmap->current_search_node = NULL;
  bitmap->searched = FALSE;

  hri_bt_reset_bitmap_data(bitmap);

  btset->pathexist = FALSE;

}




/****************************************************************/
/*!
 * \brief Starts A* search to find a path the bitmap bitmap of a bitmapset with start finish nodes
 *
 * \param qs  Start config
 * \param qf  End config
 * \param bitmapset the bitmapset
 * \param manip whether we do this for navigation or armmanipulation.
 *
 * \return path costs in case of success, and an error code else
 */
/****************************************************************/
double hri_bt_start_search(double qs[3], double qf[3], hri_bitmapset* bitmapset, int manip)
{
  hri_bitmap* bitmap;
  hri_bitmap* bitmap_oldpath = NULL;
  hri_bitmap_cell* new_search_start;
  hri_bitmap_cell* new_search_goal;
  double oldcost;

  double result;

  if(bitmapset==NULL || bitmapset->bitmap[BT_PATH]==NULL || bitmapset->bitmap[BT_OBSTACLES]==NULL || bitmapset->bitmap[BT_COMBINED] == NULL){
    PrintError(("Trying to find a path in a non existing bitmap or bitmapset\n"));
    return FALSE;
  }

  // if either of those 2 bitmaps are active in Move3d, they are up to date anyways. In MHP they are never active.
  if( !bitmapset->bitmap[BT_OBSTACLES]->active && !bitmapset->bitmap[BT_COMBINED]->active){
    // need to make sure obstacle bitmap has been initialized.
    hri_bt_create_obstacles(bitmapset);
  }
  bitmap = bitmapset->bitmap[BT_PATH];

  // get new goal cell and compare to old goal cell, to see whether we still want to go to same place
  new_search_goal  =
      hri_bt_get_closest_cell(bitmapset, bitmap,
          qf[0],
          qf[1],
          qf[2]);


  if (bitmapset->parameters->use_changepath_reluctance
      && bitmap->search_goal != NULL
      && DISTANCE3D(bitmap->search_goal->x,
          bitmap->search_goal->y,
          bitmap->search_goal->z,
          new_search_goal->x,
          new_search_goal->y,
          new_search_goal->z) <= bitmapset->pace) {
    // if we searched in this bitmap before,
    // and we search going to the same goal as last time,
    // then we look for a start cell on the path if possible to be able to compare paths costs.
    new_search_start = hri_bt_getCellOnPath(bitmapset, bitmap,
        qs[0],
        qs[1],
        qs[2]);
  } else {
    if (!manip) {
      // choose the closest grid cell
      new_search_start =
          hri_bt_get_closest_free_cell(bitmapset, bitmap,
              qs[0],
              qs[1],
              qs[2],
              bitmapset->robot->ROBOT_POS[11],
              bitmapset->parameters->start_cell_tolerance);
      if(new_search_start == NULL) {
        bitmapset->pathexist = FALSE;
        PrintWarning(("No free search start cell for (%f, %f) in cell range %f\n", qs[0], qs[1], bitmapset->parameters->start_cell_tolerance));
        return HRI_PATH_SEARCH_ERROR_NAV_START_IN_OBSTACLE;
      }
    } else {
      // choose the closest grid cell
      new_search_start =
          hri_bt_get_closest_cell(bitmapset, bitmap,
              qs[0],
              qs[1],
              qs[2]);
      if(new_search_start == NULL) {
        PrintWarning(("Search start cell does not exist for (%f, %f) \n", qs[0], qs[1]));
        bitmapset->pathexist = FALSE;
        return HRI_PATH_SEARCH_ERROR_NAV_INTERNAL_ERROR;
      }
    }
  }


  if(new_search_goal == NULL ){
    PrintWarning(("Search goal cell does not exist for (%f, %f)\n", qf[0], qf[1]));
    bitmapset->pathexist = FALSE;
    return HRI_PATH_SEARCH_ERROR_NAV_INTERNAL_ERROR;
  }




  // hri_bt_create_obstacles(bitmapset); // update obstacle map

  // the following checks are all just relevant for navigation, not for manipulation
  if (!manip) {
      if(hri_bt_isRobotOnCellInCollision(bitmapset, bitmap, new_search_goal, bitmapset->robot->ROBOT_GOTO[11], FALSE)) {
          if (FALSE) {
              bitmapset->pathexist = FALSE;
              PrintError(("Goal Position is in an obstacle or human (%f, %f) \n", qf[0], qf[1]));
              return HRI_PATH_SEARCH_ERROR_NAV_GOAL_IN_OBSTACLE;
          } else {
              new_search_goal =
                  hri_bt_get_closest_free_cell(bitmapset, bitmap,
                      qf[0],
                      qf[1],
                      qf[2],
                      bitmapset->robot->ROBOT_GOTO[11],
                      bitmapset->parameters->goal_cell_tolerance);
              if(new_search_goal == NULL) {
                  bitmapset->pathexist = FALSE;
                  PrintError(("No free Goal Position is free around (%f, %f) in cell range %d\n", qf[0], qf[1], bitmapset->parameters->goal_cell_tolerance));
                  return HRI_PATH_SEARCH_ERROR_NAV_GOAL_IN_OBSTACLE;
              }
          }
      }
  } // endif not manip

  //printf("Search goal cell  for (%f, %f), %d, %d\n", qf[0], qf[1], new_search_goal->x, new_search_goal->y);

  if (bitmapset->parameters->use_changepath_reluctance
      && bitmapset->pathexist) {
    /* reluctance to change means the robot will stay on an old path */
    // check whether new request is for the same goal as old in bitmap
    if (bitmap->search_goal == new_search_goal) {
      // store old path in case we want to keep it
      bitmap_oldpath = hri_bt_create_copy(bitmap); /* ALLOC */
    }
  }

  // reset the path, also clears data from earlier failed attempts when !pathexist
  hri_bt_reset_path(bitmapset);

  bitmap->search_start = new_search_start;
  bitmap->search_goal  = new_search_goal;


  /******** calculating the path costs *************/
  result = hri_bt_astar_bh(bitmapset, bitmap);

  if (bitmapset->parameters->use_changepath_reluctance
      && bitmap_oldpath != NULL) {
    oldcost = hri_bt_keep_old_path(bitmapset, bitmap_oldpath, bitmap, result, new_search_start);
    // check whether the robot should prefer to stay on the old path
    if (oldcost > 0) {
      result = oldcost;
      hri_bt_copy_bitmap_values(bitmap_oldpath, bitmapset->bitmap[BT_PATH]);
    }
    hri_bt_destroy_bitmap(bitmap_oldpath); /* FREE */
  } // endif oldpath was stored


  if (result > -1) {
    bitmapset->pathexist = TRUE;
    return result;
  } else {
    bitmapset->pathexist = FALSE;
    return HRI_PATH_SEARCH_ERROR_NAV_NAV_NO_PATH_FOUND;
  }

}





/****************************************************************/
/*!
 * \brief Draws active bitmaps
 *
 * \param type type of the bitmap
 *
 * \return
 */
/****************************************************************/
void g3d_hri_bt_draw_active_bitmaps(hri_bitmapset* bitmapset)
{
  int i;
  if (bitmapset != NULL && bitmapset->bitmap != NULL) {
    for (i=0; i < bitmapset->n; i++) {
      if( bitmapset->bitmap[i]->active) {
        hri_bt_show_bitmap(bitmapset, bitmapset->bitmap[i]);
      }
    }
  }
}

/****************************************************************/
/*!
 * \brief Draws the robot's target
 *
 * \param type type of the bitmap
 *
 * \return
 */
/****************************************************************/
void g3d_hri_bt_draw_targets(hri_bitmapset* bitmapset)
{
  if (bitmapset == NULL)
    return;

  if (bitmapset->BT_target_available) {
    g3d_drawDisc(bitmapset->BT_target[0], bitmapset->BT_target[1], 0, 0.5, 1, NULL);
  }
}


/****************************************************************/
/*!
 * \brief  desactivate bitmap of given type
 *
 * \param type type of the bitmap
 *
 * \return
 */
/****************************************************************/
void hri_bt_desactivate(int type, hri_bitmapset* bitmapset)
{

  hri_bitmap *bitmap = hri_bt_get_bitmap(type, bitmapset);

  if (bitmap != NULL)
    bitmap->active = FALSE;
}

/****************************************************************/
/*!
 * \brief activate bitmap of given type
 *
 * \param type type of the bitmap
 *
 * \return
 */
/****************************************************************/
int hri_bt_is_active(int type, hri_bitmapset* bitmapset)
{
  hri_bitmap *bitmap = hri_bt_get_bitmap(type, bitmapset);
  return (bitmap != NULL && bitmap->active);
}

/* REVISION */
/****************************************************************/
/*!
 * \brief refill all the active bitmaps
 *
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_refresh_all(hri_bitmapset * btset)
{
  int i;

  if(btset == NULL || btset->bitmap==NULL)
    return FALSE;

  for(i=0; i<btset->n; i++){
    if(btset->bitmap[i]->active){
      switch(btset->bitmap[i]->type){
        case BT_DISTANCE:
          hri_bt_update_distance(btset,btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dheight,
                                 btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dradius);
          break;
        case BT_VISIBILITY:
          hri_bt_update_visibility(btset,btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vheight,
                                   btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vback,
                                   btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vradius);
          break;
        case BT_HIDZONES:
          hri_bt_update_hidzones(btset,btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].hradius);
          break;
        case BT_COMBINED:
          if (! btset->bitmap[BT_OBSTACLES]->active) {
            // since loop only considers active bitmaps, we only create obstacles once.
            hri_bt_create_obstacles(btset);
          }
          hri_bt_update_combined(btset);
          break;
        case BT_OBSTACLES:
          hri_bt_create_obstacles(btset);
          break;
      }
    }
  }
  return TRUE;
}

/* REVISION */
/****************************************************************/
/*!
 * \brief refill all the active bitmaps, updates position of current human
 *
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_3drefresh_all(hri_bitmapset * btset)
{
  int l,x,y,z;
  configPt humanConf;
  double hx,hy,hz;
  double Ccoord[6];

  if(btset == NULL || btset->bitmap==NULL)
    return FALSE;

  // first update human position
  humanConf = p3d_get_robot_config(btset->human[btset->actual_human]->HumanPt);

  hx = humanConf[6] - (btset->bitmap[0]->nx * btset->pace /2);
  hy = humanConf[7] - (btset->bitmap[0]->ny * btset->pace /2);

  p3d_destroy_config(btset->human[btset->actual_human]->HumanPt,humanConf);

  p3d_mat4ExtractPosReverseOrder(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos,
                                 Ccoord, Ccoord+1, Ccoord+2, Ccoord+3, Ccoord+4, Ccoord+5);

  hz = Ccoord[2] - (btset->bitmap[0]->nz * btset->pace /2);

  hri_bt_change_bitmap_position(btset,hx,hy,hz);

  // now update cells
  for (l = 0; l < btset->n; l++) {
    if (btset->bitmap[l]->active) {

      for (x = 0; x < btset->bitmap[l]->nx; x++) {
        for (y = 0; y < btset->bitmap[l]->ny; y++) {
          for (z = 0; z < btset->bitmap[l]->nz; z++) {
            btset->bitmap[l]->data[x][y][z].val
            = btset->bitmap[l]->calculate_cell_value(btset, x, y, z);
          }
        }
      }

    }
  }

  hri_exp_fill_obstacles(btset);

  return TRUE;
}


/****************************************************************/
/*!
 * \brief resets the given bitmap data
 *
 * \param bitmap  the bitmap
 *
 * \return
 */
/****************************************************************/
void hri_bt_reset_bitmap_data(hri_bitmap* bitmap)
{
  int x,y,z;

  if(bitmap == NULL)
    return;

  for(x=0; x<bitmap->nx; x++){
    for(y=0; y<bitmap->ny; y++){
      for(z=0; z<bitmap->nz; z++){
        bitmap->data[x][y][z].val = 0;
        bitmap->data[x][y][z].h = -1;
        bitmap->data[x][y][z].g = 0;
        bitmap->data[x][y][z].parent = NULL;
        bitmap->data[x][y][z].closed = FALSE;
        bitmap->data[x][y][z].open   = FALSE;
        bitmap->data[x][y][z].x = x;
        bitmap->data[x][y][z].y = y;
        bitmap->data[x][y][z].z = z;
        bitmap->data[x][y][z].locked = FALSE;
      }
    }
  }

}

/* REVISION */
/****************************************************************/
/*!
 * \brief to link the goal position values of the form
 *
 * \param bitmap  goal position
 * \param type    position type
 *
 * \return
 */
/****************************************************************/
void hri_bt_init_TARGET(int place, int type)
{
  if(place!=0 && type!=0){
    PLACEMENT = place;
    PLCMT_TYPE = type;
  }
}

/* REVISION */
/****************************************************************/
/*!
 * \brief Gives the configuration of the goal position
 *
 *
 * \return the configuration
 */
/****************************************************************/
configPt hri_bt_set_TARGET()
{
  configPt q,q_f;
  int cost=0, places=0, min=0, mincost=9999;
  /*  int MAXX = BTSET->human[0]->joints[1]->dof_data[0].vmax , MAXY = BTSET->human[0]->joints[1]->dof_data[1].vmax; */
  double orient =   BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_NECK_PAN]->dof_data->v +
  BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[5].v;

  if(PLACEMENT==0 || PLCMT_TYPE==0)
    return NULL;

  q = p3d_copy_config(BTSET->robot, BTSET->robot->ROBOT_POS);
  if(PLCMT_TYPE == BT_TRG_BODY){
    /*if(BTSET->human[0]->joints[1]->dof_data[0].v-1>0 && ){ */

    switch(PLACEMENT){
        /*     case 1: */
        /*       if(BTSET->human[0]->joints[1]->dof_data[0].v-1000>0){ */
        /* 	q[6]=BTSET->human[0]->joints[1]->dof_data[0].v-1000; */
        /* 	q[7] = BTSET->human[0]->joints[1]->dof_data[1].v; */
        /*       } */
        /*       break; */
        /*     case 2: */
        /*       if(BTSET->human[0]->joints[1]->dof_data[0].v-1000>0 && BTSET->human[0]->joints[1]->dof_data[1].v-1000>0){ */
        /* 	q[6] = BTSET->human[0]->joints[1]->dof_data[0].v-1000; */
        /* 	q[7] = BTSET->human[0]->joints[1]->dof_data[1].v+1000; */
        /*       } */
        /*       break;  */
      case 3:
        q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*sin(orient);
        q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*cos(orient);
        break;
      case 4:
        q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)-0.7*sin(orient);
        q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)+0.7*cos(orient);
        break;
      case 5:
        q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*cos(orient);
        q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*sin(orient);
        break;
      case 6:
        q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)+0.7*sin(orient);
        q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)-0.7*cos(orient);
        break;
      case 7:
        q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*sin(orient);
        q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*cos(orient);
        break;
        /*     case 8: */
        /*       if(BTSET->human[0]->joints[1]->dof_data[0].v-1000>0 && BTSET->human[0]->joints[1]->dof_data[1].v-1000>0){ */
        /* 	q[6] = BTSET->human[0]->joints[1]->dof_data[0].v-1000; */
        /* 	q[7] = BTSET->human[0]->joints[1]->dof_data[1].v-1000; */
        /*       } */
        /*       break;  */
    }
  }
  if(PLCMT_TYPE == BT_TRG_LOOK){
    q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*cos(orient);
    q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*sin(orient);
  }
  if(PLCMT_TYPE == BT_TRG_APPROACH){
    q_f = p3d_copy_config(BTSET->robot, q);
    while(places < 5){
      if(places == 0){
        q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*sin(orient);
        q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*cos(orient);
      }
      if(places == 1){
        q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)-0.7*sin(orient);
        q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)+0.7*cos(orient);
      }
      if(places == 2){
        q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*cos(orient);
        q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*sin(orient);
      }
      if(places == 3){
        q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)+0.7*sin(orient);
        q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)-0.7*cos(orient);
      }
      if(places == 4){
        q_f[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*sin(orient);
        q_f[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*cos(orient);
      }
      hri_bt_reset_path(BTSET);
      cost = hri_bt_start_search(q,q_f,BTSET,FALSE);
      if(cost!=-1 && cost<mincost){
        mincost = cost;
        min = places;
      }
      places++;
    }

    if(min == 0){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-1*sin(orient);
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*cos(orient);
    }
    if(min == 1){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)-0.7*sin(orient);
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)+0.7*cos(orient);
    }
    if(min == 2){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*cos(orient);
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+1*sin(orient);
    }
    if(min == 3){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+0.7*cos(orient)+0.7*sin(orient);
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v+0.7*sin(orient)-0.7*cos(orient);
    }
    if(min == 4){
      q[ROBOTq_X] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v+1*sin(orient);
      q[ROBOTq_Y] = BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-1*cos(orient);
    }

    q[ROBOTq_RZ] = atan2((BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[1].v-q[ROBOTq_Y]),
                         (BTSET->human[BTSET->actual_human]->HumanPt->joints[HUMANj_BODY]->dof_data[0].v-q[ROBOTq_X]));
    if(q[ROBOTq_RZ]<0)
      q[ROBOTq_RZ] = q[ROBOTq_RZ]+M_PI;
    else
      q[ROBOTq_RZ] = q[ROBOTq_RZ]-M_PI;
    /*q[12] = M_PI;
     q[18] = 0; Only for blue humanoid robot */

  }
  /*printf("Target is %d and orientation is base: %f top:%f\n",min,q[11]*180/M_PI,q[12]*180/M_PI);*/

  BTSET->BT_target[0] = q[ROBOTq_X];
  BTSET->BT_target[1] = q[ROBOTq_Y];
  BTSET->BT_target_available = TRUE;
  return q;

}





/**************************ASTAR********************************/
/*!
 * \brief A* search: close the cell by setting the closed flag to true
 *
 * \param bitmap       the bitmap
 * \param current_cell the cell to be closed
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_close_cell(hri_bitmap* bitmap, hri_bitmap_cell* current_cell)
{
  if(current_cell == NULL)
    return FALSE;
  if(! on_map(current_cell->x, current_cell->y, current_cell->z, bitmap))
    return FALSE;

  bitmap->data[current_cell->x][current_cell->y][current_cell->z].closed = TRUE;
  return TRUE;
}

/***************************ASTAR********************************/
/*!
 * \brief Starts A* search to find a path on bitmap
 *
 * \param bitmap the bitmap
 * \param start_cell start cell
 * \param final_cell goal cell
 *
 * \return -1 in case of a problem, cost of solution else
 */
/****************************************************************/
double hri_bt_astar_bh(hri_bitmapset * btset, hri_bitmap* bitmap)
{

  hri_bitmap_cell * current_cell;;
  int reached = FALSE;
  double finalcosts = -1;

  if(bitmap->type != BT_PATH) {
    // TK: previously type was set to BT_PATH in the end, but this messes up bitmapset definitions
    PrintError(("Trying to call A star on bitmap which is not of type BT_PATH"));
    return -1;
  }

  if(bitmap->search_start == NULL || bitmap->search_goal == NULL){
    PrintError(("hri_bt_astar_bh: start/final cell is NULL\n"));
    return -1;
  }
  if(bitmap->search_start == bitmap->search_goal) {
    return 0;
  }

  current_cell = bitmap->search_start;

  hri_bt_init_BinaryHeap(bitmap); /** ALLOC **/

  if(!hri_bt_close_cell(bitmap,current_cell)){
    PrintError(("cant close start cell!\n"));
    return -1;
  }
  hri_bt_A_neigh_costs(btset, bitmap, current_cell, bitmap->search_goal);

  while(!reached) {
    if( hri_bt_A_Heap_size()==0){
      PrintError(("A*:no path found!"));
      hri_bt_destroy_BinaryHeap();
      return -1;
    }
    current_cell = hri_bt_A_remove_OL();
    if(current_cell == bitmap->search_goal) {
      reached = TRUE;
      break;
    }
    hri_bt_close_cell(bitmap, current_cell);
    hri_bt_A_neigh_costs(btset, bitmap, current_cell, bitmap->search_goal);
  }
  bitmap->searched = TRUE;

  finalcosts = hri_bt_A_CalculateCellG(btset, bitmap->search_goal, bitmap->search_goal->parent, getCellDistance(bitmap->search_goal, bitmap->search_goal->parent));
  printf("\ncost: %f \n", finalcosts);

  hri_bt_destroy_BinaryHeap();

  return finalcosts;
}



/*********************ASTAR***************************************/
/*!
 * \brief A* search: calculate neighbours
 *
 * all neighbors not opened yet will be opened,
 * all opened neighbors will be updated if they are cheaper to reach by the center cell
 *
 * \param bitmap the bitmap
 * \param center_cell whose neighbours
 * \param final_cell  goal cell
 * \param reached TRUE if we reach to the goal
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_A_neigh_costs(hri_bitmapset* btset, hri_bitmap* bitmap, hri_bitmap_cell* center_cell, hri_bitmap_cell* final_cell)
{
  int i,j,k; // loop xyz coordinates
  int x, y, z; // center cell coordinates
  int refx1, refy1, refx2, refy2; // 2step reference cell xy coordinates
  hri_bitmap_cell *current_cell, *overstepped1, *overstepped2;
  double new_cell_g, overstep_val, distance;

  if(center_cell == NULL)
    return FALSE;
  else{
    x=center_cell->x; y=center_cell->y; z=center_cell->z;
  }

  if( ! on_map(center_cell->x, center_cell->y, center_cell->z, bitmap) ) {
    PrintError(("cant get cell\n"));
    return FALSE;
  }

  /* iterate over all direct non-closed neighbors in bitmap */
  for(i=-1; i<2; i++){ // -1 to 1
    for(j=-1; j<2; j++){ // -1 to 1
      for(k=-1; k<2; k++){// -1 to 1
        if(i==0 && j==0 && k==0) continue; // center cell
        if (! on_map(x+i, y+j, z+k, bitmap)) {
          continue;
        }

        if( !(current_cell = hri_bt_get_cell(bitmap,x+i,y+j,z+k)) ){
          PrintError(("Can't get cell\n"));
          return FALSE;
        }

        if(btset->bitmap[BT_OBSTACLES]->data[x+i][y+j][z+k].val == BT_OBST_SURE_COLLISION) {
//          printf("collsion in (%d, %d)\n", x+i, y+j);
          continue; /* Is the cell in obstacle? */
        }

        /* closed cells already have a minimum path to start, and all neighbors opened */
        if(current_cell->closed) continue;

        // prevent sharp turns
        if (center_cell->parent != NULL
            && get3CellAngle(center_cell->parent, center_cell, current_cell) >  btset->parameters->angle2d_minimum) {
          continue;
        }

        /* open cells might have less g cost for going through current node */
        int manhattan_distance = ABS(i) + ABS(j) + ABS(k);
        if (manhattan_distance == 1) {
          distance= 1; // normal grid step
        } else if(manhattan_distance == 2) {
          distance = M_SQRT2; // 2d diagonal step
        } else if(manhattan_distance == 3) {
          distance = M_SQRT3; // 3d diagonal step
        }
        if (current_cell->open) {
          new_cell_g = hri_bt_A_CalculateCellG(btset, current_cell, center_cell, distance);

          if(current_cell->g > new_cell_g){
            current_cell->g =  new_cell_g;
            current_cell->parent = center_cell;
            hri_bt_A_update_cell_OL(current_cell);
          } else {
            continue;
          }

        } else { // cell was neither open nor closed
          if (CalculateCellValue(btset, bitmap, current_cell, center_cell) == FALSE) {
            continue;// leave untouched
          }
          current_cell->h = hri_bt_dist_heuristic(btset, bitmap, current_cell->x, current_cell->y, current_cell->z);
          // TK:dead code never used because of if before
          //          if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].val == BT_OBST_POTENTIAL_COLLISION
          //              && btset->manip == BT_MANIP_NAVIGATION) {
          //            fromcellno = get_direction(current_cell, center_cell);
          //            if(btset->bitmap[BT_OBSTACLES]->data[current_cell->x][current_cell->y][current_cell->z].obstacle[fromcellno]==TRUE) // it was !=, PRAGUE
          //              continue;
          //          }

          current_cell->g = hri_bt_A_CalculateCellG(btset, current_cell, center_cell, distance);
          current_cell->parent = center_cell;

          /*  printf("It is g=%f now\n",current_cell->g); */
          hri_bt_A_insert_OL(current_cell);
          current_cell->open = TRUE;
        }
      }
    }
  } // end for immediate neighbors

  /**
   * as an optimisation in 2D to basic A* over the grid, we also search the 2 step
   * diagonals that would be reached by going 1 step horizontal or vertical, and one step diagonal.
   *
   * We use the value and open properties use for single grid steps calculated earlier.
   * To be fair with costs, the cost in a cell are its own costs plus the average of the two "jumped" cells
   */
  if (bitmap->nz == 1) {
    k = 0;
    for(i=-2; i<3; i++){ // -2 to 2
      for(j=-2; j<3; j++){ // -2 to 2
                           //        for(k=-2; k<3; j++){ // -2 to 2
        if (abs(i)!=2 && abs(j)!=2 && abs(k)!=2) continue; // not a 2 step border cell
        if (i==0 || j==0 || k==0) {
          // in plane on center cell, target cells are 3 manhattan steps away
          if (abs(i) + abs(j) + abs(k) != 3) {
            continue;
          }
        } else { // 3D case
          if (abs(i) + abs(j) + abs(k) != 4) {
            continue;
          }
        }
        if (! on_map(x+i, y+j, z+k, bitmap)) {
          continue;
        }


        if( !(current_cell = hri_bt_get_cell(bitmap,x+i,y+j,z+k)) ){
          PrintError(("Can't get cell\n"));
          return FALSE;
        }

        if(btset->bitmap[BT_OBSTACLES]->data[x+i][y+j][z+k].val == BT_OBST_SURE_COLLISION) continue; /* Is the cell in obstacle? */

        /* closed cells already have a minimum path to start, and all neighbors opened */
        if(current_cell->closed) continue;

        // prevent sharp turns
        if (center_cell->parent != NULL
            && get3CellAngle(center_cell->parent, center_cell, current_cell) >  btset->parameters->angle2d_minimum) {
          continue;
        }

        // 2D code, 2 reference points stepped over
        //ref1 is horizontal / vertical stepped over cell, ref2 diagonally stepped over cell
        if (i == -2) {
          refx1 = -1;
        } else if (i == 2) {
          refx1 = 1;
        } else {
          refx1 = 0;
        }
        if (j == -2) {
          refy1 = -1;
        } else if (j == 2) {
          refy1 = 1;
        } else {
          refy1 = 0;
        }
        if (i < 0) {
          refx2 = -1;
        } else  {
          refx2 = 1;
        }
        if (j < 0) {
          refy2 = -1;
        } else  {
          refy2 = 1;
        }
        overstepped1 = hri_bt_get_cell(bitmap, x+refx1, y+refy1, z+k);
        overstepped2 = hri_bt_get_cell(bitmap, x+refx2, y+refy2, z+k);

        // only continue if both overstepped cells have been opened, and add the max val to g later
        if (overstepped1->open && overstepped2->open) {
          overstep_val =  (overstepped1->val + overstepped2->val) / 2 ;
        } else {
          /* TODO: For more completeness, check the 2 step localpath for collisions,
           * and calculate equivalent cost for a double step */
          continue;
        }

        /* open cells might have less g cost for going through current node */
        if (current_cell->open) {
          new_cell_g = overstep_val + hri_bt_A_CalculateCellG(btset, current_cell, center_cell, M_SQRT5);

          if(current_cell->g > new_cell_g){
            current_cell->g =  new_cell_g;
            current_cell->parent = center_cell;
            hri_bt_A_update_cell_OL(current_cell);
          } else {
            continue;
          }

        } else { // cell was neither open nor closed
          if (CalculateCellValue(btset, bitmap, current_cell, center_cell) == FALSE)
            continue;// leave untouched
          current_cell->h = hri_bt_dist_heuristic(btset, bitmap,current_cell->x,current_cell->y,current_cell->z);

          current_cell->g = overstep_val + hri_bt_A_CalculateCellG(btset, current_cell, center_cell, M_SQRT5);
          current_cell->parent = center_cell;

          /*  printf("It is g=%f now\n",current_cell->g); */
          hri_bt_A_insert_OL(current_cell);
          current_cell->open = TRUE;
        }
        //        }
      }
    } // end for distant diagonal neighbors
  } // end if 2d then smooth

  return TRUE;
}



/****************************************************************/
/*!
 * \brief reconstructs the distance bitmap
 *
 * \param radius radius
 * \param heigth height
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_distance(hri_bitmapset * btset, double height, double radius)
{
  hri_bitmap* bitmap;
  int i,j,k;

  if(btset==NULL)
    return FALSE;
  else
    bitmap = btset->bitmap[BT_DISTANCE];

  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dheight = height;
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].dradius = radius;
  if(!bitmap->active){
    return TRUE;
  }
  for(i=0;i<bitmap->nx;i++){
    for(j=0;j<bitmap->ny;j++){
      for(k=0;k<bitmap->nz;k++)
        btset->bitmap[BT_DISTANCE]->data[i][j][k].val = btset->bitmap[BT_DISTANCE]->calculate_cell_value(btset,i,j,k);
      //hri_bt_calc_dist_VALUE(i,j,k,0,NULL);
    }
  }

  return TRUE;
}

/****************************************************************/
/*!
 * \brief reconstructs the visibility bitmap
 *
 * \param height height
 * \param p2     back radius
 * \param p3     sides radius
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_visibility(hri_bitmapset * btset,double height, double p2, double p3)
{
  hri_bitmap* bitmap;
  int i,j,k;

  if(btset==NULL)
    return FALSE;
  else
    bitmap = btset->bitmap[BT_VISIBILITY];

  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vheight = height;
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vback = p2;
  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].vradius = p3;

  if(!bitmap->active){
    return TRUE;
  }

  //  hri_bt_reset_bitmap_data(btset->bitmap[BT_VISIBILITY]);

  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++)
        btset->bitmap[BT_VISIBILITY]->data[i][j][k].val = hri_bt_calc_vis_value(btset,i,j,k);
    }
  }

  return TRUE;
}

/****************************************************************/
/*!
 * \brief reconstructs the hidden zones bitmap
 *
 * \param radius vision radius
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_hidzones(hri_bitmapset * btset,double radius)
{
  hri_bitmap* bitmap;
  int i,j,k;
  double val;

  if (btset == NULL) {
    return FALSE;
  } else {
    bitmap = btset->bitmap[BT_HIDZONES];
  }
  /*  bitmap->p4 = radius; */

  btset->human[btset->actual_human]->state[btset->human[btset->actual_human]->actual_state].hradius = radius;

  if(!bitmap->active){
    return TRUE;
  }

  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++){
        val = hri_bt_calc_hz_value(btset,i,j,k);
        // don't allow negative values
        if(val > 0) {
          btset->bitmap[BT_HIDZONES]->data[i][j][k].val = val;
        } else {
          btset->bitmap[BT_HIDZONES]->data[i][j][k].val = 0;
        }
      }
    }
  }

  return TRUE;
}

/****************************************************************/
/*!
 * \brief reconstructs the distance bitmap
 * assumes that BT_OBSTACLE bitmap has been updated first.
 * \param radius radius
 * \param heigth height
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_update_combined(hri_bitmapset * btset)
{
  hri_bitmap* bitmap;
  int i,j,k;

  if(btset==NULL)
    return FALSE;
  else
    bitmap = btset->bitmap[BT_COMBINED];

  if(!bitmap->active){
    return TRUE;
  }

  for(i=0;i<bitmap->nx;i++){
    for(j=0;j<bitmap->ny;j++){
      for(k=0;k<bitmap->nz;k++){
        btset->bitmap[BT_COMBINED]->data[i][j][k].val = hri_bt_calc_combined_value(btset,i,j,k);
      }
    }
  }
  return TRUE;
}

int hri_bt_sit_stand(p3d_rob* human)
{
  return 0;
}

/**
 * writes the path data stored in btset->bitmap[BT_PATH] to
 * a more convenient format in btset->path. start and end waypoints are
 * added with the robots current and final position, which can deviate
 * from the grid positions and theta.
 */
int hri_bt_write_TRAJ(hri_bitmapset * btset, p3d_jnt * joint)
{
  hri_bitmap * bitmap;
  int i=0, j=0;
  int length =0;
  int numOfNodes = 0;

  if( (btset == NULL) || (btset->bitmap[BT_PATH] == NULL) || (!btset->pathexist) ){
    PrintError(("Cant write path structure\n"));
    return FALSE;
  }
  bitmap = btset->bitmap[BT_PATH];

  printf("*****Creating the path structure... ");

  btset->path = MY_ALLOC(hri_bt_path,1);    /* ALLOC */

  bitmap->current_search_node = bitmap->search_goal;
  while(bitmap->current_search_node != NULL){
    numOfNodes++;
    bitmap->current_search_node = bitmap->current_search_node->parent;
  }
  bitmap->current_search_node = bitmap->search_goal;

  int goalValid = TRUE; // goal config in obstacle (might be allowed)
  //TODO: int firstValid = TRUE; // first cell point useful?
  //TODO: int lastValid = TRUE; // last cellpont useful?


  configPt qc;
  qc = p3d_get_robot_config(btset->robot); /** ALLOC */
  qc[6]  = btset->robot->ROBOT_GOTO[ROBOTq_X];
  qc[7]  = btset->robot->ROBOT_GOTO[ROBOTq_Y];
  qc[11] = btset->robot->ROBOT_GOTO[ROBOTq_RZ];
  p3d_set_and_update_this_robot_conf(btset->robot, qc);
  if(p3d_col_test_robot_statics(btset->robot, FALSE)){
    goalValid = FALSE;
    // for visuals, set robot on last valid point
    qc = p3d_get_robot_config(btset->robot); /** ALLOC */
      qc[6]  = (bitmap->current_search_node->x * btset->pace) + btset->realx;
      qc[7]  = (bitmap->current_search_node->y * btset->pace) + btset->realy;
      qc[11] = btset->robot->ROBOT_GOTO[ROBOTq_RZ];
      p3d_set_and_update_this_robot_conf(btset->robot, qc);
  }
  p3d_destroy_config(btset->robot, qc); /** FREE */

  if(goalValid) {
    // goal config is valid
    length = numOfNodes+2; /* ADDING 2 FOR START AND GOAL CONFIGS */
    printf("... omitting goal config... ");
  } else {
    // goal config is not valid, approaching close position instead
    length = numOfNodes+1;
  }

  btset->path->xcoord = MY_ALLOC(double,length); /* ALLOC */
  btset->path->ycoord = MY_ALLOC(double,length); /* ALLOC */
  btset->path->zcoord = MY_ALLOC(double,length); /* ALLOC */
  btset->path->theta  = MY_ALLOC(double,length); /* ALLOC */
  btset->path->length = length;


  /**
    * grid start and end nodes were mere approximations
    * of the real position, we can get rid of them (though formally
    * we should check for collisions doing so)
    */

  // setting path start position to robot current position
  btset->path->xcoord[0] = btset->robot->ROBOT_POS[ROBOTq_X];
  btset->path->ycoord[0] = btset->robot->ROBOT_POS[ROBOTq_Y];
  btset->path->zcoord[0] = btset->robot->ROBOT_POS[ROBOTq_Z];
  btset->path->theta[0]  = btset->robot->ROBOT_POS[ROBOTq_RZ];

  if(goalValid) {
    //  setting path end position to robot goto position
    btset->path->xcoord[length-1] = btset->robot->ROBOT_GOTO[ROBOTq_X];
    btset->path->ycoord[length-1] = btset->robot->ROBOT_GOTO[ROBOTq_Y];
    btset->path->zcoord[length-1] = btset->robot->ROBOT_GOTO[ROBOTq_Z];
    btset->path->theta[length-1]  = btset->robot->ROBOT_GOTO[ROBOTq_RZ];
  }

  // TODO: check whether first and last grid cell make sense, to avoid hard edge, e.g. (0.05, 0.05), (0, 0), (0.1, 0.1)

  /*  We assign each array element of the path, except the first and last node, the xyz of from the btset path. */
  bitmap->current_search_node = bitmap->search_goal;
  for (j = numOfNodes; j > 0; j--) {
    btset->path->xcoord[j] = (bitmap->current_search_node->x * btset->pace) + btset->realx;
    btset->path->ycoord[j] = (bitmap->current_search_node->y * btset->pace) + btset->realy;
    btset->path->zcoord[j] = (bitmap->current_search_node->z * btset->pace) + btset->realz;
    // printf("Added node (%f,%f)\n", btset->path->xcoord[j], btset->path->ycoord[j]);
    bitmap->current_search_node = bitmap->current_search_node->parent;
  }

  /* For the angles, we calculate the angles between xy positions, averaging over the last and next angle
   * array elements [i+1] and [i-1] exist for all i since we filled the array before and we do not change the thetas of [0] and [length-1]*/
  bitmap->current_search_node = bitmap->search_goal;
  for (i = numOfNodes; i > 0; i--) {

    if (btset->path->xcoord[i+1] == btset->path->xcoord[i-1] && btset->path->ycoord[i+1] == btset->path->ycoord[i-1]) {
      // path is stupid, should never happen? -> point towards next waypoint
      btset->path->theta[i] =
      atan2(btset->path->ycoord[i+1] - btset->path->ycoord[i], btset->path->xcoord[i+1] - btset->path->xcoord[i]);
    } else {
      /* the angle between the last and the next point is geometrically the equivalent to the averange between the
       * last angle and the next angle (except for the stupid case in the if or if path nodes are on top of each other)
       */
      // point halfway angle between last point and next point.
      btset->path->theta[i] =
      atan2(btset->path->ycoord[i+1] - btset->path->ycoord[i-1], btset->path->xcoord[i+1] - btset->path->xcoord[i-1]);
    }
    //printf("Calculating theta for (%f,%f) to (%f,%f) to (%f,%f) = %f\n", btset->path->xcoord[i-1],btset->path->ycoord[i-1],btset->path->xcoord[i],btset->path->ycoord[i],btset->path->xcoord[i+1],btset->path->ycoord[i+1], btset->path->theta[i]);
  }

  printf("Path structure created*****\n");
  return TRUE;
}


int hri_bt_verify_path(hri_bitmapset * btset)
{
  int i,ntest;
  configPt qc = NULL, qg = NULL;
  int x,y,z;
  p3d_localpath * localpath;

  if(btset == NULL){
    printf("BTSET is null, cannot verify the path\n");
    return FALSE;
  }

  if(btset->path == NULL){
    printf("BTSET->path is null, cannot verify the path\n");
    return FALSE;
  }

  qc = p3d_get_robot_config(btset->robot); /* ALLOC */
  qg = p3d_get_robot_config(btset->robot); /* ALLOC */


  for(i=0; i<btset->path->length; i++){

    /** ROTATION **/
    /* coming configuration */

    qc[6]  = btset->path->xcoord[i];
    qc[7]  = btset->path->ycoord[i];
    if(i==0)
      qc[11] =  btset->path->theta[i];
    else
      qc[11] = atan2(btset->path->ycoord[i]-btset->path->ycoord[i-1],btset->path->xcoord[i]-btset->path->xcoord[i-1]);

    /* going configuration */

    qg[6]  = btset->path->xcoord[i];
    qg[7]  = btset->path->ycoord[i];
    if(i==btset->path->length-1)
      qg[11] =  btset->path->theta[i];
    else
      qg[11] = atan2(btset->path->ycoord[i+1]-btset->path->ycoord[i],btset->path->xcoord[i+1]-btset->path->xcoord[i]);

    /* test */
    if(!p3d_equal_config(btset->robot,qc, qg)){
      if( (localpath = p3d_local_planner(btset->robot, qc, qg)) ){
        if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
          destroy_list_localpath(btset->robot, localpath);
          x = (int)((btset->path->xcoord[i]-btset->realx)/btset->pace);
          y = (int)((btset->path->ycoord[i]-btset->realy)/btset->pace);
          z = (int)((btset->path->zcoord[i]-btset->realz)/btset->pace);
          btset->bitmap[BT_OBSTACLES]->data[x][y][z].val = -1;
          p3d_destroy_config(btset->robot, qc);
          p3d_destroy_config(btset->robot, qg);
          printf("Path collision at x:%d y:%d z:%d\n",x,y,z);
          return FALSE;
        }
      }
      destroy_list_localpath(btset->robot, localpath);
    }
    if(i!=btset->path->length-1){
      /** TRANSLATION **/
      /* coming configuration */

      qc[11] = qg[11];

      /* going configuration */

      qg[6]  = btset->path->xcoord[i+1];
      qg[7]  = btset->path->ycoord[i+1];

      /* test */

      if( (localpath = p3d_local_planner(btset->robot, qc, qg)) ){
        if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
          destroy_list_localpath(btset->robot, localpath);
          x = (int)((btset->path->xcoord[i]-btset->realx)/btset->pace);
          y = (int)((btset->path->ycoord[i]-btset->realy)/btset->pace);
          z = (int)((btset->path->zcoord[i]-btset->realz)/btset->pace);
          btset->bitmap[BT_OBSTACLES]->data[x][y][z].val = -1;
          p3d_destroy_config(btset->robot, qc);
          p3d_destroy_config(btset->robot, qg);
          printf("Path collision at x:%d y:%d z:%d\n",x,y,z);
          return FALSE;
        }
      }
      destroy_list_localpath(btset->robot, localpath);
    }


    //		g3d_draw_allwin_active();
  }


  p3d_destroy_config(btset->robot, qc);
  p3d_destroy_config(btset->robot, qg);


  printf("Path verified\n");
  return TRUE;

}



int hri_bt_verify_pathold(hri_bitmapset * btset)
{
  int ntest;
  p3d_localpath * localpath;
  configPt qc, qn;
  hri_bitmap * bitmap;

  if(btset == NULL){
    printf("BTSET is null, cannot verify the path\n");
    return FALSE;
  }

  if(btset->path == NULL){
    printf("BTSET->path is null, cannot verify the path\n");
    return FALSE;
  }

  bitmap = btset->bitmap[BT_PATH];

  qc = p3d_get_robot_config(btset->robot); /* ALLOC */
  qn = p3d_get_robot_config(btset->robot); /* ALLOC */

  bitmap->current_search_node = bitmap->search_goal->parent;

  qn[6]  = (bitmap->search_goal->x*btset->pace)+btset->realx;
  qn[7]  = (bitmap->search_goal->y*btset->pace)+btset->realy;
  qn[11] = atan2(bitmap->search_goal->y-bitmap->current_search_node->y,bitmap->search_goal->x-bitmap->current_search_node->x);

  while(bitmap->current_search_node != NULL){

    /* Translation */
    qc[6]  = (bitmap->current_search_node->x*btset->pace)+btset->realx;
    qc[7]  = (bitmap->current_search_node->y*btset->pace)+btset->realy;
    qc[11] = qn[11];

    if( (localpath = p3d_local_planner(btset->robot, qc, qn)) ){
      if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
        destroy_list_localpath(btset->robot, localpath);
        btset->bitmap[BT_OBSTACLES]->data[bitmap->current_search_node->x][bitmap->current_search_node->y][bitmap->current_search_node->z].val = -1;
        return FALSE;
      }
    }
    destroy_list_localpath(btset->robot, localpath);

    /* Rotation */

    qn[6] = qc[6];
    qn[7] = qc[7];
    qn[11] = qc[11];

    qc[11] = atan2(bitmap->current_search_node->y-bitmap->current_search_node->parent->y,bitmap->current_search_node->x-bitmap->current_search_node->parent->x);

    if( (localpath = p3d_local_planner(btset->robot, qc, qn)) ){
      if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){
        destroy_list_localpath(btset->robot, localpath);
        return FALSE;
      }
    }
    destroy_list_localpath(btset->robot, localpath);


    qc[6] = (bitmap->current_search_node->x*btset->pace)+btset->realx;
    qc[7] = (bitmap->current_search_node->y*btset->pace)+btset->realy;
    qc[11] = atan2(btset->robot->ROBOT_GOTO[7]-bitmap->search_goal->y,btset->robot->ROBOT_GOTO[6]-bitmap->search_goal->x);

    qn[6] = (bitmap->current_search_node->x*btset->pace)+btset->realx;
    qn[7] = (bitmap->current_search_node->y*btset->pace)+btset->realy;
    qn[11] = atan2(btset->robot->ROBOT_GOTO[7]-bitmap->search_goal->y,btset->robot->ROBOT_GOTO[6]-bitmap->search_goal->x);


    //  if( (localpath = p3d_local_planner(btset->visball, qtarget, qhuman)) ){
    // if(p3d_unvalid_localpath_test(btset->robot, localpath, &ntest)){

  }

  return TRUE;

}




int hri_bt_destroy_path(hri_bitmapset* bitmapset)
{
  if(bitmapset->path == NULL){
    PrintWarning(("Trying to destroy a non existing path"));
    return FALSE;
  }

  MY_FREE(bitmapset->path->xcoord,double,bitmapset->path->length);
  MY_FREE(bitmapset->path->ycoord,double,bitmapset->path->length);
  MY_FREE(bitmapset->path->zcoord,double,bitmapset->path->length);
  MY_FREE(bitmapset->path->theta,double,bitmapset->path->length);

  MY_FREE(bitmapset->path,hri_bt_path,1);

  return TRUE;
}

/* REVISION */
void hri_bt_print_PATH(hri_bitmapset* bitmapset)
{
  int i=0;

  if(bitmapset->path == NULL)
    return;

  printf("\nPath length = %d\n",bitmapset->path->length);

  for(i=0; i<bitmapset->path->length; i++){
    printf("no %d, %f, %f, %f, %f\n",i,bitmapset->path->xcoord[i],bitmapset->path->ycoord[i],bitmapset->path->zcoord[i],bitmapset->path->theta[i]);
  }

}


int hri_bt_opt_path(hri_bitmap* bitmap)
{
  hri_bitmap_cell * current, * prev;
  double p_ratio, c_ratio;

  if(bitmap->search_goal != NULL &&
     bitmap->search_goal->parent !=NULL &&
     bitmap->search_goal->parent->parent != NULL &&
     bitmap->search_goal->parent->parent->parent != NULL){

    prev = bitmap->search_goal;
    current = prev->parent->parent;


    while(current->parent != NULL){

      if(current->parent->y - current->y)
        c_ratio = (current->parent->x - current->x)/(current->parent->y - current->y);
      else
        c_ratio = 9999;

      if(prev->parent->y - prev->y)
        p_ratio = (prev->parent->x - prev->x)/(prev->parent->y - prev->y);
      else
        p_ratio = 9999;

      if(c_ratio == p_ratio)
        prev->parent = current->parent;
      else{
        prev = current;
        p_ratio = c_ratio;
      }

      if(prev->parent==NULL || prev->parent->parent==NULL ||
         prev->parent->parent->parent == NULL)
        break;
      else
        current = prev->parent->parent;
    }

  }


  prev = bitmap->search_goal;
  current = prev->parent;

  if(current->y - prev->y)
    p_ratio = (current->x - prev->x)/(current->y - prev->y);
  else
    p_ratio = 9999;

  while(current->parent != NULL){
    if(current->parent->y - current->y)
      c_ratio = (current->parent->x - current->x)/(current->parent->y - current->y);
    else
      c_ratio = 9999;
    if(c_ratio == p_ratio)
      prev->parent = current->parent;
    else{
      prev = current;
      p_ratio = c_ratio;
    }
    current = current->parent;
  }

  return TRUE;

}

int hri_bt_write_path2file()
{
  FILE* f = NULL;
  int i;

  if( hri_bt_write_TRAJ(BTSET,BTSET->robot->joints[1]) == FALSE ){
    PrintError(("Cant write path structure\n"));
    return FALSE;
  }

  f = fopen("path1.dat","w");

  for(i=0; i<BTSET->path->length; i++){
    fprintf(f,"%f %f\n",BTSET->path->xcoord[i], BTSET->path->ycoord[i]);
  }

  fclose(f);

  return TRUE;

}

int hri_bt_calculate_bitmap_path(hri_bitmapset * btset, p3d_rob *robotPt, configPt qs, configPt qg, int manip)
{
  p3d_graph *G;
  p3d_node  *Ns=NULL,*Ng=NULL;
  p3d_list_node *graph_node;
  configPt  q_s=NULL, q_g=NULL;
  double searchgoal[3], searchstart[3];

  if(qs == NULL){
    PrintInfo(("hri_hri_planner : ERROR : no initial configuration\n"));
    return(FALSE);
  }
  if(qg == NULL){
    PrintInfo(("hri_hri_planner : ERROR : no final configuration\n"));
    return(FALSE);
  }
  if(p3d_equal_config(robotPt, qs, qg)) {
    PrintInfo(("hri_hri_planner : ERROR : qs = qg\n"));
    return(FALSE);
  }

  searchstart[0] = qs[ROBOTq_X]; searchstart[1] = qs[ROBOTq_Y]; searchstart[2] = qs[ROBOTq_Z];
  searchgoal[0]  = qg[ROBOTq_X];  searchgoal[1] = qg[ROBOTq_Y];  searchgoal[2] = qg[ROBOTq_Z];

  //  if(!btset->pathexist){
  //	while(!pathisOK){
  ChronoOn();
  if(hri_bt_start_search(searchstart, searchgoal, btset, manip) < 0 ){ /* here we find the path */
    PrintInfo(("hri_planner : ERROR : A*: no path found\n"));
    return(FALSE);
  }

  ChronoPrint("A* STAR - SEARCH TIME");
  ChronoOff();

  //		hri_bt_write_TRAJ(btset, btset->robot->joints[1]);

  //			pathisOK = hri_bt_verify_path(btset);

  //		}
  //  }

  if(!BTGRAPH)     G = hri_bt_create_graph(robotPt);
  else             G = BTGRAPH;

  robotPt->GRAPH = G;
  graph_node = G->nodes;

  if(Ns == NULL) {
    q_s = p3d_copy_config(robotPt, qs);
    Ns = p3d_APInode_make(G,q_s);
    p3d_insert_node(G,Ns);
    p3d_create_compco(G,Ns);
    Ns->type = ISOLATED;
  }

  if(Ng == NULL) {
    q_g = p3d_copy_config(robotPt, qg);
    Ng = p3d_APInode_make(G,q_g);
    p3d_insert_node(G,Ng);
    p3d_create_compco(G,Ng);
    Ng->type = ISOLATED;
  }

  /* Initialize some data in the graph for the A* graph search */
  G->search_start = Ns;
  G->search_goal = Ng;
  G->search_done = FALSE;

  btset->changed = FALSE;

  //hri_bt_write_path2file();

  //hri_bt_opt_path(BTSET->bitmap[BT_PATH]);
  //hri_bt_write_path2file();
  hri_bt_bitmap_to_GRAPH(btset,G,btset->bitmap[BT_PATH]);

  /* hri_print_info_graph(G);*/
  MY_ALLOC_INFO("After hri_hri_planner");

  return(TRUE);
}


/*********************************************************/
/* Cree une graphe pour le robot dans le contexte bitmap */
/* courant                                               */
/* In :                                                  */
/* Out : le graphe                                       */
/*********************************************************/
p3d_graph * hri_bt_create_graph(p3d_rob* Robot)
{
  p3d_graph * Graph;

  Graph      = p3d_allocinit_graph();
  Graph->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
  Graph->rob = Robot;

  if (Robot != NULL) {
    if (Robot->GRAPH != NULL) {
      p3d_del_graph(Robot->GRAPH);
    }
    Robot->GRAPH = Graph;
  }

  return Graph;
}


void g3d_hri_bt_draw_active_3dbitmaps(hri_bitmapset* bitmapset)
{
  int i;

  if(bitmapset==NULL)
    return ;

  for(i=0; i<bitmapset->n; i++)
    if(bitmapset->bitmap[i]!=NULL && bitmapset->bitmap[i]->active)
      hri_bt_show_3dbitmap(bitmapset,bitmapset->bitmap[i]);

}

void  hri_bt_show_3dbitmap(hri_bitmapset* btset,hri_bitmap* bitmap)
{
  int i,j,k;
  double colorvector[4];

  colorvector[0] = 1;       //red
  colorvector[1] = 0;       //green
  colorvector[2] = 0;       //blue
  colorvector[3] = 0.75;    //transparency

  if(bitmap->type == BT_3D_PATH){
    hri_bt_show_path(btset,bitmap);
    return;
  }

  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++){

        colorvector[1] = bitmap->data[i][j][k].val;

        switch(bitmap->type){
          case BT_3D_DISTANCE:

            /* g3d_drawSphere(i*btset->pace+btset->realx,j*btset->pace+btset->realy, */
            /* 			 k*btset->pace+btset->realz,0.01, Any,colorvector); */
            //g3d_drawDisc(i*bitmap->pace+bitmap->realx,j*bitmap->pace+bitmap->realy,
            //	       k*bitmap->pace+bitmap->realz,bitmap->pace/2, Red,NULL);

            //g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx,
            //		  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy,
            //		  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz,
            //		  Any,colorvector);
            g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                            i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                            Any,colorvector);
            g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
                            i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
                            Any,colorvector);
            g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
                            i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
                            Any,colorvector);
            break;

          case BT_3D_VISIBILITY:
            /* g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
            /* 	  		  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
            /* 	  		  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
            /* 	  		  Any,colorvector); */
            g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                            i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                            Any,colorvector);
            g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
                            i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
                            Any,colorvector);
            g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
                            i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
                            Any,colorvector);


            break;

            /* 	case BT_3D_HCOMFORT: */
            /* 	  g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
            /* 			  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
            /* 			  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
            /* 			  Any,colorvector); */
            /* 	  break;  */

            /* 	case BT_3D_RREACH: */
            /* 	  g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
            /* 			  j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
            /* 			  k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
            /* 			  Any,colorvector); */
            /* 	  break;  */

            /* 	case BT_3D_PATH: */
            /* 	  if(bitmap->data[i][j][k].open) */
            /* 	    g3d_draw_a_box2(i*btset->pace+btset->realx,(i+1)*btset->pace+btset->realx, */
            /* 			    j*btset->pace+btset->realy,(j+1)*btset->pace+btset->realy, */
            /* 			    k*btset->pace+btset->realz,(k+1)*btset->pace+btset->realz, */
            /* 			    Any,colorvector); */
            /* 	  break;  */

          case BT_3D_OBSTACLES:
            if(bitmap->data[i][j][k].val < -1){
              g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                              i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                              Black,NULL);
              g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
                              i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
                              Black,NULL);
              g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
                              i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
                              Black,NULL);
            }
            else{
              if(bitmap->data[i][j][k].val > 0){
                g3d_drawOneLine(i*btset->pace+btset->realx-0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                                i*btset->pace+btset->realx+0.01,j*btset->pace+btset->realy,k*btset->pace+btset->realz,
                                Red,NULL);
                g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy-0.01,k*btset->pace+btset->realz,
                                i*btset->pace+btset->realx,j*btset->pace+btset->realy+0.01,k*btset->pace+btset->realz,
                                Red,NULL);
                g3d_drawOneLine(i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz-0.01,
                                i*btset->pace+btset->realx,j*btset->pace+btset->realy,k*btset->pace+btset->realz+0.01,
                                Red,NULL);
              }
            }
            break;

        }
      }
    }
  }
}

double hri_bt_min_cell_limited(hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z, double xlimit, double ylimit, double zlimit )
{
  int i,j,k;
  double mincost=100;
  double cost;
  int xb, xe, yb, ye, zb, ze;

  xb = (int)( (bitmap->nx - xlimit/btset->pace)/2 );
  xe = (int)( (bitmap->nx + xlimit/btset->pace)/2 );
  yb = (int)( (bitmap->ny - ylimit/btset->pace)/2 );
  ye = (int)( (bitmap->ny + ylimit/btset->pace)/2 );
  zb = (int)( (bitmap->nz - zlimit/btset->pace)/2 );
  ze = (int)( (bitmap->nz + zlimit/btset->pace)/2 );

  for(i=xb; i<xe; i+=2){
    for(j=yb; j<ye; j+=2){
      for(k=zb; k<ze; k+=2){
        if(btset->bitmap[BT_3D_OBSTACLES]->data[i][j][k].val == -1)
          continue;
        cost = bitmap->calculate_cell_value(btset,i,j,k);
        if(cost < 0)
          continue;
        if(mincost > cost){
          mincost =  cost;
          *x=i;  *y=j;  *z=k;
        }
      }
    }
  }

  return mincost;
}


/**
 * searches the cell with the smallest cost >=0.
 *
 * Returns the costs, and updates x y z.
 */
double hri_bt_min_cell(hri_bitmapset * btset,hri_bitmap * bitmap, int *x, int *y, int *z)
{
  int i,j,k;
  double mincost=100;
  double cost;

  // use += 2 o reduce search space
  for(i=0; i<bitmap->nx; i+=2){
    for(j=0; j<bitmap->ny; j+=2){
      for(k=0; k<bitmap->nz; k+=2){
        if(btset->bitmap[BT_3D_OBSTACLES]->data[i][j][k].val <= -1)
          continue;
        cost = bitmap->calculate_cell_value(btset,i,j,k);
        if(cost < 0)
          continue;
        if(mincost > cost){
          mincost =  cost;
          *x=i;  *y=j;  *z=k;
        }
      }
    }
  }

  return mincost;
}

/**
 * searches the cell with the maximal cost >0.
 *
 * Returns the costs, and updates x y z.
 */
double hri_bt_max_cell(hri_bitmapset * btset,hri_bitmap * bitmap, int *x, int *y, int *z)
{
  int i,j,k;
  double maxcost=0;
  double cost;

  // use += 2 o reduce search space
  for(i=0; i<bitmap->nx; i+=2){
    for(j=0; j<bitmap->ny; j+=2){
      for(k=0; k<bitmap->nz; k+=2){
        cost = bitmap->calculate_cell_value(btset,i,j,k);
        if(cost <= 0)
          continue;
        if(maxcost < cost){
          maxcost =  cost;
          *x=i;  *y=j;  *z=k;
        }
      }
    }
  }
  return maxcost;
}

int hri_bt_calculate_bitmap_pathwGIK(hri_bitmapset * btset, p3d_vector3 start, p3d_vector3 goal, int manip)
{
  p3d_graph *G;
  p3d_list_node *graph_node;
  configPt q_s=NULL, q_g=NULL, qsaved;
  p3d_vector3 startt[3], goall[3];
  int i;
  double coord[3], orient[3];

  if(start== NULL || goal==NULL){
    PrintError((" start/goal points are null\n"));
    return FALSE;
  }
  if(start[0] == goal[0] && start[1] == goal[1] && start[2] == goal[2]){
    PrintError((" start/goal points are equal\n"));
    return FALSE;
  }

  for(i=0; i<3; i++){
    startt[0][i] = start[i];
    startt[1][i] = start[i];
    startt[2][i] = start[i];
  }
  for(i=0; i<3; i++){
    goall[0][i] = goal[i];
    goall[1][i] = goal[i];
    goall[2][i] = goal[i];
  }

  if(!BTGRAPH)     G = hri_bt_create_graph(btset->robot);
  else             G = BTGRAPH;

  btset->robot->GRAPH = G;
  graph_node = G->nodes;

  qsaved = p3d_get_robot_config(btset->robot);

  /* Start node in the graph */
  q_s = p3d_get_robot_config(btset->robot);
  hri_gik_compute(btset->robot, HRI_GIK, 500, 0.01, startt, &q_s, NULL);
  //q_s = p3d_get_robot_config(btset->robot);

  G->search_start  = p3d_APInode_make(G,q_s);
  p3d_insert_node(G,G->search_start);
  p3d_create_compco(G,G->search_start);
  G->search_start->type = ISOLATED;

  /* Goal node in the graph */

  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);

  /* for 2nd task if it exists */
  p3d_mat4ExtractPosReverseOrder(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_NECK_TILT]->abs_pos,
                                 coord, coord+1, coord+2, orient, orient+1, orient+2);
  goall[1][0] = coord[0];
  goall[1][1] = coord[1];
  goall[1][2] = coord[2]+0.1;
  /*   */

  q_g = p3d_get_robot_config(G->rob);
  hri_gik_compute(btset->robot, HRI_GIK, 500, 0.01, goall, &q_g, NULL);
  //q_g = p3d_get_robot_config(G->rob);

  // Should copy q_g to ROBOT_GOTO
  p3d_copy_config_into(G->rob, q_g, &G->rob->ROBOT_GOTO);

  G->search_goal  = p3d_APInode_make(G,q_g);
  p3d_insert_node(G,G->search_goal);
  p3d_create_compco(G,G->search_goal);
  G->search_goal->type = ISOLATED;


  if(!btset->pathexist){
    ChronoOn();

    // Warning here condition must be negative of positive ???

    if(hri_bt_start_search(start, goal, btset, manip) < 0){ /* here we find the path */
      PrintInfo(("hri_planner : ERROR : A*: no path found\n"));
      return(FALSE);
    }

    ChronoPrint("A STAR - TIME");
    ChronoOff();
  }

#ifdef HRI_JIDO
  hri_bt_write_TRAJ(btset,btset->robot->joints[ROBOTj_GRIP]);
  hri_bt_print_PATH(btset);
#endif

  G->search_done = FALSE;

  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);


  hri_bt_bitmap_to_graphwGIK(btset,G,btset->bitmap[BT_PATH]);

  p3d_destroy_config(btset->robot,qsaved);

  return(TRUE);
}

/****************************************************************/
/*!
 * \brief create the graph regarding to the path found on bitmap
 *
 * \param G      the graph
 * \param bitmap the bitmap
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_bitmap_to_graphwGIK(hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap)
{
  configPt q,qsaved;
  p3d_node *prev_node;
  double dist = 0;
  int done = FALSE;
  p3d_node *NewNode = NULL;
  int count=0;
  p3d_vector3 next_target[3];
  int i, res = FALSE;

  if(bitmap->searched)
    bitmap->current_search_node = bitmap->search_goal;
  else
    return FALSE;


  qsaved = p3d_get_robot_config(btset->robot);

  prev_node =  G->last_node->N;
  q = p3d_copy_config(G->rob, G->search_goal->q);
  while(!done){

    /* if(!HRI_GIK_CONTINUOUS) */
    /*       p3d_set_and_update_this_robot_conf(btset->robot,qsaved); */

    if(bitmap->current_search_node == bitmap->search_start)
      done = TRUE;

    for(i=0; i<3; i++){ /* 3 tasks */
      next_target[i][0] = bitmap->current_search_node->x * btset->pace + btset->realx;
      next_target[i][1] = bitmap->current_search_node->y * btset->pace + btset->realy;
      next_target[i][2] = bitmap->current_search_node->z * btset->pace + btset->realz;
    }

    /* if(giktaskcounter == 0){ */
    /*       p3d_mat4ExtractPos(btset->human[btset->actual_human]->HumanPt->joints[HUMANj_LHAND]->abs_pos, */
    /* 			 coord ,coord+1 ,coord+2 ,orient, orient+1, orient+2);  */

    /*       next_target[1][0] = coord[0]; */
    /*       next_target[1][1] = coord[1]; */
    /*       next_target[1][2] = coord[2]; */
    /*       giktaskcounter=0; */
    /*     } */
    /*     else{ */
    /*       next_target[1][0] = bitmap->current_search_node->x * btset->pace + btset->realx; */
    /*       next_target[1][1] = bitmap->current_search_node->y * btset->pace + btset->realy; */
    /*       next_target[1][2] = bitmap->current_search_node->z * btset->pace + btset->realz; */
    /*     } */
    /*     giktaskcounter++; */

    res = hri_gik_compute(INTERPOINT->robot, HRI_GIK, 200, 0.01, next_target, &q, NULL);
    printf("added a node %d\n",res);
    if(HRI_GIK_CONTINUOUS){
      p3d_set_and_update_this_robot_conf(INTERPOINT->robot,q);
    }

    q = p3d_get_robot_config(G->rob);

    if( /* (count==5) && */ (bitmap->current_search_node != bitmap->search_goal) &&
       (bitmap->current_search_node != bitmap->search_start)){
      count = 0;

      if(!p3d_equal_config(G->rob, q, G->search_start->q)){
        NewNode = p3d_APInode_make(G, q);
        NewNode->type = LINKING;
        p3d_insert_node(G, NewNode);
        dist = p3d_APInode_dist(G,prev_node,NewNode);
        p3d_create_edges(G,prev_node,NewNode,dist);
        p3d_add_node_compco(NewNode, prev_node->comp, TRUE);
        prev_node = NewNode;
      }
    }
    bitmap->current_search_node = bitmap->current_search_node->parent;
    count++;
  }
  dist = p3d_APInode_dist(G,prev_node,G->search_start);
  p3d_create_edges(G,prev_node ,G->search_start ,dist);
  p3d_merge_comp(G, G->search_start->comp, &(prev_node->comp));


  p3d_set_and_update_this_robot_conf(INTERPOINT->robot,qsaved);
  p3d_destroy_config(btset->robot,qsaved);

  return TRUE;

}

/**
 * finds the n smalles values
 */
void hri_bt_min_cell_n(hri_bitmapset * btset, hri_bitmap * bitmap, int *x, int *y, int *z, double * cost, int n)
{
  int i,j,k;
  int index ;

  for(i=0; i<n; i++)
    cost[i] = 1000;

  for(i=0; i<bitmap->nx; i++){
    for(j=0; j<bitmap->ny; j++){
      for(k=0; k<bitmap->nz; k++){

        index = insert2table(bitmap->calculate_cell_value(btset,i,j,k),i,j,k,
                             cost,x,y,z, n);

      }
    }
  }
}

/**
 * insert value into table by inserting before the first value that is greater than this value.
 * if the table was sorted ascending before, it will still be after.
 */
static int insert2table(double value, int cx, int cy, int cz, double * Table,
                        int * x, int * y, int * z, int l)
{
  int i,j;

  if(value != -1){
    for(i=0; i<l; i++){
      if(value < Table[i]){
        for(j=l-1; j>i; j--){
          Table[j] = Table[j-1];
          x[j] = x[j-1];
          y[j] = y[j-1];
          z[j] = z[j-1];
        }
        Table[i] = value;
        x[i] = cx;
        y[i] = cy;
        z[i] = cz;
        return i;
      }
    }
  }
  return -1;
}

/**
 * sets bitmap cells of bitmap 3d obstacles by testing ech position for collisions using visball
 */
int hri_bt_create_precise_obstacles(hri_bitmapset * bitmapset)
{
  configPt visq;
  hri_bitmap * bitmap;
  int x,y,z;

  bitmap = bitmapset->bitmap[BT_3D_OBSTACLES];

  visq = p3d_get_robot_config(bitmapset->visball);

  for(x=0; x<bitmap->nx; x++){
    for(y=0; y<bitmap->ny; y++){
      for(z=0; z<bitmap->nz; z++){
        visq[6]  = x*bitmapset->pace+bitmapset->realx;
        visq[7]  = y*bitmapset->pace+bitmapset->realy;
        visq[8]  = z*bitmapset->pace+bitmapset->realz;
        p3d_set_and_update_this_robot_conf(bitmapset->visball, visq);
        if(p3d_col_test_robot_statics(bitmapset->visball,FALSE))
          bitmapset->bitmap[BT_3D_OBSTACLES]->data[x][y][z].val = -2;
        else
          bitmapset->bitmap[BT_3D_OBSTACLES]->data[x][y][z].val = 0;
      }
    }
  }
  p3d_destroy_config(bitmapset->visball, visq);


  return TRUE;
}


int hri_bt_calculate_bitmap_pathwR6IK(hri_bitmapset * btset, p3d_vector3 start, p3d_vector3 goal, int manip)
{
  p3d_graph *G;
  p3d_list_node *graph_node;
  configPt q_s=NULL, q_g=NULL, qsaved;

  if(start== NULL || goal==NULL){
    PrintError((" start/goal points are null\n"));
    return FALSE;
  }
  if(start[0] == goal[0] && start[1] == goal[1] && start[2] == goal[2]){
    PrintError((" start/goal points are equal\n"));
    return FALSE;
  }

  if(!BTGRAPH)     G = hri_bt_create_graph(btset->robot);
  else             G = BTGRAPH;

  btset->robot->GRAPH = G;
  graph_node = G->nodes;

  qsaved = p3d_get_robot_config(btset->robot);

  /* Start node in the graph */
  q_s = p3d_get_robot_config(btset->robot);
  p3d_copy_config_into(G->rob, q_s, &G->rob->ROBOT_POS);

  //hri_compute_R6IK(btset->robot,btset->object,q_s);

  G->search_start  = p3d_APInode_make(G,q_s);
  p3d_insert_node(G,G->search_start);
  p3d_create_compco(G,G->search_start);
  G->search_start->type = ISOLATED;

  /* Goal node in the graph */

  q_g = p3d_get_robot_config(G->rob);

/*   if(!hri_compute_R6IK(btset->robot,btset->object,q_g)){ */
/*     printf("Goal position is in collision\n"); */
/*     return FALSE; */
/*   } */
  /*  configPt q_deg; */
  /*   q_deg =  p3d_get_robot_config(G->rob); */
  /*   p3d_convert_config_rad_to_deg(btset->robot,q_g,&q_deg); */
  /*   // Should copy q_g to ROBOT_GOTO */
  p3d_copy_config_into(G->rob, q_g, &G->rob->ROBOT_GOTO);
  /*   return FALSE;      */
  G->search_goal  = p3d_APInode_make(G,q_g);
  p3d_insert_node(G,G->search_goal);
  p3d_create_compco(G,G->search_goal);
  G->search_goal->type = ISOLATED;

  if(!btset->pathexist){
    ChronoOn();
    if(hri_bt_start_search(start, goal, btset, manip) < 0){ /* here we find the path */
      PrintInfo(("hri_planner : ERROR : A*: no path found\n"));
      return(FALSE);
    }

    ChronoPrint("A STAR - TIME");
    ChronoOff();
  }

#ifdef HRI_JIDO
  hri_bt_write_TRAJ(btset,btset->robot->joints[ROBOTj_GRIP]);
  hri_bt_print_PATH(btset);
#endif

  G->search_done = FALSE;

  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);

  hri_bt_bitmap_to_graphwR6IK(btset,G,btset->bitmap[BT_PATH]);

  p3d_destroy_config(btset->robot,qsaved);

  return(TRUE);
}

/****************************************************************/
/*!
 * \brief create the graph regarding to the path found on bitmap
 *
 * \param G      the graph
 * \param bitmap the bitmap
 *
 * \return FALSE in case of a problem
 */
/****************************************************************/
int hri_bt_bitmap_to_graphwR6IK(hri_bitmapset * btset, p3d_graph *G, hri_bitmap* bitmap)
{
  configPt q,qsaved;
  p3d_node *prev_node;
  double dist = 0;
  int done = FALSE;
  p3d_node *NewNode = NULL;
  int count=0;

  if(bitmap->searched)
    bitmap->current_search_node = bitmap->search_goal;
  else
    return FALSE;

  //q_o = p3d_get_robot_config(btset->object); /* ALLOC */
  qsaved = p3d_get_robot_config(btset->robot);

  //qtemp = p3d_copy_config(btset->robot,btset->robot->ROBOT_POS);

  prev_node =  G->last_node->N;

  while(!done){

    if(bitmap->current_search_node == bitmap->search_start)
      done = TRUE;

    //q_o[6] = bitmap->current_search_node->x * btset->pace + btset->realx;
    //q_o[7] = bitmap->current_search_node->y * btset->pace + btset->realy;
    //q_o[8] = bitmap->current_search_node->z * btset->pace + btset->realz;
    /*     q_o[9] = -90*M_PI/180; */
    /*     q_o[10] =-90*M_PI/180; */
    /*     q_o[11] = 113*M_PI/180; */

    //p3d_set_and_update_this_robot_conf(btset->object,q_o);
    //   p3d_set_and_update_this_robot_conf(btset->robot,qtemp);
    //q = p3d_copy_config(G->rob, G->search_goal->q);
    //hri_compute_R6IK(btset->robot,btset->object,q);
    //p3d_set_and_update_this_robot_conf(btset->robot,q);
    //p3d_destroy_config(btset->object, q_o);

    q = bitmap->current_search_node->q;

    if( (bitmap->current_search_node != bitmap->search_goal) &&
       (bitmap->current_search_node != bitmap->search_start)){
      count = 0;

      if(!p3d_equal_config(G->rob, q, G->search_start->q)){
        NewNode = p3d_APInode_make(G, q);
        NewNode->type = LINKING;
        p3d_insert_node(G, NewNode);
        dist = p3d_APInode_dist(G,prev_node,NewNode);
        p3d_create_edges(G,prev_node,NewNode,dist);
        p3d_add_node_compco(NewNode, prev_node->comp, TRUE);
        prev_node = NewNode;
      }
    }
    bitmap->current_search_node = bitmap->current_search_node->parent;
    count++;
  }
  dist = p3d_APInode_dist(G,prev_node,G->search_start);
  p3d_create_edges(G,prev_node ,G->search_start ,dist);
  p3d_merge_comp(G, G->search_start->comp, &(prev_node->comp));


  p3d_set_and_update_this_robot_conf(btset->robot,qsaved);
  p3d_destroy_config(btset->robot,qsaved);

  return TRUE;

}

gnuplot_ctrl * hri_bt_init_gnuplot(double xmin, double xmax, double ymin, double ymax,double zmin, double zmax)
{
  gnuplot_ctrl * h;

  h = gnuplot_init();

  if(h == NULL){
    printf("Gnuplot Init problem");
    return h;
  }

  gnuplot_cmd(h,(char*)"set term x11");
  gnuplot_cmd(h,(char*)"set xrange [%f:%f]",xmin,xmax);
  gnuplot_cmd(h,(char*)"set yrange [%f:%f]",ymin,ymax);
  gnuplot_cmd(h,(char*)"set zrange [%f:%f]",zmin,zmax);
  gnuplot_cmd(h,(char*)"set size ratio 1");

  return h;
}

gnuplot_ctrl * hri_bt_init_gnuplot_bitmap(hri_bitmapset * btset, int btno)
{
  gnuplot_ctrl * h;

  h = gnuplot_init();

  if(h == NULL){
    printf("Gnuplot Init problem");
    return h;
  }

  gnuplot_cmd(h,(char*)"set term x11");
  gnuplot_cmd(h,(char*)"set xrange [%f:%f]",btset->realx,btset->bitmap[btno]->nx*btset->pace+btset->realx);
  gnuplot_cmd(h,(char*)"set yrange [%f:%f]",btset->realy,btset->bitmap[btno]->ny*btset->pace+btset->realy);
  gnuplot_cmd(h,(char*)"set zrange [%f:%f]",btset->realz,btset->bitmap[btno]->nz*btset->pace+btset->realz);

  return h;

}


int hri_bt_gnuplot_bitmap(gnuplot_ctrl * h,hri_bitmapset * btset, int btno, double exclude)
{

  char myfile[] = "temp.dat" ;

  if(h==NULL){
    printf("Plot not initialized\n");
    return FALSE;
  }

  hri_exp_save(btset,btset->bitmap[btno],myfile,exclude);

  gnuplot_cmd(h, (char*)"splot '%s' using 1:2:3:4 with points palette " , myfile);

  return TRUE;
}




/**
 * changes state of human between standing and sitting.
 */
int hri_change_human_state(hri_human * human, int state, configPt  config )
{
  if(config == NULL || human == NULL)
    return FALSE;

  if (human->actual_state != state &&
      ((state == BT_STANDING) || (state == BT_SITTING) || (state == BT_MOVING))) {
    hri_set_human_state(human, state, config);
  }
  else
    return FALSE;

  return TRUE;
}



int hri_set_human_state(hri_human * human, int state, configPt config )
{
  return hri_set_human_state_SICK(human, state,config, 1);
}

/**
 * set state adjusting for e.g. sitting position,
 * if the SICK laser coordinates are the center of the detecte legs,
 * the torso of a sitting person is not between the legs in XY
 */
int hri_set_human_state_SICK(hri_human * human, int state, configPt config, int adjustForSick )
{
  if(config == NULL) {
    PrintError(("Config is NULL when setting human state.\n"));
    return FALSE;
  }

  if(strcasestr(human->HumanPt->name,"achile")){
     // if human model is ACHILE
     if(state == BT_SITTING){
       if (adjustForSick && human->actual_state != BT_SITTING) {
         // adjust for sick laser detecting legs, not torso
         config[6] = config[6]-0.45*cos(config[11]);
         config[7] = config[7]-0.45*sin(config[11]);
       }
       config[8] = human->state[state].c7;
       config[33] = human->state[state].c3;
       config[35] = human->state[state].c4;
       config[40] = human->state[state].c1;
       config[42] = human->state[state].c2;
     } else if(state == BT_STANDING || state == BT_STANDING_TRANSPARENT) {
       // no need to adjust, don't know why
       //    if (human->actual_state == BT_SITTING) {
       //      // adjust for sick laser detecting legs, not torso
       //      config[6] = config[6]+0.33*cos(config[11]);
       //      config[7] = config[7]+0.33*sin(config[11]);
       //    }
       config[8] = human->state[state].c7;
       config[33] = human->state[state].c3;
       config[35] = human->state[state].c4;
       config[40] = human->state[state].c1;
       config[42] = human->state[state].c2;
     }  else if(state == BT_MOVING){
       // no need to adjust, don't know why
       //    if (human->actual_state == BT_SITTING) {
       //      // adjust for sick laser detecting legs, not torso
       //      config[6] = config[6]+0.33*cos(config[11]);
       //      config[7] = config[7]+0.33*sin(config[11]);
       //    }
       config[8] = human->state[state].c7;
       config[32] = human->state[state].c3;
       config[35] = human->state[state].c4;
       config[39] = human->state[state].c1;
       config[42] = human->state[state].c2;
     } else { // unknown state
       PrintError(("Unknown human state: %i\n", state));
       return FALSE;
     }
  } else {
    // if human model is SUPERMAN
    if(state == BT_SITTING){
      if (adjustForSick && human->actual_state != BT_SITTING) {
        // adjust for sick laser detecting legs, not torso
        config[6] = config[6]-0.33*cos(config[11]);
        config[7] = config[7]-0.33*sin(config[11]);
      }
      config[8] = human->state[state].c7;
      config[43] = human->state[state].c1;
      config[44] = human->state[state].c2;
      config[46] = human->state[state].c3;
      config[47] = human->state[state].c4;
      config[50] = human->state[state].c5;
      config[53] = human->state[state].c6;
      /* Right Hand */
      config[66] = config[6] + cos(config[11]-0.4)*0.5; /* REVIEW 0.4 --> 0.2 */
      config[67] = config[7] + sin(config[11]-0.4)*0.5;
      config[68] = config[68]-0.34+0.1;
      /* Left Hand */
      config[72] = config[6] + cos(config[11]+0.4)*0.5;
      config[73] = config[7] + sin(config[11]+0.4)*0.5;
      config[74] = config[74]-0.34+0.1;
    } else if(state == BT_STANDING || state == BT_STANDING_TRANSPARENT) {
      // no need to adjust, don't know why
      //    if (human->actual_state == BT_SITTING) {
      //      // adjust for sick laser detecting legs, not torso
      //      config[6] = config[6]+0.33*cos(config[11]);
      //      config[7] = config[7]+0.33*sin(config[11]);
      //    }
      config[8] = human->state[state].c7;
      config[43] = human->state[state].c1;
      config[44] = human->state[state].c2;
      config[46] = human->state[state].c3;
      config[47] = human->state[state].c4;
      config[50] = human->state[state].c5;
      config[53] = human->state[state].c6;
      /* Right Hand */
      config[66] = config[6] + cos(config[11]-0.4)*0.5;
      config[67] = config[7] + sin(config[11]-0.4)*0.5;
      config[68] = 1.1;
      /* Left Hand */
      config[72] = config[6] + cos(config[11]+0.4)*0.5;
      config[73] = config[7] + sin(config[11]+0.4)*0.5;
      config[74] = 1.1;
    }  else if(state == BT_MOVING){
      // no need to adjust, don't know why
      //    if (human->actual_state == BT_SITTING) {
      //      // adjust for sick laser detecting legs, not torso
      //      config[6] = config[6]+0.33*cos(config[11]);
      //      config[7] = config[7]+0.33*sin(config[11]);
      //    }
      config[8] = human->state[state].c7;
      config[43] = human->state[state].c1;
      config[44] = human->state[state].c2;
      config[46] = human->state[state].c3;
      config[47] = human->state[state].c4;
      config[50] = human->state[state].c5;
      config[53] = human->state[state].c6;
      /* Right Hand */
      config[66] = config[6] + cos(config[11]-0.4)*0.5;
      config[67] = config[7] + sin(config[11]-0.4)*0.5;
      config[68] = 1.1;
      /* Left Hand */
      config[72] = config[6] + cos(config[11]+0.4)*0.5;
      config[73] = config[7] + sin(config[11]+0.4)*0.5;
      config[74] = 1.1;
    } else { // unknown state
      PrintError(("Unknown human state: %i\n", state));
      return FALSE;
    }
  }
  human->actual_state = state;

  // dirty workaround until MHP offers a better way to set transparent
  if(state == BT_MOVING || state == BT_STANDING_TRANSPARENT){
    human->transparent = TRUE;
  } else {
    human->transparent = FALSE;
  }
  return TRUE;
}
