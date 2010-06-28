#ifndef _BITMAP_H
#define _BITMAP_H


//AKP
#ifdef USE_MIGHTABILITY_MAPS
#include "Mightability_Maps.h"
#endif


/** types of bitmap and index of type in bitmapset array */
#define BT_VISIBILITY 0
#define BT_DISTANCE   1
#define BT_HIDZONES   2
#define BT_OBSTACLES  4
#define BT_VELOCITY   3
#define BT_COMBINED   5
#define BT_PATH       6

#define BT_3D_VISIBILITY 0
#define BT_3D_DISTANCE 1
#define BT_3D_HCOMFORT 2
#define BT_3D_RREACH 3
#define BT_3D_OBSTACLES  4
#define BT_3D_COMBINED 5
#define BT_3D_PATH 6

#define BT_BITMAP_NO 7
#define BT_HUMAN_NO 5


#define BTS_SIZE 10 /* maximum number of bitmaps allowed in a bitmapset */


/* how big a cell value must be at least to be considered non-zero for safety and comfort
 * this makes robot movement more legible, and stabilizes robot path.
 * The reference is that path cost from cell to cell is 1
 */
#define BT_NAVIG_THRESHOLD 10

// this defines the grid density in move3d
#define BT_SAMPLING 0.05 //0.05
#define BT_3DR_SAMPLING 0.03
#define BT_3D_SAMPLING 0.2

#define BT_COMBINE_MAX  217
#define BT_COMBINE_SUM  218

#define BT_TRG_LOOK 311
#define BT_TRG_BODY 312
#define BT_TRG_APPROACH 313

// the number of human states as declared next
#define BT_STATE_NO 4

#define BT_STANDING 0
#define BT_SITTING  1
#define BT_MOVING  2
// transparent look like normal, but means the planner may let the robot go through the human at high costs
#define BT_STANDING_TRANSPARENT 3

/** used fot btset->manip */
#define BT_MANIP_NAVIGATION 0
#define BT_MANIP_MANIPULATION 1
#define BT_MANIP_REACH 2

/* function parameter and cell value*/
#define BT_OBST_POTENTIAL_HUMAN_COLLISION -1
/* function parameter and cell value*/
#define BT_OBST_SURE_COLLISION -2
/* function parameter only */
#define BT_OBST_POTENTIAL_OBJECT_COLLISION -3
/* function parameter only */
#define BT_OBST_MARK_CORRIDOR -4
/* grid cell value flag */
#define BT_OBST_POTENTIAL_CORRIDOR_MARK -4
/* grid cell value flag */
#define BT_OBST_SURE_CORRIDOR_MARK -5


/** Error codes for hri_bt_start_search */
#define HRI_PATH_SEARCH_ERROR_NAV_START_IN_OBSTACLE -1
#define HRI_PATH_SEARCH_ERROR_NAV_GOAL_IN_OBSTACLE -2
#define HRI_PATH_SEARCH_ERROR_NAV_HUMAN_TOO_CLOSE -3
#define HRI_PATH_SEARCH_ERROR_NAV_INTERNAL_ERROR -4
#define HRI_PATH_SEARCH_ERROR_NAV_NAV_NO_PATH_FOUND -5
#define HRI_PATH_SEARCH_ERROR_NAV_NAV_ALREADY_REACHED -6



#define BT_DIRECTION_NORTH     0
#define BT_DIRECTION_NORTHEAST 1
#define BT_DIRECTION_EAST      2
#define BT_DIRECTION_SOUTHEAST 3
#define BT_DIRECTION_SOUTH     4
#define BT_DIRECTION_SOUTHWEST 5
#define BT_DIRECTION_WEST      6
#define BT_DIRECTION_NORTHWEST 7

extern int PLACEMENT;  /* changed by form, n,ne,e,se,s, etc. */
extern int PLCMT_TYPE; /* changed by form, look, body, go */
extern int GIK_VIS;    /* changed by form, no of iterations between visual updates (bottom bar on form) */

typedef struct bitmap_set hri_bitmapset;

typedef struct bitmap_cell{
  int x;
  int y;
  int z;                          /* = 1 for NHP */
  double val;                     /* cost */
  double h;                       /* astar: heuristic, the estimated cost from this node to end */
  double g;                       /* astar: g, the cost from start to this node */
  struct bitmap_cell * parent;    /* astar: cell's parent */
  int closed;                     /* astar: TRUE if cell's closed */
  int open;                       /* astar: TRUE if cell's open */
  int locked;

  configPt q;   // manipulation, array of actuator configurations

#ifdef USE_MIGHTABILITY_MAPS
  struct Mightability_Map_set Mightability_Map;//AKP
  struct Mightability_map_cell_object_info Mightability_map_cell_obj_info;//AKP
#endif
  

} hri_bitmap_cell;

/* states of humans e.g. sitting standing*/
typedef struct state{
  char name[20];

  double dheight;  // distance
  double dradius;
  double vheight;  // visibility
  double vback;
  double vradius;
  double hradius; // hiddens

  // configurations of human skeletton joints (only those that differ between standing and sitting)
  double c1;
  double c2;
  double c3;
  double c4;
  double c5;
  double c6;
  double c7;

} hri_human_state;

typedef struct human{
  p3d_rob * HumanPt;
  int id; // obsolete?
  int exists;
  /** whether to consider human as non obstacles, merely weight */
  int transparent;
  /* number of possible states for this human (e.g. handicaped humans have different states) */
  int states_no;
  int actual_state;
  /* possible states */
  hri_human_state * state;
  /* obsolete, was used after human change */
  int coord_changed;
} hri_human;

typedef struct bt_path{
  double* xcoord;
  double* ycoord;
  double* zcoord;
  double* theta;
  int length;
} hri_bt_path;

/* eg visibility, distance, hiddens, final */
typedef struct bitmap{
  int type;     /* bitmap type                 */
  int id;       /* bitmap id (not used for the moment) */

  long nx;      /* number of cells (usually < 1000) */
  long ny;
  long nz;

  // contains calculated cell weights (calculated on demand, e.g. for find path or for visualisation)
  struct bitmap_cell *** data;

  struct bitmap_cell * search_start;
  struct bitmap_cell * search_goal;
  struct bitmap_cell * current_search_node;
  int searched; // whether this bitmap contains data of a previous search (is dirty)

  // inputs to the search algorithm (current, goto)
  configPt start_config; // robot current configuration
  configPt goal_config;  // also arm configuration when moving and at end

  int active; // 1 if visible in form

  // function to calculate this bitmaps cell value
  double (*calculate_cell_value)(struct bitmap_set*,int x, int y, int z);

} hri_bitmap;



/**
 * structure containing the parameters to use
 */
typedef struct astar_parameters{

  /*
   * Additional cost for moving within region of potential collision
   */
  int soft_collision_base_cost;
  int soft_collision_distance_weight;

  /* By how much to multiply the grid distance as cost */
  int path_length_weight;

  /*
   * how many grid cells the robot actual position may deviate from
   * a previously planned path to consider the robot on this cell off the path
   */
  int path_reuse_cell_startcell_tolerance;
  /* how much better in % of costs a new path must be to beat an old path */
  int path_reuse_threshold;
  /* flag to activate reluctance behavior, which prefers an
   * existing path if the new one is not much better.
   * This could in some cases help for later algorithms */
  int use_changepath_reluctance;
  /* how many grid steps away a free start cell may be found from the robot real position */
  int start_cell_tolerance;

  int use_corridors;

  double corridor_Costs;
} hri_astar_parameters;



/* a container for differents kinds of bitmaps */
struct bitmap_set{
  double realx;     /* real coordinates of cell 0,0,0 */
  double realy;
  double realz;

  double pace;      /* real sampling pace of cells */

  int max_size;             /* max size of bitmaps array  */
  int n;                    /* number of bitmaps in array */
  hri_bitmap ** bitmap;     /* bitmaps */
  int pathexist;            /* TRUE if path has been already calculated with success */
  int combine_type;         // sum or max
  int changed;              // for forms interface
  int human_no;
  int actual_human;         // target human for handover

  hri_human ** human;
  p3d_rob * robot;
  
  p3d_rob * visball;        // virtual move3d object for calculating hidden zones
  p3d_rob * object;         // for grasping

  struct bt_path * path;    // found path

  /* for visioning in PSP */
  double BT_target[2];
  int BT_target_available;

  int manip; // BT_MANIP_... type of bitmap, one of (manip, reach, navigation)

  struct astar_parameters * parameters;
};



#endif
