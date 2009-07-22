#ifndef _BITMAP_H
#define _BITMAP_H

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
#define BT_STATE_NO 3

#define BTS_SIZE 10 /* maximum number of bitmaps allowed in a bitmapset */


/* how big a cell value must be at least to be considered non-zero for safety and comfort
 * this makes robot movement more legible, and stabilizes robot path.
 * The reference is that path cost from cell to cell is 1
 */
#define BT_NAVIG_THRESHOLD 10

// this defines the grid density in move3d
#define BT_SAMPLING 0.05 //0.05
#define BT_3DR_SAMPLING 0.03
#define BT_3D_SAMPLING 0.1

#define BT_COMBINE_MAX  217
#define BT_COMBINE_SUM  218

#define BT_TRG_LOOK 311
#define BT_TRG_BODY 312
#define BT_TRG_APPROACH 313

#define BT_STANDING 0
#define BT_SITTING  1
#define BT_MOVING  2

/** used fot btset->manip */
#define BT_MANIP_NAVIGATION 0
#define BT_MANIP_MANIPULATION 1
#define BT_MANIP_REACH 2

#define BT_OBST_SURE_COLLISION -2
//#define BT_OBST_POTENTIAL_COLLISION  -1

/*
 * Additional cost for moving within region of potential collision
 */
#define BT_OBST_POTENTIAL_COLLISION_MIN_COST 15
#define BT_OBST_POTENTIAL_COLLISION_FACTOR 8

/* By how much to multiply the grid distance as cost */
#define BT_DISTANCE_WEIGHT 60

/*
 * how many grid cells the robot actual position may deviate from
 * a previously planned path to consider the robot on this cell of the path
 */
#define BT_PATH_OLDPATH_FINDCELL_TOLERANCE 3

#define BT_PATH_RELUCTANCE_BUFFER 30 /* how much better in % of costs a new path must be to beat an old path */
#define BT_PATH_USE_RELUCTANCE 1 /* flag to activate behavior */

#define BT_PATH_DISLOCATE_HUMANS 1

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
  int id;
  int exists;
  int states_no; // number of possible states for this human (e.g. handicaped humans have different states)
  int actual_state;
  hri_human_state * state;
  int coord_changed; // obsolete, was used after human change
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

};

#endif
