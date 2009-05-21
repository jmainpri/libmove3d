#ifndef _BITMAP_H
#define _BITMAP_H

#define BT_VISIBILITY 0
#define BT_DISTANCE   1
#define BT_HIDZONES   2
#define BT_OBSTACLES  3
#define BT_VELOCITY   4
#define BT_COMBINED   5
#define BT_PATH       6

#define BT_BITMAP_NO 7
#define BT_HUMAN_NO 5
#define BT_STATE_NO 2

#define BT_SAMPLING 0.05

#define BT_COMBINE_MAX  217
#define BT_COMBINE_SUM  218

#define BT_TRG_LOOK 311
#define BT_TRG_BODY 312
#define BT_TRG_APPROACH 313

#define BT_STANDING 0
#define BT_SITTING  1

int PLACEMENT;
int PLCMT_TYPE;
int GIK_VIS;

typedef struct bitmap{
  double realx;
  double realy;
   
  double pace;  /* real sampling pace of cells */
  int type;     /* bitmap type                 */
  int id;       /* bitmap id                   */
  long nx;      /* cell number                 */
  long ny;    
  struct bitmap_cell ** data; /* bitmap */

  struct bitmap_cell * search_start;
  struct bitmap_cell * search_goal;
  struct bitmap_cell * current_search_node;
  int searched;

  int active;

} p3d_bitmap;


typedef struct bitmap_cell{
  int x;
  int y;
  double val;                     /* cost */
  double h;                       /* astar: heuristic */
  double g;                       /* astar: g */
  struct bitmap_cell * parent;    /* astar: cell's parent */
  int closed;                     /* astar: TRUE if cell's closed */
  int open;                       /* astar: TRUE if cell's open */
  int locked;

} p3d_bitmap_cell;

typedef struct state{
  char name[20];

  double dheight;
  double dradius;
  double vheight;
  double vback;
  double vsides;
  double hradius;

  double c1;
  double c2;
  double c3;
  double c4;
  double c5;
  double c6;
  double c7;

} p3d_human_state;

typedef struct human{
  p3d_rob * HumanPt;
  int id;
  int exists;
  int states_no;
  int actual_state;
  p3d_human_state * state;
  int coord_changed;
} p3d_human;

typedef struct bt_path{
  double* xcoord;
  double* ycoord;
  double* theta;
  int length;
} p3d_bt_path;


/* a container for differents kinds of bitmaps */
typedef struct bitmap_set{
  int size;                 /* max size of bitmaps array  */
  /*  int n;                     number of bitmaps in array */
  p3d_bitmap ** bitmap;     /* bitmaps */
  int pathexist;            /* TRUE if path has been already calculated */
  int combine_type;
  int changed;
  int human_no;
  int actual_human;
  
  p3d_human ** human;
  p3d_rob * robot;
  p3d_rob * visball;

  struct bt_path * path;
    
  /* for visioning */
  double BT_target[2];
  int BT_target_available;



 
} p3d_bitmapset;



#endif
