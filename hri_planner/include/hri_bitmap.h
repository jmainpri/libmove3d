#ifndef _BITMAP_H
#define _BITMAP_H

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
#define BT_STATE_NO 2

#define BT_SAMPLING 0.3 //0.05
#define BT_3DR_SAMPLING 0.03
#define BT_3D_SAMPLING 0.1

#define BT_COMBINE_MAX  217
#define BT_COMBINE_SUM  218

#define BT_TRG_LOOK 311
#define BT_TRG_BODY 312
#define BT_TRG_APPROACH 313

#define BT_STANDING 0
#define BT_SITTING  1

extern int PLACEMENT;
extern int PLCMT_TYPE;
extern int GIK_VIS;

typedef struct bitmap_set hri_bitmapset;

typedef struct bitmap_cell{
  int x;
  int y;
  int z; 
  double val;                     /* cost */
  double h;                       /* astar: heuristic */
  double g;                       /* astar: g */
  struct bitmap_cell * parent;    /* astar: cell's parent */
  int closed;                     /* astar: TRUE if cell's closed */
  int open;                       /* astar: TRUE if cell's open */
  int locked;
	
  int obstacle[8];

  configPt q;

} hri_bitmap_cell;

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

} hri_human_state;

typedef struct human{
  p3d_rob * HumanPt;
  int id;
  int exists;
  int states_no;
  int actual_state;
  hri_human_state * state;
  int coord_changed;
} hri_human;

typedef struct bt_path{
  double* xcoord;
  double* ycoord;
  double* zcoord;
  double* theta;
  int length;
} hri_bt_path;

typedef struct bitmap{
  int type;     /* bitmap type                 */
  int id;       /* bitmap id                   */

  long nx;      /* cell number                 */
  long ny;    
  long nz; 
  struct bitmap_cell *** data;
  
  struct bitmap_cell * search_start;
  struct bitmap_cell * search_goal;
  struct bitmap_cell * current_search_node;
  int searched;

  configPt start_config;
  configPt goal_config;
  
  int active;
  
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
  int pathexist;            /* TRUE if path has been already calculated */
  int combine_type;
  int changed;
  int human_no;
  int actual_human;
  
  hri_human ** human;
  p3d_rob * robot;
  p3d_rob * visball;
  p3d_rob * object;
  
  struct bt_path * path;
  
  /* for visioning */
  double BT_target[2];
  int BT_target_available;

  int manip;
  
};

#endif
