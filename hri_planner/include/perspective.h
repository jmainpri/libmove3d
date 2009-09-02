#ifndef _PERSPECTIVE_H
#define _PERSPECTIVE_H

//int npspconf;
//configPt *list_psp_configs;

/* typedef enum{ */
/*   PSP_INSIDE, */
/*   PSP_OUTSIDE, */
/*    PSP_CUTTED_UP, */

/* }PSP_VIS_STATUS; */

#define PSP_MAX_COLOR_IDX 1.0
#define PSP_MAX_OBJ_NUM 255
#define OFF_LINE

typedef enum{
  PSP_NO_TASK,           // ordered by cost: taking less costly point first
  PSP_GIVE_TASK,        // Taking points as they are generated (non-ordered)
  PSP_PICK_TASK,             // Searching by a ramdom method
  PSP_TAKE_FROM
}PSP_TASK_TYPE;



typedef enum{
  PSP_AROUND,            // all around the human
  PSP_FRONT,             // only to search in front of the human
  PSP_AROUND_WTRAJ,
  PSP_FRONT_WTRAJ,
  PSP_AROUND_RANDOM,
  PSP_FRONT_RANDOM,
  PSP_AROUND_COMPLETE,
  PSP_FRONT_COMPLETE
}PSP_SEARCH_METHOD;

typedef enum{
  PSP_ORDERED,           // ordered by cost: taking less costly point first
  PSP_SECUENTIAL,        // Taking points as they are generated (non-ordered)
  PSP_RANDOM,             // Searching by a ramdom method
  PSP_RANDOM_LIST
}PSP_SEARCH_TYPE;

typedef enum{
  PSP_FFFO,              // First Found First Out (first acceptable conf. found is the one it takes)
  PSP_BCF,                // Best Cost Found (checks all points and takes best one)
  PSP_DEEP
}PSP_SEARCH_GOAL;

typedef enum{
  PSP_SRCHM_METHOD,              // Three parameters for searching
  PSP_SRCHM_TYPE,                //
  PSP_SRCHM_GOAL                 // 
}PSP_SEARCH;

typedef enum{
  PSP_OBSERVED,
  PSP_NON_OBSERVED,
  PSP_NOT_AVAILABLE
}PSP_VIS_STATUS;

typedef enum{
  PSP_St_OBSERVED,
  PSP_St_OBSERVABLE,
  PSP_St_OBS_HIGH_COST,
  PSP_St_NOT_IN_RANGE,
  PSP_St_HIDDEN
}PSP_STATUS;

typedef struct{
  char *name; //element's name
  int type; //type robot/obstacle/Body
  int x;   //Position x,y,z
  int y;
  int z;
  int status; //status of the element
}psp_obs_element;

typedef struct{
  int ne;
  psp_obs_element **elements;
}psp_lst_elements;



typedef struct{
  int id;            // ID of the point
  int status;        // status of the element
  int segment;       // coordinate segment
  int layer;         // coordiante layer
  double cost;       // cost of desired position
  double quality;    // quality of the point
  double utility;    // utility of the point
  p3d_vector3  pos;  // Position x,y,z
}psp_obs_vertex;


typedef struct{
  int nv;           // number of vertex
  int currentVert;  // current vertex
  int nl;           // number of layers
  int ns;           // number of segments
  int chosen;       // chosen vertex
  psp_obs_vertex vertex[5000]; //List of vertex for ordered methods
  psp_obs_vertex grid[100][50];  //grid of vertex for optimization function
}psp_lst_vertex;


typedef struct{
  int nv;           // number of vertex
  int currentVert;  // current vertex
  psp_obs_vertex vertex[1000]; //List of vertex for ordered methods
}psp_lst_vertex_tmp;

typedef struct{
  p3d_vector3 origin; //Origin Point of the cone (in the sharpen extremity)
  double h;           //height of the cone
  double angle;       //Aperture angle
}psp_cone;

typedef struct{
  p3d_vector3 position;
  double distMin;
  double distMax;
}psp_searchball;

typedef int (*fct_psp_task_eval)(p3d_rob*, void*, configPt*, double*, double*);

extern p3d_rob *PSP_ROBOT;

extern int PSP_init_grid;

extern int PSP_DEACTIVATE_AUTOHIDE;

extern int PSP_NUM_OBJECTS;
extern int PSP_CURR_DRAW_OBJ;

extern int PSP_DRAW_OBJ_ARRAY [];

extern float PSP_DRAW_OBJ_COL_INDEX[];

#endif
//  p3d_poly elem;
//  p3d_Matrix4 abs_pos;
