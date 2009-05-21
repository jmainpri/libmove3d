#ifndef _PERSPECTIVE_H
#define _PERSPECTIVE_H

//int npspconf;
//configPt *list_psp_configs;

/* typedef enum{ */
/*   PSP_INSIDE, */
/*   PSP_OUTSIDE, */
/*    PSP_CUTTED_UP, */

/* }PSP_VIS_STATUS; */


typedef enum{
  PSP_OBSERVED,
  PSP_NON_OBSERVED,
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
  int id;           //ID of the point
  int status;       //status of the element
  double cost;      //cost of desired position
  p3d_vector3  pos; //Position x,y,z
}psp_obs_vertex;


typedef struct{
  int nv;
  int currentVert;
  psp_obs_vertex vertex[250];
}psp_lst_vertex;


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


extern p3d_rob *PSP_ROBOT;


extern int PSP_DEACTIVATE_AUTOHIDE;

#endif
//  p3d_poly elem;
//  p3d_Matrix4 abs_pos;
