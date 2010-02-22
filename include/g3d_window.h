#ifndef _G3D_WIN_H
#define _G3D_WIN_H

#ifndef PROTO
	#include <stdlib.h>
	#include <math.h>
	#include "GL/gl.h"
	#include "GL/glu.h"
	#ifdef UNIX
//		#include "GL/glx.h"
//		#include "forms.h"
	#endif
#endif

#define INIT_AZ 0.3 /* 20 degrees */
#define INIT_EL 0.5 /* 30 degrees */
#define GAIN_AZ 1.5
#define GAIN_EL 1.5

/** @defgroup graphic 
* The graphic module contains display functions, mainly based upon OpenGL functions.
 */


typedef enum {
  NORMAL,
  OBJECTIF,
  DIFFERENCE
} g3d_window_draw_mode;

typedef enum {
  G3D_PERSPECTIVE,
  G3D_ORTHOGRAPHIC,
} g3d_projection_mode;


typedef struct g3d_win G3D_Window;

struct g3d_win {
  char       name[256];
  void       *form;
  void       *canvas;
  void       *mcamera_but;
  void       (*fct_draw)(void);
  pp3d_matrix4  cam_frame;
  pp3d_matrix4 (*fct_mobcam)(void);
  float      bg[3];
  float      size;
  GLdouble   x,y,z,el,az,zo;
  p3d_vector4  up;
  GLdouble   sx,sy,sz,sel,saz,szo;
  p3d_vector4  sup;
  int FILAIRE, CONTOUR, GHOST, GOURAUD, BB, ACTIVE, list;
  GLfloat    frustum[6][4]; /* 6 x 4 flottants correspondant au coeffs de frustum de vue*/
  G3D_Window *next;
  g3d_projection_mode projection_mode;
#ifdef PLANAR_SHADOWS
  //! pointer to an additional display function, that can be called from any source file
  void (*fct_draw2) ();

  //! pointer to a function that is called by pressing a key (see g3d_window.c)
  void (*fct_key1) ();
  //! pointer to another function that is called by pressing a key (see g3d_window.c)
  void (*fct_key2) ();

  //! position of the light source that creates the shadows
  GLfloat lightPosition[4];

  //! floor color
  GLdouble floorColor[3]; 

  //! wall color
  GLdouble wallColor[3]; 

  //! floor plane equation
  GLdouble floorPlane[4];

  //! equations of the wall planes
  GLdouble wallPlanes[4][4];

  //! shadow projection matrix onto the floor
  GLdouble floorShadowMatrix[16];

  //! shadow projection matrices onto the walls
  GLdouble wallShadowMatrix[4][16];

#endif
  //! flag to tell wether or not the frame at the focus point of the camera is drawn
  unsigned int displayFrame;

  //! flag to tell wether or not the current robot joints are drawn
  unsigned int displayJoints;

  //! flag to tell wether or not OpenGL will use lighting:
  unsigned int enableLight;

  //! flag to tell wether or not the planar shadows will be displayed:
  unsigned int displayShadows;

  //! flag to tell wether or not the walls will be displayed:
  unsigned int displayWalls;

  //! flag to tell wether or not the floor will be displayed:
  unsigned int displayFloor; 

  //! flag to tell wether or not the floor tiles will be displayed:
  unsigned int displayTiles; 

  //! this flag is used when planar shadows are enabled to indicate that all bodies must be drawn in black
  //! with no lighting:
  unsigned int allIsBlack; 

#ifdef HRI_PLANNER
  int point_of_view;                    /* Boolean for  another perspective */ 
  int win_perspective;                  /* Boolean to know if it is a perspective window */    
  g3d_window_draw_mode draw_mode;       /* Boolean to know if we'll draw only the objective or in a different color from obstacles */
#endif

};

#endif
