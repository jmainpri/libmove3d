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

typedef enum {
  NORMAL,
  OBJECTIF,
  DIFFERENCE
}g3d_window_draw_mode;

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
  GLfloat floorColor[3]; 

  //! floor plane equation
  GLfloat floorPlane[4];

  //! equations of the wall planes
  GLfloat wallPlanes[4][4];

  //! shadow projection matrix onto the floor
  GLfloat floorShadowMatrix[16];

  //! shadow projection matrices onto the walls
  GLfloat wallShadowMatrix[4][16];


  //! shadow density (shadowContrast must be > 0 and < 1); the more shadowContrast is close
  //! to 1, the smallest will be the contrast between shaded and enlightened zones
  GLfloat shadowContrast;
#endif

  //flag to tell wether or not the current robot joints are drawn
  unsigned displayJoints;

  //booleen pour indiquer si on affiche les ombres ou pas:
  unsigned displayShadows;
  //booleen pour indiquer si on affiche les murs ou pas:
  unsigned displayWalls;


  unsigned displayFloor; //Boolean to enable/disable floor
  unsigned displayTiles; //Boolean to enable/disable floor tiles


#ifdef HRI_PLANNER
  int point_of_view;                    /* Boolean for  another perspective */ 
  int win_perspective;                  /* Boolean to know if it is a perspective window */    
  g3d_window_draw_mode draw_mode;       /* Boolean to know if we'll draw only the objective or in a different color from obstacles */
#endif

};

#endif
