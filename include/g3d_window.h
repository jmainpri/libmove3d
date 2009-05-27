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
  int FILAIRE, CONTOUR, GOURAUD, ACTIVE, list;
  GLfloat    frustum[6][4]; /* 6 x 4 flottants correspondant au coeffs de frustum de vue*/
  G3D_Window *next;

#ifdef PLANAR_SHADOWS

  //pointeur vers une fonction d'affichage supplémentaire, appelable depuis n'importe quel fichier source
	void (*fct_draw2) ();
  //pointeur vers une fonction appelée en appuyant sur une touche du clavier (voir g3d_window.c)
	void (*fct_key) ();

    //position de la source de lumière qui projette les ombres:
	GLfloat lightPosition[4];

    //équation du plan du sol:
	GLfloat floorPlane[4];

    //équations des plans des murs:
	GLfloat wallPlanes[4][4];

    //matrice de projection des ombres sur le plan du sol:
	GLfloat floorShadowMatrix[16];

    //matrices de projection des ombres sur les plans des murs:
	GLfloat wallShadowMatrix[4][16];

    //densité des ombres (il faut 0 < shadowContrast < 1); plus shadowContrast est proche de 1
    //moins le contraste est grand entre les zones d'ombre et de lumière.:
	GLfloat shadowContrast;
#endif
  //booleen pour indiquer si on affiche les ombres ou pas:
	unsigned displayShadows;
  //booleen pour indiquer si on affiche les murs ou pas:
  unsigned displayWalls;


  unsigned displayFloor; //Boolean to enable/disable floor

  unsigned displayTiles; //Boolean to enable/disable tiles on the floor

#ifdef HRI_PLANNER
  int point_of_view;                    /* Boolean for  another perspective */ 
  int win_perspective;                  /* Boolean to know if it is a perspective window */    
  g3d_window_draw_mode draw_mode;       /* Boolean to know if we'll draw only the objective or in a different color from obstacles */
#endif

};

#endif
