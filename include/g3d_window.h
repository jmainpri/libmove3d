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
};

#endif
