#ifndef _G3D_STATES_H
#define _G3D_STATES_H

#ifdef USE_SHADERS
 #include <GL/glew.h>
#endif

#ifndef PROTO
        #include <stdlib.h>
        #include <math.h>
#if defined(MACOSX) 
#if defined(QT_LIBRARY) && defined (CXX_PLANNER)
        #include "gl.h"
        #include "glu.h"
#else
        #include <glu.h>
#endif
#else
		#include "GL/gl.h"
		#include "GL/glu.h"
#endif
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

//! This enum is used to know if, for a g3d_window, which kind of OpenGL projection to use.
typedef enum {
  G3D_PERSPECTIVE,
  G3D_ORTHOGRAPHIC,
} g3d_projection_mode;

//! This enum is used to know if, for a g3d_window, we will display only transparent object, only opaque objects, display both or disable transparency.
typedef enum {
  G3D_TRANSPARENT,
  G3D_OPAQUE,
  G3D_TRANSPARENT_AND_OPAQUE,
  G3D_NO_TRANSPARENCY,
} g3d_transparency_mode;


typedef struct g3d_states
{
  float      bg[3];
  float      size;
  GLdouble   x,y,z,el,az,zo;
  p3d_vector4  up;
  GLdouble   sx,sy,sz,sel,saz,szo;
  p3d_vector4  sup;
  int FILAIRE, CONTOUR, GHOST, GOURAUD, BB, ACTIVE, list;
  GLfloat    frustum[6][4]; /* 6 x 4 flottants correspondant au coeffs de frustum de vue*/
  GLdouble   fov; /*!< camera field of view angle (IN DEGREES) */
  p3d_vector3 cameraPosition; /*!< position of the camera used to set the light position if cameraBoundedLight is true*/
  
  g3d_projection_mode projection_mode; /*!< defines the kind of OpenGL projection to use */

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

  //! choose which objects to display according to their transparency
  g3d_transparency_mode transparency_mode; 


  //! flag to tell if events (key press, mouse clicking) are autorized or not.
  //! This avoids modifying something by accident. 
  unsigned int eventsEnabled;

  //! flag to tell wether or not we enable the face culling test of OpenGL.
  //! It is much better to enable face culling but this requires that the faces of the 3D models are correctly oriented. 
  unsigned int cullingEnabled;

  //! flag to tell wether or not the frame at the focus point of the camera is drawn
  unsigned int displayFrame;

  //! flag to tell wether or not the current robot joints are drawn
  unsigned int displayJoints;

  //! flag to tell wether or not OpenGL will use lighting:
  unsigned int enableLight;
  
  //! this flag is used to bound the light position on the camera position:
  unsigned int cameraBoundedLight; 

  //! flag to tell wether or not the planar shadows will be displayed:
  unsigned int displayShadows;

  //! flag to tell wether or not the walls will be displayed:
  unsigned int displayWalls;

  //! flag to tell wether or not the floor will be displayed:
  unsigned int displayFloor;

  //! flag to tell wether or not the floor tiles will be displayed:
  unsigned int displayTiles; 
  
  //! flag to tell wether or not antialiasing will be activated:
  unsigned int enableAntialiasing;

  //! flag to tell wether or not shaders will be activated:
  unsigned int enableShaders;

  //! this flag is used when planar shadows are enabled to indicate that all bodies must be drawn in black
  //! with no lighting:
  unsigned int allIsBlack; 

  //! used to display the logo texture (texture ID used by OpenGL):
  GLuint logoTexture; 

//! flag to tell wether or not the LAAS logo will be displayed:
  unsigned int enableLogo; 

} g3d_states;

#endif
