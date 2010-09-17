/* 
 *  This file includes the prototypes of the functions defined in the
 *  directory graphic and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef GRAPHIC_PKG_H
#define GRAPHIC_PKG_H

#ifdef UNIX
#endif


/* struct */
#include "p3d_sys.h"
#include "p3d_type.h"
#include "p3d_matrix.h"
#include "polyhedre.h"
#include "p3d_poly.h"
#include "p3d.h"

#include "traj.h"
#include "localpath.h"
#include "device.h"

#include "g3d_states.h"

#ifdef WITH_XFORMS
#include "g3d_window.h"
#include "forms.h"
#else
#include "qtWindow/qtOpenGL/qtG3DWindow.hpp"
#include <iostream>
#endif

/* QT OpenGL*/
extern void g3d_draw_env();
extern void g3d_draw_object_moved(p3d_obj *o, int coll, G3D_Window* win);


/* globals */

extern int G3D_DRAW_TRACE;
extern int G3D_DRAW_OCUR_SPECIAL;
extern int boxlist;	/* liste opengl pour la boite materialisant
			   l'environnment */
extern int p3d_numcoll;	/* Variables externes pour le CC */

#ifdef CXX_PLANNER
#include <vector>
namespace API { class Trajectory; }
extern std::vector<API::Trajectory> trajToDraw;
#endif

#ifdef QT_GL
#include "qtWindow/qtOpenGL/Widget.hpp"
#endif

/* proto */
#include "../graphic/proto/graphic_proto.h"

#ifdef USE_SHADERS
 #include "../graphic/proto/g3d_extensions_proto.h"

 //! array of OpenGL IDs of the existing shader programs:
 extern GLuint G3D_PROGRAMS[100];

 //! number of existing shaders:
 extern unsigned int G3D_NB_PROGRAMS;

 //! index (in the array 'programs') of the current program (the one that is used now):
 extern unsigned int G3D_CURRENT_PROGRAM;
#endif


#endif /* #ifndef GRAPHIC_PKG_H */
