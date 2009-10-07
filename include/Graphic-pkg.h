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

#include "g3d_window.h"
#include "forms.h"

extern void g3d_draw_env();
extern void g3d_draw_object_moved(p3d_obj *o, int coll, G3D_Window* win);
extern G3D_Window* win_test;


/* globals */

extern int G3D_DRAW_TRACE;
extern int G3D_DRAW_OCUR_SPECIAL;
extern int boxlist;	/* liste opengl pour la boite materialisant
			   l'environnment */
extern int p3d_numcoll;	/* Variables externes pour le CC */

#ifdef CXX_PLANNER
#include "../planning_api/planningAPI.hpp"
class Trajectory;
extern std::vector<Trajectory> trajToDraw;
#endif

#ifdef QT_GL
#include "../qtWindow/qtOpenGL/g3dQtConnection.hpp"
extern Move3D2OpenGl* pipe2openGl;
#endif

/* proto */
#include "../graphic/proto/graphic_proto.h"

#ifdef UNIX
#endif

#endif /* #ifndef GRAPHIC_PKG_H */
