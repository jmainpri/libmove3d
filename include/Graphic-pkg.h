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
// ----------------------------------------------------------
extern void qt_canvas_viewing(int mouse_press,int button);
extern void qt_get_win_mouse(int* i, int *j);
extern void qt_get_fl_win();
extern void qt_calc_cam_param();
extern void qt_change_mob_frame(G3D_Window* win,pp3d_matrix4 frame);
extern void qt_reset_mob_frame(G3D_Window* win);
extern G3D_Window * qt_get_cur_g3d_win();

extern p3d_vector4 JimXc;
extern p3d_vector4 JimXw;
extern p3d_vector4 Jimup;

#include <QtCore/QWaitCondition>
#include <QtCore/QMutex>

extern QWaitCondition* waitDrawAllWin;
extern QMutex* lockDrawAllWin;

#include "qtWindow/qtOpenGL/g3dQtConnection.hpp"

extern Move3D2OpenGl* pipe2openGl;
// ----------------------------------------------------------
#endif

/* proto */
#include "../graphic/proto/graphic_proto.h"

#ifdef UNIX
#endif

#endif /* #ifndef GRAPHIC_PKG_H */
