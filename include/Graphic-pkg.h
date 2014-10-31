/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
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
#include "g3d_window.h"

#if defined( QT_LIBRARY )
#include "../graphic/proto/g3d_newWindow.hpp"
#include <iostream>
#endif

#include <vector>

/* QT OpenGL*/
extern void g3d_draw_env(int opengl_context);
extern void g3d_draw_object_moved(p3d_obj *o, int coll, G3D_Window* win, int opengl_context);


/* globals */

extern int G3D_DRAW_TRACE;
extern int G3D_DRAW_OCUR_SPECIAL;
extern int boxlist;	/* liste opengl pour la boite materialisant
			   l'environnment */
extern int p3d_numcoll;	/* Variables externes pour le CC */
extern std::vector<p3d_matrix4*> global_FramesToDraw;

#if defined( QT_GL ) && defined( CXX_PLANNER )
#include "../qtWindow/qtOpenGL/Widget.hpp"
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
