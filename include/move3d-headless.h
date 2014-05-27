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
 *  directory move3d and the files defining the structure appearing in 
 *  these prototypes.
 */

#ifndef MOVE3D_PKG_H
#define MOVE3D_PKG_H

// Compile without graphical interface
//#ifndef WITH_XFORMS

#include "P3d-pkg.h"
// Functions Without XFORMS
extern int fct_stop(void);
extern void fct_draw(void);
extern int g3d_get_KCD_CHOICE_IS_ACTIVE();
extern int p3d_get_user_drawnjnt(void);
extern void p3d_set_user_drawnjnt(int jnt);
extern p3d_traj *p3d_graph_to_traj ( p3d_rob *robotPt );
extern void g3d_add_traj ( char *name, int i , p3d_rob* rob = NULL , p3d_traj* traj = NULL );
extern void g3d_add_config_to_ui( char *name, p3d_rob *robotPt , configPt q );

#ifdef WITH_XFORMS
//TODO: these prototypes belongs to move3d-studio. They correspond to GUI code, and should
// be removed from the move3d library
//#include "move3d.h"

extern void MovieDrawGraph(); // required by graphic/g3d_draw_graph.c
extern void CB_DiffusionMethod_obj(FL_OBJECT *obj, long arg); // required by lightPlanner/lightPlanner.c
extern void FORMrobot_update ( int ir ); // required by lightPlanner/ManipulationPlanner.cpp
extern void CB_del_param_obj ( FL_OBJECT *ob, long arg ); // required by planner/p3d_graph.c
extern void p3d_printTrajGraphContactPdbFiles(char* filePrefix, int index, p3d_rob *robotPt); // required by planner/p3d_graph.c
extern int p3d_get_user_drawnjnt ( void ); // required by planner/p3d_NodeWeight.c

#endif

//#endif

#endif /* #ifndef MOVE3D_PKG_H */
