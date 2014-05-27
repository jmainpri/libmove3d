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
#ifndef MOVE3D_H
#define MOVE3D_H

#ifndef PROTO

//#include "glcanvas.h"
#ifdef WITH_XFORMS
	#include "forms.h"
	#include "g3d_states.h"
	#include "g3d_window.h"
#else
	#include "../graphic/proto/g3d_newWindow.hpp"
#endif

#endif

/* taille fenetre graphique definie dans FORMmain.c */
#define G3D_WINSIZE 600
#define G3D_WINSIZE_WIDTH 800
#define G3D_WINSIZE_HEIGHT 600

/* la fenetre graphique move3d initialisee dans FORMmain.c*/
extern G3D_Window *G3D_WIN;

/* col checking actif ou non pour les visualisations , initialise dans FORMmain.c */
extern int        G3D_ACTIVE_CC;

/* Structure des menu forms */
#define MAX_DDLS 3600      // < modif Juan
#define MAX_NJNTS_IN_ROBOTFORM 300  // < modif Juan
#ifdef WITH_XFORMS
typedef struct {
  FL_FORM *ROBOT_FORM;
  FL_OBJECT  *GOTO_OBJ;
  FL_OBJECT  *POSITION_OBJ[MAX_DDLS];
  FL_OBJECT  *RADIUS_OBJ;
  FL_OBJECT  *DMAX_OBJ;
  FL_OBJECT  *DMAX_BUTTON_OBJ;
  FL_OBJECT *g3d_trajnum_obj;
  FL_OBJECT *g3d_trajmove_obj[4];
  int       g3d_trajnum;       /* number of trajectories of this robot */
    /*  int g3d_trajnum_npt;  */
  double pos_on_traj; /* current position on path */
  int PLANNER_CHOICE;
  FL_OBJECT  *BEGTRAJ_OBJ;
  FL_OBJECT  *ENDTRAJ_OBJ;
  FL_OBJECT  *ADDTRAJ_OBJ;
  FL_OBJECT  *SHOWTRAJ_OBJ;
  FL_OBJECT  *MOVIETRAJ_OBJ;
  FL_OBJECT  *WRITEPATH_OBJ;
  /* Debut Modification Thibaut */
  FL_OBJECT  *PRINT_CONFIGURATION_OBJ;
  /* Fin Modification Thibaut */
  /* Debut Modification Fabien */
  FL_OBJECT  *CONFIG_OBJ;
  FL_OBJECT  *NEW_CONFIG_OBJ;
  FL_OBJECT  *DEL_CONFIG_OBJ;
  FL_OBJECT  *SET_CONFIG_OBJ;
  /* Fin Modification Fabien */
  FL_OBJECT  *DISPLAY_FILTERBOX_OBJ;
  FL_OBJECT  *ADAPT_FILTERBOX_OBJ;
  FL_OBJECT  *KINE_CONSTRAINTS_OBJ;
  FL_OBJECT  *LOAD_PATH_OBJ;
  FL_OBJECT  *COMPUTE_OBJ;
  FL_OBJECT  *TRACK_OBJ;
  FL_OBJECT  *SAVE_JOINTMAP_OBJ;
  int        TRACK_TRAJ;
} MENU_ROBOT;

typedef struct {
	FL_FORM     *ROBOT_FORM;
        FL_OBJECT  *BOUNDS_OBJ[MAX_DDLS];
        FL_OBJECT  *CANCEL_OBJ;
        FL_OBJECT  *OK_OBJ;
} MENU_FILTER;

#define MAX_CNTRT_TP 50

typedef struct {
	FL_FORM    *CNTRT_FORM;
        FL_OBJECT  *LIST_OBJ[MAX_CNTRT_TP];
        FL_OBJECT  *ARROWS_OBJ[MAX_CNTRT_TP];
        FL_OBJECT  *NEW_OBJ;
        FL_OBJECT  *OK_OBJ;
        FL_OBJECT  *RLG_OBJ;
} MENU_CONSTRAINTS;

typedef struct {
	FL_FORM    *CNTRT_SET_FORM;
        FL_OBJECT  *LIST_OBJ[MAX_CNTRT_TP];
        FL_OBJECT  *DONE_OBJ;
} MENU_CONSTRAINTS_SETTING;
#endif


#endif
