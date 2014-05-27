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
#ifndef _G3D_WIN_H
#define _G3D_WIN_H

typedef struct g3d_win G3D_Window;

struct g3d_win {
  char       name[256];
  void       *form;
  void       *canvas;
  void       *mcamera_but;
  void       (*fct_draw)(int);
  pp3d_matrix4 cam_frame;
  pp3d_matrix4 (*fct_mobcam)(void);

  G3D_Window *next;

  g3d_states vs; //! viewer state

  //! pointer to an additional display function, that can be called from any source file
  void (*fct_draw2) ();

  //! pointer to a function that is called by pressing a key (see g3d_window.c)
  void (*fct_key1) ();
  //! pointer to another function that is called by pressing a key (see g3d_window.c)
  void (*fct_key2) ();

// TODO: Because of the removal of hri_planner from BioMove3D (to libhri), this flag is problematic:
// - libmove3d p3d_rob struct is compiled without this part
// - libhri/move3d-studio use this part, resulting in structs that have different sizes.
//#ifdef HRI_PLANNER_GUI
//
  int point_of_view;                    /* Boolean for  another perspective */ 
  int win_perspective;                  /* Boolean to know if it is a perspective window */    
  g3d_window_draw_mode draw_mode;       /* Boolean to know if we'll draw only the objective or in a different color from obstacles */
//#endif

};

// Funtion when not drawing, to be assigned to 
// all function pointers that are defined outside libmove3d
void dummy_void();

// Function pointers 
// to external drawing functionalities
extern void (*ext_g3d_export_cpp_graph)();
extern void (*ext_g3d_draw_cost_features)();
extern void (*ext_g3d_draw_allwin_active)();
extern void (*ext_g3d_draw_hri_features)();
extern void (*ext_g3d_draw_remote)();
extern void (*ext_g3d_draw_multi_thread)();

#endif
