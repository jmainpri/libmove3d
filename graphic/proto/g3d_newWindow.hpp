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
#ifndef QTG3DWINDOW_HPP
#define QTG3DWINDOW_HPP

#include "device.h"

typedef struct g3d_win G3D_Window;

typedef struct g3d_cam_param 
{
	p3d_vector3 Xc;
	p3d_vector3 Xw;
	p3d_vector3 up;
} g3d_cam_param;

// G3D window class that holds the 
// 3d window structs
class qtG3DWindow
{
public:
	qtG3DWindow();
	
private:
	void newG3dWindow();
	double size;
};

// Funtions to be used 
// in the higher level modules
void calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw);
void qt_canvas_viewing(int mouse_press, int button);
void qt_change_mob_frame(G3D_Window* win,pp3d_matrix4 frame);
void qt_reset_mob_frame(G3D_Window* win);
void init_all_draw_functions_dummy();

G3D_Window * qt_get_cur_g3d_win();

// Function pointers 
// to external drawing functionalities
extern void (*ext_g3d_calc_cam_param)(g3d_cam_param& p);
extern void (*ext_g3d_get_win_mouse)(int* x, int* y);
extern void (*ext_g3d_add_traj_to_ui)(char* name,int i, p3d_rob* rob , p3d_traj* traj);
extern void (*ext_g3d_add_config_to_ui)(char* name,p3d_rob* rob,double* q);


#endif // QTG3DWINDOW_HPP
