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
