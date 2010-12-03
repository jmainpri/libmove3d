#ifndef QTG3DWINDOW_HPP
#define QTG3DWINDOW_HPP

#include "p3d_matrix.h"

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

G3D_Window * qt_get_cur_g3d_win();

// Function pointer 
// to external functionalities
extern void (*ext_g3d_export_cpp_graph)();
extern void (*ext_g3d_draw_allwin_active)();
extern void (*ext_calc_cam_param)(g3d_cam_param& p);
extern void (*ext_get_win_mouse)(int* x, int* y);
extern void (*ext_g3d_draw_cost_features)();
extern void (*ext_qt_add_traj)(char* name,int i);

#endif // QTG3DWINDOW_HPP
