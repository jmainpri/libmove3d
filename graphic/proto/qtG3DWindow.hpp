#ifndef QTG3DWINDOW_HPP
#define QTG3DWINDOW_HPP

#include "p3d_matrix.h"

typedef struct g3d_win G3D_Window;

//void qt_calc_cam_param();
void calc_cam_param(G3D_Window *win, p3d_vector4 Xc, p3d_vector4 Xw);

extern void (*ext_g3d_draw_allwin_active)();
extern void (*ext_calc_cam_param)();
extern void (*ext_get_win_mouse)(int* x, int* y);

class qtG3DWindow
{
public:
    qtG3DWindow();

private:
    void newG3dWindow();
		double size;
};

#endif // QTG3DWINDOW_HPP
