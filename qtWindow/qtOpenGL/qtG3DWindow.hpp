#ifndef QTG3DWINDOW_HPP
#define QTG3DWINDOW_HPP

typedef struct g3d_win G3D_Window;

#include "Graphic-pkg.h"

void qt_calc_cam_param();

class qtG3DWindow
{
public:
    qtG3DWindow();

private:
    void newG3dWindow();
	double size;
};

#endif // QTG3DWINDOW_HPP
