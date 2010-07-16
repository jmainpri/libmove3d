#ifndef QTG3DWINDOW_HPP
#define QTG3DWINDOW_HPP

typedef struct g3d_win G3D_Window;

#include "Graphic-pkg.h"

struct g3d_win 
{
	char       name[256];
	void       *form;
	void       *canvas;
	void       *mcamera_but;
	void       (*fct_draw)(void);
	pp3d_matrix4  cam_frame;
	pp3d_matrix4 (*fct_mobcam)(void);
	
	G3D_Window *next;
	
	g3d_states vs; //! viewer state
	
	//! pointer to an additional display function, that can be called from any source file
	void (*fct_draw2) ();
	
	//! pointer to a function that is called by pressing a key (see g3d_window.c)
	void (*fct_key1) ();
	//! pointer to another function that is called by pressing a key (see g3d_window.c)
	void (*fct_key2) ();
	
#ifdef HRI_PLANNER
	int point_of_view;                    /* Boolean for  another perspective */ 
	int win_perspective;                  /* Boolean to know if it is a perspective window */    
	g3d_window_draw_mode draw_mode;       /* Boolean to know if we'll draw only the objective or in a different color from obstacles */
#endif
};

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
