


#ifndef __CEXTRACT__

extern G3D_Window *g3d_show_persp_win();
extern void g3d_set_win_draw_mode(G3D_Window *w,g3d_window_draw_mode mode);
extern void g3d_refresh_win2(G3D_Window *w); 
extern void g3d_draw_win2(G3D_Window *win) ;
extern void g3d_set_light_persp(void);
extern int canvas_expose_special(FL_OBJECT *ob, Window win, int w, int h, XEvent *xev, void *ud); 
#endif /* __CEXTRACT__ */
