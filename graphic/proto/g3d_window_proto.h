/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

extern G3D_Window *g3d_new_win ( const char *name, int w, int h, float size );
extern G3D_Window *g3d_new_win_wo_buttons( char *name, int w, int h, float size ); 
extern void g3d_del_win ( G3D_Window *win );
extern int g3d_win_id ( G3D_Window *win );
extern void g3d_refresh_allwin_active ( void );
extern void g3d_event_win ( G3D_Window *g3dwin, int event, int xpos, int ypos, void* data );
extern void g3d_set_light ( void );
extern void g3d_draw_allwin ( void );
extern void g3d_draw_allwin_active ( void );
extern void g3d_print_allwin ( void );
extern void g3d_resize_win ( G3D_Window *win, float w, float h, float size );
extern void g3d_resize_allwin_active ( float w, float h, float size );
extern void g3d_set_win_bgcolor ( G3D_Window *win, float r, float v, float b );
extern void g3d_set_win_floor_color ( G3D_Window *win, float r, float v, float b );
extern void g3d_set_win_camera ( G3D_Window *win, float ox, float oy, float oz, float dist, float az, float el, float up0, float up1, float up2 );
extern void g3d_set_win_center ( G3D_Window *win, float ox, float oy, float oz );
extern void g3d_save_win_camera ( G3D_Window *win );
extern void g3d_load_saved_camera_params(double* params);
extern void g3d_restore_win_camera ( G3D_Window *win );
extern void g3d_set_win_fct_mobcam ( G3D_Window *win, pp3d_matrix4 (*fct)(void) );
extern void g3d_set_mobile_camera_activate ( G3D_Window *win, int mode );
extern void g3d_set_win_drawer ( G3D_Window *win, void (*fct)(void) );
extern void g3d_init_allwin_booleans ( void );
extern void g3d_beg_poly ( void );
extern void g3d_end_poly ( void );
extern void g3d_draw_frame ( void );
extern G3D_Window *g3d_get_cur_win ( void );
extern G3D_Window *g3d_get_cmc_win ( void );
extern void g3d_build_shadow_matrices(G3D_Window *win);
extern G3D_Window *g3d_get_win_by_name(char *s);
extern double g3d_get_light_factor(void);
extern void g3d_set_picking(unsigned int enabled);
extern int g3d_export_GL_display(char *filename);
#endif /* __CEXTRACT__ */
