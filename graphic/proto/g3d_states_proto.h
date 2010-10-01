#ifndef G3D_STATES_PROTO_H
#define G3D_STATES_PROTO_H

#include "g3d_states.h"

g3d_states g3d_init_viewer_state(double size);
int g3d_free_viewer_state(g3d_states vs);
void g3d_findPlane( GLdouble plane[4], GLdouble v0[3], GLdouble v1[3], GLdouble v2[3] );
extern void g3d_build_shadow_matrices(g3d_states &vs);
void g3d_set_win_bgcolor(g3d_states &vs, float r, float v, float b);
void g3d_set_win_floor_color(g3d_states &vs, float r, float v, float b);
void g3d_set_win_wall_color(g3d_states &vs, float r, float v, float b);
void g3d_set_win_camera(g3d_states &vs, float ox,float oy, float oz,
                   float dist, float az, float el,
                   float up0, float up1, float up2);
void g3d_print_win_camera(g3d_states &vs);
void g3d_set_win_center(g3d_states &vs, float ox,float oy, float oz);
void g3d_set_light (g3d_states &vs);
void g3d_save_win_camera(g3d_states &vs);
void g3d_restore_win_camera(g3d_states &vs);
void get_lookat_vector(g3d_states &vs, p3d_vector4 Vec);
void get_pos_cam_matrix(g3d_states &vs, p3d_matrix4 Transf);
void g3d_move_win_camera_forward(g3d_states &vs, float d );
void g3d_move_win_camera_sideways(g3d_states &vs, float d );
void g3d_rotate_win_camera_rz(g3d_states &vs, float d );
void g3d_zoom_win_camera(g3d_states &vs, float d );
void recalc_cam_up(g3d_states &vs, p3d_matrix4 transf);
void recalc_mouse_param(g3d_states &vs, p3d_vector4 Xc, p3d_vector4 Xw);
void g3d_init_OpenGL();
int g3d_load_next_shader();
int g3d_no_shader();
int g3d_use_shader();
void g3d_set_projection_matrix(g3d_projection_mode mode);
int g3d_export_OpenGL_display(char *filename);
void g3d_set_dim_light();
void g3d_set_default_material();
void g3d_set_shade_material();
void g3d_draw_frame(void);

extern void g3d_load_saved_camera_params(double* params);
extern void g3d_restore_win_camera ( g3d_states &vs );
extern int g3d_set_camera_parameters_from_frame(p3d_matrix4 frame, g3d_states &vs);
extern int g3d_save_state(g3d_win *win, g3d_states *st);
extern int g3d_load_state(g3d_win *win, g3d_states *st);
extern int g3d_checkGLerrors(char *message);

int g3d_load_logo_texture(g3d_states &vs);
int g3d_display_logo(g3d_states &vs, float offsetX, float offsetY, float scale);

#endif // G3D_STATES_PROTO_H
