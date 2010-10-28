/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */


extern void g3d_set_draw_coll ( int n );
extern void g3d_reinit_graphics ( void );
extern void g3d_draw ( void );
extern void g3d_draw_obstacles ( G3D_Window* win );
extern void g3d_draw_robots ( G3D_Window *win );
extern void g3d_draw_env_box ( void );
extern void g3d_draw_robot ( int ir, G3D_Window* win );
extern void g3d_draw_env(void);
extern void g3d_draw_env_custom(void);
extern void g3d_draw_obstacle(G3D_Window *win);
extern void g3d_draw_body(int coll, G3D_Window *win);
extern void g3d_draw_object(p3d_obj *o, int coll, G3D_Window *win);
extern void p3d_drawRobotMoveMeshs(void);
extern void g3d_draw_obj_BB(p3d_obj *o);
extern int compute_wall_dimensions(double *_size, double *_xmin, double *_xmax, double *_ymin, double *_ymax, double *_zmin, double *_zmax);
extern void buildShadowMatrix( GLdouble fMatrix[16], GLfloat fLightPos[4], GLdouble fPlane[4] );
extern int g3d_draw_tiled_floor(GLdouble color[3], float dx, float dy, float xmin, float xmax, float ymin, float ymax, float zmin, float zmax, float shadowContrast);
extern void g3d_draw_floor(GLdouble color[3], int tiles) ;
extern void g3d_draw_wall(int wall, GLdouble color[3], int quadsPerEdge) ;
extern void g3d_draw_backwall(int wall);
extern void g3d_draw_and_col_allwin_active(void);
extern void showConfig(configPt conf);
extern void showConfig_2(configPt conf);
extern int g3d_does_robot_hide_object(p3d_matrix4 camera_frame, double camera_fov, p3d_rob *robot, p3d_rob *object, double *result);
