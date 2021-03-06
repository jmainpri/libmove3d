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

extern void p3d_to_gl_matrix(p3d_matrix4 T, GLfloat mat[16]);
extern void g3d_draw_text(char* string);
extern void g3d_set_color_vect ( int color, GLdouble color_vect[4] );
extern void g3d_get_color_vect(int color, GLdouble color_vect[4]);
extern void g3d_set_color ( int color, GLdouble color_vect[4] );
extern void g3d_set_custom_color_draw(p3d_rob* r, int draw_custom);
extern void g3d_set_custom_color_vect(GLdouble color_vect[4]);
extern void g3d_get_custom_color_vect(GLdouble color_vect[4]);
extern void g3d_drawDisc ( double x, double y, double z, float r, int color, double color_vect[4] );
extern void g3d_drawSphere ( double x, double y, double z, float r);
extern void g3d_drawColorSphere ( double x, double y, double z, float r, int color, double color_vect[4] );
extern void g3d_drawCircle ( double x, double y, double r, int color, double *color_vect, double width);
extern void g3d_drawOneLine ( double x1, double y1, double z1, double x2, double y2, double z2, int color, double *color_vect );
extern void g3d_draw_rep_obj ( p3d_jnt *jnt, double a, int num );
extern void g3d_draw_simple_box(double x1,double x2,double y1,double y2,double z1,double z2, int color, int fill, double width);
extern void g3d_draw_complex_black_box(double p2[8][3]);
extern void g3d_draw_a_box ( double x1, double x2, double y1, double y2, double z1, double z2, int c, int fill );
extern void g3d_draw_rectangle(float bottomLeftCornerX, float bottomLeftCornerY, float z, float dimX, float dimY);
extern void g3d_draw_tesselated_rectangle(float bottomLeftCornerX, float bottomLeftCornerY, float z, float dimX, float dimY, float delta);
extern void g3d_draw_AA_box(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
extern int BoxInFrustum_obj ( p3d_obj *o, G3D_Window *win );
extern int SphereInFrustum ( G3D_Window *win, float x, float y, float z, float radius );
extern void g3d_extract_frustum ( G3D_Window *win );
extern int g3d_calcule_resolution ( G3D_Window *win, p3d_poly *p );


extern void g3d_draw_primitive ( G3D_Window *win, p3d_poly *p, int fill , int opengl_context);
extern void g3d_draw_poly ( p3d_poly *p, G3D_Window *win, int coll, int fill , int opengl_context);
extern void g3d_draw_poly_with_color(p3d_poly *p,G3D_Window *win,int coll,int fill,double color, int opengl_context);
extern void g3d_draw_poly_special ( p3d_poly *p, G3D_Window *win, int color, int opengl_context );
extern void g3d_init_all_poly_gouraud ( int opengl_context );
extern void g3d_init_all_poly ( int opengl_context );
extern void g3d_delete_all_poly ( int mode, int opengl_context );
extern void GLUerrorCallback ( GLenum errorCode );

extern void g3d_init_box2 ( p3d_poly *p, int fill , int opengl_context);
extern void g3d_init_box ( p3d_poly* p, int fill , int opengl_context);
extern void g3d_init_cylindreGLU2 ( p3d_poly* p, int fill, float base_radius, float top_radius , int opengl_context);
extern void g3d_init_cylindreGLU ( p3d_poly* p, int fill, float base_radius, float top_radius , int opengl_context);
extern void g3d_init_sphereGLU ( p3d_poly* p, int fill , int opengl_context);
extern void g3d_init_polyquelconque ( p3d_poly *p, int fill , int opengl_context);
extern void g3d_init_poly ( p3d_poly *p, int fill , int opengl_context);
extern void g3d_delete_poly ( p3d_poly *p, int mode , int opengl_context);

extern void g3d_draw_solid_cone(double radius, double height, int nbSegments);
extern void g3d_draw_arrow(p3d_vector3 p1, p3d_vector3 p2, double red, double green, double blue);
extern void g3d_drawRepMoveObj(p3d_matrix4 frame ,double length, int axis);
extern void g3d_drawSphMoveObj(p3d_matrix4 mat ,double length);
extern int g3d_lineLineIntersect( p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_vector3 p4, p3d_vector3 *pa, p3d_vector3 *pb, double *mua, double *mub);
extern void g3d_draw_frame(p3d_matrix4 frame, double length);
extern int g3d_draw_cylinder(p3d_vector3 p1, p3d_vector3 p2, double radius, unsigned int nbSegments);
extern void g3d_rgb_from_int(int i, double color[4]);
extern void g3d_rgb_from_hue(double x, double color[4]);
extern int g3d_draw_robot_joints(p3d_rob* robot, double size );
extern int g3d_draw_robot_kinematic_chain(p3d_rob* robot);
extern int g3d_draw_p3d_polyhedre ( p3d_polyhedre *polyhedron );
extern int g3d_circle_table(double **sint, double **cost, const int n);
extern void g3d_draw_solid_sphere(double radius, int nbSegments);
extern void g3d_draw_solid_sphere(double x, double y, double z, double radius, int nbSegments);
extern void g3d_draw_solid_cylinder(double radius, double length, int nbSegments);
extern void g3d_draw_solid_cylinder(double x, double y, double radius, double length, int nbSegments);
extern int g3d_draw_body_normals(p3d_obj *obj, double length);
extern int g3d_draw_robot_normals(p3d_rob *robot, double length);
extern int g3d_draw_body_vertex_normals(p3d_obj *obj, double length);
extern int g3d_draw_robot_vertex_normals(p3d_rob *robot, double length);
extern int g3d_is_poly_transparent(p3d_poly *p);
extern void g3d_draw_ellipsoid(double a, double b, double c, int nbSegments);
extern void g3d_draw_wire_ellipsoid(double a, double b, double c);
extern int g3d_draw_collision_cloud();
extern int g3d_draw_poly_curvature(p3d_polyhedre *poly);
