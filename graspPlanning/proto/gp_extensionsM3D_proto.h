#ifndef GP_EXTENSIONS_M3D_PROTO_H
#define GP_EXTENSIONS_M3D_PROTO_H

#include "Graphic-pkg.h"

extern void logfile ( const char *format, ... );


extern int p3d_get_obj_pos(p3d_obj *o, p3d_matrix4 pose);

extern int p3d_save_in_OBJ_format ( p3d_polyhedre *polyhedron, char *name );

extern p3d_polyhedre * p3d_copy_polyhedre ( p3d_polyhedre *polyhedron );

extern int p3d_compute_vertex_normals(p3d_polyhedre *polyhedron);

extern int p3d_print_face_neighbours(p3d_polyhedre *polyhedron, char *filename);

extern void p3d_matrix4_to_OpenGL_format(p3d_matrix4 source, GLfloat dest[16]);

extern void Gb_th_matrix4(Gb_th *th, p3d_matrix4 mat);

extern void Gb_matrix4_th(p3d_matrix4 mat, Gb_th *th);

extern int solve_trigonometric_equation(double a, double b, double c, double *x1, double *x2);

extern void draw_frame(p3d_matrix4 frame, double length);

extern void draw_frame0(p3d_matrix4 frame, double length);

extern int export_scene_to_POVRAY(char *foldername, char *filename);

extern int export_p3d_polyhedre_to_POVRAY(p3d_polyhedre *polyhedron, char *filename);

extern int export_primitive_to_POVRAY(char *object_name, int entity_type, p3d_primitive *prim, char *filename);

extern int export_p3d_obj_to_POVRAY(p3d_obj *object, char *filename);

extern void get_sample2D(int n, p3d_vector2 origin, double factor, p3d_vector2 result);

extern void get_sample3D(int n, p3d_vector3 origin, double factor, p3d_vector3 result);

extern int gpExport_bodies_for_coldman(p3d_rob *robot);

extern int gpExport_obstacles_for_coldman();

extern int gpPolyhedron_AABB(p3d_polyhedre *poly, double &xmin, double &xmax, double &ymin, double &ymax, double &zmin, double &zmax);

extern int gpObj_AABB(p3d_obj *obj, double &xmin, double &xmax, double &ymin, double &ymax, double &zmin, double &zmax);

extern int gpPrint_robot_AABBs(p3d_rob *robot);

#endif
