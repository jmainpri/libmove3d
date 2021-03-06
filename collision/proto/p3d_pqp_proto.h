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


#ifndef P3D_PQP_PROTO_H
#define P3D_PQP_PROTO_H

#include <../collision/PQP/include/p3d_pqp.h>

extern void pqp_set_collision_message(unsigned int set);

extern void pqp_set_tolerance_value(double value);

extern void pqp_get_tolerance_value(double* value);

extern void pqp_set_tolerance_flag(bool value);

extern bool pqp_get_tolerance_flag(void);

extern int pqp_update_BB(p3d_obj* obj);

extern int pqp_get_obj_pos(p3d_obj *o, p3d_matrix4 pose);

extern int pqp_set_obj_pos(p3d_obj *o, p3d_matrix4 pose, unsigned int update_graphics);

extern p3d_obj *pqp_get_previous_body(p3d_obj *body);

extern int pqp_create_pqpModel(p3d_obj *obj);

extern void p3d_start_pqp();

extern int pqp_check_collision_pair_validity();

extern int pqp_create_collision_pairs();

extern int pqp_is_pure_graphic(p3d_obj* obj);

extern int pqp_is_pair_always_inactive(p3d_obj* o1, p3d_obj* o2);

extern int pqp_print_collision_pairs();

extern int pqp_fprint_collision_pairs(char *filename);

extern int pqp_is_collision_pair_activated(p3d_obj *o1, p3d_obj *o2);

extern void p3d_end_pqp();

extern double pqp_triangle_area(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3);

extern int pqp_is_face_degenerate(p3d_polyhedre *polyhedron, unsigned int face_index);

extern int pqp_activate_object_collision(p3d_obj *obj);

extern int pqp_deactivate_object_collision(p3d_obj *obj);

extern int pqp_activate_object_object_collision(p3d_obj *o1, p3d_obj *o2);

extern int pqp_deactivate_object_object_collision(p3d_obj *o1, p3d_obj *o2);

extern int pqp_activate_robot_robot_collision(p3d_rob *robot1, p3d_rob *robot2);

extern int pqp_deactivate_robot_robot_collision(p3d_rob *robot1, p3d_rob *robot2);

extern int pqp_activate_robot_environment_collision(p3d_rob *robot);

extern int pqp_deactivate_robot_environment_collision(p3d_rob *robot);

extern int pqp_activate_robot_object_collision(p3d_rob *rob, p3d_obj *obj);

extern int pqp_deactivate_robot_object_collision(p3d_rob *rob, p3d_obj *obj);

extern int pqp_activate_robot_selfcollision(p3d_rob *rob);

extern int pqp_deactivate_robot_selfcollision(p3d_rob *rob);

extern int pqp_activate_robot_collision(p3d_rob *rob);

extern int pqp_deactivate_robot_collision(p3d_rob *rob);

extern int pqp_activate_object_environment_collision(p3d_obj *obj);

extern int pqp_deactivate_object_environment_collision(p3d_obj *obj);

extern int pqp_activate_all_collisions();

extern int pqp_deactivate_all_collisions();

extern pqp_triangle* pqp_triangulate_face(p3d_polyhedre *polyhedron, unsigned int face_index, unsigned int *nb_triangles);

extern int pqp_is_point_in_triangle(p3d_vector2 p, p3d_vector2 a, p3d_vector2 b, p3d_vector2 c);

extern pqp_triangle* pqp_triangulate_polygon(p3d_vector2 *vertices, int nb_vertices, unsigned int *nb_triangles);

extern int pqp_draw_triangle(p3d_obj *object, unsigned int index, double red, double green, double blue);

extern int pqp_draw_model(p3d_obj *object, double red, double green, double blue);

extern void pqp_draw_all_models();

extern int pqp_get_OBB_first_level(p3d_obj *object, double box[8][3] );

extern void pqp_draw_OBB_first_level(p3d_obj *object );

extern void pqp_draw_OBBs_recursive(p3d_obj *object, double M[4][4], int bn, int currentLevel, int level);

extern int pqp_draw_OBBs(p3d_obj *object, int level);

extern void pqp_draw_all_OBBs(int level, int opengl_context);

extern int pqp_top_OBB(p3d_obj *object, double &tx, double &ty, double &tz, double &ax, double &ay, double &az, double &xmin, double &xmax, double &ymin, double &ymax, double &zmin, double &zmax);

extern void pqp_save_model(p3d_obj *object, const char *filename);

extern void pqp_orthogonal_projection_point_onto_plane(p3d_vector3 point, poly_plane plane, p3d_vector3 result);

extern void pqp_orthogonal_vector(p3d_vector3 v, p3d_vector3 result);

extern void pqp_orthonormal_basis(p3d_vector3 u, p3d_vector3 v, p3d_vector3 w);

extern int pqp_collision_test(p3d_obj *o1, p3d_obj *o2);

extern double pqp_distance(p3d_obj *o1, p3d_obj *o2, p3d_vector3 closest_point1, p3d_vector3 closest_point2);

extern int pqp_robot_selfcollision_test(p3d_rob *robot);

extern int pqp_robot_environment_collision_test(p3d_rob *robot);

extern int pqp_robot_robot_collision_test(p3d_rob *robot1, p3d_rob *robot2);

extern int pqp_robot_robot_collision_test_without_contact_surface(p3d_rob *robot1, p3d_rob *robot2);

extern int pqp_robot_obj_collision_test(p3d_rob *robot, p3d_obj *obj);

extern int pqp_obj_environment_collision_test(p3d_obj *obj);

extern int pqp_robot_all_no_self_collision_test(p3d_rob *robot);

extern int pqp_robot_all_collision_test(p3d_rob *robot);

extern int pqp_all_collision_test();

extern double pqp_robot_environment_distance(p3d_rob *robot, p3d_vector3 closest_point_rob, p3d_vector3 closest_point_obst);

extern double pqp_robot_robot_distance(p3d_rob *robot1, p3d_rob *robot2, p3d_vector3 closest_point_rob1, p3d_vector3 closest_point_rob2);

extern double pqp_robot_robot_weighted_distance(p3d_rob *robot1, p3d_rob *robot2);

extern int pqp_tolerance(p3d_obj *o1, p3d_obj *o2, double tolerance);

extern int pqp_colliding_pair(p3d_obj **o1, p3d_obj **o2);

extern int pqp_print_colliding_pair();

extern int pqp_colliding_obj_name_pair(char obj_1_name[200], char obj_2_name[200]);//AKP
#endif


