#ifndef _P3D_POLY_H
#define _P3D_POLY_H

/**********************************************************/
/* Fichier de prototypes pour la creation de polyedre     */
/**********************************************************/

/* pour faire l interface entre p3d et polyhedre */
/*#include "polyhedre.h" */

#define p3d_error_value poly_error_value

#define p3d_index poly_index
#define p3d_plane poly_plane
#define p3d_edge_g poly_edge
#define p3d_face poly_face
#define p3d_polyhedre poly_polyhedre

#define p3d_f_2_v3 poly_f_2_v3
#define p3d_d_2_v3 poly_d_2_v3
#define p3d_v3_2_f poly_v3_2_f
#define p3d_v3_2_d poly_v3_2_d
#define p3d_plane_2_d poly_plane_2_d

#define p3d_show_error_on_shell poly_show_error_on_shell
#define p3d_get_error_value poly_get_error_value
#define p3d_error poly_error

#define p3d_init_poly poly_init_poly
#define p3d_create_poly poly_create_poly
#define p3d_destroy_poly poly_destroy_poly
#define p3d_destroy_edges poly_destroy_edges
#define p3d_destroy_planes poly_destroy_planes

#define p3d_get_name poly_get_name
#define p3d_get_nb_points poly_get_nb_points
#define p3d_get_nb_points_in_face poly_get_nb_points_in_face
#define p3d_get_nb_faces poly_get_nb_faces
#define p3d_get_nb_edges_in_face poly_get_nb_edges_in_face
#define p3d_get_nb_edges poly_get_nb_edges
#define p3d_get_index_point_in_face poly_get_index_point_in_face
#define p3d_get_edge_points poly_get_edge_points
#define p3d_get_edge_faces poly_get_edge_faces
#define p3d_get_plane poly_get_plane
#define p3d_get_plane_2_d poly_get_plane_2_d
#define p3d_get_point_2_v3 poly_get_point_2_v3
#define p3d_get_point_2_d poly_get_point_2_d
#define p3d_get_poly_pos poly_get_poly_pos
#define p3d_get_point_in_pos_in_face poly_get_point_in_pos_in_face
#define p3d_get_point_in_edge poly_get_point_in_edge
#define p3d_get_poly_mat poly_get_poly_mat

#define p3d_find_point poly_find_point
#define p3d_find_pos_in_face poly_find_pos_in_face
#define p3d_find_edge poly_find_edge

#define p3d_add_point poly_add_point
#define p3d_pos_poly_by_mat poly_pos_poly_by_mat
#define p3d_scale_poly poly_scale_poly
#define p3d_set_poly_pos poly_set_poly_pos

#define p3d_build_edges poly_build_edges
#define p3d_build_plane_face poly_build_plane_face
#define p3d_build_planes poly_build_planes
#define p3d_build_face poly_build_face
#define p3d_compute_poly_BB poly_compute_poly_BB 


/* fonctions propres a p3d */
/*#include "p3d.h" */

#endif
