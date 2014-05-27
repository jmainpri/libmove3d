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
/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

extern void poly_f_2_v3 ( float x, float y, float z, p3d_vector3 *vector );
extern void poly_d_2_v3 ( double x, double y, double z, p3d_vector3 *vector );
extern void poly_v3_2_f ( p3d_vector3 *vector, float *x, float *y, float *z );
extern void poly_v3_2_d ( p3d_vector3 *vector, double *x, double *y, double *z );
extern void poly_plane_2_d ( poly_plane *plane, double *a, double *b, double *c, double *d );
extern int set_poly_show_error_on_shell ( int value );
extern int poly_get_error_value ( void );
extern int poly_error ( void );
extern void poly_init_poly ( poly_polyhedre *polyhedre, char *name );
extern poly_polyhedre *poly_create_poly ( char *name );
extern void poly_destroy_poly ( poly_polyhedre *polyhedre );
extern void poly_destroy_edges ( poly_polyhedre *polyhedre );
extern void poly_destroy_planes ( poly_polyhedre *polyhedre );
extern char *poly_get_name ( poly_polyhedre *polyhedre );
extern unsigned int poly_get_nb_points ( poly_polyhedre *polyhedre );
extern unsigned int poly_get_nb_points_in_face ( poly_polyhedre *polyhedre, poly_index face );
extern poly_index_p p3d_get_points_in_face ( poly_polyhedre *polyhedre, poly_index face );
extern unsigned int poly_get_nb_faces ( poly_polyhedre *polyhedre );
extern unsigned int poly_get_nb_edges_in_face ( poly_polyhedre *polyhedre, poly_index face );
extern unsigned int poly_get_nb_edges ( poly_polyhedre *polyhedre );
extern int poly_get_index_point_in_face ( poly_polyhedre *polyhedre, poly_index face, poly_index index );
extern int poly_get_edge_points ( poly_polyhedre *polyhedre, poly_index edge, poly_index *p1, poly_index *p2 );
extern int poly_get_edge_faces ( poly_polyhedre *polyhedre, poly_index edge, poly_index *f1, poly_index *f2 );
extern int poly_add_point ( p3d_vector3 point, poly_polyhedre *polyhedre );
extern int poly_find_point ( p3d_vector3 point, poly_polyhedre *polyhedre );
extern int poly_get_point_2_v3 ( poly_polyhedre *polyhedre, poly_index index, p3d_vector3 *vector );
extern int poly_get_point_2_d ( poly_polyhedre *polyhedre, poly_index index, double *x, double *y, double *z );
extern void poly_scale_poly ( poly_polyhedre *polyhedre, double scaleX, double scaleY, double scaleZ );
extern void poly_pos_poly_by_mat ( poly_polyhedre *polyhedre, p3d_matrix4 mat );
extern void poly_get_poly_pos ( poly_polyhedre *polyhedre, p3d_matrix4 mat );
extern p3d_matrix4 *poly_get_poly_mat ( poly_polyhedre *polyhedre );
extern void poly_set_poly_pos ( poly_polyhedre *polyhedre, p3d_matrix4 mat );
extern int poly_find_pos_in_face ( poly_polyhedre *polyhedre, poly_index index, poly_index point );
extern int poly_get_point_in_pos_in_face ( poly_polyhedre *polyhedre, poly_index face, poly_index point, double *x, double *y, double *z );
extern int poly_get_point_in_edge ( poly_polyhedre *polyhedre, poly_index edge, poly_index point, double *x, double *y, double *z );
extern int poly_find_edge ( poly_polyhedre *polyhedre, poly_index p1, poly_index p2 );
extern int poly_build_edge ( poly_polyhedre *polyhedre, poly_index f, poly_index p1, poly_index p2 );
extern int poly_build_edges ( poly_polyhedre *polyhedre );
extern int poly_build_plane_face ( poly_polyhedre *polyhedre, poly_index numero );
extern int poly_get_plane ( poly_polyhedre *polyhedre, poly_index index, poly_plane *plane );
extern int poly_get_plane_2_d ( poly_polyhedre *polyhedre, poly_index index, double *a, double *b, double *c, double *d );
extern int poly_build_planes ( poly_polyhedre *polyhedre );
extern int poly_build_face ( poly_index *the_indexs, unsigned int nombre, poly_polyhedre *polyhedre );
extern int p3d_poly_is_convex ( poly_polyhedre *p );
extern int p3d_poly_facet_is_convex ( poly_polyhedre *p, int f_id );
extern void poly_compute_poly_BB ( poly_polyhedre *p, double *xmin, double *xmax, double *ymin, double *ymax, double *zmin, double *zmax );

extern p3d_triangle* p3d_triangulate_polygon(p3d_vector2 *vertices, int nb_vertices, unsigned int *nb_triangles);
extern p3d_triangle* p3d_triangulate_face(poly_index *the_indexs, unsigned int nb_points, poly_polyhedre *polyhedron, unsigned int *nb_triangles);
extern int p3d_compute_face_areas_and_centroids(poly_polyhedre *poly);
extern int p3d_compute_area(poly_polyhedre *poly, double &area);
extern int p3d_compute_edges_and_face_neighbours(poly_polyhedre *polyhedron);
extern int p3d_compute_vertex_normals(poly_polyhedre *poly);
extern int p3d_export_as_OFF(poly_polyhedre *poly);

extern double p3d_triangle_area(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3);
extern int p3d_compute_poly_centroid(p3d_polyhedre *poly);

#ifdef GRASP_PLANNING
extern int p3d_compute_mean_curvature(p3d_polyhedre *poly);
#endif

#endif /* __CEXTRACT__ */
