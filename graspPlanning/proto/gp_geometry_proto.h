#ifndef GP_GEOMETRY_PROTO_H
#define GP_GEOMETRY_PROTO_H

extern int gpLine_triangle_intersection( p3d_vector3 c1, p3d_vector3 c2, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_vector3 intersection);

extern int gpRay_triangle_intersection( p3d_vector3 origin, p3d_vector3 direction, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_vector3 intersection);

extern int gpLine_segment_plane_intersection( p3d_plane plane, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 result);

extern double gpPoint_to_line_segment_distance(p3d_vector3 p, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 closestPoint);

extern int gpTriangle_plane_intersection( p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_plane plane, p3d_vector3 result1, p3d_vector3 result2);

extern int gpCheck_triangle_plane_intersection( p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_plane plane);

extern int gpPlane_plane_intersection( p3d_plane *plane1, p3d_plane *plane2, p3d_vector3 point_on_line, p3d_vector3 line_direction);

extern p3d_plane gpPlane_from_point_and_normal(p3d_vector3 p, p3d_vector3 normal);

extern p3d_plane gpTransform_plane_equation(p3d_matrix4 T, p3d_plane plane);

extern int gpLine_segment_sphere_intersection( p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 center, double radius, p3d_vector3 result1, p3d_vector3 result2);

extern void gpPoint_in_triangle_from_parameters( double alpha, double beta, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, p3d_vector3 result);

extern int gpParameters_in_triangle_from_point( p3d_vector3 p, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, double *alpha, double *beta);

extern void gpOrthogonal_projection_point_onto_plane( p3d_vector3 point, p3d_plane plane, p3d_vector3 result);

extern void gpOrthogonal_vector( p3d_vector3 v, p3d_vector3 result);

extern void gpOrthonormal_basis( p3d_vector3 u, p3d_vector3 v, p3d_vector3 w);

extern double gpPoint_to_triangle_distance( p3d_vector3 point, p3d_vector3 p0, p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 closestPoint);

extern int gpPoint_to_polyhedron_distance2(p3d_vector3 point, p3d_polyhedre *polyhedron, double &distance, p3d_vector3 closestPoint);

extern void gpDraw_plane( p3d_plane plane);

extern void gpDraw_plane(p3d_vector3 normal, double offset, double d);

extern int gpLine_line_intersection2D( double point1[2], double direction1[2], double point2[2], double direction2[2], double result[2]);

extern int gpSegment_segment_intersection2D( double a1[2], double b1[2], double a2[2], double b2[2], double result1[2], double result2[2]);

extern int gpIs_point_on_segment2D( double p[2], double a[2], double b[2]);

extern double gpTriangle_area( p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3);

extern double gpPolygon_area( double(*vertices)[2], int nb_vertices);

extern int gpSave_polygon( char *name, double(*vertices)[2], int nb_vertices);

extern int gpIs_polygon_simple( double(*vertices)[2], int nb_vertices);

extern int gpIs_polygon_ccw( double(*vertices)[2], int nb_vertices);

extern int gpIs_point_in_polygon( double point[2], double(*vertices)[2], int nb_vertices);

extern int gpPolygon_polygon_inclusion( double(*vertices1)[2], int nb_vertices1, double(*vertices2)[2], int nb_vertices2);

extern double gpTetrahedron_volume(p3d_vector3 a, p3d_vector3 b, p3d_vector3 c, p3d_vector3 d);

extern void gpSpherical_edge_projection(p3d_vector3 x1, p3d_vector3 x2, double a, p3d_vector3 v);

extern int gpSample_sphere_surface(double radius, unsigned int nb_samples, std::vector<gpVector3D> &samples);

extern p3d_vector3 * gpSample_triangle_surface(p3d_vector3 p1, p3d_vector3 p2, p3d_vector3 p3, double step, unsigned int *nb_samples)
;

extern int gpIs_point_in_triangle(p3d_vector3 point, p3d_vector3 a, p3d_vector3 b, p3d_vector3 c);

extern int gpPoint_to_polyhedron_distance(p3d_vector3 point, p3d_polyhedre *polyhedron, double &distance, p3d_vector3 closestPoint);

extern int gpSample_polyhedron_AABB(p3d_polyhedre *polyhedron, double step, std::vector<gpVector3D> &samples);

#endif
