/*
 *   This file was automatically generated by version 1.7 of cextract.
 *   Manual editing not recommended.

 */
#ifndef __CEXTRACT__

extern void set_DO_KCD_GJK ( int val );
extern void kcd_assign_tolerance ( double val );
extern void kcd_get_tolerance ( double *tol );
extern int kcd_tolerance_is_zero ( void );
extern int obb_disjoint_and_distance ( kcd_matrix4 Place, double a[3], double b[3], double *kcd_obb_distance );
extern int obb_disjoint ( kcd_matrix4 Place, double a[3], double b[3] );
extern int obb_disjoint_trees ( int mobj_id, kcd_matrix4 a_box_pos, kcd_matrix4 inv_a_pos, double a[4], kcd_bb *second_box, int with_report, double *minimal_distance_estimate, int *nearest_obstacle );
extern int collision_exists_between_2_robot_obbtrees ( kcd_bb *rob_box, kcd_matrix4 invM2, kcd_matrix4 mat2, kcd_matrix4 mat1, kcd_matrix4 thisM1, kcd_bb *second_box, kcd_matrix4 *THISmat8, kcd_matrix4 *THISmat9, int with_report, double *distance_estimate, int *nearest_obstacle );

#endif /* __CEXTRACT__ */
