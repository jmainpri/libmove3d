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

extern void kcd_mo_moved ( int kcd_obj_id, int is_moved );
extern int kcd_get_is_moved ( int mo_id );
extern int kcd_is_initialized ( void );
extern int kcd_get_nof_mos ( void );
extern int kcd_get_nof_grps ( void );
extern int kcd_get_nof_sos ( void );
extern void deconnect_kcd_data ( void );
extern void kcd_remember_scene ( int kcd_scene_id );
extern void kcd_beg_scene ( int tot_nof_prims, int nof_st_obj, int nof_mo_grps, int nof_mov_obj );
extern int kcd_end_scene ( void );
extern void kcd_clean_up ( void );
extern void kcd_beg_obj ( int can_move );
extern int kcd_end_obj ( void );
extern int kcd_add_prim ( void *primPt );
extern int kcd_def_mo_grp ( int *arr_kcd_ids, int nof_mo_ids );
extern void kcd_def_mo_no_grp ( void );
extern void kcd_act_mo ( int mo_id );
extern void kcd_deact_mo ( int mo_id );
extern void kcd_act_grp ( int grp_id );
extern void kcd_deact_grp ( int grp_id );
extern void kcd_act_obst ( int ext_obj_id );
extern void kcd_deact_obst ( int ext_obj_id );
extern int kcd_obst_is_act ( int ext_obj_id );
extern void kcd_act_mo_pair ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern void kcd_deact_mo_pair ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern void kcd_act_col_mo_pair ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern void kcd_deact_col_mo_pair ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern void kcd_act_dist_mo_pair ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern void kcd_deact_dist_mo_pair ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern void kcd_act_mo_env ( kcd_col_handle * handlePt, int kcd_id );
extern void kcd_deact_mo_env ( kcd_col_handle * handlePt, int kcd_id );
extern void kcd_act_col_mo_env ( kcd_col_handle * handlePt, int kcd_id );
extern void kcd_deact_col_mo_env ( kcd_col_handle * handlePt, int kcd_id );
extern void kcd_act_dist_mo_env ( kcd_col_handle * handlePt, int kcd_id );
extern void kcd_deact_dist_mo_env ( kcd_col_handle * handlePt, int kcd_id );
extern void kcd_act_grp_pair ( kcd_col_handle * handlePt, int grp_id1, int grp_id2 );
extern void kcd_deact_grp_pair ( kcd_col_handle * handlePt, int grp_id1, int grp_id2 );
extern void kcd_act_grp_env ( kcd_col_handle * handlePt, int grp_id );
extern void kcd_deact_grp_env ( kcd_col_handle * handlePt, int grp_id );
extern void kcd_deactivate_all_mo ( kcd_col_handle * handlePt );
extern int kcd_mo_is_act ( int mo_id );
extern int kcd_grp_is_act ( int grp_id );
extern int kcd_mo_belongs_to_group ( int mobj_id );
extern int kcd_mo_pair_is_act ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern int kcd_col_mo_pair_is_act ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern int kcd_dist_mo_pair_is_act ( kcd_col_handle * handlePt, int kcd_id1, int kcd_id2 );
extern int kcd_mo_env_is_act ( kcd_col_handle * handlePt, int kcd_id );
extern int kcd_col_mo_env_is_act ( kcd_col_handle * handlePt, int kcd_id );
extern int kcd_dist_mo_env_is_act ( kcd_col_handle * handlePt, int kcd_id );
extern int kcd_grp_pair_is_act ( kcd_col_handle * handlePt, int grp_id1, int grp_id2 );
extern int kcd_grp_env_is_act ( kcd_col_handle * handlePt, int grp_id );
extern kcd_col_handle * kcd_create_handle ( void );
extern kcd_col_handle * kcd_copy_handle ( kcd_col_handle * handlePt );
extern void kcd_copy_handle_into ( kcd_col_handle * src_handlePt, kcd_col_handle * dest_handlePt );
extern void kcd_add_handle_into ( kcd_col_handle * src_handlePt, kcd_col_handle * dest_handlePt );
extern void kcd_sub_handle_into ( kcd_col_handle * src_handlePt, kcd_col_handle * dest_handlePt );
extern void kcd_deactivate_all_handle ( kcd_col_handle * handlePt );
extern void kcd_destroy_handle ( kcd_col_handle * handlePt );
extern kcd_col_handle * kcd_get_cur_handle ( void );
extern void kcd_set_cur_handle ( kcd_col_handle * handlePt );
extern void kcd_set_aabb_on_mo ( int kcd_ext_o );
extern int kcd_robot_collides_itself ( int robot_number, int with_report, double *min_dist, int *near_obs );
extern int kcd_robot_collides_robot ( int robot_number1, int robot_number2, int with_report );
extern int kcd_robot_vs_robot ( int robot_number1, int robot_number2, int with_report, double *min_dist_estimate, int *nearest_obstacle );
extern int kcd_robot_collides ( int robot_id, int with_report, double *min_dist_estimate, int *nearest_obstacle );
extern int kcd_collision_exists ( int with_report, double *min_dist_estimate );
extern int kcd_mo_collides ( int mobj_id, int with_report, double *min_dist_estimate, int *nearest_obstacle );
extern int kcd_robot_collides_mo ( int robot_id, int mo_id );
extern int kcd_robot_vs_mo ( int robot_id, int mo_id, int with_report, double *min_dist_estimate, int *nearest_obstacle );
extern int kcd_mo_collides_mo ( int mo1, int mo2 );
extern int kcd_mo_vs_mo ( int mo1, int mo2, int with_report, double *min_dist_estimate, int *nearest_obstacle );
extern void kcd_set_tolerance ( double epsilon );
extern void kcd_ignore_tolerance ( void );
extern void kcd_set_relative_error ( double value );
extern double kcd_get_relative_error ( void );
extern void kcd_init_min_vol_of_obj_detail ( double one_dim );
extern void kcd_set_min_vol_of_obj_detail ( double one_dim );
extern void kcd_ignore_min_vol_of_obj_detail ( void );
extern int kcd_report_get_cor_mo ( int kcd_obj_id );
extern int kcd_robot_collides_something ( int i, int with_report, double *min_dist_estimate );
extern int kcd_robot_collides_something_except_specified_robot ( int i, int deviceToIgnore, int with_report, double *min_dist_estimate );
extern int kcd_nof_obst_to_add ( void );
extern void kcd_addObst_beg_scene ( int current_scene, int tot_nof_prims, int nof_st_obj );
extern void kcd_addObst_end_scene ( void );
extern void kcd_addObst_beg_obj ( void );
extern int kcd_addObst_add_prim ( void *primPt );
extern int kcd_addObst_end_obj ( void );
extern int kcd_get_nof_objects ( void );

#endif /* __CEXTRACT__ */
