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

extern void initialize_flat_value ( pflat_trailer_str trailer_params, double l1, double l2, double phi_max );
extern void trailer_conv_sub_path_fpath ( p3d_rob *robotPt, p3d_sub_trailer_data * sub_trailer_dataPt, FLAT_LOCAL_PATH_STR *fpathPt );
extern p3d_localpath * p3d_alloc_trailer_localpath ( p3d_rob *robotPt, configPt q_init, configPt q_cusp, configPt q_end, double alpha_0, double alpha_1, double u_start, double u_end, int lp_id, int symmetric );
extern int is_valid_trailer ( p3d_rob *robotPt, p3d_trailer_data *trailer_dataPt, int symmetric );
extern double p3d_sub_trailer_distance ( p3d_sub_trailer_data *sub_trailer_dataPt );
extern double p3d_trailer_dist ( p3d_rob *robotPt, p3d_localpath *localpathPt );
extern void p3d_destroy_sub_trailer_data ( p3d_rob* robotPt, p3d_sub_trailer_data* sub_trailer_dataPt, int q_init_is_there );
extern void p3d_destroy_trailer_data ( p3d_rob* robotPt, p3d_trailer_data* trailer_dataPt );
extern void p3d_trailer_destroy ( p3d_rob* robotPt, p3d_localpath* localpathPt );
extern configPt p3d_sub_trailer_config_at_distance ( p3d_rob *robotPt, p3d_sub_trailer_data *sub_trailer_dataPt, double d_lp );
extern configPt p3d_sub_trailer_config_at_param ( p3d_rob *robotPt, p3d_sub_trailer_data *sub_trailer_dataPt, double u_lp );
extern configPt p3d_trailer_config_at_distance ( p3d_rob *robotPt, p3d_localpath *localpathPt, double d_lp );
extern configPt p3d_trailer_config_at_param ( p3d_rob *robotPt, p3d_localpath *localpathPt, double u_lp );
extern double p3d_trailer_stay_within_dist ( p3d_rob* robotPt, p3d_localpath* localpathPt, double u_lp, int dir, double *distances );
extern p3d_localpath *p3d_copy_trailer_localpath ( p3d_rob* robotPt, p3d_localpath* localpathPt );
extern void p3d_extract_sub_trailer ( p3d_rob *robotPt, p3d_sub_trailer_data *sub_trailer_dataPt, double d_lp1, double d_lp2, double *Tab_new_u );
extern p3d_localpath *p3d_extract_trailer ( p3d_rob *robotPt, p3d_localpath *localpathPt, double d_lp1, double d_lp2 );
extern p3d_localpath *p3d_extract_trailer_by_param ( p3d_rob *robotPt, p3d_localpath *localpathPt, double u1, double u2 );
extern double p3d_trailer_cost ( p3d_rob *robotPt, p3d_localpath *localpathPt );
extern p3d_localpath *p3d_simplify_trailer ( p3d_rob *robotPt, p3d_localpath *localpathPt, int *need_colcheck );
extern int p3d_write_trailer ( FILE *file, p3d_rob* robotPt, p3d_localpath* localpathPt );
extern double alpha_max ( int deriv_order );
extern double dC_ds ( double *Tab_gamma );
extern void compute_alphas ( p3d_rob *robotPt, configPt q_init, configPt q_end, double *alphas, int symmetric );
extern void lm_destroy_trailer_params ( p3d_rob *robotPt, void *local_method_params );
extern p3d_localpath *p3d_trailer_localplanner ( p3d_rob *robotPt, double *qi, double *qf, int* ikSol );
extern p3d_localpath *p3d_nocusp_trailer_localplanner ( p3d_rob *robotPt, double *qi, double *qf, int* ikSol );
extern int p3d_create_trailer_local_method_for_robot ( p3d_rob *robotPt, double *dtab, int *itab );
extern int p3d_create_trailer_local_method ( double *dtab, int *itab );
extern pflat_trailer_str lm_create_trailer ( p3d_rob *robotPt, double *dtab, int *itab );
extern pflat_trailer_str lm_get_trailer_lm_param ( p3d_rob *robotPt );
extern p3d_localpath *p3d_read_trailer_localpath_symmetric ( p3d_rob *robotPt, FILE *file, double version );
extern p3d_localpath *p3d_read_trailer_localpath_not_symmetric ( p3d_rob *robotPt, FILE *file, double version );

#endif /* __CEXTRACT__ */
