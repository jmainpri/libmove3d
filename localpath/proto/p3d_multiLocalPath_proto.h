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

#ifdef MULTILOCALPATH
#ifndef P3D_MULTILOCALPATH_PROTO
#define  P3D_MULTILOCALPATH_PROTO

extern void p3d_destroy_multiLocalPath_data(p3d_rob* robotPt);
extern void p3d_multiLocalPath_destroy(p3d_rob* robotPt, p3d_localpath* localpathPt);
extern configPt p3d_multiLocalPath_config_at_param(p3d_rob *robotPt, p3d_localpath *localpathPt, double param);
extern double p3d_multiLocalPath_length(p3d_rob* robotPt, p3d_localpath* localpathPt);
extern double p3d_multiLocalPath_stay_within_dist(p3d_rob* robotPt, p3d_localpath* localpathPt, double parameter, whichway dir, double *distances);
extern p3d_localpath *p3d_copy_multiLocalPath_localpath(p3d_rob* robotPt, p3d_localpath* localpathPt);
extern p3d_localpath *p3d_extract_multiLocalPath(p3d_rob *robotPt, p3d_localpath *localpathPt, double l1, double l2);
extern double p3d_multiLocalPath_cost(p3d_rob *robotPt, p3d_localpath *localpathPt);
extern p3d_localpath *p3d_simplify_multiLocalPath(p3d_rob *robotPt, p3d_localpath *localpathPt, int *need_colcheck);
extern p3d_localpath *p3d_multiLocalPath_localplanner(p3d_rob *robotPt, p3d_softMotion_data** softMotion_data, configPt qi, configPt qf, configPt qfp1, int* ikSol);
extern void lm_destroy_multiLocalPath_params(p3d_rob *robotPt, void *paramPt);
extern int p3d_write_multiLocalPath(FILE* filePtr, p3d_rob* robotPt, p3d_localpath* localpathPt);
extern int p3d_multiLocalPath_get_value_groupToPlan(p3d_rob* robotPt, const int mgID);
extern void p3d_multiLocalPath_set_groupToPlan(p3d_rob* robotPt, int mgID, int value, int updateJointSampling = TRUE) ;
extern configPt p3d_separateMultiLocalPathConfig(p3d_rob *r, configPt refConfig, configPt config, int mlpID, p3d_multiLocalPathJoint ** mlpJoints);
extern void p3d_multiLocalPath_set_groupToPlan_by_name(p3d_rob* robotPt, char* name, int flag) ;
extern void p3d_multiLocalPath_disable_all_groupToPlan(p3d_rob* robotPt, int updateJointSampling = TRUE);
extern int p3d_multiLocalPath_get_group_by_name(p3d_rob* robotPt, char* name);

extern void p3d_multiLocalPath_enable_all_groupToPlan(p3d_rob* robotPt);
extern int p3d_multiLocalPath_update_joint_sampling_activation(p3d_rob* robotPt);

extern int p3d_multilocapath_print_group_info(p3d_rob* robotPt);
#endif
#endif
