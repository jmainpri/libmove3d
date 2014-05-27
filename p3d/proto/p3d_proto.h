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
 *    File generated automatically. Do not edit by hand.
 */

#include"../p3d/proto/p3d_rw_util_proto.h"
#include"../p3d/proto/p3d_config_proto.h"
#include"../p3d/proto/p3d_del_proto.h"
#include"../p3d/proto/p3d_info_proto.h"
#include"../p3d/proto/p3d_set_proto.h"
#include"../p3d/proto/p3d_setpos_proto.h"
#include"../p3d/proto/p3d_env_proto.h"
#include"../p3d/proto/p3d_prim_proto.h"
#include"../p3d/proto/p3d_rw_util_proto.h"
#include"../p3d/proto/p3d_rw_env_proto.h"
#include"../p3d/proto/p3d_rw_traj_proto.h"
#include"../p3d/proto/p3d_rw_scenario_proto.h"
#include"../p3d/proto/p3d_get_proto.h"
#include"../p3d/proto/p3d_poly_proto.h"
#include"../p3d/proto/polyhedre_proto.h"
#include"../p3d/proto/p3d_matrix_proto.h"
#ifdef P3D_CONSTRAINTS
#include"../p3d/proto/p3d_constraints_proto.h"
#endif
#include"../p3d/proto/p3d_jacobian_proto.h"
#include"../p3d/proto/p3d_autocol_proto.h"
#include"../p3d/proto/p3d_halton_proto.h"
#include"../p3d/proto/p3d_joints_proto.h"
#include"../p3d/proto/p3d_jnt_translate_proto.h"
#include"../p3d/proto/p3d_jnt_fixed_proto.h"
#include"../p3d/proto/p3d_jnt_rotate_proto.h"
#include"../p3d/proto/p3d_jnt_base_proto.h"
#include"../p3d/proto/p3d_jnt_freeflyer_proto.h"
#include"../p3d/proto/p3d_jnt_plan_proto.h"
#include"../p3d/proto/p3d_jnt_knee_proto.h"
#include"../p3d/proto/p3d_rw_jnt_proto.h"
#include"../p3d/proto/p3d_BB_proto.h"
#include"../p3d/proto/p3d_ik_proto.h"
#include"../p3d/proto/p3d_rlg_proto.h"
#include"../p3d/proto/p3d_parallel_proto.h"
#include"../p3d/proto/p3d_ik_kuka_proto.h"
#include"../p3d/proto/p3d_human_arm_ik_proto.h"
#include"../p3d/proto/p3d_ik_pa10_proto.h"
#include"../p3d/proto/p3d_ik_lwr_proto.h"
#include"../p3d/proto/p3d_ik_pr2_proto.h"
#include"../p3d/proto/p3d_copy_robot.h"
#include"../p3d/proto/p3d_random_proto.h"
#include"../p3d/proto/p3d_rwXmlBasics_proto.h"
#include"../p3d/proto/p3d_rwXmlTraj_proto.h"
