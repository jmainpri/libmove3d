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

extern int s_p3d_col_env_active_context ( void );
extern void p3d_col_env_switch_env ( p3d_type_col_env_context type );
extern void p3d_col_env_switch_robot ( p3d_rob * robotPt, p3d_type_col_env_context type );
extern void p3d_col_env_restore ( void );
extern void p3d_col_env_start ( void );
extern void p3d_col_env_stop ( void );
extern void p3d_col_activate_obj_obj ( p3d_obj *obj1, p3d_obj *obj2 );
extern void p3d_col_activate_obj_env ( p3d_obj *obj );
extern void p3d_col_activate_rob_obj ( p3d_rob *rob, p3d_obj *obj );
extern void p3d_col_activate_rob_env ( p3d_rob *robotPt );
extern void p3d_col_activate_rob_rob ( p3d_rob *rob1, p3d_rob *rob2 );
extern void p3d_col_activate_obj_rob(p3d_obj *obj, p3d_rob *rob);
extern void p3d_col_activate_rob_all_rob ( p3d_rob *robotPt );
extern void p3d_col_activate_obj_all_rob(p3d_obj *obj);
extern void p3d_col_activate_rob ( p3d_rob *rob );
extern void p3d_col_deactivate_all ( void );
extern void p3d_col_activate_env ( void );
extern void p3d_col_activate_robots ( void );
extern void p3d_col_deactivate_obj_obj ( p3d_obj *obj1, p3d_obj *obj2 );
extern void p3d_col_deactivate_obj_env ( p3d_obj *obj );
extern void p3d_col_deactivate_rob_obj ( p3d_rob *rob, p3d_obj *obj );
extern void p3d_col_deactivate_rob_env ( p3d_rob *robotPt );
extern void p3d_col_deactivate_rob_rob ( p3d_rob *rob1, p3d_rob *rob2 );
extern void p3d_col_deactivate_obj_rob(p3d_obj *obj, p3d_rob *rob);
extern void p3d_col_deactivate_rob ( p3d_rob *rob );
extern void p3d_col_deactivate_obj_all_rob(p3d_obj *obj);

#endif /* __CEXTRACT__ */
