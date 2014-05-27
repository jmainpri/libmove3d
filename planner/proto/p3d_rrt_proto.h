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

#ifndef __CEXTRACT__

extern int p3d_get_rrt_maxn_fails(void);
extern p3d_node *hrm_nearest_neighbor(p3d_rob *rob, configPt q, p3d_compco *comp, double *distPt, int bio_ligand_mode);
extern p3d_node *hrm_extend_on_surface(p3d_rob *rob, p3d_graph *G, p3d_node *Nnear, configPt qnew);
extern p3d_node *p3d_get_current_rrt_Nnear ( void );
extern void p3d_set_current_rrt_Nnear(p3d_node *Nnear);
extern void p3d_set_save_intermediate_rrt_N(int val);
extern int p3d_get_save_intermediate_rrt_N ( void );
extern p3d_list_node* is_config_inside_tree_shape ( p3d_rob* robotPt, p3d_compco* comp, configPt q );
extern double hrm_dist_config(p3d_rob *robotPt, configPt q_i, configPt q_f);
extern int hrm_expand_one_rrt ( p3d_graph *G, p3d_compco **CompPt, configPt q, int sample_passive);
extern int p3d_expand_start_rrt ( p3d_graph *G, int (*fct_stop)(void) );
extern int p3d_expand_bidir_rrt ( p3d_graph *G, int (*fct_stop)(void) );
extern int p3d_resize_rrt_box(p3d_rob* robotPt, configPt box_env[], p3d_node* NewNode, int koef);
#endif /* __CEXTRACT__ */
