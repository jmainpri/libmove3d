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
extern int bio_expand_side_chain(p3d_graph*G, p3d_node* last_node, int_tab* sc_lignand_col_list);
extern p3d_node*bio_expandnode_ligand(p3d_graph* G,p3d_node* Nnear, configPt qexp_direction, 
				int_tab** sc_ligand_col_list, int* is_ligand_autocolPt, 
				      int* is_ligand_bb_colPt);
extern int bio_expand_side_chain(p3d_graph*G, p3d_node* last_node, int_tab* sc_lignand_col_list); 
extern int bio_expand_ligand(p3d_graph*G, configPt q_lig, int_tab** sc_ligand_col_list, int*  is_ligand_autocolPt, 
			     int* is_ligand_bb_colPt);
extern int bio_start_lig_sc_rrt(p3d_graph *G,int (*fct_stop)(void));
