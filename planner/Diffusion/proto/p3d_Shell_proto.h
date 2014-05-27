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

/**
 * p3d_init_pb_variables
 * Init the graph variables needed to  use the shells
 * @param[In] GraphPt: the robot graph 
 */
void p3d_init_pb_variables(p3d_graph* GraphPt);

/**
 * hrm_selected_pb_node
 * return the node selected thinks to the shell method
 * @param[In] GraphPt: pointer to the robot graph
 * @param[In] q: the configuration used to get the direction of 
 * expansion
 * @param[In comp: the connected componant to extend
 * @return: the selected node of the componant 
 */
p3d_node* hrm_selected_pb_node(p3d_graph* GraphPt, 
			       configPt q, p3d_compco *comp);
/*
 * update_parent_nfails
 * update the n_fail_extend value of the parent node 
 * of a given node
 * @param[In]: The given node
 * @return: TRUE if the n_fail_extend value of the parent node
 * has been set to rrt_maxn_fails.
 */
int update_parent_nfails(p3d_node *N);

/**
 * Get The number of time a new node 
 * has been created without fail
 * @return: The  number of time a 
 * new node has been created without fail
 */
int p3d_GetNGood(void);

/**
 * p3d_SetNGood
 * Set The number of time a new node 
 * has been created without fail
 * @param[In]: The  number of time a 
 * new node has been created without fail
 */
void p3d_SetNGood(int NGood);



#endif

