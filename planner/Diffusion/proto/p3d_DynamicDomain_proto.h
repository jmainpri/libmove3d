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
 * p3d_SetLambda(double L)
 * Set the value of the lambda parameter used to
 * define the size of the dynamic sampling domain
 * @param[In] Lambda: the Lambda value
 */
void p3d_SetLambda(double Lambda);

/**
 * p3d_GetLambda(void)
 * Get the value of the lambda parameter used to
 * define the size of the dynamic sampling domain
 * @return: the value of the lambda parameter 
 */
double p3d_GetLambda(void);

/**
 * p3d_resize_rrt_box_ORIGINAL
 * Old version of p3d_ResizeDynDomain
 * No more used
 */
int p3d_resize_rrt_box_ORIGINAL(p3d_rob* robotPt, configPt box_env[], p3d_node* NewNode, int koef);
/**
 * p3d_InitSpaceCostParam
 * Initialize some parameters for dealing 
 * with dynamic domain methods
 * @param[In] GraphPt: the robot graph 
 * @param[In] Ns: Initial node of the graph 
 * @param[In] Ng: Goal node of the graph 
 */
void p3d_InitDynDomainParam(p3d_graph* GraphPt, p3d_node* Ns, p3d_node* Ng);

/**
 * p3d_ResizeDynDomain
 * Resize the Dynamic Domain of a componant when a new 
 * node is added. The Dynamic Domain is then used as a restricted 
 * region of sampling to balance the greediness of the expansion 
 * process with more refinement of the tree.
 * The field box_env_small of the componected componant of the node
 *  is modified.
 * @param[In]: robotPt: A pointer to the robot
 * @param[In]: NewNodePt : Pointer for the New added node
 * return: TRUE if the process of resizing succeeded
 */
int p3d_ResizeDynDomain(p3d_rob* robotPt, p3d_node* NewNodePt);
#endif
