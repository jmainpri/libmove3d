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
 * p3d_SetIsWeightedChoice
 * Set if the nodes are weighted or not
 * param[In]: IsWeightedChoice: should be TRUE if
 * the nodes are weighted.
 */
void p3d_SetIsWeightedChoice(int IsWeightedChoice);

/**
 * p3d_GetIsWeightedChoice
 * Get if the nodes are weighted or not
 * @return: TRUE if the nodes are weighted.
 */
int p3d_GetIsWeightedChoice(void);

/**
 * p3d_GetNodeWeightStrategy
 * Get the strategy used to weight a node
 * @return:  the strategy used to weight a node.
 */
int p3d_GetNodeWeightStrategy(void);

/**
 * p3d_SetNodeWeightStrategy
 * Set the strategy used to weight a node
 * @param[In]:  the strategy used to weight a node.
 */
void p3d_SetNodeWeightStrategy(int NodeWeightStrat);

/**
 * p3d_GetNodeWeight
 * Get the weight of a given node depending
 * of the weighting strategy
 * @param[In]: The given node
 * @return: the weight of the node depending of the 
 * weighting strategy. 
 * Note: currently the returned value is different than the
 * NodePt->weight field. Should be modified to remove ambiguity
 */
double p3d_GetNodeWeight(p3d_node* NodePt);

/**
 * p3d_SetNodeWeight
 * Set the weight of a given node depending
 * of the weighting strategy. The function modify
 * the  field NodePt->weight.
 * @param[In]: GraphPt: the robot graph
 * @param[In]: NodePt: the given node
 */
void p3d_SetNodeWeight(p3d_graph* GraphPt, p3d_node* NodePt);

double p3d_get_rate_n_expan_without_w_improve(void);
double ffo_dist_from_root_pos(configPt q);

int p3d_get_w_inc_dir(void);
void p3d_init_root_weight(p3d_graph *G);


void p3d_SetStopWeightAndSign(double stop_weight, int sign_stop_weight);
void p3d_GetStopWeightAndSign(double *stop_weightPt, int *sign_stop_weightPt);

void p3d_SetDiffuStoppedByWeight(double stopped_by_weight);
int p3d_GetDiffuStoppedByWeight(void);

void p3d_SetIsWeightStopCondition(int IsWeightStopCondition);
int p3d_GetIsWeightStopCondition(void);
#endif
