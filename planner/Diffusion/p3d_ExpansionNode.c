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
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"



/**
 * Flag  checking if there is 
 * or not a  maximum number of failures allowed  
 * for a node until it is discarded for selection
 *Should be TRUE if there is a maximal number of failures
 */
static int IS_MAX_EXPAND_NODE_FAILURE = FALSE;

/**
 * the percentage  of K-nearest neighbors in a compoant, 
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 */
static int K_NEAREST_PERCENT = 10;

/**
 * p3d_GetNearestExpandPercent
 * Get for a componant the percent of K-nearest neighbors
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 * @return:  the percentage of K-nearest neighbor 
 * in the componant.
 */
int p3d_GetNearestExpandPercent(void) {
  return K_NEAREST_PERCENT;
}

/**
 * SetNbNearestExpand
 * Set for a componant the percent of K-nearest neighbors
 * inside which a node is selected for expansion
 * if the appropriate expansion method is selected.
 * @param[In]: KNearestPercent: the percentage of K-nearest neighbor 
 * in the componant.
 */
void p3d_SetNearestExpandPercent(int KNearestPercent) {
  K_NEAREST_PERCENT = KNearestPercent;
}


/**
 * SelectExpansionNode
 * Main function selecting a node to expand
 * for a connected componant
 * @param[in] GraphPt: Pointer to the robot graph
 * @param[in] CompToExpandPt: the connected componant to expand
 * @param[in] DirectionConfig the configuration previously 
 * selected as  direction of expansion
 * @return: The node of the connected componant selected
 * or NULL if no node has been found
 */
p3d_node* SelectExpansionNode(p3d_graph* GraphPt, p3d_compco* CompToExpandPt, 
			      configPt DirectionConfig) {
  p3d_node* ExpansionNodePt =NULL;
  int KNearest = -1;
  int NearestPercent;


 if((GraphPt == NULL) || (CompToExpandPt == NULL)) {
   PrintInfo(("Warning : Try to Select an expansion node with wrong graph structures \n"));
    return NULL;
  }

 switch(ENV.getInt(Env::ExpansionNodeMethod)) {

 case NEAREST_EXP_NODE_METH:
   /* Choose the nearest node of the componant*/
   ExpansionNodePt = NearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig);
   break;

 case K_NEAREST_EXP_NODE_METH:
   /* Select randomly among the K nearest nodes of a componant */
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(CompToExpandPt->nnode))/100.));
   ExpansionNodePt = KNearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig,
					    KNearest);
   break;
 case BEST_SCORE_EXP_METH:
   /* Select the node which has the best score: weight*dist */
   ExpansionNodePt = NearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig);
   break;
 case K_BEST_SCORE_EXP_METH:
   NearestPercent = p3d_GetNearestExpandPercent();
   KNearest = MAX(1,(int)((NearestPercent*(CompToExpandPt->nnode))/100.));
   ExpansionNodePt = KNearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig, KNearest);
   break;
 case RANDOM_IN_SHELL_METH:
   /* Select randomly among all the nodes inside a given portion of shell */
   ExpansionNodePt = hrm_selected_pb_node(GraphPt,
					  DirectionConfig,
					  CompToExpandPt);
   break;
 case RANDOM_NODE_METH:
   ExpansionNodePt = p3d_RandomNodeFromComp(CompToExpandPt);
   break;
 default:
   /* By default return the nearest node of the componant */
   ExpansionNodePt = NearestWeightNeighbor(GraphPt, CompToExpandPt, DirectionConfig);
  }
 return ExpansionNodePt;
}




