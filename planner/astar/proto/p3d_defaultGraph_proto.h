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

extern void * p3d_validSearch(void * vGraph);
extern void p3d_initSearch(void * vGraph);
extern int p3d_endSearch(void * node, void * graph, int (*fct_end)(void *, void *));
extern void p3d_recordSolution(void * vGoal , int n, void * vGraph);
extern int p3d_isNodeInPath(void * bestNodePt, void * N);
extern double p3d_computeHeurist(void * node, double fct_heurist(void * n1, void *n2), void * graph);

/**************** Heuristics *********************/

extern double p3d_heurist(void * n1, void *n2);
extern int p3d_valid(void * node, void * edge, void * graph);
extern int p3d_end(void * n1, void * n2);

/************** Ebt Nodes *********************/

extern void p3d_setEbtNodeOpened(void * node, int state);
extern void p3d_setEbtNodeClosed(void * node, int state);
extern void p3d_setEbtPathNodeOpened(void * node, int state);
extern void p3d_setEbtPathNodeClosed(void * node, int state);
extern int p3d_ebtNodeOpened(void * node);
extern int p3d_ebtNodeClosed(void * node);
extern int ebtBestNode ( void *n1, void *n2 );

/************** Nodes/Edges ******************/

extern void* p3d_getNodeListEdges(void * node);
extern void* p3d_getEdgeInitialNode(void * listEdge);
extern void* p3d_getEdgeFinalNode(void * listEdge);
extern void* p3d_getEdge(void * listEdge);
extern void* p3d_getNextEdge(void * listEdge);
extern double p3d_getNodeG(void * node);
extern double p3d_getListEdgeCost(void * listEdge);
extern void p3d_updateNode(void * vNode, void * bestNode, void * listEdge, double g, double h);
extern dbl_list* p3d_getNodeEdgeCostList(void * node);
extern int p3d_isReductibleCycle(void *graph, dbl_list *listNode, void *traj);
extern int p3d_NodeNbEdges(void *node);
extern void p3d_updateNodeMany(p3d_path_nodes *newNodeOfPath, double g, void * listEdge, void * node);
#endif
