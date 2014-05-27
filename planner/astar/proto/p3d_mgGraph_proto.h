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
#ifdef MULTIGRAPH

#ifndef __CEXTRACT__
#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

extern void * p3d_mgValidSearch(void * vGraph);
extern void p3d_mgInitSearch(void * vGraph);
extern int p3d_mgEndSearch(void * node, void * graph, int (*fct_end)(void *, void *));
extern void p3d_mgRecordSolution(void * vGoal , int n, void * vGraph);
extern int p3d_mgIsNodeInPath(void * bestNodePt, void * N);
extern double p3d_mgComputeHeurist(void * node, double fct_heurist(void * n1, void *n2), void * graph);

/**************** Heuristics *********************/

extern double p3d_mgHeurist(void * n1, void *n2);
extern int p3d_mgEnd(void * n1, void * n2);

/************** Ebt Nodes *********************/

extern void p3d_setEbtMgNodeOpened(void * node, int state);
extern void p3d_setEbtMgNodeClosed(void * node, int state);
extern int p3d_ebtMgNodeOpened(void * node);
extern int p3d_ebtMgNodeClosed(void * node);
extern int ebtBestMgNode(void *n1, void *n2);

/************** Nodes/Edges ******************/

extern void* p3d_getMgNodeListEdges(void * node);
extern void* p3d_getMgEdgeInitialNode(void * listEdge);
extern void* p3d_getMgEdgeFinalNode(void * listEdge);
extern void* p3d_getMgEdge(void * listEdge);
extern void* p3d_getNextMgEdge(void * listEdge);
extern double p3d_getMgNodeG(void * node);
extern double p3d_getMgListEdgeCost(void * listEdge);
extern void p3d_updateMgNode(void * vNode, void * bestNode, void * listEdge, double g, double h);
extern dbl_list* p3d_getMgNodeEdgeCostList(void * node);
extern int p3d_MgNodeNbEdges(void *node);

#endif

#endif
