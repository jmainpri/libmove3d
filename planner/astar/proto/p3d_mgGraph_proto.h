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
