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
