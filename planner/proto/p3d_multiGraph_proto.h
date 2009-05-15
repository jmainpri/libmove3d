#ifdef MULTIGRAPH
#ifndef __CEXTRACT__
extern void p3d_setActiveDof(p3d_rob * r, int mgNum);
extern void p3d_activateMgAutocol(p3d_rob * r, int mgNum);
extern void p3d_deactivateMgAutocol(p3d_rob * r, int mgNum);
extern void p3d_deactivateMgAutocolOnly(p3d_rob * r, int mgNum);
extern p3d_graph * p3d_setRandomMultiGraphAndActiveDof(p3d_rob * r, int * random);
extern p3d_graph * p3d_setMultiGraphAndActiveDof(p3d_rob * r, int mgNum);
extern void p3d_setAllDofActive(p3d_rob * r);
extern void p3d_setAllDofPassive(p3d_rob * r);
extern void p3d_flatMultiGraph(p3d_rob * r, int global);
extern int p3d_fillFlatMultiGraph(p3d_rob * r, p3d_node ** node, p3d_multiGraphJoint ** mgJoints, int mgNum, int mode);
extern int p3d_specificFillFlatMultiGraph(p3d_rob * r, p3d_node ** node, int mgNum);
extern void p3d_convertFsgToGraph(p3d_graph * graph, p3d_flatSuperGraph *fsg);
extern void p3d_resetMultiGraph(p3d_rob * r);
extern int p3d_jointInMultigraph(p3d_rob * r, int jointId);
extern void p3d_extractMultigraphStartGoal(p3d_rob * r, configPt qs, configPt qg, configPt * q_s, configPt * q_g);
extern void p3d_testFsgNodesConnection(p3d_rob * r, p3d_flatSuperGraph *fsg, p3d_flatSuperGraphNode * node);
extern configPt* p3d_mergeMultiGraphNodes(p3d_rob *r, int nNodes, p3d_node ** nodes, p3d_multiGraphJoint ** mgJoints, int* nbConfigs);
extern configPt p3d_mergeMultiGraphConfig(p3d_rob *r, int nConfig, configPt *configs, p3d_multiGraphJoint ** mgJoints);
extern p3d_flatSuperGraphNode * p3d_isConfigInSuperGraph(p3d_rob* robot, p3d_flatSuperGraph * fsg, configPt q);
extern configPt p3d_mergeTwoMultiGraphConfigs(p3d_rob *r, configPt q1, configPt q2, configPt trunk, int nbMg1, p3d_multiGraphJoint **mgj1, int nbMg2, p3d_multiGraphJoint **mgj2);
extern int p3d_doIncrementalConstruction(int state);
extern void p3d_addFsgNodeInGraph(p3d_flatSuperGraph *fsg, p3d_flatSuperGraphNode * node);
extern void p3d_connectFsgNodes(p3d_flatSuperGraph *fsg, p3d_flatSuperGraphNode * n1, p3d_flatSuperGraphNode * n2, double dist);
extern void p3d_createRobotFlatSuperGraph (p3d_rob *r);
extern void p3d_initFlatSuperGraph (p3d_flatSuperGraph *fsg);
extern p3d_flatSuperGraphNode * p3d_createFlatSuperGraphNode (p3d_rob *r, p3d_flatSuperGraph *fsg, p3d_node ** nodes, configPt q);
extern p3d_flatSuperGraphEdge * p3d_createFlatSuperGraphEdge(p3d_flatSuperGraphNode * fsgN1, p3d_flatSuperGraphNode * fsgN2, double dist);
extern p3d_fsgListNode * p3d_createFsgListNode(p3d_flatSuperGraphNode * fsgNode);
extern p3d_fsgListEdge * p3d_createFsgListEdge(p3d_flatSuperGraphEdge * fsgEdge);
extern void p3d_pushFsgListNode(p3d_fsgListNode ** list, p3d_fsgListNode * nodeToInsert);
extern void p3d_insertAfterFsgListNode(p3d_fsgListNode * prevNode, p3d_fsgListNode * nodeToInsert);
extern void p3d_insertBeforeFsgListNode(p3d_fsgListNode * nextNode, p3d_fsgListNode * nodeToInsert);
extern void p3d_removeFsgListNode (p3d_fsgListNode *fsgListNode);
extern void p3d_pushFsgListEdge(p3d_fsgListEdge ** list, p3d_fsgListEdge * edgeToInsert);
extern void p3d_insertAfterFsgListEdge(p3d_fsgListEdge * prevEdge, p3d_fsgListEdge * edgeToInsert);
extern void p3d_insertBeforeFsgListEdge(p3d_fsgListEdge * nextEdge, p3d_fsgListEdge * edgeToInsert);
extern void p3d_removeFsgListEdge (p3d_fsgListEdge *fsgListEdge);
extern void p3d_delFlatSuperGraph (p3d_rob * r, p3d_flatSuperGraph * fsg);
extern void p3d_delFsgListNode(p3d_rob * r, p3d_fsgListNode * list);
extern void p3d_delFsgListEdge(p3d_fsgListEdge * list);
extern void p3d_delNodeFromFsgListNode(p3d_rob * r,p3d_fsgListNode * node);
extern void p3d_delNodeFromFsgListEdge(p3d_fsgListEdge * node);
extern void p3d_delFsgNode(p3d_rob * r, p3d_flatSuperGraphNode * node);
extern void p3d_delFsgEdge(p3d_flatSuperGraphEdge * edge);
#endif
#endif
