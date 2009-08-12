#ifdef MULTIGRAPH

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

void * p3d_mgValidSearch(void * vGraph){
  p3d_flatSuperGraph * graph = (p3d_flatSuperGraph *)vGraph;
  if ((graph->search_start != NULL) && (graph->search_goal != NULL)){
    return (void *)graph->search_start;
  }
  return NULL;
}

void p3d_mgInitSearch(void * vGraph){
  int i;
  p3d_flatSuperGraph * graph = (p3d_flatSuperGraph *) vGraph;
  p3d_fsgListNode  *lnodes = graph->nodes;
  p3d_flatSuperGraphNode   *node;

  for(i = 0; i < graph->nNodes; i++) {
    node = lnodes->node;
    node->opened = 0;
    node->closed = 0;
    lnodes = lnodes->next;
  }

  node  = graph->search_start;
  node->search_from = NULL;
  node->search_to = NULL;
  node->f = 0.0;
  node->g = 0.0;
  node->h = 0.0;

  graph->search_goal->search_from = NULL;
  graph->search_goal->search_to = NULL;
}

int p3d_mgEndSearch(void * node, void * graph, int (*fct_end)(void *, void *)){
  return fct_end(((p3d_flatSuperGraph *)graph)->search_goal,node);
}

void p3d_mgRecordSolution(void * vGoal , int n, void * vGraph){
  p3d_flatSuperGraph * graph = (p3d_flatSuperGraph *)vGraph;
  p3d_flatSuperGraphNode * goal = (p3d_flatSuperGraphNode *)vGoal;
  p3d_flatSuperGraphNode * node = goal ;
  int l = 1;

  graph->search_done = TRUE;
  graph->search_numdev = n;

  if(goal == NULL) {
    graph->search_path      = FALSE;
    graph->search_goal_sol  = NULL;
    graph->search_cost      = 0.0;
  } else {
    graph->search_path      = TRUE;
    graph->search_goal_sol  = goal;
    graph->search_cost      = goal->f;

    while(node->search_from != NULL) {
      l++;
      node->search_from->search_to = node;
      node = node->search_from;
    }
    graph->search_path_length = l;
  }
}

int p3d_mgIsNodeInPath(void * bestNodePt, void * N){
  p3d_flatSuperGraphNode * parentNode = (p3d_flatSuperGraphNode *)bestNodePt;
  while(parentNode != NULL) {
    if(parentNode == (p3d_flatSuperGraphNode *)N) {
      return TRUE;
    }
    parentNode = parentNode->search_from;
  }
  return FALSE;
}

double p3d_mgComputeHeurist(void * node, double fct_heurist(void * n1, void *n2), void * graph){
  if(fct_heurist != NULL) {
    return (*fct_heurist)(node,(void *)((p3d_flatSuperGraph *)graph)->search_goal);
  } else {
    return(0);
  }
}

/**************** Heuristics *********************/

double p3d_mgHeurist(void * n1, void *n2){
p3d_rob *rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  // In case of Cost Spaces, we don't give a minorant heuristic
  // as it is non trivial. However epsilon*dist seems correct
  // Where epsilon is the small value put in the edges costs for 
  // downhillslopes.  
  if(ENV.getBool(Env::isCostSpace) == TRUE) {
     return 0;
  }
  return p3d_dist_config(rob, ((p3d_flatSuperGraphNode *)n1)->q, ((p3d_flatSuperGraphNode *)n2)->q);
}

int p3d_mgEnd(void * n1, void * n2){
  p3d_rob *rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  return p3d_equal_config(rob, ((p3d_flatSuperGraphNode *)n1)->q, ((p3d_flatSuperGraphNode *)n2)->q);
}

/************** Ebt Nodes *********************/

void p3d_setEbtMgNodeOpened(void * node, int state){
  ((p3d_flatSuperGraphNode *)node)->opened = state;
}

void p3d_setEbtMgNodeClosed(void * node, int state){
  ((p3d_flatSuperGraphNode *)node)->closed = state;
}

int p3d_ebtMgNodeOpened(void * node){
  return ((p3d_flatSuperGraphNode *)node)->opened;
}

int p3d_ebtMgNodeClosed(void * node){
  return ((p3d_flatSuperGraphNode *)node)->closed;
}

/************** Nodes/Edges ******************/

void* p3d_getMgNodeListEdges(void * node){
  return (void *)((p3d_flatSuperGraphNode *)node)->fsgEdges;
}

void* p3d_getMgEdgeInitialNode(void * listEdge){
  return (void *)((p3d_fsgListEdge *)listEdge)->edge->node1;
}

void* p3d_getMgEdgeFinalNode(void * listEdge){
  return (void *)((p3d_fsgListEdge *)listEdge)->edge->node2;
}

void* p3d_getMgEdge(void * listEdge){
  return (void *)((p3d_fsgListEdge *)listEdge)->edge;
}

void* p3d_getNextMgEdge(void * listEdge){
 return (void *)((p3d_fsgListEdge *)listEdge)->next;
}

double p3d_getMgNodeG(void * node){
  return ((p3d_flatSuperGraphNode *) node)->g;
}

double p3d_getMgListEdgeCost(void * listEdge){
  return ((p3d_fsgListEdge *)listEdge)->edge->cost;
}

void p3d_updateMgNode(void * vNode, void * bestNode, void * listEdge, double g, double h){
  p3d_flatSuperGraphNode * node = (p3d_flatSuperGraphNode *) vNode;

  node->search_from = (p3d_flatSuperGraphNode *)bestNode;
  node->edge_from = ((p3d_fsgListEdge *)listEdge)->edge;
  node->g = g;
  node->h = h;
  node->f = node->g + node->h;
}

dbl_list* p3d_getMgNodeEdgeCostList(void * node){
  return ((p3d_flatSuperGraphNode *)node)->orderCostListEdge;
}

// int p3d_isReductibleCycle(void *graph, dbl_list *listNode, void *traj){
//   int isReductible = 0;
//   p3d_traj* current_trajPt = p3d_create_traj_from_list_nodes((p3d_graph *)graph, listNode);//create trajectory
//   p3d_set_nretract(0);//TODO verifier si remet nretract a l'etat
//   isReductible = p3d_test_projection(((p3d_graph *)graph)->rob, current_trajPt, (p3d_traj*) traj,p3d_get_Nstep());
//   p3d_del_traj(current_trajPt);
//   return isReductible;
// }

int p3d_MgNodeNbEdges(void *node){
  return ((p3d_flatSuperGraphNode *)node)->nEdges;
}

// void p3d_updateNodeMany(p3d_path_nodes *newNodeOfPath, double g, void * listEdge, void * node){
//   newNodeOfPath->g = g  + ((p3d_list_edge *)listEdge)->E->longueur;
//   ((p3d_node *)newNodeOfPath->N)->search_from = ((p3d_node*)node);
// }

/*******************************************************/
/* Fonction de classement pour les ebt                 */
/* connexes                                            */
/* In : les deux noeuds                                */
/* Out : le code du meilleur noeud                     */
/*******************************************************/
extern int GlobalOrdering;
int ebtBestMgNode(void *n1, void *n2) {
  p3d_flatSuperGraphNode *node1, *node2;
  double dist1, dist2;

  dbl_list* CListEdge1, * CListEdge2;
  int res;
  node1 = (p3d_flatSuperGraphNode *)n1;
  node2 = (p3d_flatSuperGraphNode *)n2;

  if (GlobalOrdering == TRUE) {
    CListEdge1 = node1->orderCostListEdge;
    CListEdge2 = node2->orderCostListEdge;
    res = dbl_list_test_equal(CListEdge1, CListEdge2, costBestEdge);
    if (res >= 0) return 1;
    return -1;
  }
  dist1 = node1->f;
  dist2 = node2->f;
  return (dist1 < dist2) ? -1 : (dist1 > dist2) ? 1 : -1;
}

#endif
