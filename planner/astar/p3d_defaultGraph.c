#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

void * p3d_validSearch(void * vGraph){
  p3d_graph * graph = (p3d_graph *)vGraph;
  if ((graph->search_start != NULL) && (graph->search_goal != NULL)){
    return (void *)graph->search_start;
  }
  return NULL;
}

void p3d_initSearch(void * vGraph){
  int i;
  p3d_graph * graph = (p3d_graph *) vGraph;
  p3d_list_node  *lnodes = graph->nodes;
  p3d_node   *node;


  for(i=1;i<=graph->nnode;i++) {
    node = lnodes->N;
    node->opened = 0;
    node->closed = 0;
    lnodes = lnodes->next;
  }

  node              = graph->search_start;
  node->search_from = NULL;
  node->search_to   = NULL;
  node->f           = .0;
  node->g           = .0;

  graph->search_goal->search_from = NULL;
  graph->search_goal->search_to = NULL;
}

int p3d_endSearch(void * node, void * graph, int (*fct_end)(void *, void *)){
  return fct_end(((p3d_graph *)graph)->search_goal,node);
}

void p3d_recordSolution(void * vGoal , int n, void * vGraph){
  p3d_graph * graph = (p3d_graph *)vGraph;
  p3d_node * goal = (p3d_node *)vGoal;
  p3d_node * node = goal ;
  int l = 1;

  graph->search_done    = TRUE;
  graph->search_numdev  = n;

  if(goal == NULL) {
    graph->search_path      = FALSE;
    graph->search_goal_sol  = NULL;
    graph->search_cost      = .0;
  }
  else {
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

int p3d_isNodeInPath(void * bestNodePt, void * N){
  p3d_node * parentNode = (p3d_node *)bestNodePt;
  while(parentNode != NULL) {
    if(parentNode == (p3d_node *)N) {
      return TRUE;
    }
    parentNode= parentNode->search_from;
  }
  return FALSE;
}

double p3d_computeHeurist(void * node, double fct_heurist(void * n1, void *n2), void * graph){
  double h;

  if(fct_heurist != NULL) {
    h = (*fct_heurist)(node,(void *)((p3d_graph *)graph)->search_goal);
    return(h);
  } else {
    return(.0);
  }
}

/**************** Heuristics *********************/

double p3d_heurist(void * n1, void *n2){
p3d_rob *rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  // In case of Cost Spaces, we don't give a minorant heuristic
  // as it is non trivial. However epsilon*dist seems correct
  // Where epsilon is the small value put in the edges costs for 
  // downhillslopes.  
  if(p3d_GetIsCostFuncSpace() == TRUE) {
     return 0;
  }
  return p3d_dist_config(rob, ((p3d_node *)n1)->q, ((p3d_node *)n2)->q);
}

int p3d_valid(void * node, void * edge, void * graph){
  if((graph!=NULL) && (edge!=NULL) && (node!=NULL)) {
    return(TRUE);
  } else {
    PrintInfo(("ERROR : Unvalid Node, Edge or graph in astar/p3d_valid\n"));
    return(FALSE);
  }
}

int p3d_end(void * n1, void * n2){
  p3d_rob *rob = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

  return p3d_equal_config(rob, ((p3d_node *)n1)->q, ((p3d_node *)n2)->q);
}

/************** Ebt Nodes *********************/

void p3d_setEbtNodeOpened(void * node, int state){
  ((p3d_node *)node)->opened = state;
}

void p3d_setEbtNodeClosed(void * node, int state){
  ((p3d_node *)node)->closed = state;
}

void p3d_setEbtPathNodeOpened(void * node, int state){
  ((p3d_path_nodes *)node)->opened = state;
}

void p3d_setEbtPathNodeClosed(void * node, int state){
  ((p3d_path_nodes *)node)->closed = state;
}


int p3d_ebtNodeOpened(void * node){
  return ((p3d_node *)node)->opened;
}

int p3d_ebtNodeClosed(void * node){
  return ((p3d_node *)node)->closed;
}

/************** Nodes/Edges ******************/

void* p3d_getNodeListEdges(void * node){
  return (void *)((p3d_node *)node)->edges;
}

void* p3d_getEdgeInitialNode(void * listEdge){
  return (void *)((p3d_list_edge *)listEdge)->E->Ni;
}

void* p3d_getEdgeFinalNode(void * listEdge){
  return (void *)((p3d_list_edge *)listEdge)->E->Nf;
}

void* p3d_getEdge(void * listEdge){
  return (void *)((p3d_list_edge *)listEdge)->E;
}

void* p3d_getNextEdge(void * listEdge){
 return (void *)((p3d_list_edge *)listEdge)->next;
}

double p3d_getNodeG(void * node){
  return ((p3d_node *) node)->g;
}

double p3d_getListEdgeCost(void * listEdge){
  return ((p3d_list_edge *)listEdge)->E->cost;
}

void p3d_updateNode(void * vNode, void * bestNode, void * listEdge, double g, double h){
  p3d_node * node = (p3d_node *) vNode;

  node->search_from = (p3d_node *)bestNode;
  node->edge_from = ((p3d_list_edge *)listEdge)->E;
  node->g = g;
  node->h = h;
  node->f = node->g + node->h;
}

dbl_list* p3d_getNodeEdgeCostList(void * node){
  return ((p3d_node *)node)->orderCostListEdge;
}

int p3d_isReductibleCycle(void *graph, dbl_list *listNode, void *traj){
  int isReductible = 0;
  p3d_traj* current_trajPt = p3d_create_traj_from_list_nodes((p3d_graph *)graph, listNode);//create trajectory
  int nretract = p3d_get_nretract();
  p3d_set_nretract(0);
  isReductible = p3d_test_projection(((p3d_graph *)graph)->rob, current_trajPt, (p3d_traj*) traj,p3d_get_Nstep());
  p3d_set_nretract(nretract);
  p3d_del_traj(current_trajPt);
  return isReductible;
}

int p3d_NodeNbEdges(void *node){
  return ((p3d_node *)node)->nedge;
}

void p3d_updateNodeMany(p3d_path_nodes *newNodeOfPath, double g, void * listEdge, void * node){
  newNodeOfPath->g = g  + ((p3d_list_edge *)listEdge)->E->longueur;
  ((p3d_node *)newNodeOfPath->N)->search_from = ((p3d_node*)node);
}
