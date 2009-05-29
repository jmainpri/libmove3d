
/*******************************
General purpose Astar Algo
Written by T. Simeon 
nic@grasp.cis.upenn.edu
U.P.E.N.N july 89
Modified by M. Gharbi
LAAS January 09
*******************************/

#include "Util-pkg.h"
#include "P3d-pkg.h"
#include "Planner-pkg.h"

/******************* Static functions **********************/
static p3d_path_nodes*  p3d_allocinit_path_nodes(void);
static void p3d_destroy_path_nodes(p3d_path_nodes* path_nodes);

/**
 * Main function of the A* algorithm (astar),
 * extracting a solution path from a graph structure.
 * The algorithm is based on Equilibrated Binary Trees
 * (EBT) in order to increase the efficiency (see ebt.c)
 * @param graph The graph to extract from the path.
 * @param (* fct_validSearch)(void *) Check if we have a start and a goal node in the graph structure.
 * @param (* fct_initSearch)(void *) Init the graph structure and the nodes for the search.
 * @param (* fct_endSearch)(void *, void *, int (* fct_end)(void *, void *)) Detect if we reatch the goal node.
 * @param (* fct_recordSolution)(void *, int , void *) Save the path into the graph structure.
 * @param (* fct_isNodeInPath)(void *, void *) Check if the node is in the current generated path.
 * @param (* fct_computeHeurist)(void *, double (* fct_heurist)(void *, void *), void *) Some test before calling the fct_heurist.
 * @param (* fct_heurist)(void *, void *) The heuristic function evaluating the cost to reach the goal from the
 * given node (can take for exemple the distance to the goal).
 * @param (* fct_valid)(void *, void *, void *) A function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing edges are valid).
 * @param (* fct_end)(void *, void *) Function testing if node extended reached the goal.
 * @param (* fct_setEbtNodeOpened)(void *, int ) Set node opened value for the EBT.
 * @param (* fct_setEbtNodeClosed)(void *, int ) Set node closed value for the EBT.
 * @param (* fct_ebtNodeOpened)(void *) Get the node opened value.
 * @param (* fct_ebtNodeClosed)(void *) Get the node closed value.
 * @param (* fct_getNodeListEdges)(void *) Get the list of edges connecting the node.
 * @param (* fct_getEdgeFinalNode)(void *) Get the the final node of the given edge.
 * @param (* fct_getEdge)(void *) Get the edge from the listEdge.
 * @param (* fct_getNextEdge)(void *) Get the next Edge of the list.
 * @param (* fct_getNodeG)(void *) Get the g value of a node.
 * @param (* fct_getEdgeCost)(void *) Get the cost of a node.
 * @param (* fct_updateNode)(void *, void *, void *, double , double ) Update the node astar variables.
 * @return TRUE if a solution path has been found and FALSE otherwise.
 */
int p3d_astar(void *graph,
              void* (*fct_validSearch)(void *),
              void (*fct_initSearch)(void *),
              int (*fct_endSearch)(void *, void *, int (*fct_end)(void *, void *)),
              void (*fct_recordSolution)(void *, int, void*),
              int (*fct_isNodeInPath)(void *, void *),
              double(*fct_computeHeurist)(void *, double(*fct_heurist)(void *, void *), void *),
              double(*fct_heurist)(void *, void *),
              int (*fct_valid)(void *, void *, void *),
              int (*fct_end)(void *, void *),
              void (*fct_setEbtNodeOpened)(void *, int),
              void (*fct_setEbtNodeClosed)(void *, int),
              int (*fct_ebtNodeOpened)(void *),
              int (*fct_ebtNodeClosed)(void *),
              int (*fct_ebtBestNode)(void *, void *),
              void* (*fct_getNodeListEdges)(void *),
              void* (*fct_getEdgeFinalNode)(void *),
              void* (*fct_getEdge)(void *),
              void* (*fct_getNextEdge)(void *),
              double(*fct_getNodeG)(void*),
              double(*fct_getEdgeCost)(void *),
              void (*fct_updateNode)(void *, void *, void *, double, double)) {
  void *open = NULL;
  int num_dev  = 0;
  void *bestNodePt = NULL, *V = NULL, *startNode = NULL;
  void *listEdgePt = NULL;
  double hCurrent = 0.0, gCurrent = 0.0;
  /*   double edgeCost; */
  if (!(startNode = fct_validSearch((void*)graph))) {
    fprintf(stderr, "Warning: start/goal undefined\n");
    return(FALSE);
  }
  fct_initSearch((void *)graph);
  EBTInsertNode((pEBTNode *)(&open), (char *)startNode, fct_ebtBestNode);
  fct_setEbtNodeOpened(startNode, TRUE);
  fct_setEbtNodeClosed(startNode, TRUE);
  while (TRUE) {
    if (EBT_EMPTY(open)) {
      /* The path doesn't exist */
      fct_recordSolution(NULL, num_dev, graph);
      return(FALSE);
    }

    EBT_GET_BEST(bestNodePt, &open);
    fct_setEbtNodeOpened(bestNodePt, FALSE);

    if (fct_endSearch(bestNodePt, graph, fct_end) == TRUE) {
      /* A path has been found */
      fct_recordSolution(bestNodePt, num_dev, graph);

      /*deasallocation of the memory*/
      while (!EBT_EMPTY(open)) {
        EBT_GET_BEST(V, &open);
        fct_setEbtNodeOpened(V, FALSE);
      }
      return(TRUE);
    }

    fct_setEbtNodeClosed(bestNodePt, TRUE);
    num_dev++;
    /* No solution found yet, bestNode
       has to be developped */
    listEdgePt = fct_getNodeListEdges(bestNodePt);
    /*Propagation along all the outcoming nodes*/
    while (listEdgePt != NULL) {
      V = fct_getEdgeFinalNode(listEdgePt);
      if (fct_valid != NULL) {
        if (!(*fct_valid)(V, fct_getEdge(listEdgePt), graph)) {
          /*We have a validation funcion which invalidate the edge*/
          /*the arc is not considered*/
          listEdgePt = fct_getNextEdge(listEdgePt);
          continue;
        }
      }
      if (fct_isNodeInPath(bestNodePt, V) == TRUE) {
        listEdgePt = fct_getNextEdge(listEdgePt);
        continue;
      }
      gCurrent = fct_getNodeG(bestNodePt);

      hCurrent = fct_getEdgeCost(listEdgePt);//fct_computeHeurist(V, fct_heurist, graph);
      if ((fct_ebtNodeOpened(V) == FALSE) && (fct_ebtNodeClosed(V) == FALSE)) {
        fct_updateNode(V, bestNodePt, listEdgePt, gCurrent, hCurrent);
        //  listEdgePt->E->cost = edgeCost;
        EBTInsertNode((pEBTNode *)(&open), (char *)V, fct_ebtBestNode);
        fct_setEbtNodeOpened(V, TRUE);
        fct_setEbtNodeClosed(V, TRUE);
      } else {
        if (fct_getNodeG(V) > gCurrent) {
          fct_updateNode(V, bestNodePt, listEdgePt, gCurrent, hCurrent);
          //listEdgePt->E->cost = edgeCost;
          if (fct_ebtNodeClosed(V) == FALSE) {
            EBT_INSERT(V, &open);
            fct_setEbtNodeOpened(V, TRUE);
            fct_setEbtNodeClosed(V, TRUE);
          }
        }
      }
      listEdgePt = fct_getNextEdge(listEdgePt);
    }
  }
}



/**
 * Graph search function that find the
 * path such that the highest cost is minimised,
 * and all the successive lower cost in case of equality
 * between two paths.
 * @param graph The graph to extract from the path.
 * @param (* fct_validSearch)(void *) Check if we have a start and a goal node in the graph structure.
 * @param (* fct_initSearch)(void *) Init the graph structure and the nodes for the search.
 * @param (* fct_endSearch)(void *, void *, int (* fct_end)(void *, void *)) Detect if we reatch the goal node.
 * @param (* fct_recordSolution)(void *, int , void *) Save the path into the graph structure.
 * @param (* fct_isNodeInPath)(void *, void *) Check if the node is in the current generated path.
 * @param (* fct_computeHeurist)(void *, double (* fct_heurist)(void *, void *), void *) Some test before calling the fct_heurist.
 * @param (* fct_heurist)(void *, void *) The heuristic function evaluating the cost to reach the goal from the
 * given node (can take for exemple the distance to the goal).
 * @param (* fct_valid)(void *, void *, void *) A function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing edges are valid).
 * @param (* fct_end)(void *, void *) Function testing if node extended reached the goal.
 * @param (* fct_setEbtNodeOpened)(void *, int ) Set node opened value for the EBT.
 * @param (* fct_setEbtNodeClosed)(void *, int ) Set node closed value for the EBT.
 * @param (* fct_ebtNodeOpened)(void *) Get the node opened value.
 * @param (* fct_ebtNodeClosed)(void *) Get the node closed value.
 * @param (* fct_getNodeListEdges)(void *) Get the list of edges connecting the node.
 * @param (* fct_getEdgeFinalNode)(void *) Get the the final node of the given edge.
 * @param (* fct_getEdge)(void *) Get the edge from the listEdge.
 * @param (* fct_getNextEdge)(void *) Get the next Edge of the list.
 * @param (* fct_getNodeEdgeCostList)(void *) Get the edge cost list of a node.
 * @param (* fct_updateNode)(void *, void *, void *, double , double ) Update the node's astar variables.
 * @return TRUE if a solution path has been found and FALSE otherwise.
 */

int p3d_orderingSearch(void *graph,
                       void* (*fct_validSearch)(void *),
                       void (*fct_initSearch)(void *),
                       int (*fct_endSearch)(void *, void *, int (*fct_end)(void *, void *)),
                       void (*fct_recordSolution)(void *, int, void*),
                       int (*fct_isNodeInPath)(void *, void *),
                       double(*fct_computeHeurist)(void *, double(*fct_heurist)(void *, void *), void *),
                       double(*fct_heurist)(void *, void *),
                       int (*fct_valid)(void *, void *, void *),
                       int (*fct_end)(void *, void *),
                       void (*fct_setEbtNodeOpened)(void *, int),
                       void (*fct_setEbtNodeClosed)(void *, int),
                       int (*fct_ebtNodeOpened)(void *),
                       int (*fct_ebtNodeClosed)(void *),
                       void* (*fct_getNodeListEdges)(void *),
                       void* (*fct_getEdgeFinalNode)(void *),
                       void* (*fct_getEdge)(void *),
                       void* (*fct_getNextEdge)(void *),
                       dbl_list*(*fct_getNodeEdgeCostList)(void *),
                       void (*fct_updateNode)(void *, void *, void *, double, double)) {
  void *open = NULL;
  int num_dev  = 0, res = 0;
  void *bestNodePt = NULL, *V = NULL, *startNode = NULL;
  void *listEdgePt = NULL;
  dbl_list* tmpOrderCostListEdge = NULL, *tmpList = NULL;

  if (!(startNode = fct_validSearch(graph))) {
    fprintf(stderr, "Warning: start/goal undefined\n");
    return(FALSE);
  }
  fct_initSearch(graph);
  EBT_INSERT(startNode, &open);
  fct_setEbtNodeOpened(startNode, TRUE);
  fct_setEbtNodeClosed(startNode, TRUE);
  tmpList = fct_getNodeEdgeCostList(startNode);
  tmpList = dbl_list_pointer_init();
  while (TRUE) {
    if (EBT_EMPTY(open)) {
      /* The path doesn't exist */
      fct_recordSolution(NULL, num_dev, graph);
      return(FALSE);
    }

    EBT_GET_BEST(bestNodePt, &open);
    fct_setEbtNodeOpened(bestNodePt, FALSE);

    if (fct_endSearch(bestNodePt, graph, fct_end) == TRUE) {
      /* A path has been found */
      fct_recordSolution(bestNodePt, num_dev, graph);

      /*deasallocation of the memory*/
      while (!EBT_EMPTY(open)) {
        EBT_GET_BEST(V, &open);
        fct_setEbtNodeOpened(bestNodePt, FALSE);
      }
      return(TRUE);
    }

    fct_setEbtNodeClosed(bestNodePt, TRUE);
    num_dev++;
    /* No solution found yet, bestNode
       has to be developped */

    listEdgePt = fct_getNodeListEdges(bestNodePt);
    /*Propagation along all the outcoming nodes*/
    while (listEdgePt != NULL) {
      V = fct_getEdgeFinalNode(listEdgePt);
      if (fct_valid != NULL) {
        if (!(*fct_valid)(V, fct_getEdge(listEdgePt), graph)) {
          /*We have a validation funcion which invalidate the edge*/
          /*te arc is not considered*/
          listEdgePt = fct_getNextEdge(listEdgePt);
          continue;
        }
      }
      if ((fct_ebtNodeOpened(V) == FALSE) && (fct_ebtNodeClosed(V) == FALSE)) {
        fct_updateNode(V, bestNodePt, listEdgePt, 0.0, 0.0);

        tmpList = fct_getNodeEdgeCostList(V);
        tmpList = dbl_list_copy(fct_getNodeEdgeCostList(bestNodePt));
        dbl_list_insert_sorted_link(tmpList,  fct_getEdge(listEdgePt), sortCostBestEdge);

        EBT_INSERT(V, &open);
        fct_setEbtNodeOpened(V, TRUE);
        fct_setEbtNodeClosed(V, TRUE);
      } else {
        tmpOrderCostListEdge =  dbl_list_copy(fct_getNodeEdgeCostList(bestNodePt));
        dbl_list_insert_sorted_link(tmpOrderCostListEdge, fct_getEdge(listEdgePt) , sortCostBestEdge);
        res = dbl_list_test_equal(tmpOrderCostListEdge, fct_getNodeEdgeCostList(V), costBestEdge);
        if (res < 0) {
          dbl_list_destroy(fct_getNodeEdgeCostList(V));
          tmpList = fct_getNodeEdgeCostList(V);
          tmpList = tmpOrderCostListEdge;

          fct_updateNode(V, bestNodePt, listEdgePt, 0.0, 0.0);

          if (fct_ebtNodeClosed(V) == FALSE) {
            fct_setEbtNodeClosed(V, FALSE);
            EBT_INSERT(V, &open);
            fct_setEbtNodeOpened(V, TRUE);
            fct_setEbtNodeClosed(V, TRUE);
          }
        } else {
          dbl_list_destroy(tmpOrderCostListEdge);
        }
      }
      listEdgePt = fct_getNextEdge(listEdgePt);
    }
  }
}


/**********************************************************************/
static int MAX_NB_TEST_PATH = 10;
static int NNODE_MAX = 10;

/**
 * Function of the A* algorithm (astar),
 * extracting a group of solution paths from a graph structure.
 * The algorithm is based on Equilibrated Binary Trees
 * (EBT) in order to increase the efficiency (see ebt.c)
 * @param graph The graph to extract from the path.
 * @param traj The returned Trajectory (not sure).
 * @param (* fct_validSearch)(void *) Check if we have a start and a goal node in the graph structure.
 * @param (* fct_initSearch)(void *) Init the graph structure and the nodes for the search.
 * @param (* fct_endSearch)(void *, void *, int (* fct_end)(void *, void *)) Detect if we reatch the goal node.
 * @param (* fct_recordSolution)(void *, int , void *) Save the path into the graph structure.
 * @param (* fct_isNodeInPath)(void *, void *) Check if the node is in the current generated path.
 * @param (* fct_computeHeurist)(void *, double (* fct_heurist)(void *, void *), void *) Some test before calling the fct_heurist.
 * @param (* fct_heurist)(void *, void *) The heuristic function evaluating the cost to reach the goal from the
 * given node (can take for exemple the distance to the goal).
 * @param (* fct_valid)(void *, void *, void *) A function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing edges are valid).
 * @param (* fct_end)(void *, void *) Function testing if node extended reached the goal.
 * @param (* fct_setEbtNodeOpened)(void *, int ) Set node opened value for the EBT.
 * @param (* fct_setEbtNodeClosed)(void *, int ) Set node closed value for the EBT.
 * @param (* fct_ebtNodeOpened)(void *) Get the node opened value.
 * @param (* fct_ebtNodeClosed)(void *) Get the node closed value.
 * @param (* fct_getNodeListEdges)(void *) Get the list of edges connecting the node.
 * @param (* fct_getEdgeInitialNode)(void *) Get the the initial node of the given edge.
 * @param (* fct_getEdgeFinalNode)(void *) Get the the final node of the given edge.
 * @param (* fct_getEdge)(void *) Get the edge from the listEdge.
 * @param (* fct_getNextEdge)(void *) Get the next Edge of the list.
 * @param (* fct_isReductibleCycle)(void *, dbl_list *, void *) Test if the selected path is reductible or not.
 * @param (* fct_NodeNbEdges)(void *) Get the number of edges starting from a given node.
 * @param (* fct_updateNodeMany)(p3d_path_nodes *, double , void *, void *) Update the node's astar variables
 * @return TRUE if a solution path has been found and FALSE otherwise.
 */

int p3d_astar_many(void *graph, void* traj,
                          void* (*fct_validSearch)(void *),
                          void (*fct_initSearch)(void *),
                          int (*fct_endSearch)(void *, void *, int (*fct_end)(void *, void *)),
                          void (*fct_recordSolution)(void *, int, void*),
                          int (*fct_isNodeInPath)(void *, void *),
                          double(*fct_computeHeurist)(void *, double(*fct_heurist)(void *, void *), void *),
                          double(*fct_heurist)(void *, void *),
                          int (*fct_valid)(void *, void *, void *),
                          int (*fct_end)(void *, void *),
                          void (*fct_setEbtPathNodeOpened)(void *, int),
                          void (*fct_setEbtPathNodeClosed)(void *, int),
                          void* (*fct_getNodeListEdges)(void *),
                          void* (*fct_getEdgeInitialNode)(void *),
                          void* (*fct_getEdgeFinalNode)(void *),
                          void* (*fct_getEdge)(void *),
                          void* (*fct_getNextEdge)(void *),
                          int (*fct_isReductibleCycle)(void *, dbl_list *, void *),
                          int (*fct_NodeNbEdges)(void*),
                          void (*fct_updateNodeMany)(p3d_path_nodes *, double, void *, void *)) {
  void *open = NULL;
  int num_dev = 0, i = 0, nb_path = 0;
  void *U_best = NULL, *V = NULL, *startNode = NULL;
  void *list_edgePt = NULL;
  p3d_path_nodes *nodeOfPath = NULL, *NewNodeOfPath = NULL;

  if (!(startNode = fct_validSearch(graph))) {
    fprintf(stderr, "Warning: start/goal undefined\n");
    return(FALSE);
  }
  fct_initSearch(graph);

  // insertion the first node
  nodeOfPath = p3d_allocinit_path_nodes();
  nodeOfPath->list_node = dbl_list_pointer_init();
  dbl_list_concat_link(nodeOfPath->list_node, startNode);
  nodeOfPath->N = startNode;
  nodeOfPath->nnode = 1;
  nodeOfPath->h = fct_computeHeurist(nodeOfPath->N, fct_heurist, graph);//distance entre graph->search_start et graph->search_goal
  nodeOfPath->g = 0.0;
  nodeOfPath->f = nodeOfPath->h + nodeOfPath->g;

  EBT_INSERT_PATH(nodeOfPath, &open);//mise dans l'arbre binaire
  fct_setEbtPathNodeOpened(nodeOfPath, TRUE);
  fct_setEbtPathNodeClosed(nodeOfPath, TRUE);

  while (TRUE) {
    if (EBT_EMPTY(open)) {//si l'arbre est vide
      return(FALSE);
    }
    if (nb_path > MAX_NB_TEST_PATH) {//si le nombre limit de tests a ete atteint
      while (!EBT_EMPTY(open)) {//vider l'arbre et desalouer les noeuds
        EBT_GET_BEST_PATH(nodeOfPath, &open);
        fct_setEbtPathNodeOpened(nodeOfPath, FALSE);
        p3d_destroy_path_nodes(nodeOfPath);
      }
      return(FALSE);
    }
    EBT_GET_BEST_PATH(nodeOfPath, &open);//remplissage de nodeOfPath et supression du meilleur element de open
    fct_setEbtPathNodeOpened(nodeOfPath, FALSE);
    U_best = nodeOfPath->N;//meilleur noeud

    if (fct_endSearch(U_best, graph, fct_end) == TRUE) {// si U_best == graph->search_goal
      nb_path++;
      if (fct_isReductibleCycle(graph, nodeOfPath->list_node, traj)) {
        /*the cycle is reductible */
        while (!EBT_EMPTY(open)) {
          EBT_GET_BEST_PATH(nodeOfPath, &open);
          fct_setEbtPathNodeOpened(nodeOfPath, FALSE);
          p3d_destroy_path_nodes(nodeOfPath);
        }
        return(TRUE);
      }
    } else {
      /* U_best has to be developped */
      if (nodeOfPath->nnode < NNODE_MAX) {
        num_dev++;
        /* consider all the edges starting from U_best */
        list_edgePt = fct_getNodeListEdges(U_best);
        for (i = 1;i <= fct_NodeNbEdges(U_best);i++) {
          /* consider all the neigbors of U_best */
          if ((!dbl_list_find_by_data(nodeOfPath->list_node, fct_getEdgeFinalNode(list_edgePt), NULL))) { //Nf appartient a path_nodes->list_node
            V = fct_getEdgeFinalNode(list_edgePt);
            /* Check if this arc has to be considered */
            if ((fct_valid != NULL) && ((*fct_valid)(V, fct_getEdge(list_edgePt), graph))){
              NewNodeOfPath = p3d_allocinit_path_nodes();
              NewNodeOfPath->list_node = dbl_list_copy(nodeOfPath->list_node);
              dbl_list_concat_link(NewNodeOfPath->list_node, V);
              NewNodeOfPath->nnode = nodeOfPath->nnode + 1;
              NewNodeOfPath->N = V;
              NewNodeOfPath->h = fct_computeHeurist(V, fct_heurist, graph);
              fct_updateNodeMany(NewNodeOfPath, nodeOfPath->g, list_edgePt, U_best);
              NewNodeOfPath->f = NewNodeOfPath->h + NewNodeOfPath->g;
              EBT_INSERT_PATH(NewNodeOfPath, &open);
              fct_setEbtPathNodeOpened(NewNodeOfPath, TRUE);
              fct_setEbtPathNodeClosed(NewNodeOfPath, TRUE);
            }
          }
          /* Possibilite de faire la desallocation a verifier*/
          list_edgePt = fct_getNextEdge(list_edgePt);
        }
      }
    }
    p3d_destroy_path_nodes(nodeOfPath);
    nodeOfPath  = NULL;
  }
}

/**
 * Allocate the p3d_path_nodes structure
 * @return the allocated pathNode
 */

static p3d_path_nodes*  p3d_allocinit_path_nodes(void) {
  p3d_path_nodes* path_nodes;
  path_nodes = MY_ALLOC(p3d_path_nodes, 1);

  path_nodes->f = 0.0;
  path_nodes->g = 0.0;
  path_nodes->h = 0.0;
  path_nodes->opened = 0;
  path_nodes->closed = 0;
  path_nodes->nnode = 0;
  path_nodes->N = NULL;

  return path_nodes;
}

/**
 * Destroy the given p3d_path_nodes
 * @param path_nodes the variable to destroy
 */

static void p3d_destroy_path_nodes(p3d_path_nodes* path_nodes) {
  dbl_list_destroy(path_nodes->list_node);
  path_nodes->list_node = NULL;
  MY_FREE(path_nodes, p3d_path_nodes, 1);
}
