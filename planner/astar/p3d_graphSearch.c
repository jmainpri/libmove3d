#include "P3d-pkg.h"
#include "Planner-pkg.h"

/**
 * p3d_graph_search
 * Main function of the A* algorithm (astar), 
 * extracting a solution path from a graph structure.
 *  The algorithm is based on Equilibrated Binary Trees
 * (EBT) in order to increase the efficiency (see ebt.c) 
 * @param graph: the graph to extract from the path.
 * @param (*fct_heurist): the heuristic function evaluating 
 * the cost to reach the goal from the given node (can take for exemple
 * the distance to the goal)
 * @param (*fct_valid): a function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing
 * edges are valid).
 * @param (*fct_end): Function testing if node extended reached the goal.
 * @param graphType: The type of graph used.
 * @return: TRUE if a solution path has been found and FALSE otherwise. 
 */

int p3d_graph_search(void *graph,
         double (*fct_heurist)(void *, void *),
         int (*fct_valid)(void *, void *, void *),
         int (*fct_end)(void *, void *), int graphType) {
  switch (graphType){
    case DEFAULTGRAPH :{
      return p3d_astar(graph, p3d_validSearch, p3d_initSearch, p3d_endSearch, p3d_recordSolution, p3d_isNodeInPath, p3d_computeHeurist, p3d_heurist, p3d_valid, p3d_end, p3d_setEbtNodeOpened, p3d_setEbtNodeClosed, p3d_ebtNodeOpened, p3d_ebtNodeClosed, ebtBestNode, p3d_getNodeListEdges, p3d_getEdgeFinalNode, p3d_getEdge, p3d_getNextEdge, p3d_getNodeG, p3d_getListEdgeCost, p3d_updateNode);
    }
#ifdef MULTIGRAPH
    case MGGRAPH :{
      return p3d_astar(graph, p3d_mgValidSearch, p3d_mgInitSearch, p3d_mgEndSearch, p3d_mgRecordSolution, p3d_mgIsNodeInPath, p3d_mgComputeHeurist, p3d_mgHeurist, p3d_valid, p3d_mgEnd, p3d_setEbtMgNodeOpened, p3d_setEbtMgNodeClosed, p3d_ebtMgNodeOpened, p3d_ebtMgNodeClosed, ebtBestMgNode, p3d_getMgNodeListEdges, p3d_getMgEdgeFinalNode, p3d_getMgEdge, p3d_getNextMgEdge, p3d_getMgNodeG, p3d_getMgListEdgeCost, p3d_updateMgNode);
    }
#endif
    default :{
      return p3d_astar(graph, p3d_validSearch, p3d_initSearch, p3d_endSearch, p3d_recordSolution, p3d_isNodeInPath, p3d_computeHeurist, p3d_heurist, p3d_valid, p3d_end, p3d_setEbtNodeOpened, p3d_setEbtNodeClosed, p3d_ebtNodeOpened, p3d_ebtNodeClosed,  ebtBestNode, p3d_getNodeListEdges, p3d_getEdgeFinalNode, p3d_getEdge, p3d_getNextEdge, p3d_getNodeG, p3d_getListEdgeCost, p3d_updateNode);
    }
  }
}

/**
 * Graph search function that find the
 * path such that the highest cost is minimised,
 * and all the successive lower cost in case of equality
 * between two paths.
 * @param graph: the graph to extract from the path.
 * @param (*fct_heurist): the heuristic function evaluating
 * the cost to reach the goal from the given node (can take for exemple
 * the distance to the goal)
 * @param (*fct_valid): a function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing
 * edges are valid).
 * @param (*fct_end): Function testing if node extended reached the goal.
 * @param graphType: The type of graph used.
 * @return: TRUE if a solution path has been found and FALSE otherwise.
 *
 */
int p3d_OrderingGraphSearch(void *graph,
         double (*fct_heurist)(void *, void *),
         int (*fct_valid)(void *, void *, void *),
         int (*fct_end)(void *, void *), int graphType) {
  switch (graphType){
    case DEFAULTGRAPH :{
      return p3d_orderingSearch(graph, p3d_validSearch, p3d_initSearch, p3d_endSearch, p3d_recordSolution, p3d_isNodeInPath, p3d_computeHeurist, p3d_heurist, p3d_valid, p3d_end, p3d_setEbtNodeOpened, p3d_setEbtNodeClosed, p3d_ebtNodeOpened, p3d_ebtNodeClosed, p3d_getNodeListEdges, p3d_getEdgeFinalNode, p3d_getEdge, p3d_getNextEdge, p3d_getNodeEdgeCostList, p3d_updateNode);
    }
#ifdef MULTIGRAPH
    case MGGRAPH :{
      return FALSE;
    }
#endif
    default :{
      return p3d_orderingSearch(graph, p3d_validSearch, p3d_initSearch, p3d_endSearch, p3d_recordSolution, p3d_isNodeInPath, p3d_computeHeurist, p3d_heurist, p3d_valid, p3d_end, p3d_setEbtNodeOpened, p3d_setEbtNodeClosed, p3d_ebtNodeOpened, p3d_ebtNodeClosed, p3d_getNodeListEdges, p3d_getEdgeFinalNode, p3d_getEdge, p3d_getNextEdge, p3d_getNodeEdgeCostList, p3d_updateNode);
    }
  }
}

/**
 * Function of the A* algorithm (astar),
 * extracting a group of solution paths from a graph structure.
 * The algorithm is based on Equilibrated Binary Trees
 * (EBT) in order to increase the efficiency (see ebt.c)
 * @param graph: the graph to extract from the path.
 * @param traj: The returned Trajectory (not sure).
 * @param (*fct_heurist): the heuristic function evaluating
 * the cost to reach the goal from the given node (can take for exemple
 * the distance to the goal)
 * @param (*fct_valid): a function testing if an edge/arc is allowed.
 * Currently this function doesn't make any collision tes (ie. assume that the existing
 * edges are valid).
 * @param (*fct_end): Function testing if node extended reached the goal.
 * @param graphType: The type of graph used.
 * @return: TRUE if a solution path has been found and FALSE otherwise.
 *
 */
int p3d_graph_many_search(void *graph, double (*fct_heurist)(void *, void *),
                          int (*fct_valid)(void *, void *, void *),int (*fct_end)(void *, void *), void* traj, int graphType){
  switch (graphType){
    case DEFAULTGRAPH :{
      return p3d_astar_many(graph, traj, p3d_validSearch, p3d_initSearch, p3d_endSearch, p3d_recordSolution, p3d_isNodeInPath, p3d_computeHeurist, p3d_heurist, p3d_valid, p3d_end, p3d_setEbtPathNodeOpened, p3d_setEbtPathNodeClosed,p3d_getNodeListEdges, p3d_getEdgeInitialNode, p3d_getEdgeFinalNode, p3d_getEdge, p3d_getNextEdge, p3d_isReductibleCycle, p3d_NodeNbEdges, p3d_updateNodeMany);
    }
#ifdef MULTIGRAPH
    case MGGRAPH :{
      return FALSE;
    }
#endif
    default :{
      return p3d_astar_many(graph, traj, p3d_validSearch, p3d_initSearch, p3d_endSearch, p3d_recordSolution, p3d_isNodeInPath, p3d_computeHeurist, p3d_heurist, p3d_valid, p3d_end, p3d_setEbtPathNodeOpened, p3d_setEbtPathNodeClosed,p3d_getNodeListEdges, p3d_getEdgeInitialNode, p3d_getEdgeFinalNode, p3d_getEdge, p3d_getNextEdge, p3d_isReductibleCycle, p3d_NodeNbEdges, p3d_updateNodeMany);
    }
  }
}
