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

extern int p3d_astar(void *graph,
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
              void (*fct_updateNode)(void *, void *, void *, double, double));

extern int p3d_orderingSearch(void *graph,
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
                      dbl_list* (*fct_getNodeEdgeCostList)(void *),
                      void (*fct_updateNode)(void *, void *, void *, double, double));

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
                    void (*fct_updateNodeMany)(p3d_path_nodes *, double, void *, void *));
#endif /* __CEXTRACT__ */
