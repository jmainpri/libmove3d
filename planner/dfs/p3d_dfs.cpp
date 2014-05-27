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
#include "./include/p3d_dfs.h"

/**
 * @brief Default constructor
 */
Dfs::Dfs(){
}

/**
 * @brief Default destructor
 */
Dfs::~Dfs(){
}

/**
 * @brief Deep-First Search main function. This fuction is used to search nodes in the graph connected to the given node througth one or more valid edges.
 * @param graph The graph to search inside
 * @param node The node to search from
 */
void Dfs::deepFirstSearch(void* graph, void* node){
  void* successor;        /* successor vertex */

  this->setDiscovered(node, true);
  this->processNode(node);

  for (int i = 0; i < this->getNodeNbEdges(node); i++) {
    if (this->validEdge(node, i) == true) {
      successor = this->getEdgeNode(node, i);
      if (this->getDiscovered(successor) == false) {
        this->deepFirstSearch(graph, successor);
      } else {
        if (this->getProceeded(node) == false){
          this->processEdge(node, i);
        }
      }
    }
  }
  this->setProceeded(node, true);
}
