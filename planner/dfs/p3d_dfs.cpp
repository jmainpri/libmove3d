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
