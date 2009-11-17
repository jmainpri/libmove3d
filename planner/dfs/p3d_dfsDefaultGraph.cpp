#include "./include/p3d_dfsDefaultGraph.h"

/**
 * @brief Default constructor
 */
DfsDefaultGraph::DfsDefaultGraph(){
  _listNode = NULL;
}
/**
 * @brief Default destructor
 */
DfsDefaultGraph::~DfsDefaultGraph(){
  this->deleteNodeList();
}

/**
 * @brief The user have to call this function to use the DFS alogorithm
 * @param graph The graph to search inside
 * @param node The node to search from
 * @return The list of node found by the dfs
 */
void* DfsDefaultGraph::p3d_dfs(void* graph, void* node){
  this->deleteNodeList();
  this->initGraph(graph);
  this->deepFirstSearch(graph, node);
  return _listNode;
}

/**
 * @brief Delete the node list (private attribute)
 */
void DfsDefaultGraph::deleteNodeList(void){
  p3d_list_node* listNode = NULL;
  for(; _listNode; _listNode = listNode){
    listNode = ((p3d_list_node*)_listNode)->next;
    MY_FREE(_listNode, p3d_list_node, 1);
  }
}

/**
 * @brief Initialize dfs attributes for all the graph nodes
 * @param g the graph to initialize
 */
void DfsDefaultGraph::initGraph(void* g){
  p3d_graph* graph = (p3d_graph*)g;
  for(p3d_list_node* listNode = graph->nodes; listNode; listNode = listNode->next){
    listNode->N->discovered = FALSE;
    listNode->N->processed = FALSE;
  }
}

/**
 * @brief Set the given nodes discovered variable to value
 * @param n The node
 * @param value The value
 */
void DfsDefaultGraph::setDiscovered(void* n, bool value){
  p3d_node* node = (p3d_node*)n;
  node->discovered = value;
}

/**
 * @brief Get the given nodes discovered variable value
 * @param n The node
 * @return The value of discoverd variable
 */
bool DfsDefaultGraph::getDiscovered(void* n){
  return ((p3d_node*)n)->discovered;
}

/**
 * @brief Set the given nodes processed variable to value
 * @param n The node
 * @param value The value
 */
void DfsDefaultGraph::setProceeded(void* n, bool value){
  p3d_node* node = (p3d_node*)n;
  node->processed = value;
}

/**
 * @brief Get the given nodes processed variable value
 * @param n The node
 * @return The value of processed variable
 */
bool DfsDefaultGraph::getProceeded(void* n){
  return ((p3d_node*)n)->processed;

}

/**
 * @brief This function determines the action to do on the found node. Here we add the node to the listNode
 * @param n The node
 */
void DfsDefaultGraph::processNode(void* n){
  p3d_node* node = (p3d_node*)n;
  _listNode = p3d_add_node_to_list(node, (p3d_list_node*)_listNode);
}

/**
 * @brief This function determines the action to do on the found edge. Here we do nothing
 * @param n The node
 * @param id The edge id in the node structure
 */
void DfsDefaultGraph::processEdge(void* n, int id){
}

/**
 * @brief Get the number of edges connecting this node his neigbours
 * @param n The node
 * @return 
 */
int DfsDefaultGraph::getNodeNbEdges(void* n){
  return ((p3d_node*)n)->nedge;
}

/**
 * @brief Get the id'th edge on the nodes edges list
 * @param n The node
 * @param id The edge id in the node structure
 * @return 
 */
void* DfsDefaultGraph::getEdgeNode(void* n, int id){
  p3d_node* node = (p3d_node*)n;
  p3d_list_edge* lEdge = node->edges;
  for(int i = 0; i < id; i++){
    lEdge = lEdge->next;
  }
  return (void*)lEdge->E->Nf;
}

/**
 * @brief Check if the edge is valid or not
 * @param n The node
 * @param id The edge id in the node structure
 * @return True if the node is valid, False otherwise
 */
bool DfsDefaultGraph::validEdge(void* n, int id){
  p3d_node* node = (p3d_node*)n;
  p3d_list_edge* lEdge = node->edges;
  for(int i = 0; i < id; i++){
    lEdge = lEdge->next;
  }
  return !lEdge->E->unvalid;
}
