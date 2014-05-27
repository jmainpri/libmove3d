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
#ifdef MULTIGRAPH
#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Localpath-pkg.h"

static void p3d_writeGraphNodes(p3d_rob* robot, p3d_flatSuperGraph *graph, xmlNodePtr parent);
static void writeXmlConfig(p3d_rob* robot, p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent);
static void writeXmlEdge(p3d_flatSuperGraph *graph, p3d_flatSuperGraphEdge * edge, xmlNodePtr parent);
static void writeXmlNodeEdges(p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent);
static void writeXmlNode(p3d_rob* robot, p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent);
static xmlNodePtr p3d_writeSuperGraphRootNode(p3d_flatSuperGraph * graph, xmlNodePtr root);
static void p3d_writeMultiGraphJoints(p3d_multiGraph * graph, xmlNodePtr cur);
static void writeXmlSubNodes(p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent);

void p3d_writeMultiGraph(void * graph, const char* file, xmlNodePtr root){
  p3d_multiGraph * multiGraph = (p3d_multiGraph *) graph;
  p3d_rob* robot = (p3d_rob*)p3d_sel_desc_num(P3D_ROBOT, p3d_get_rob_nid_by_name(multiGraph->robotName));

  p3d_writeMultiGraphJoints(multiGraph, root);
  //Write the graphs for each part
  for(int i = 0; i < multiGraph->nbGraphs; i++){
    p3d_writeDefaultGraph(multiGraph->graphs[i], file, root);
  }
  //Write the SuperGraph
  p3d_writeSuperGraph(robot, multiGraph->fsg, root);
}

/*   MultiGraph Writing   */
void p3d_writeMultiGraphRootNode(void * graph, xmlNodePtr root){
  p3d_multiGraph * mg = (p3d_multiGraph*) graph;
  char str[80];

  xmlNewProp (root, xmlCharStrdup("type"), xmlCharStrdup("MGGRAPH"));
  xmlNewProp (root, xmlCharStrdup("envName"), xmlCharStrdup(mg->envName));
  xmlNewProp (root, xmlCharStrdup("robotName"), xmlCharStrdup(mg->robotName));
  sprintf(str, "%d", mg->nbGraphs);
  xmlNewProp (root, xmlCharStrdup("numGraphs"), xmlCharStrdup(str));
}

static void p3d_writeMultiGraphJoints(p3d_multiGraph * graph, xmlNodePtr cur){
  xmlNodePtr node = NULL;
  char str[80];

  for(int i = 0; i < graph->nbGraphs; i++){
    node = xmlNewChild (cur, NULL, xmlCharStrdup("mgJoint"), NULL);
    sprintf(str, "%d", i);
    xmlNewProp (node, xmlCharStrdup("id"), xmlCharStrdup(str));
    sprintf(str, "%d", graph->mgJoints[i]->nbJoints);
    xmlNewProp (node, xmlCharStrdup("numJoints"), xmlCharStrdup(str));
    for(int j = 0; j < graph->mgJoints[i]->nbJoints; j++){
      sprintf(str, "%d", graph->mgJoints[i]->joints[j]);
      xmlNewChild (node, NULL, xmlCharStrdup("joint"), xmlCharStrdup(str));
    }
  }
}

/*   SuperGraph Writing   */

void p3d_writeSuperGraph(p3d_rob* robot, p3d_flatSuperGraph * graph, xmlNodePtr root){
  xmlNodePtr cur = NULL;
  //Write root node
  cur = p3d_writeSuperGraphRootNode(graph, root);
  //Write The compco
  p3d_writeGraphNodes(robot, graph, cur);
}

static xmlNodePtr p3d_writeSuperGraphRootNode(p3d_flatSuperGraph * graph, xmlNodePtr root){
  xmlNodePtr node = NULL;
  char str[80];

  node = xmlNewChild (root, NULL, xmlCharStrdup("graph"), NULL);
  xmlNewProp (node, xmlCharStrdup("type"), xmlCharStrdup("SUPERGRAPH"));
  sprintf(str, "%d", graph->nNodes);
  xmlNewProp (node, xmlCharStrdup("numNodes"), xmlCharStrdup(str));
  sprintf(str, "%d", graph->nEdges);
  xmlNewProp (node, xmlCharStrdup("numEdges"), xmlCharStrdup(str));
  return node;
}

static void p3d_writeGraphNodes(p3d_rob* robot, p3d_flatSuperGraph *graph, xmlNodePtr parent){
  p3d_fsgListNode *list_node = graph->nodes;

  while(list_node != NULL) {
    writeXmlNode(robot, graph,list_node->node, parent);
    list_node = list_node->next;
  }
}

static void writeXmlNode(p3d_rob* robot, p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent){
  xmlNodePtr cur = xmlNewChild(parent, NULL, xmlCharStrdup("node"), NULL);
  char str[80];

  sprintf(str, "%d", node->num);
  xmlNewProp (cur, xmlCharStrdup("id"), xmlCharStrdup(str));

  writeXmlConfig(robot, graph, node, cur);
  writeXmlSubNodes(graph, node, cur);
  writeXmlNodeEdges(graph, node, cur);
}

/*Need the default graph be declared and filled first*/
static void writeXmlSubNodes(p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent){
  xmlNodePtr cur = xmlNewChild(parent, NULL, xmlCharStrdup("subNodes"), NULL);
  char str[80];

  sprintf(str, "%d", node->nNodes);
  xmlNewProp (cur, xmlCharStrdup("numNodes"), xmlCharStrdup(str));
  for(int i = 0; i < node->nNodes; i++){
    xmlNodePtr tmp = xmlNewChild(cur, NULL, xmlCharStrdup("node"), NULL);
    sprintf(str, "%d", node->nodes[i]->num);
    xmlNewProp (tmp, xmlCharStrdup("id"), xmlCharStrdup(str));
    sprintf(str, "%d", i);
    xmlNewProp (tmp, xmlCharStrdup("idGraph"), xmlCharStrdup(str));
  }
}

static void writeXmlConfig(p3d_rob* robot, p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent){
  xmlNodePtr config = NULL;
  char str[80];
  configPt q;

  config = xmlNewChild(parent, NULL, xmlCharStrdup("config"), NULL);
  q = p3d_copy_config_rad_to_deg(robot, node->q);
  sprintf(str, "%d", robot->nb_dof);
  xmlNewProp (config, xmlCharStrdup("num"), xmlCharStrdup(str));
  for(int i=0; i < robot->nb_dof; i++){
    sprintf(str, "%f", q[i]);
    xmlNewChild(config, NULL, xmlCharStrdup("dofVal"), xmlCharStrdup(str));
  }
  p3d_destroy_config(robot, q);
}

static void writeXmlNodeEdges(p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode * node, xmlNodePtr parent){
  xmlNodePtr edges = NULL;
  char str[80];
  p3d_fsgListEdge *listEdges = NULL;

  edges = xmlNewChild(parent, NULL, xmlCharStrdup("nodeEdges"), NULL);
  sprintf(str, "%d", node->nEdges);
  xmlNewProp (edges, xmlCharStrdup("num"), xmlCharStrdup(str));
  listEdges = node->fsgEdges;
  for(; listEdges; listEdges = listEdges->next){
    writeXmlEdge(graph, listEdges->edge, edges);
  }
}

static void writeXmlEdge(p3d_flatSuperGraph *graph, p3d_flatSuperGraphEdge * edge, xmlNodePtr parent){
  xmlNodePtr xmlEdge = NULL, tmp = NULL;
  char str[80];

  xmlEdge = xmlNewChild(parent, NULL, xmlCharStrdup("edge"), NULL);

  tmp = xmlNewChild(xmlEdge, NULL, xmlCharStrdup("edgeNode"),NULL);
  sprintf(str, "%d", edge->node2->num);
  xmlNewProp(tmp, xmlCharStrdup("id"), xmlCharStrdup(str));
  /* tmp = xmlNewChild(xmlEdge, NULL, xmlCharStrdup("cost"),NULL);
    sprintf(str, "%d", edge->cost);
    xmlNewProp(tmp, xmlCharStrdup("value"), xmlCharStrdup(str)); */
}
#endif
