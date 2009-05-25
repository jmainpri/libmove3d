#include "Planner-pkg.h"
#include "Move3d-pkg.h"

static int checkGraphValidity(p3d_multiGraph * g, p3d_env* env, p3d_rob * robot, xmlNodePtr cur);
static int readMgJoints(p3d_multiGraph * mg, xmlNodePtr parent);
static int readMultiGraph(p3d_rob* robot, p3d_multiGraph * mg, const char * file, xmlNodePtr parent);
static int readSuperGraph(p3d_flatSuperGraph *fsg, p3d_rob* robot, xmlNodePtr parent);
static int readGraphInfos(p3d_flatSuperGraph * graph, xmlNodePtr cur);
static int readXmlNode(p3d_flatSuperGraph* graph, p3d_rob* robot, xmlNodePtr cur, xmlNodePtr* edgeTab);
static int readXmlSubNodes(p3d_multiGraph * mg, p3d_node ** subNodes, xmlNodePtr parent);
static int processXmlEdges(p3d_flatSuperGraph* graph, xmlNodePtr* edgeTab);
static int readXmlEdges(p3d_flatSuperGraph* graph, p3d_flatSuperGraphNode *node, xmlNodePtr cur);
static int readXmlEdgeNodes(p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode *node, xmlNodePtr cur);

int p3d_readMgGraph(xmlNodePtr cur, const char *file) {
  p3d_rob* robot = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
  p3d_multiGraph *mg = p3d_cloneMultiGraph(robot, robot->mg);
  p3d_graph * xyzGraph = XYZ_GRAPH;
  p3d_graph * robotGraph = robot->GRAPH; //in case of the selected robot is not the right one

  XYZ_GRAPH = NULL;
  if (robot->GRAPH != NULL) {
    robot->GRAPH = NULL;
  }

  for(int i = 0; robot->mg && robot->mg->graphs && i < robot->mg->nbGraphs; i++){
    robot->mg->graphs[i] = NULL;
  }

  if (!checkGraphValidity(robot->mg, (p3d_env*)p3d_get_desc_curid(P3D_ENV), robot, cur) || !readMultiGraph(robot, robot->mg, file, cur)) {
    //error restore the old mg and XYZ_GRAPH
    p3d_del_multiGraph(robot, robot->mg);
    robot->mg = mg;
    XYZ_GRAPH = xyzGraph;
    robot->GRAPH = robotGraph;
    return FALSE;
  }

  printf("Graph parsed sucessfully\n");
  if (xyzGraph != robot->GRAPH) {/* Effacement de l'ancien graphe */
    if (robotGraph != NULL) {
      p3d_del_graph(robotGraph);
    }
  }
  if (xyzGraph != NULL) {
    p3d_del_graph(xyzGraph);
  }
  p3d_del_multiGraph(robot, mg);
  XYZ_GRAPH = NULL;
  XYZ_GRAPH = p3d_create_graph();
  p3d_convertFsgToGraph(XYZ_GRAPH, robot->mg->fsg);
  robot->GRAPH = XYZ_GRAPH;
  XYZ_GRAPH->file = MY_STRDUP(file);
  return TRUE;
}

static int readMultiGraph(p3d_rob* robot, p3d_multiGraph * mg, const char * file, xmlNodePtr parent) {
  xmlNodePtr cur = parent->xmlChildrenNode->next;

  for (int i = 0; cur; cur = cur->next) {
    if (!xmlStrcmp(cur->name, xmlCharStrdup("graph"))) {
      if (!xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("type")), xmlCharStrdup("SUPERGRAPH"))) {
        if(!mg->fsg){
          mg->fsg = MY_ALLOC(p3d_flatSuperGraph, 1);
          p3d_initFlatSuperGraph(mg->fsg);
        }
        if (!readSuperGraph(mg->fsg, robot, cur)) {
          return FALSE;
        }
      } else if (!xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("type")), xmlCharStrdup("DEFAULTGRAPH"))) {
        XYZ_GRAPH = NULL;
        robot->GRAPH = NULL;
        if (!p3d_readDefaultGraph(cur, file)) {
          return FALSE;
        }
        robot->mg->graphs[i] = XYZ_GRAPH;
        i++;
      }
    }
  }
  return TRUE;
}

static int readSuperGraph(p3d_flatSuperGraph* fsg, p3d_rob* robot, xmlNodePtr parent) {
  xmlNodePtr cur = parent, *edgeTab = NULL;
  int nbNodes = 0, nbEdges = 0;
  if (!readGraphInfos(fsg, cur)) {
    printf("Error in graph parse: Can not read the graph infos\n");
    return FALSE;
  }
  //reset the number of node and edges(we increment the graph->nnode at each node addition)
  nbNodes = fsg->nNodes;
  fsg->nNodes = 0;
  nbEdges = fsg->nEdges;
  fsg->nEdges = 0;

  edgeTab = MY_ALLOC(xmlNodePtr, nbNodes);
  for (int i = 0; i < nbNodes; i++) {
    edgeTab[i] = NULL;
  }

  cur = cur->xmlChildrenNode;
  for (;cur != NULL; cur = cur->next) {
    if (!xmlStrcmp(cur->name, xmlCharStrdup("node"))) {
      if (!readXmlNode(fsg, robot, cur, edgeTab)) {
        printf("Error in graph parse: Can not read the node\n");
        for (int i = 0; i < fsg->nNodes; i++) {
          if (edgeTab[i] != NULL) {
            MY_FREE(edgeTab[i], xmlNode, 1);
          }
        }
        return FALSE;
      }
    } else if (xmlStrcmp(cur->name, xmlCharStrdup("text"))) {
      printf("Warning in graph parse: Unknown tag %s\n", (char*)cur->name);
    }
  }
  if (nbNodes != fsg->nNodes) {//compare the nodes numbers
    printf("Error in graph parse: All nodes are not parsed\n");
    for (int i = 0; i < fsg->nNodes; i++) {
      if (edgeTab[i] != NULL) {
        MY_FREE(edgeTab[i], xmlNode, 1);
      }
    }
    return FALSE;
  }
  if (!processXmlEdges(fsg, edgeTab)) {
    printf("Error in graph parse: Can not process nodes Edges\n");
    for (int i = 0; i < fsg->nNodes; i++) {
      if (edgeTab[i] != NULL) {
        MY_FREE(edgeTab[i], xmlNode, 1);
      }
    }
    return FALSE;
  }
  for (int i = 0; i < fsg->nNodes; i++) {
    if (edgeTab[i] != NULL) {
      MY_FREE(edgeTab[i], xmlNode, 1);
    }
  }
  return TRUE;
}


static int readGraphInfos(p3d_flatSuperGraph * graph, xmlNodePtr cur) {
  xmlChar *tmp = NULL;
  if ((tmp = xmlGetProp(cur, xmlCharStrdup("numNodes"))) != NULL) {
    sscanf((char *) tmp, "%d", &(graph->nNodes));
  } else {
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  if ((tmp = xmlGetProp(cur, xmlCharStrdup("numEdges"))) != NULL) {
    sscanf((char *) tmp, "%d", &(graph->nEdges));
  } else {
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  return TRUE;
}

static int checkGraphValidity(p3d_multiGraph * mg, p3d_env* env, p3d_rob * robot, xmlNodePtr cur) {

  //Compare the environment names
  if (xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("envName")), xmlCharStrdup(env->name))) {
    printf("Error in graph parse: environment needed : %s , environment read : %s\n", env->name, (char*)xmlGetProp(cur, xmlCharStrdup("envName")));
    return FALSE;
  }
  if (xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("robotName")), xmlCharStrdup(robot->name))) {
    printf("Error in graph parse: this graph does not the selected robot\n");
    return FALSE;
  }
  if (atoi((char *)xmlGetProp(cur, xmlCharStrdup("numGraphs"))) != mg->nbGraphs) {
    printf("Error in graph parse: Graph number needed : %d , graph number read : %s\n", mg->nbGraphs, (char*)xmlGetProp(cur, xmlCharStrdup("numGraphs")));
    return FALSE;
  }
  if (!readMgJoints(mg, cur)) {
    printf("Error in graph parse: the Multigraph Joints does not missmatch\n");
    return FALSE;
  }
  return TRUE;
}

static int readMgJoints(p3d_multiGraph * mg, xmlNodePtr parent) {
  xmlNodePtr xmlMgJoint = parent->xmlChildrenNode->next;

  for (; xmlMgJoint; xmlMgJoint = xmlMgJoint->next) {
    if (!xmlStrcmp(xmlMgJoint->name, xmlCharStrdup("mgJoint"))) {
      xmlChar * charTmp = NULL;
      int mgJointId = atoi((char *)xmlGetProp(xmlMgJoint, xmlCharStrdup("id")));
      int numJoints = atoi((char *)xmlGetProp(xmlMgJoint, xmlCharStrdup("numJoints")));
      if (numJoints != mg->mgJoints[mgJointId]->nbJoints) {
        return FALSE;
      }
      xmlNodePtr xmlJoint = xmlMgJoint->xmlChildrenNode->next;
      for (int i = 0; xmlJoint != NULL; xmlJoint = xmlJoint->next) {
        if (!xmlStrcmp(xmlJoint->name, xmlCharStrdup("joint"))) {
          charTmp = xmlNodeGetContent(xmlJoint);
          if (atoi((char *)charTmp) != mg->mgJoints[mgJointId]->joints[i]) {
            return FALSE;
          }
          xmlFree(charTmp);
          i++;
        } else if (xmlStrcmp(xmlJoint->name, xmlCharStrdup("text"))) {
          printf("Warning in mgJoint parse: Unknown tag %s\n", (char*)xmlJoint->name);
        }
      }
    }
  }
  return TRUE;
}

static int readXmlNode(p3d_flatSuperGraph* graph, p3d_rob * robot, xmlNodePtr cur, xmlNodePtr* neigTab) {
  xmlNodePtr tmp = NULL;
  xmlChar * tmpChar = NULL;
  p3d_flatSuperGraphNode * node  = NULL;
  int idNode = 0;
  configPt config = NULL;
  p3d_node ** subNodes = MY_ALLOC(p3d_node*, robot->mg->nbGraphs);

  if ((tmpChar = xmlGetProp(cur, xmlCharStrdup("id"))) != NULL) {
    sscanf((char *) tmpChar, "%d", &(idNode));
    xmlFree(tmpChar);
  } else {
    xmlFree(tmpChar);
    MY_FREE(subNodes, p3d_node*, robot->mg->nbGraphs);
    return FALSE;
  }

  tmp = cur->xmlChildrenNode;
  for (;tmp != NULL; tmp = tmp->next) {
    if (!xmlStrcmp(tmp->name, xmlCharStrdup("config"))) {
      config = readXmlConfig(robot,  tmp);
      if (config == NULL) {
        printf("Error in node parse: Can not read the node config\n");
        MY_FREE(subNodes, p3d_node*, robot->mg->nbGraphs);
        return FALSE;
      }
    }else if (!xmlStrcmp(tmp->name, xmlCharStrdup("subNodes"))) {
      if (!readXmlSubNodes(robot->mg, subNodes, tmp)) {
        printf("Error in node parse: Can not read the node sub nodes\n");
        MY_FREE(subNodes, p3d_node*, robot->mg->nbGraphs);
        return FALSE;
      }
    }else if (!xmlStrcmp(tmp->name, xmlCharStrdup("nodeEdges"))) {
      neigTab[idNode] = xmlCopyNode(tmp, 1);
    }else if (xmlStrcmp(tmp->name, xmlCharStrdup("text"))) {
        printf("Warning in node parse: Unknown tag %s\n", (char*)tmp->name);
      }
  }
  node = p3d_createFlatSuperGraphNode(robot, graph, subNodes, config);
  p3d_addFsgNodeInGraph(graph, node);
  node->num = idNode;
  return TRUE;
}

static int readXmlSubNodes(p3d_multiGraph * mg, p3d_node ** subNodes, xmlNodePtr parent) {
  xmlChar * charTmp = NULL;
  xmlNodePtr tmp = NULL;

  int nbGraphs = 0;
  if ((charTmp = xmlGetProp(parent, xmlCharStrdup("numNodes"))) != NULL) {
    sscanf((char *) charTmp, "%d", &(nbGraphs));
    xmlFree(charTmp);
    if (mg->nbGraphs != nbGraphs) {
      printf("Error in subNodes parse: check the number of subNodes\n");
      return FALSE;
    } else {
      tmp = parent->xmlChildrenNode->next;
      for (; tmp != NULL; tmp = tmp->next) {
        int nodeId = 0, nodeGraph = 0;
        if (!xmlStrcmp(tmp->name, xmlCharStrdup("node"))) {
          if ((charTmp = xmlGetProp(tmp, xmlCharStrdup("id"))) != NULL) {
            sscanf((char *) charTmp, "%d", &(nodeId));
            xmlFree(charTmp);
          } else {
            printf("Error in subNodes parse: Can not read the subNode id\n");
            xmlFree(charTmp);
            return FALSE;
          }
          if ((charTmp = xmlGetProp(tmp, xmlCharStrdup("idGraph"))) != NULL) {
            sscanf((char *) charTmp, "%d", &(nodeGraph));
            xmlFree(charTmp);
          } else {
            printf("Error in subNodes parse: Can not read the subNode Graph id\n");
            xmlFree(charTmp);
            return FALSE;
          }
          subNodes[nodeGraph] = p3d_getNodeInGraphByNum(mg->graphs[nodeGraph], nodeId);
        } else if (xmlStrcmp(tmp->name, xmlCharStrdup("text"))) {
          printf("Warning in subNodes parse: Unknown tag %s\n", (char*)tmp->name);
        }
      }
    }
  } else {
    printf("Error in subNodes parse: Can not read the number of subNodes\n");
    return FALSE;
  }
  return TRUE;
}

static int processXmlEdges(p3d_flatSuperGraph* graph, xmlNodePtr* neigTab) {
  p3d_fsgListNode * lnode = NULL;

  for (int i = 0; i < graph->nNodes; i++) {
    if (neigTab[i] != NULL) {
      for (lnode = graph->nodes; lnode != NULL; lnode = lnode->next) {
        if (lnode->node->num == i) {
          break;
        }
      }
      if (!lnode || !readXmlEdges(graph, lnode->node, neigTab[i])) {
        printf("Error in graph parse: Can not read nodes neigbors\n");
        return FALSE;
      }
    }
  }
  return TRUE;
}

static int readXmlEdges(p3d_flatSuperGraph* graph, p3d_flatSuperGraphNode *node, xmlNodePtr cur) {
  xmlNodePtr edge = cur->xmlChildrenNode;

  for (; edge != NULL; edge = edge->next) {
    if (!xmlStrcmp(edge->name, xmlCharStrdup("edge"))) {
      readXmlEdgeNodes(graph, node, edge);
    } else if (xmlStrcmp(edge->name, xmlCharStrdup("text"))) {
      printf("Warning in neighbor parse: Unknown tag %s\n", (char*)edge->name);
      return FALSE;
    }
  }
  return TRUE;
}

static int readXmlEdgeNodes(p3d_flatSuperGraph *graph, p3d_flatSuperGraphNode *node, xmlNodePtr cur) {
  xmlNodePtr edgeNode = cur->xmlChildrenNode;
  xmlChar* tmp = NULL;
  int nodeId = 0;
  double size = P3D_HUGE;
  p3d_fsgListNode * lnode = NULL;

  for (; edgeNode != NULL; edgeNode = edgeNode->next) {
    if (!xmlStrcmp(edgeNode->name, xmlCharStrdup("edgeNode"))) {
      if ((tmp = xmlGetProp(edgeNode, xmlCharStrdup("id"))) != NULL) {
        sscanf((char *)tmp, "%d", &nodeId);
        xmlFree(tmp);
        for (lnode = graph->nodes; lnode != NULL; lnode = lnode->next) {
          if (lnode->node->num == nodeId) {
            break;
          }
        }
        if (/* size != -1 && */ lnode != NULL) {
          p3d_connectFsgNodes(graph, node, lnode->node, size);
          return TRUE;
        }
      } else {
        printf("Error in edge parse: No id found\n");
        xmlFree(tmp);
        return FALSE;
      }
    }/* else if(!xmlStrcmp(edgeNode->name, xmlCharStrdup("cost"))){
          if((tmp = xmlGetProp(edgeNode, xmlCharStrdup("value"))) != NULL){
            sscanf((char *)tmp, "%lf", &size);
            xmlFree(tmp);
            if(size != -1 && lnode != NULL){
              p3d_connectFsgNodes(graph, node, lnode->node, size);
              return TRUE;
            }
          } else{
        printf("Error in edge parse: size not found\n");
        xmlFree(tmp);
        return FALSE;
      }
    }*/
    else if (xmlStrcmp(edgeNode->name, xmlCharStrdup("text"))) {
      printf("Warning in edge parse: Unknown tag %s\n", (char*)edgeNode->name);
      return FALSE;
    }
  }
  return TRUE;
}
