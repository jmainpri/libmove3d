#include "Planner-pkg.h"

static int checkGraphValidity(p3d_graph ** g, p3d_env* env, p3d_rob * robot, xmlNodePtr cur);
static int readGraph(p3d_graph * graph, xmlNodePtr parent);
static int readGraphInfos(p3d_graph * graph, xmlNodePtr cur);
static int readXmlComp(p3d_graph* graph, xmlNodePtr cur, xmlNodePtr* neigTab);
static int readXmlNode(p3d_graph* graph, p3d_compco * comp, xmlNodePtr cur, xmlNodePtr* neigTab);
static int readXmlNodeInfos(p3d_node* node, xmlNodePtr cur);
static int readXmlIkSol(p3d_rob *robot, p3d_node *node, xmlNodePtr cur);
static int readXmlConfig(p3d_rob *robot, p3d_node *node, xmlNodePtr cur);
static int processXmlEdges(p3d_graph* graph, xmlNodePtr* neigTab);
static int readXmlEdges(p3d_graph* graph, p3d_node *node, xmlNodePtr cur);
static int readXmlEdgeNodes(p3d_graph *graph, p3d_node *node, xmlNodePtr cur);

int p3d_readDefaultGraph(xmlNodePtr cur, const char *file){
  p3d_graph *graph = XYZ_GRAPH;
  XYZ_GRAPH = NULL;
  if (graph != NULL){
    graph->rob->GRAPH = NULL;
  }
  if (!checkGraphValidity(&XYZ_GRAPH, (p3d_env*)p3d_get_desc_curid(P3D_ENV), (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT), cur) || !readGraph(XYZ_GRAPH, cur)){
    if (graph!=NULL) {  /* Restauration de l'ancien graphe */
      XYZ_GRAPH = graph;
      graph->rob->GRAPH = graph;
    }
    return FALSE;
  }
  printf("Graph parsed sucessfully\n");
  if (graph!=NULL)    /* Effacement de l'ancien graphe */
    { p3d_del_graph(graph); }
  XYZ_GRAPH->rob->GRAPH = XYZ_GRAPH;
  XYZ_GRAPH->file = MY_STRDUP(file);
  return TRUE;
}

static int readGraph(p3d_graph * graph, xmlNodePtr parent){
  xmlNodePtr cur = parent, *neigTab = NULL;
  int nnodes = 0;
  if(!readGraphInfos(graph, cur)){
    printf("Error in graph parse: Can not read the graph infos\n");
    return FALSE;
  }
  nnodes = graph->nnode;
  graph->nnode = 0; //reset the number of node (we increment the graph->nnode at each node addition)
  neigTab = MY_ALLOC(xmlNodePtr, nnodes);
  for(int i = 0; i < nnodes; i++){
    neigTab[i] = NULL;
  }
  cur = cur->xmlChildrenNode;
  for(;cur != NULL; cur = cur->next){
    if (!xmlStrcmp(cur->name, xmlCharStrdup("comp"))){
      if(!readXmlComp(graph, cur, neigTab)){
        printf("Error in graph parse: Can not read the comp\n");
        for(int i = 0; i < graph->nnode; i++){
          if(neigTab[i] != NULL){
            MY_FREE(neigTab[i], xmlNode, 1);
          }
        }
        return FALSE;
      }
    }else if(xmlStrcmp(cur->name, xmlCharStrdup("text"))){
      printf("Warning in graph parse: Unknown tag %s\n", (char*)cur->name);
    }
  }
  if(nnodes != graph->nnode){//compare the nodes numbers
    printf("Error in graph parse: All nodes are not parsed\n");
    for(int i = 0; i < graph->nnode; i++){
      if(neigTab[i] != NULL){
        MY_FREE(neigTab[i], xmlNode, 1);
      }
    }
    return FALSE;
  }
  if(!processXmlEdges(graph, neigTab)){
    printf("Error in graph parse: Can not process nodes neigbors\n");
    for(int i = 0; i < graph->nnode; i++){
      if(neigTab[i] != NULL){
        MY_FREE(neigTab[i], xmlNode, 1);
      }
    }
//     MY_FREE(neigTab,xmlNodePtr,graph->nnode);
    return FALSE;
  }
  for(int i = 0; i < graph->nnode; i++){
    if(neigTab[i] != NULL){
      MY_FREE(neigTab[i], xmlNode, 1);
    }
  }
//   MY_FREE(neigTab,xmlNodePtr,graph->nnode);
  return TRUE;
}

static int readGraphInfos(p3d_graph * graph, xmlNodePtr cur){
  xmlChar *tmp = NULL;
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numNodes"))) != NULL){
    sscanf((char *) tmp,"%d", &(graph->nnode));
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numQ"))) != NULL){
    sscanf((char *) tmp,"%d", &(graph->nb_q));
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numQClosed"))) != NULL){
    sscanf((char *) tmp,"%d", &(graph->nb_q_closed));
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numQBkbFree"))) != NULL){
    sscanf((char *) tmp,"%d", &(graph->nb_bkb_q_free));
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numQFree"))) != NULL){
    sscanf((char *) tmp,"%d", &(graph->nb_q_free));
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("time"))) != NULL){
    sscanf((char *) tmp,"%lf", &(graph->time));
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numLocalCall"))) != NULL){
    sscanf((char *) tmp,"%d", &(graph->nb_local_call));
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numTestColl"))) != NULL){
    sscanf((char *) tmp,"%d", &(graph->nb_test_coll));
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  if((tmp = xmlGetProp(cur, xmlCharStrdup("hHCount"))) != NULL){
    sscanf((char *) tmp,"%lu", &(graph->hhCount));
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  xmlFree(tmp);
  return TRUE;
}

static int checkGraphValidity(p3d_graph ** g, p3d_env* env, p3d_rob * robot, xmlNodePtr cur){
  p3d_graph *graph = p3d_allocinit_graph();
  graph->env = env;
  graph->rob = robot;

  //Compare the environment names
  if(xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("envName")), xmlCharStrdup(env->name))){
    printf("Error in graph parse: environment needed : %s , environment read : %s\n", env->name, (char*)xmlGetProp(cur, xmlCharStrdup("envName")));
    p3d_del_graph(graph);
    return FALSE;
  }
  if(xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("robotName")), xmlCharStrdup(robot->name))){
    for(int i = 0; i < env->nr; i++) {
      if(!strcmp(env->robot[i]->name, (char*)xmlGetProp(cur, xmlCharStrdup("robotName")))) {
        graph->rob = env->robot[i];
        *g = graph;
        return TRUE;
      }
    }
    printf("Error in graph parse: this graph does not match any robot in this enviroment\n");
    p3d_del_graph(graph);
    return FALSE;
  }
  *g = graph;
  return TRUE;
}

static int readXmlComp(p3d_graph* graph, xmlNodePtr cur, xmlNodePtr* neigTab){
  xmlNodePtr xmlNode = NULL;
  p3d_compco * comp = p3d_create_void_compco(graph);

  sscanf((char *) xmlGetProp(cur, xmlCharStrdup("id")),"%d", &(comp->num));

  xmlNode = cur->xmlChildrenNode;
  for(; xmlNode != NULL; xmlNode = xmlNode->next){
    if (!xmlStrcmp(xmlNode->name, xmlCharStrdup("node"))){
      if(!readXmlNode(graph, comp, xmlNode, neigTab)){
        printf("Error in comp parse: Can not read the node\n");
        p3d_remove_compco(graph, comp);
        return FALSE;
      }
    }else if(xmlStrcmp(xmlNode->name, xmlCharStrdup("text"))){
      printf("Warning in comp parse: Unknown tag %s\n", (char*)xmlNode->name);
    }
  }
  return TRUE;
}

static int readXmlNode(p3d_graph* graph, p3d_compco * comp, xmlNodePtr cur, xmlNodePtr* neigTab){
  xmlNodePtr tmp = NULL;
  p3d_node * node  = p3d_allocinit_node();
  int idNode = 0;

  if(!readXmlNodeInfos(node, cur)){
    printf("Error in node parse: Can not read the node infos\n");
    MY_FREE(node, p3d_node, 1);
    return FALSE;
  }
  idNode = node->num;

  tmp = cur->xmlChildrenNode;
  for(;tmp != NULL; tmp = tmp->next){
    if(!xmlStrcmp(tmp->name, xmlCharStrdup("iksol"))){
      if(!readXmlIkSol(graph->rob, node, tmp)){
        printf("Error in node parse: Can not read the node ikSol\n");
        MY_FREE(node, p3d_node, 1);
        return FALSE;
      }
    }
    if(!xmlStrcmp(tmp->name, xmlCharStrdup("config"))){
      if(!readXmlConfig(graph->rob, node, tmp)){
        printf("Error in node parse: Can not read the node config\n");
        MY_FREE(node, p3d_node, 1);
        return FALSE;
      }
    }
//     if(!xmlStrcmp(tmp->name, xmlCharStrdup("neighbor"))){
//       neigTab[node->num] = xmlCopyNode(tmp, 1);
//     }
    if(!xmlStrcmp(tmp->name, xmlCharStrdup("nodeEdges"))){
      neigTab[node->num] = xmlCopyNode(tmp, 1);
    }
  }

  p3d_add_node_compco(node, comp);
  p3d_insert_node_in_graph(graph, node);
  node->num = idNode;
  return TRUE;
}

static int readXmlNodeInfos(p3d_node* node, xmlNodePtr cur){
  xmlChar *tmp = NULL;
  
  if((tmp = xmlGetProp(cur, xmlCharStrdup("id"))) != NULL){
    sscanf((char *) tmp,"%d", &(node->num));
    xmlFree(tmp);
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  if((tmp = xmlGetProp(cur, xmlCharStrdup("numFailExtend"))) != NULL){
    sscanf((char *) tmp,"%d", &(node->n_fail_extend));
    xmlFree(tmp);
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  if((tmp = xmlGetProp(cur, xmlCharStrdup("weight"))) != NULL){
    sscanf((char *) tmp,"%lf", &(node->weight));
    xmlFree(tmp);
  }else{
    xmlFree(tmp);
    return FALSE;
  }
  return TRUE;
}

static int readXmlIkSol(p3d_rob *robot, p3d_node *node, xmlNodePtr cur){
  xmlChar * charTmp = NULL;
  xmlNodePtr tmp = NULL;
  int nCntrts = 0;
  if((charTmp = xmlGetProp(cur, xmlCharStrdup("num"))) != NULL){
    sscanf((char *) charTmp,"%d", &(nCntrts));
    xmlFree(charTmp);
    if(robot->cntrt_manager->ncntrts != nCntrts){
      printf("Error in ikSol parse: check the constraints number in node %d\n", node->num);
      return FALSE;
    }else{
      node->iksol = MY_ALLOC(int, nCntrts);
      for(int i = 0; i < nCntrts; i++){
        (node->iksol)[i] = 0;
      }
      tmp = cur->xmlChildrenNode;
      for(int i = 0 ; tmp != NULL; i++, tmp = tmp->next){
        if(!xmlStrcmp(tmp->name, xmlCharStrdup("cntrtSol"))){
          charTmp = xmlNodeGetContent(tmp);
          sscanf((char *)charTmp, "%d", &((node->iksol)[i]));
          xmlFree(charTmp);
        }else if(xmlStrcmp(tmp->name, xmlCharStrdup("text"))){
          printf("Warning in ikSol parse: Unknown tag %s\n", (char*)tmp->name);
        }
      }
    }
  }else{
    printf("Error in graph parse: Can not read the number of constraints\n");
    return FALSE;
  }
  return TRUE;
}

static int readXmlConfig(p3d_rob *robot, p3d_node *node, xmlNodePtr cur){
  xmlChar * charTmp = NULL;
  xmlNodePtr tmp = NULL;
  int nDof = 0;
  configPt config = NULL;

  if((charTmp = xmlGetProp(cur, xmlCharStrdup("num"))) != NULL){
    sscanf((char *) charTmp,"%d", &(nDof));
    xmlFree(charTmp);
    if(robot->nb_dof != nDof){
      printf("Error in config parse: check the DoF number in node %d\n", node->num);
      return FALSE;
    }else{
      config = p3d_alloc_config(robot);
      for(int i = 0; i < nDof; i++){
        config[i] = 0;
      }
      tmp = cur->xmlChildrenNode;
      for(int i = 0 ; tmp != NULL; tmp = tmp->next){
        if(!xmlStrcmp(tmp->name, xmlCharStrdup("dofVal"))){
          charTmp = xmlNodeGetContent(tmp);
          sscanf((char *)charTmp, "%lf", &(config[i]));
          xmlFree(charTmp);
          i++;
        }else if(xmlStrcmp(tmp->name, xmlCharStrdup("text"))){
          printf("Warning in config parse: Unknown tag %s\n", (char*)tmp->name);
        }
      }
      node->q = p3d_copy_config_deg_to_rad(robot,config);
      p3d_destroy_config(robot, config);
    }
  }else{
    printf("Error in graph parse: Can not read the number of constraints\n");
    return FALSE;
  }
  return TRUE;
}

static int processXmlEdges(p3d_graph* graph, xmlNodePtr* neigTab){
  p3d_list_node * lnode = NULL;

  for(int i = 0; i < graph->nnode; i++){
    if(neigTab[i] != NULL){
      for(lnode = graph->nodes; lnode != NULL; lnode = lnode->next){
        if(lnode->N->num == i){
          break;
        }
      }
      if(!lnode || !readXmlEdges(graph, lnode->N, neigTab[i])){
        printf("Error in graph parse: Can not read nodes neigbors\n");
        return FALSE;
      }
    }
  }
  return TRUE;
}

static int readXmlEdges(p3d_graph* graph, p3d_node *node, xmlNodePtr cur){
  xmlNodePtr edge = cur->xmlChildrenNode;

  for(; edge != NULL; edge = edge->next){
    if(!xmlStrcmp(edge->name, xmlCharStrdup("edge"))){
      readXmlEdgeNodes(graph, node, edge);
    }else if(xmlStrcmp(edge->name, xmlCharStrdup("text"))){
      printf("Warning in neighbor parse: Unknown tag %s\n", (char*)edge->name);
      return FALSE;
    }
  }
  return TRUE;
}

static int readXmlEdgeNodes(p3d_graph *graph, p3d_node *node, xmlNodePtr cur){
  xmlNodePtr edgeNode = cur->xmlChildrenNode;
  xmlChar* tmp = NULL;
  int nodeId = 0;
  double size = -1;
  p3d_list_node * lnode = NULL;
  
  for(; edgeNode != NULL; edgeNode = edgeNode->next){
    if(!xmlStrcmp(edgeNode->name, xmlCharStrdup("edgeNode"))){
      if((tmp = xmlGetProp(edgeNode, xmlCharStrdup("id"))) != NULL){
        sscanf((char *)tmp, "%d", &nodeId);
        xmlFree(tmp);
        for(lnode = graph->nodes; lnode != NULL; lnode = lnode->next){
          if(lnode->N->num == nodeId){
            break;
          }
        }
        if(size != -1){
          p3d_create_one_edge(graph, node, lnode->N, size);
          return TRUE;
        }
      }else{
        printf("Error in edge parse: No id found\n");
        xmlFree(tmp);
        return FALSE;
      }
    }else if(!xmlStrcmp(edgeNode->name, xmlCharStrdup("localpath"))){
      if((tmp = xmlGetProp(edgeNode, xmlCharStrdup("size"))) != NULL){
        sscanf((char *)tmp, "%lf", &size);
        xmlFree(tmp);
        if(lnode != NULL){
          p3d_create_one_edge(graph, node, lnode->N, size);
          return TRUE;
        }
      }else{
        printf("Error in edge parse: No localpath size found\n");
        xmlFree(tmp);
        return FALSE;
      }
    }else if(xmlStrcmp(edgeNode->name, xmlCharStrdup("text"))){
      printf("Warning in neighbor parse: Unknown tag %s\n", (char*)edgeNode->name);
      return FALSE;
    }
  }
  return TRUE;
}
