#include "Planner-pkg.h"
#include "P3d-pkg.h"

static void p3d_writeXmlGraph(void *graph, const char *file, void (*writeSpeGraph)(void * graph, const char *file, xmlNodePtr root), void (*writeRootNode)(void* graph, xmlNodePtr root));

static void p3d_writeXmlGraph(void *graph, const char *file, void (*writeSpeGraph)(void * graph, const char *file, xmlNodePtr root), void (*writeRootNode)(void* graph, xmlNodePtr root)){
  xmlDocPtr doc = NULL;
  xmlNodePtr root = NULL;

//Creating the file Variable version 1.0
  doc = xmlNewDoc(xmlCharStrdup("1.0"));

//Writing the graph
  root = xmlNewNode (NULL, xmlCharStrdup("move3dGraph"));
  xmlDocSetRootElement(doc, root);
  writeRootNode(graph, root);
  writeSpeGraph(graph, file, root);

//Writing the file on HD
  xmlSaveFormatFile (file, doc, 1);
  xmlFreeDoc(doc);
}

void p3d_writeGraph(void *graph, const char *file, int graphType){
  switch (graphType){
    case DEFAULTGRAPH:{
      p3d_writeXmlGraph(graph, file, p3d_writeDefaultGraph, p3d_writeDefaultGraphRootNode);
      break;
    }
#ifdef MULTIGRAPH
    case MGGRAPH : {
      p3d_writeXmlGraph(graph, file, p3d_writeMultiGraph, p3d_writeMultiGraphRootNode);
      break;
    }
#endif
  }
}

int p3d_readGraph(const char *file, int graphType){
  xmlDocPtr doc;
  xmlNodePtr cur;
  int parseDone = FALSE;

  doc = xmlParseFile(file);
  if (doc == NULL ) {
    fprintf(stderr,"Document not parsed successfully. \n");
    return 0;
  }
  cur = xmlDocGetRootElement(doc);
  if (cur == NULL) {
    fprintf(stderr,"Empty document\n");
    xmlFreeDoc(doc);
    return 0;
  }
  if (xmlStrcmp(cur->name, xmlCharStrdup("move3dGraph"))) {
    fprintf(stderr,"Document of the wrong type\n");
    xmlFreeDoc(doc);
    return 0;
  }
  if (!xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("type")), xmlCharStrdup("DEFAULTGRAPH"))){
    parseDone = p3d_readDefaultGraph(cur->xmlChildrenNode->next, file);
  }
#ifdef MULTIGRAPH
  else if (!xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("type")), xmlCharStrdup("MGGRAPH"))){
    parseDone = p3d_readMgGraph(cur, file);
  }
#endif
  else{
    fprintf(stderr,"Non supported graph type or wrong format\n");
    xmlFreeDoc(doc);
    return 0;
  }
  xmlFreeDoc(doc);
  return parseDone;
}

configPt readXmlConfig(p3d_rob *robot, xmlNodePtr cur){
  xmlChar * charTmp = NULL;
  xmlNodePtr tmp = NULL;
  int nDof = 0;
  configPt config = NULL, q = NULL;
  
  if((charTmp = xmlGetProp(cur, xmlCharStrdup("num"))) != NULL){
    sscanf((char *) charTmp,"%d", &(nDof));
    xmlFree(charTmp);
    if(robot->nb_dof != nDof){
      printf("Error in config parse: check the DoF number\n");
      return NULL;
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
      q = p3d_copy_config_deg_to_rad(robot, config);
      p3d_destroy_config(robot, config);
    }
  }else{
    printf("Error in graph parse: Can not read the number of constraints\n");
    return NULL;
  }
  return q;
}
