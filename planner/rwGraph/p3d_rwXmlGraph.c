#include "Planner-pkg.h"

static void p3d_writeXmlGraph(void *graph, const char *file, void (*writeSpeGraph)(void * graph, const char *file, xmlNodePtr root));

static void p3d_writeXmlGraph(void *graph, const char *file, void (*writeSpeGraph)(void * graph, const char *file, xmlNodePtr root)){
  xmlDocPtr doc = NULL;
  xmlNodePtr root = NULL;

//Creating the file Variable version 1.0
  doc = xmlNewDoc(xmlCharStrdup("1.0"));

//Writing the graph
  root = xmlNewNode (NULL, xmlCharStrdup("move3dGraph"));
  xmlDocSetRootElement(doc, root);
  writeSpeGraph(graph, file, root);

//Writing the file on HD
  xmlSaveFormatFile (file, doc, 1);
  xmlFreeDoc(doc);
}

void p3d_writeGraph(void *graph, const char *file, int graphType){
  switch (graphType){
    case DEFAULTGRAPH:{
      p3d_writeXmlGraph(graph, file, p3d_writeDefaultGraph);
      break;
    }
    case MGGRAPH : {
      //TODO
      break;
    }
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
  }else if (!xmlStrcmp(xmlGetProp(cur, xmlCharStrdup("type")), xmlCharStrdup("MGGRAPH"))){
//     parseDone = p3d_readXmlMultiGraph(cur);
  }else{
    fprintf(stderr,"Non supported graph type or wrong format\n");
    xmlFreeDoc(doc);
    return 0;
  }
  xmlFreeDoc(doc);
  return parseDone;
}
