#include "Planner-pkg.h"
#include "Collision-pkg.h"
#include "Localpath-pkg.h"
#include "P3d-pkg.h"
#include "Util-pkg.h"

static void writeXmlComp(p3d_graph *g, p3d_compco * c, xmlNodePtr parent);
static void writeXmlIkSol(p3d_graph *graph, p3d_node * node, xmlNodePtr parent);
static void writeXmlConfig(p3d_graph *graph, p3d_node * node, xmlNodePtr parent);
static void writeXmlEdge(p3d_graph *graph, p3d_edge * edge, xmlNodePtr parent);
static void writeXmlNodeEdges(p3d_graph *graph, p3d_node * node, xmlNodePtr parent);
static void writeXmlNode(p3d_graph *graph, p3d_node * node, xmlNodePtr parent);
static xmlNodePtr writeGraphRootNode(void * graph, xmlNodePtr root, const char* file);
static void p3d_writeGraphComp(void *graph, xmlNodePtr parent);
static void writeXmlLocalpath(p3d_graph *graph, p3d_edge * edge, xmlNodePtr xmlEdge);

void p3d_writeDefaultGraph(void * graph, const char* file, xmlNodePtr root){
  xmlNodePtr cur = NULL;
  //Write root node
  cur = writeGraphRootNode(graph, root, file);
  //Write The compco
  p3d_writeGraphComp(graph, cur);
}

void p3d_writeDefaultGraphRootNode(void * graph, xmlNodePtr root){
  xmlNewProp(root, xmlCharStrdup("type"), xmlCharStrdup("DEFAULTGRAPH"));
}

static xmlNodePtr writeGraphRootNode(void * g, xmlNodePtr root, const char* file){
  p3d_graph * graph = (p3d_graph*) g;
  xmlNodePtr node = NULL;
  char str[80];

  if (graph->file != NULL){
    MY_STRFREE(graph->file);
  }
  graph->file = MY_STRDUP(file);

  node = xmlNewChild (root, NULL, xmlCharStrdup("graph"), NULL);
  xmlNewProp (node, xmlCharStrdup("type"), xmlCharStrdup("DEFAULTGRAPH"));
  xmlNewProp (node, xmlCharStrdup("envName"), xmlCharStrdup(graph->env->name));
  xmlNewProp (node, xmlCharStrdup("robotName"), xmlCharStrdup(graph->rob->name));
  sprintf(str, "%d", graph->nnode);
  xmlNewProp (node, xmlCharStrdup("numNodes"), xmlCharStrdup(str));
  sprintf(str, "%d", graph->nb_q);
  xmlNewProp (node, xmlCharStrdup("numQ"), xmlCharStrdup(str));
  if(p3d_random_loop_generator_ok()) {
    sprintf(str, "%d", graph->nb_q_closed);
    xmlNewProp (node, xmlCharStrdup("numQClosed"), xmlCharStrdup(str));
    if(p3d_col_get_mode() != p3d_col_mode_bio) {   // IF MODE BIO !!!
      sprintf(str, "%d", graph->nb_bkb_q_free);
      xmlNewProp (node, xmlCharStrdup("numQBkbFree"), xmlCharStrdup(str));
    }
  }
  sprintf(str, "%d", graph->nb_q_free);
  xmlNewProp (node, xmlCharStrdup("numQFree"), xmlCharStrdup(str));
  sprintf(str, "%f", graph->time);
  xmlNewProp (node, xmlCharStrdup("time"), xmlCharStrdup(str));
  sprintf(str, "%d", graph->nb_local_call);
  xmlNewProp (node, xmlCharStrdup("numLocalCall"), xmlCharStrdup(str));
  sprintf(str, "%d", graph->nb_test_coll);
  xmlNewProp (node, xmlCharStrdup("numTestColl"), xmlCharStrdup(str));
  sprintf(str, "%lu", graph->hhCount);
  xmlNewProp (node, xmlCharStrdup("hHCount"), xmlCharStrdup(str));

  return node;
}

static void p3d_writeGraphComp(void *g, xmlNodePtr parent){
  p3d_graph* graph = (p3d_graph*)g;
  p3d_compco * comp = graph->comp;
  for(; comp; comp = comp->suiv){
    writeXmlComp(graph,comp,parent);
  }
}

static void writeXmlComp(p3d_graph *graph, p3d_compco *comp, xmlNodePtr parent){
  xmlNodePtr cur = xmlNewChild(parent, NULL, xmlCharStrdup("comp"), NULL);
  char str[80];
  p3d_list_node *list_node = NULL;

  sprintf(str, "%d", comp->num);
  xmlNewProp (cur, xmlCharStrdup("id"), xmlCharStrdup(str));

  list_node = comp->dist_nodes;
  while(list_node != NULL) {
    writeXmlNode(graph,list_node->N,cur);
    list_node = list_node->next;
  }
}

static void writeXmlIkSol(p3d_graph *graph, p3d_node * node, xmlNodePtr parent){
  xmlNodePtr iksol = NULL;
  char str[80];

  if(node->iksol){
    iksol = xmlNewChild(parent, NULL, xmlCharStrdup("iksol"), NULL);
    sprintf(str, "%d", graph->rob->cntrt_manager->ncntrts);
    xmlNewProp (iksol, xmlCharStrdup("num"), xmlCharStrdup(str));
    for(int i = 0; i < graph->rob->cntrt_manager->ncntrts; i++){
      sprintf(str, "%d", node->iksol[i]);
      xmlNewChild(iksol, NULL, xmlCharStrdup("cntrtSol"), xmlCharStrdup(str));
    }
  }
}

static void writeXmlConfig(p3d_graph *graph, p3d_node * node, xmlNodePtr parent){
  xmlNodePtr config = NULL;
  char str[80];
  configPt q;

  config = xmlNewChild(parent, NULL, xmlCharStrdup("config"), NULL);
  q = p3d_copy_config_rad_to_deg(graph->rob,node->q);
  sprintf(str, "%d", graph->rob->nb_dof);
  xmlNewProp (config, xmlCharStrdup("num"), xmlCharStrdup(str));
  for(int i=0; i < graph->rob->nb_dof; i++){
    sprintf(str, "%f", q[i]);
    xmlNewChild(config, NULL, xmlCharStrdup("dofVal"), xmlCharStrdup(str));
  }
  p3d_destroy_config(graph->rob, q);
}

static void writeXmlEdge(p3d_graph *graph, p3d_edge * edge, xmlNodePtr parent){
  xmlNodePtr xmlEdge = NULL, tmp = NULL;
  char str[80];

  xmlEdge = xmlNewChild(parent, NULL, xmlCharStrdup("edge"), NULL);

//   tmp = xmlNewChild(xmlEdge, NULL, xmlCharStrdup("edgeNode"),NULL);
//   sprintf(str, "%d", edge->Ni->num);
//   xmlNewProp(tmp, xmlCharStrdup("id"), xmlCharStrdup(str));
  tmp = xmlNewChild(xmlEdge, NULL, xmlCharStrdup("edgeNode"),NULL);
  sprintf(str, "%d", edge->Nf->num);
  xmlNewProp(tmp, xmlCharStrdup("id"), xmlCharStrdup(str));

  writeXmlLocalpath(graph, edge, xmlEdge);


//   tmp = xmlNewChild(xmlEdge, NULL, xmlCharStrdup("localpath"),NULL);
//   xmlNewProp(tmp, xmlCharStrdup("type"), xmlCharStrdup(p3d_local_getname_planner(edge->planner)));
//   sprintf(str, "%f", edge->longueur);
//   xmlNewProp(tmp, xmlCharStrdup("size"), xmlCharStrdup(str));
//
//   if ((edge->planner == P3D_RSARM_PLANNER) || (edge->planner == P3D_DUBINS_PLANNER)) {
//     plm_reeds_shepp_str rs_paramPt = lm_get_reeds_shepp_lm_param(graph->rob);
//     double radius = 0;
//     if (rs_paramPt == NULL) {
//       radius = -1;
//     } else {
//       radius = rs_paramPt->radius;
//     }
//     sprintf(str, "%f", radius);
//     xmlNewProp(tmp, xmlCharStrdup("radius"), xmlCharStrdup(str));
//   }
}

static void writeXmlNodeEdges(p3d_graph *graph, p3d_node * node, xmlNodePtr parent){
  xmlNodePtr edges = NULL;
  char str[80];
  p3d_list_edge *listEdges;

  edges = xmlNewChild(parent, NULL, xmlCharStrdup("nodeEdges"), NULL);
  sprintf(str, "%d", node->nedge);
  xmlNewProp (edges, xmlCharStrdup("num"), xmlCharStrdup(str));
  listEdges = node->edges;
  for(; listEdges; listEdges = listEdges->next){
    writeXmlEdge(graph, listEdges->E, edges);
  }
}

static void writeXmlNode(p3d_graph *graph, p3d_node * node, xmlNodePtr parent){
  xmlNodePtr cur = xmlNewChild(parent, NULL, xmlCharStrdup("node"), NULL);
  char str[80];

  sprintf(str, "%d", node->num);
  xmlNewProp (cur, xmlCharStrdup("id"), xmlCharStrdup(str));
  sprintf(str, "%d", node->n_fail_extend);
  xmlNewProp (cur, xmlCharStrdup("numFailExtend"), xmlCharStrdup(str));
  sprintf(str, "%f", node->weight);
  xmlNewProp (cur, xmlCharStrdup("weight"), xmlCharStrdup(str));

  writeXmlIkSol(graph, node, cur);
  writeXmlConfig(graph, node, cur);
//   writeXmlNeighbor(graph, node, cur);
  writeXmlNodeEdges(graph, node, cur);
}

static void writeXmlLocalpath(p3d_graph *graph, p3d_edge * edge, xmlNodePtr xmlEdge){
	xmlNodePtr xmlLocalpath = NULL, tmp = NULL;
	char str[80];

	xmlLocalpath = xmlNewChild(xmlEdge, NULL, xmlCharStrdup("localpath"),NULL);
	xmlNewProp(xmlLocalpath, xmlCharStrdup("type"), xmlCharStrdup(p3d_local_getname_planner(edge->planner)));
  sprintf(str, "%f", edge->longueur);
	xmlNewProp(xmlLocalpath, xmlCharStrdup("size"), xmlCharStrdup(str));
#ifdef MULTILOCALPATH
	if (edge->planner == P3D_MULTILOCALPATH_PLANNER) {
		sprintf(str, "%d", graph->rob->mlp->nblpGp);
		xmlNewProp(xmlLocalpath, xmlCharStrdup("nbGroup"), xmlCharStrdup(str));
		for(int i=0; i < graph->rob->mlp->nblpGp; i++){
			tmp = xmlNewChild(xmlLocalpath, NULL, xmlCharStrdup("sub_localpath"),NULL);
			sprintf(str, "%d", p3d_multiLocalPath_get_value_groupToPlan(graph->rob, i));
			xmlNewProp(tmp, xmlCharStrdup("groupActivated"), xmlCharStrdup(str));
			xmlNewProp(tmp, xmlCharStrdup("type"), xmlCharStrdup(p3d_local_getname_planner(graph->rob->mlp->mlpJoints[i]->lplType)));
			if ((graph->rob->mlp->mlpJoints[i]->lplType == P3D_RSARM_PLANNER) || (graph->rob->mlp->mlpJoints[i]->lplType == P3D_DUBINS_PLANNER)) {
				plm_reeds_shepp_str rs_paramPt = lm_get_reeds_shepp_lm_param(graph->rob);
				double radius = 0;
				if (rs_paramPt == NULL) {
					radius = -1;
				} else {
					radius = rs_paramPt->radius;
				}
				sprintf(str, "%f", radius);
				xmlNewProp(tmp, xmlCharStrdup("radius"), xmlCharStrdup(str));
			}
		}
	} else {
#endif
		if ((edge->planner == P3D_RSARM_PLANNER) || (edge->planner == P3D_DUBINS_PLANNER)) {
			plm_reeds_shepp_str rs_paramPt = lm_get_reeds_shepp_lm_param(graph->rob);
			double radius = 0;
			if (rs_paramPt == NULL) {
				radius = -1;
			} else {
				radius = rs_paramPt->radius;
			}
			sprintf(str, "%f", radius);
			xmlNewProp(tmp, xmlCharStrdup("radius"), xmlCharStrdup(str));
		}
#ifdef MULTILOCALPATH
	}
#endif
 return;
}
