/*
 * planner_functions.cpp
 *
 *  Created on: Jul 26, 2009
 *      Author: jmainpri
 */

#include "planners_cxx.hpp"
#include "plannerFunctions.hpp"

bool p3d_run_rrt(p3d_graph* GraphPt,int (*fct_stop)(void), void (*fct_draw)(void))
{
  GraphPt = GraphPt ? GraphPt : p3d_create_graph();

  Graph* _Graph;

#ifdef LIST_OF_PLANNERS
  RRT* rrt = (RRT*)plannerlist[0];
#else
  WorkSpace* ws = new WorkSpace();
  RRT* rrt = new RRT(ws);
#endif

  int nb_added_nodes = rrt->init();

 _Graph = rrt->getActivGraph();

  printf("nb nodes %d\n",_Graph->getNodes().size());

  nb_added_nodes += rrt->expand(GraphPt, fct_stop, fct_draw);

  printf("nb added nodes %d\n", nb_added_nodes);
  printf("nb graph : %d\n", rrt->getActivRobot()->nbGraph());
  printf("nb nodes %d\n",_Graph->getNodes().size());
  bool res = rrt->trajFound();

#ifndef LIST_OF_PLANNERS
  delete rrt;
  delete ws;
#endif

  return res;
}

int p3d_run_vis_prm(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
  int ADDED;

  Graph_Pt = Graph_Pt ? Graph_Pt : p3d_create_graph();

#ifdef LIST_OF_PLANNERS
  Vis_PRM* vprm = (Vis_PRM*)plannerlist[1];
#else
  WorkSpace* ws = new WorkSpace();
  Vis_PRM* vprm = new Vis_PRM(ws);
#endif

  ADDED = vprm->init();

  Graph* _Graph = vprm->getActivGraph();

  ADDED += vprm->expand(Graph_Pt, fct_stop, fct_draw);

  printf("nb added nodes %d\n", ADDED);
  printf("nb graph : %d\n", vprm->getActivRobot()->nbGraph());
  printf("nb nodes %d\n",_Graph->getNodes().size());
  *fail = !vprm->trajFound();

#ifndef LIST_OF_PLANNERS
  delete vprm;
  delete ws;
#endif

  return ADDED;
}


int p3d_run_prm(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
  int ADDED;

  Graph_Pt = Graph_Pt ? Graph_Pt : p3d_create_graph();

#ifdef LIST_OF_PLANNERS
  PRM* prm = (PRM*)plannerlist[2];
#else
  WorkSpace* ws = new WorkSpace();
  PRM* prm = new PRM(ws);
#endif

  ADDED = prm->init();

  Graph* _Graph = prm->getActivGraph();

  ADDED += prm->expand(Graph_Pt, fct_stop, fct_draw);

  printf("nb added nodes %d\n", ADDED);
  printf("nb graph : %d\n", prm->getActivRobot()->nbGraph());
  printf("nb nodes %d\n",_Graph->getNodes().size());
  *fail = !prm->trajFound();

#ifndef LIST_OF_PLANNERS
  delete prm;
  delete ws;
#endif

  return ADDED;
}

int p3d_run_acr(p3d_graph* Graph_Pt, int* fail, int (*fct_stop)(void), void (*fct_draw)(void))
{
  int ADDED;

  Graph_Pt = Graph_Pt ? Graph_Pt : p3d_create_graph();

#ifdef LIST_OF_PLANNERS
  ACR* acr = (ACR*)plannerlist[3];
#else
  WorkSpace* ws = new WorkSpace();
  ACR* acr = new ACR(ws);
#endif

  ADDED = acr->init();

  Graph* _Graph = acr->getActivGraph();

  ADDED += acr->expand(Graph_Pt, fct_stop, fct_draw);

  printf("nb added nodes %d\n", ADDED);
  printf("nb graph : %d\n", acr->getActivRobot()->nbGraph());
  printf("nb nodes %d\n",_Graph->getNodes().size());
  *fail = !acr->trajFound();

#ifndef GLOBAL
  delete acr;
  delete ws;
#endif

  return ADDED;
}



