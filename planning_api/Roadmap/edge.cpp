//
// C++ Implementation: edge
//
// Description:
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include "../planningAPI.hpp"

//constructor and destructor
Edge::Edge(Graph* G, p3d_edge* E)
{
  _Edge = E;
  _Graph = G;
  _Robot = G->getRobot();
  _Long = _Edge->longueur;
  _Start = _Graph->getNode(E->Ni);
  _End = _Graph->getNode(E->Nf);
}

Edge::Edge(Graph* G, Node* N1, Node* N2, double Long)
{
  int *ikSol = NULL;

  _Edge = MY_ALLOC(p3d_edge, 1);
  _Edge->Ni = N1->getNodeStruct();
  _Edge->Nf = N2->getNodeStruct();
  _Edge->path = p3d_local_planner_multisol(G->getRobot()->getRobotStruct(), N1->getConfiguration()->getConfigurationStruct(), N2->getConfiguration()->getConfigurationStruct(), ikSol);

  _Edge->planner = p3d_local_get_planner();

//voir pour la longueur
  _Edge->longueur = Long;

  p3d_SetEdgeCost(_Edge);
  _Edge->sens_edge = 1;
  _Edge->visible = 0;
  _Edge->unvalid = 0;
  _Edge->for_cycle = 0;


  _Graph = G;
  _Robot = G->getRobot();
  _Long = Long;
  _Start = N1;
  _End = N2;
}

Edge::~Edge()
{
  _Start->~Node();
  _End->~Node();
}

//Accessors
p3d_edge* Edge::getEdgeStruct()
{
  return _Edge;
}

Graph* Edge::getGraph()
{
  return _Graph;
}

Robot* Edge::getRobot()
{
  return _Robot;
}

double Edge::longueur()
{
  return _Long;
}

Node* Edge::getStart()
{
  return _Start;
}

Node* Edge::getEnd()
{
  return _End;
}


