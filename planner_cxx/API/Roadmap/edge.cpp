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

#include "Localpath-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

Edge::Edge(Graph* G, unsigned int i, unsigned int j)
{
	_Graph = G;
	
	_Start = _Graph->getNode(i);
    _End = _Graph->getNode(j);
	
	_Long = _Start->dist(_End);
	
	Edge* tmpEdge = new Edge(_Graph,_Start,_End,_Long);
	_Edge = tmpEdge->_Edge;
}

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

//constructor and destructor
Edge::Edge(cpp_Graph* G, p3d_edge* E)
{
    _Edge = E;
    //_Graph = G;
    _Robot = G->getRobot();
    _Long = _Edge->longueur;
//    _Start =	G->getNode(E->Ni);
//    _End =		G->getNode(E->Nf);
}

Edge::Edge(Graph* G, Node* N1, Node* N2, double Long)
{
    int *ikSol = NULL;

    _Edge = MY_ALLOC(p3d_edge, 1);
    _Edge->Ni = N1->getNodeStruct();
    _Edge->Nf = N2->getNodeStruct();

    _Edge->path = p3d_local_planner_multisol(
            G->getRobot()->getRobotStruct(),
            N1->getConfiguration()->getConfigStruct(),
            N2->getConfiguration()->getConfigStruct(),
            ikSol);

    _Edge->planner = p3d_local_get_planner();

    //voir pour la longueur
    _Edge->longueur = Long;
    _Edge->sens_edge = 1;
    _Edge->visible = 0;
    _Edge->unvalid = 0;
    _Edge->for_cycle = 0;


    _Graph = G;
    _Robot = G->getRobot();
	p3d_SetEdgeCost(_Robot->getRobotStruct(),_Edge);
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

double Edge::getEdgeCost()
{
	p3d_SetEdgeCost(_Robot->getRobotStruct(),_Edge);
	return p3d_getEdgeCost(_Edge);
}

shared_ptr<LocalPath> Edge::getLocalPath()
{
	shared_ptr<LocalPath> ptrLP(new LocalPath(_Robot,_Edge->path));
	return ptrLP;	
}

