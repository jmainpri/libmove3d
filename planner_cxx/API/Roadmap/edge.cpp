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

#include "planningAPI.hpp"

#include "Localpath-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

Edge::Edge(Graph* G, unsigned int i, unsigned int j)
{
	m_Graph = G;
	
	m_Start = m_Graph->getNode(i);
  m_End = m_Graph->getNode(j);
	
	m_Long = m_Start->dist(m_End);
	
	Edge* tmpEdge = new Edge(m_Graph,m_Start,m_End,m_Long);
	m_Edge = tmpEdge->m_Edge;
	
	m_is_BGL_Descriptor_Valid = false;
}

//constructor and destructor
Edge::Edge(Graph* G, p3d_edge* E)
{
    m_Edge = E;
    m_Graph = G;
    m_Robot = G->getRobot();
    m_Long = m_Edge->longueur;
    m_Start = m_Graph->getNode(E->Ni);
    m_End = m_Graph->getNode(E->Nf);
	
	m_is_BGL_Descriptor_Valid = false;
	
}

//constructor and destructor
//Edge::Edge(cppm_Graph* G, p3d_edge* E)
//{
//    m_Edge = E;
//    //m_Graph = G;
//    m_Robot = G->getRobot();
//    m_Long = m_Edge->longueur;
//    m_Start =	G->getNode(E->Ni);
//    m_End =		G->getNode(E->Nf);
//}

Edge::Edge(Graph* G, Node* N1, Node* N2, double Long)
{
    int *ikSol = NULL;

    m_Edge = MY_ALLOC(p3d_edge, 1);
    m_Edge->Ni = N1->getNodeStruct();
    m_Edge->Nf = N2->getNodeStruct();

    m_Edge->path = p3d_local_planner_multisol(
            G->getRobot()->getRobotStruct(),
            N1->getConfiguration()->getConfigStruct(),
            N2->getConfiguration()->getConfigStruct(),
            ikSol);

    m_Edge->planner = p3d_local_get_planner();

    //voir pour la longueur
    m_Edge->longueur = Long;
    m_Edge->sens_edge = 1;
    m_Edge->visible = 0;
    m_Edge->unvalid = 0;
    m_Edge->for_cycle = 0;


    m_Graph = G;
    m_Robot = G->getRobot();
		p3d_SetEdgeCost(m_Robot->getRobotStruct(),m_Edge);
    m_Long = Long;
    m_Start = N1;
    m_End = N2;
	
	m_is_BGL_Descriptor_Valid = false;
}

Edge::~Edge()
{
	delete m_Edge;
}

//Accessors
p3d_edge* Edge::getEdgeStruct()
{
    return m_Edge;
}

Graph* Edge::getGraph()
{
    return m_Graph;
}

Robot* Edge::getRobot()
{
    return m_Robot;
}

double Edge::longueur()
{
    return m_Long;
}

Node* Edge::getStart()
{
    return m_Start;
}

Node* Edge::getEnd()
{
    return m_End;
}

double Edge::getEdgeCost()
{
	p3d_SetEdgeCost(m_Robot->getRobotStruct(),m_Edge);
	return p3d_getEdgeCost(m_Edge);
}

shared_ptr<LocalPath> Edge::getLocalPath()
{
	shared_ptr<LocalPath> ptrLP(new LocalPath(m_Robot,m_Edge->path));
	return ptrLP;	
}

BGL_Edge Edge::getDescriptor()
{
	if (m_is_BGL_Descriptor_Valid) 
	{
		return m_BGL_Descriptor;
	}
	else 
	{
		m_is_BGL_Descriptor_Valid = true;
		return m_Graph->findEdgeDescriptor(this);
	}
}
