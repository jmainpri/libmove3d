//
// C++ Implementation: graph
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

#include "Planner-pkg.h"
#include "Util-pkg.h"

using namespace std;
using namespace tr1;

// Constructors
//----------------------------------------------
cpp_Graph::cpp_Graph(Robot* R, p3d_graph* G) : m_Robot(R)
{
    if (G)
    {
		this->import(G);
    }
}

/**
  * Fonction that imports a Graph
  */
void cpp_Graph::import(p3d_graph* G)
{
	cout << "Importing the graph" << endl;
	
    if (G->nodes)
    {
        p3d_list_node* l = G->nodes;
        while (l)
        {
            m_Nodes.push_back(new Node(this, l->N));
            l = l->next;
        }
    }
    if (G->edges)
    {
        p3d_list_edge* l = G->edges;
        while (l)
        {
            m_Edges.push_back(new Edge(this, l->E));
            l = l->next;
        }
    }
	if (G->comp)
    {
        p3d_compco* l = G->comp;
        while (l)
        {
            m_Comp.push_back(new ConnectedComponent(this, l));
            l = l->suiv;
        }
    }
	
    this->setName();
}

/**
 * Fonction that imports a Graph
 */
p3d_graph* cpp_Graph::exportGraphStruct()
{
    p3d_graph* G = p3d_allocinit_graph();
	
    G->env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
    G->rob = m_Robot->getRobotStruct();
	
    m_Robot->getRobotStruct()->GRAPH = G;
	
	G->nnode = static_cast<int>(m_Nodes.size());
	G->nedge = static_cast<int>(m_Edges.size());
	
    if (STAT)
    {
        G->stat = createStat();
    }
    else
    {
        G->stat = NULL;
    }
	
	cout << "m_Nodes.size() = " << m_Nodes.size() << endl;
	
	p3d_list_node* ln = new p3d_list_node;
	p3d_list_node* prevNode;
	ln->prev = NULL;
	G->nodes = ln;
	for (unsigned int i = 0; i < m_Nodes.size(); i++) 
	{
		ln->N = m_Nodes[i]->getNodeStruct();
		
		if ( i < m_Nodes.size() - 1 ) 
		{
			prevNode = ln;
			prevNode->next = new p3d_list_node;
			ln = prevNode->next;
			ln->prev = prevNode;
		}
		else 
		{
			ln->next = NULL;
		}
		
		//cout << "New Node in list" << i << endl;
	}
	G->last_node = ln;
	
	
	cout << "m_Edges.size() = " << m_Edges.size() << endl;
	
	p3d_list_edge* le = new p3d_list_edge;
	p3d_list_edge* prevEdge;
	le->prev = NULL;
	G->edges = le;
	for (unsigned int i = 0; i < m_Edges.size(); i++) 
	{
		//m_Edges.push_back(new Edge(this, l->E));
		le->E = m_Edges[i]->getEdgeStruct();
		
		if ( i < m_Edges.size() - 1 ) 
		{
			prevEdge = le;
			prevEdge->next = new p3d_list_edge;
			le = prevEdge->next;
			le->prev = prevEdge;
		}
		else 
		{
			le->next = NULL;
		}
		
		//cout << "New Edges in list" << i << endl;
	}
	G->last_edge = le;
	
	
	p3d_compco* lc;
	p3d_compco* prevComp;
	lc->prec = NULL;
	lc = m_Comp[0]->getCompcoStruct();
	G->comp = lc;
	for (unsigned int i = 1; i < m_Comp.size(); i++) 
	{
		prevComp = lc;
		lc = m_Comp[i]->getCompcoStruct();
		prevComp->suiv = lc;
		lc->prec = prevComp;
	}
	lc->suiv = NULL;
	//G->last_edge = lc;
	
	return G;
}

// Destructors
//----------------------------------------------

cpp_Graph::~cpp_Graph()
{
//    p3d_del_graph(_Graph);
    this->freeResources();
}

/**
  * Frees the Nodes and Edges
  */
void cpp_Graph::freeResources()
{

    for(unsigned i=0;i<m_Nodes.size();i++)
    {
        delete m_Nodes[i];
    }

    for(unsigned i=0;i<m_Edges.size();i++)
    {
        delete m_Edges[i];
    }

}
