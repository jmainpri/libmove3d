/*
 *  compco.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "graph.hpp"
#include "compco.h"

#include "Planner-pkg.h"

#include <iterator>

using namespace std;

ConnectedComponent::ConnectedComponent(Graph* G, p3d_compco* C) :
m_Compco(C),
m_Graph(G) 
{
	
}

ConnectedComponent::ConnectedComponent(Graph* G, Node* N) :
m_Graph(G) 
{
	p3d_create_compco(G->getGraphStruct(), N->getNodeStruct());
	addNode(N);
}

ConnectedComponent::~ConnectedComponent()
{
	p3d_remove_compco(m_Graph->getGraphStruct(),m_Compco);
}

void ConnectedComponent::addNode(Node* N)
{
	m_Nodes.push_back(N);
}

void ConnectedComponent::addToReachableList(ConnectedComponent* Comp)
{
	// This function is a copy of
	// p3d_add_compco_to_reachable_list
	m_CanReach.push_back(Comp);
}

void ConnectedComponent::addToReachableListAndUpdatePredecessors(ConnectedComponent* CAdd)
{
	// This function is a copy of
	// p3d_add_compco_to_reachable_list_and_update_predecessors
	
	m_CanReach.push_back(CAdd);
	
	vector<ConnectedComponent*> v = m_Graph->getConnectedComponents();
	vector<ConnectedComponent*>::iterator CompCo;
	
	for (CompCo = v.begin(); CompCo!=v.end(); ++CompCo) 
	{
		if( (*CompCo)->isLinkedToCompco( this ) )
		{
			(*CompCo)->addToReachableList( CAdd );
		}
	}
}

bool ConnectedComponent::isLinkedToCompco(ConnectedComponent* Comp)
{
	// This function is a copy of
	// p3d_compco_linked_to_compco
	
	if( find (m_CanReach.begin(), m_CanReach.end(), Comp ) == m_CanReach.end() )
		return false;
	else
		return true;
}

void ConnectedComponent::mergeWith(ConnectedComponent& Comp)
{
	// This function is a copy of
	 //p3d_merge_comp which updates the 
	// Graph and the connected components
	
	//int nnode1,nnode2;
	p3d_list_node *list_node;
	p3d_list_compco * ListCompcoScan;
	
	//nnode1 = c1->nnode;
	//nnode2 = c2->nnode;
	
	if( use_p3d_structures )
	{
		list_node = Comp.getCompcoStruct()->dist_nodes;
		while (list_node) {
			p3d_add_node_compco(list_node->N, m_Compco, TRUE);
			list_node = list_node->next;
		}
	}
	
	/* The nodes of C2 are now in C1 */
	for(unsigned int i=0;i<Comp.m_Nodes.size();i++)
	{
		Node* tmpNode = Comp.m_Nodes[i];
		tmpNode->setCompco( this ); addNode( tmpNode );
	}
	
	/* All the compcos that can reach C2 can now reach C1 */
	if (m_Graph->getGraphStruct()->oriented) 
	{
		ListCompcoScan = Comp.getCompcoStruct()->canreach;
		while (ListCompcoScan != NULL) 
		{
			p3d_add_compco_to_reachable_list_and_update_predecessors(
																															 m_Graph->getGraphStruct(), 
																															 getCompcoStruct(), 
																															 ListCompcoScan->comp);
			
			ListCompcoScan = ListCompcoScan->next;
		}
	}
	
	/* C2 is deleted from the graph */
	m_Graph->deleteCompco( &Comp );
	
	if (m_Graph->getGraphStruct()->oriented)
	{
		m_Graph->mergeCheck();
	}
}
