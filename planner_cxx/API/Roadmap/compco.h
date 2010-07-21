/*
 *  compco.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 CNRS/LAAS. All rights reserved.
 *
 */
#ifndef CPP_COMPCO_HPP
#define CPP_COMPCO_HPP

#include <vector>

class Node;
class Graph;

#ifndef _ROADMAP_H
struct compco;
#endif

class ConnectedComponent
{
public:
	ConnectedComponent(Graph* G, compco* Comp);
	ConnectedComponent(Graph* G, Node* N);
	~ConnectedComponent();
	
	/**
	 * Returns the Connected component
	 * Structure
	 */
	compco* getCompcoStruct() { return m_Compco; }
	
	/**
	 * Add the compco to the reachable Compco
	 */
	void addToReachableList(ConnectedComponent* Comp);
	
	/**
	 * Add the compco to the reachable Compco and update predecessors
	 */
	void addToReachableListAndUpdatePredecessors(ConnectedComponent* Comp);
	
	/**
	 * Returns the ith node of the compco
	 * @param ith the id of the node in the connected component
	 */
//	Node* getNode(unsigned int ith) { return m_Nodes[i]; }
	
	/**
	 * Adds a node to the connected component
	 * @param N the node to add 
	 */
	void addNode(Node* N);
	
	/**
	 * Merge the component with another
	 * @param Component
	 */
	void mergeWith(ConnectedComponent& Comp);
	
	/**
	 * Can reach the compco
	 */
	bool isLinkedToCompco(ConnectedComponent* Comp);
	
private:
	compco*			m_Compco;
	Graph*			m_Graph;
	
	std::vector<Node*> m_Nodes;
	std::vector<ConnectedComponent*> m_CanReach;
};
#endif