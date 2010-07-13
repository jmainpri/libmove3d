/*
 *  compco.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */
#ifndef CPP_COMPCO_HPP
#define CPP_COMPCO_HPP

class Graph;

#ifndef _ROADMAP_H
struct compco;
#endif

class ConnectedComponent
{
public:
	//ConnectedComponent(cpp_Graph* G, p3d_compco* C);
	ConnectedComponent(Graph* G, compco* C);
	
	~ConnectedComponent();
	
	compco* getCompcoStruct() { return m_compco; }
	
private:
	compco*			m_compco;
	Graph*			m_graph;
};
#endif