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

class ConnectedComponent
{
public:
	//ConnectedComponent(cpp_Graph* G, p3d_compco* C);
	ConnectedComponent(Graph* G, p3d_compco* C);
	
	~ConnectedComponent();
	
	p3d_compco* getCompcoStruct() { return m_compco; }
	
private:
	p3d_compco* m_compco;
};
#endif