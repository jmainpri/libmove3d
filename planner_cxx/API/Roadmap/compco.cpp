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

ConnectedComponent::ConnectedComponent(Graph* G, p3d_compco* C) :
m_compco(C),
m_graph(G) 
{
	
}

ConnectedComponent::~ConnectedComponent()
{
	p3d_remove_compco(m_graph->getGraphStruct(),m_compco);
}