/*
 *  HRICS_NaturalCell.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "HRICS_NaturalGrid.h"
#include "HRICS_NaturalCell.h"
#include "../HRICS_Natural.h"

using namespace std;
using namespace tr1;
using namespace HRICS;

NaturalCell::NaturalCell() :
	m_Open(false),
	m_Closed(false),
	m_CostIsComputed(false),
	m_NbDirections(1.0),
	m_Cost(-1)
{
	
}

NaturalCell::NaturalCell(int i, Vector3i coord , Vector3d corner, NaturalGrid* grid) :
	API::ThreeDCell(i,corner,grid),
	m_Open(false),
	m_Closed(false),
	m_CostIsComputed(false),
	m_NbDirections(1.0),
	m_Cost(-1)
{
    m_Coord = coord;
    m_v0 = new double[3]; m_v1 = new double[3]; m_v2 = new double[3]; m_v3 = new double[3];
    m_v4 = new double[3]; m_v5 = new double[3]; m_v6 = new double[3]; m_v7 = new double[3];
	
    m_v0[0] = _corner[0] + _grid->getCellSize()[0];
    m_v0[1] = _corner[1] + _grid->getCellSize()[1];
    m_v0[2] = _corner[2] + _grid->getCellSize()[2];
	
    m_v1[0] = _corner[0] ;
    m_v1[1] = _corner[1] + _grid->getCellSize()[1];
    m_v1[2] = _corner[2] + _grid->getCellSize()[2];
	
    m_v2[0] = _corner[0] ;
    m_v2[1] = _corner[1] ;
    m_v2[2] = _corner[2] + _grid->getCellSize()[2];
	
    m_v3[0] = _corner[0] + _grid->getCellSize()[0];
    m_v3[1] = _corner[1] ;
    m_v3[2] = _corner[2] + _grid->getCellSize()[2];
	
    m_v4[0] = _corner[0] + _grid->getCellSize()[0];
    m_v4[1] = _corner[1] ;
    m_v4[2] = _corner[2] ;
	
    m_v5[0] = _corner[0] + _grid->getCellSize()[0];
    m_v5[1] = _corner[1] + _grid->getCellSize()[1];
    m_v5[2] = _corner[2] ;
	
    m_v6[0] = _corner[0] ;
    m_v6[1] = _corner[1] + _grid->getCellSize()[1];
    m_v6[2] = _corner[2] ;
	
    m_v7[0] = _corner[0] ;
    m_v7[1] = _corner[1] ;
    m_v7[2] = _corner[2] ;
}

void NaturalCell::createDisplaylist()
{
	Vector3d center = getCenter();
	
	m_list=glGenLists(1);
	glNewList(m_list, GL_COMPILE);
	g3d_draw_solid_sphere(center[0], center[1], center[2], ENV.getDouble(Env::CellSize)/2, 20);
	//g3d_drawSphere(center[0], center[1], center[2], ENV.getDouble(Env::CellSize)/2 );
	glEndList();
}
void NaturalCell::draw()
{
    double colorvector[4];
	
    colorvector[0] = 0.0;       //red
    colorvector[1] = 0.0;       //green
    colorvector[2] = 0.0;       //blue
    colorvector[3] = 0.01;       //transparency
	
	double Cost = this->getCost();
	
	//cout << "draw Cell, Cost = " << Cost << endl;
	
	if (Cost != 0.0) 
	{
		Cost *= 6;
		GroundColorMix(colorvector,Cost/*ENV.getDouble(Env::colorThreshold2)*/,0,1);
		g3d_set_color(Any,colorvector);
		glCallList(m_list);
		//cout << "Robot : " << dynamic_cast<NaturalGrid*>(_grid)->getRobot()->getName() << "Draw Sphere, Cost = " << Cost << endl;
	}
	else 
	{
//		g3d_set_color(Any,colorvector);
//		glCallList(m_list);
	}


}

int NaturalCell::setRobotToStoredConfig()
{
	NaturalGrid* grid = dynamic_cast<NaturalGrid*>(_grid);
	
	if(this->getCost() != 0.0)
	{
		//m_QStored->print();
		return grid->getRobot()->setAndUpdateMultiSol(*m_QStored);
	}
	
	return -1;
}

double NaturalCell::getCost()
{
	if (!m_CostIsComputed) 
	{
		
		Vector3d center = getCenter();
		NaturalGrid* grid = dynamic_cast<NaturalGrid*>(_grid);
		
		int ObjectDofIndex = grid->getNaturalCostSpace()->getObjectDof();
		
		shared_ptr<Configuration> q;
		
		m_Cost = 0.0;
		m_NbDirections = 360;
		
		for (unsigned int i=0; i<m_NbDirections; i++) 
		{
			q = grid->getNaturalCostSpace()->getRobot()->shoot();
			
			q->getConfigStruct()[ObjectDofIndex+0] = center[0];
			q->getConfigStruct()[ObjectDofIndex+1] = center[1];
			q->getConfigStruct()[ObjectDofIndex+2] = center[2];
			
//			q->getConfigStruct()[32] = 0.000000;
//			q->getConfigStruct()[33] = 0.000000;
//			q->getConfigStruct()[34] = -0.785398;
			
			if( q->setConstraintsWithSideEffect() && !q->IsInCollision() )
			{
				//m_Cost += grid->getNaturalCostSpace()->getCost();
				//cout << "Center :" << endl << center << endl;
				//cout << rob->getName() << endl;
				//m_QStored = q;
				//m_CostIsComputed = true;
				m_Cost += 1.0;
				//m_QStored->print(true);
				//return 1.0;
			}
		}
		
		if(m_Cost != 0.0)
		{
			m_Cost *= 1;
			//cout << "Cost = " << m_Cost << endl;
		}
		m_CostIsComputed = true;
	}
	
	return m_Cost;
}

void NaturalCell::setBlankCost()
{ 
	 m_CostIsComputed = false; 
	 //this->resetExplorationStatus(); 
}