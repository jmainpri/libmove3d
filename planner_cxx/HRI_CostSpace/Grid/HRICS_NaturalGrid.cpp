/*
 *  HRICS_NaturalGrid.cpp
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

NaturalGrid::NaturalGrid() :
	m_firstDisplay(true)
{
}

NaturalGrid::NaturalGrid(vector<int> size) :
	m_firstDisplay(true)
{
	
}

NaturalGrid::NaturalGrid(double pace, vector<double> envSize) :
	API::ThreeDGrid(pace,envSize),
	m_firstDisplay(true)
{
    createAllCells();
    cout << "Number total of cells = " << _nbCellsX*_nbCellsY*_nbCellsZ << endl;
}

/*!
 * \brief Virtual function that creates a new Cell
 *
 * \param integer index
 * \param integer x
 * \param integer y
 * \param integer z
 */
API::ThreeDCell* NaturalGrid::createNewCell(int index, int x, int y, int z )
{
    Vector3i pos;
	
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
	
    //    cout << "( "<<x<<" , "<<y<<" , "<<z<<" ) "<< endl;
	
    if (index == 0)
    {
        return new NaturalCell( 0, pos ,_originCorner , this );
    }
    return new NaturalCell( index, pos , computeCellCorner(x,y,z) , this );
}

/*!
 * \brief Compute Grid Cost
 */
void NaturalGrid::computeAllCellCost()
{
    int nbCells = this->getNumberOfCells();
    shared_ptr<Configuration> robotConf = getRobot()->getCurrentPos();
	
    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->getCost();
    }
    getRobot()->setAndUpdate(*robotConf);
    API_activeGrid = this;
}


/*!
 * \brief Reset Grid Cost
 */
void NaturalGrid::resetCellCost()
{
    int nbCells = this->getNumberOfCells();
	
    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->setBlankCost();
    }
}

int NaturalGrid::robotConfigInCell(int i)
{
	return dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->setRobotToStoredConfig();
}

Robot* NaturalGrid::getRobot()
{ 
	return this->getNaturalCostSpace()->getRobot(); 
}

void NaturalGrid::draw()
{
	shared_ptr<Configuration> q_Actual = getRobot()->getCurrentPos();
	
	int nbCells = this->getNumberOfCells();
	
	if (m_firstDisplay) 
	{
		for(int i=0; i<nbCells; i++)
		{
			dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->createDisplaylist();
		}
		
		m_firstDisplay = false;
		cout << "First Draw of natural grid" << endl;
	}
	
    for(int i=0; i<nbCells; i++)
    {
        dynamic_cast<NaturalCell*>( BaseGrid::getCell(i) )->draw();
    }
	
	getRobot()->setAndUpdate(*q_Actual);
}
