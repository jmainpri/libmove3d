/*
 *  HRICS_NaturalGrid.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef HRICS_NATURALGRID_H_
#define HRICS_NATURALGRID_H_

#include "../../API/planningAPI.hpp"

namespace HRICS
{
	class Natural;
    class NaturalGridCell;
	
    class NaturalGrid : public API::ThreeDGrid
    {
    public:
        NaturalGrid();
        NaturalGrid( std::vector<int> size );
        NaturalGrid(double pace, std::vector<double> envSize);
		
        API::ThreeDCell* createNewCell(int index, int x, int y, int z );
        void computeAllCellCost();
		int robotConfigInCell(int i);
        void draw();
        void resetCellCost();

		void setNaturalCostSpace(Natural* NCS) { m_NaturalCostSpace = NCS; }
		Natural* getNaturalCostSpace() { return m_NaturalCostSpace; }
		
		Robot* getRobot();
		
    private:
		Natural*	m_NaturalCostSpace;
		bool		m_firstDisplay;
    };
}

#endif