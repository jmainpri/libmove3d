/*
 *  HRICS_NaturalCell.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef HRICS_NATURALCELL_H_
#define HRICS_NATURALCELL_H_

#include "../../API/planningAPI.hpp"

namespace HRICS
{
	class NaturalCell : public API::ThreeDCell
	{
		
	public:
		NaturalCell();
		NaturalCell(int i, Vector3i pos , Vector3d corner, NaturalGrid* grid);
		
		~NaturalCell() { }
		
		double getCost();
		
		void setBlankCost();
		
		Vector3i getCoord() { return m_Coord; }
		
		bool getOpen() { return m_Open; }
		void setOpen() { m_Open = true; }
		
		bool getClosed() { return m_Closed; }
		void setClosed() { m_Closed = true; }
		
		void resetExplorationStatus();
		
		void createDisplaylist();
		
		void draw();
		
		bool readCellFromXml(xmlNodePtr cur);
		
		int setRobotToStoredConfig();
		
	private:
		
		Vector3i m_Coord;
		
		double* m_v0; double* m_v1; double* m_v2; double* m_v3;
		double* m_v4; double* m_v5; double* m_v6; double* m_v7;
		
		bool m_Open;
		bool m_Closed;
		
		bool m_CostIsComputed;
		double m_Cost;
		
		unsigned int m_NbDirections;
		
		std::tr1::shared_ptr<Configuration> m_QStored;
		
		GLint m_list;
	};
}


#endif
