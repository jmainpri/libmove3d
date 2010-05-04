/*
 *  HRICS_Natural.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef HRICS_NATURAL_HPP
#define HRICS_NATURAL_HPP

#include "../API/planningAPI.hpp"
#include "Grid/HRICS_NaturalGrid.h"

/**
 @ingroup HRICS
 */
namespace HRICS
{
	/**
	 * Natural Motion and Arm Confort
	 */
	class Natural 
	{
	public:
		Natural(Robot* R);
		~Natural();
		
		double getCost();
		double getCustomDistConfig(std::tr1::shared_ptr<Configuration> q);
		Vector3d sampleSphere();
		void makeNaturalGrid();
		void initNaturalJustin();
		int getObjectDof() { return m_IndexObjectDof; }
		NaturalGrid* getGrid() { return m_Grid; }
		Robot* getRobot() { return m_Robot; }
		
	private:
		bool			m_debug;
		
		int				m_IndexObjectDof;
		
		Robot*			m_Robot;
		NaturalGrid*	m_Grid;
		
		enum Kinematic 
		{
			Default,
			Justin,
			Achile
		};
		
		Kinematic m_KinType;
		
		std::tr1::shared_ptr<Configuration> m_q_Confort;
		std::tr1::shared_ptr<Configuration> m_q_Init;
		std::tr1::shared_ptr<Configuration> m_q_Goal;
	};
}

#endif