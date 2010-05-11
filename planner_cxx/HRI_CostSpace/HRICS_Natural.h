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

#ifdef HRI_PLANNER
#include "Hri_planner-pkg.h"
#endif

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
		
		/**
		 * Initilize the parameters for the 
		 * Natural Cost space
		 */
		void initNaturalJustin();
		void initNaturalAchile();
		
		/**
		 * Get the cost of the current configuration
		 */
		
		double getCost();
		double getCost(const Vector3d& WSPoint);
		
		/**
		 * Simple number of IK Cost
		 */
		double getNumberOfIKCost(const Vector3d& WSPoint);
		
		/**
		 * Get the 3 component of natural
		 * cost space
		 */
		double getJointDisplacement();
		double getEnergy();
		double getDiscomfort();
		
		void computeNaturalGrid();
	
		std::vector<double> getHeigthFromConfort();
		double getCustomDistConfig(std::tr1::shared_ptr<Configuration> q);
		Vector3d sampleSphere();
		void makeNaturalGrid();
		
		/**
		 * Basic accesors
		 */
		int getObjectDof() { return m_IndexObjectDof; }
		NaturalGrid* getGrid() { return m_Grid; }
		Robot* getRobot() { return m_Robot; }
		
		
	private:
		bool			m_debug;
		
		int				m_IndexObjectDof;
		
		bool			m_computeNbOfIK;
		
		Robot*			m_Robot;
		NaturalGrid*	m_Grid;
		
		enum Kinematic 
		{
			Default,
			Justin,
			Achile
		};
		
#ifdef HRI_PLANNER
		HRI_AGENTS* m_Agents;
#endif
		
		Kinematic m_KinType;
		
		/**
		 * @brief The Confort configuration
		 */
		std::tr1::shared_ptr<Configuration> m_q_Confort;
		
		/**
		 * @brief Weights associated to confort
		 */
		std::tr1::shared_ptr<Configuration> m_q_ConfortWeigths;
		
		/**
		 * Mass times gravity constant = potential energy
		 * @bref Mass (Weigts) associated with each arm
		 */
		std::vector<double> m_mg;
		
		std::tr1::shared_ptr<Configuration> m_q_Init;
		std::tr1::shared_ptr<Configuration> m_q_Goal;
	};
}

#endif