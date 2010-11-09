/*
 *  HRICS_Natural.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 27/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef HRICS_NATURAL_HPP
#define HRICS_NATURAL_HPP

#include "planningAPI.hpp"
#include "hri_costspace/Grid/HRICS_NaturalGrid.h"
#include "hri_costspace/Grid/HRICS_NaturalCell.h"

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
		Natural();
		Natural(Robot* R);
		~Natural();
		
		/**
		 * Initilize the parameters for the 
		 * Natural Cost space
		 */
		void initGeneral();
		void initNaturalJustin();
		void initNaturalAchile();
		
		void printBodyPos();
		
		/**
		 * Get the cost of the current configuration
		 */
		
		double getConfigCost();
		double getCost(const Eigen::Vector3d& WSPoint, bool useLeftvsRightArm, bool withEffect = false);
		double getCostInGrid(const Eigen::Vector3d& WSPoint);
		
		/**
		 * Get the 3 component of natural
		 * cost space
		 */
		double getJointDisplacement();
		double getEnergy();
		double getDiscomfort();
		
		/**
		 * Others
		 */
		double basicNaturalArmCost(bool useLeftvsRightArm);
		/*
		double akinRightArmReachCost();
		double akinLeftArmReachCost();
		 */
		std::vector<double> getUpperBodyHeigth();
		double getCustomDistConfig(Configuration& q);
		double getJointLimits(Configuration& q);
		
		/**
		 * Simple number of IK Cost
		 */
		double getNumberOfIKCost(const Eigen::Vector3d& WSPoint);
		
#ifdef HRI_PLANNER
		/**
		 * Compute if the Workspace Point is Reachable
		 * Leaves the Robot in the Configuration
		 */
		bool computeIsReachable( const Eigen::Vector3d& WSPoint, bool leftArm);
#endif
		
		/**
		 * Computation on the Grid
		 */ 
		Eigen::Transform3d getGridOriginMatrix();
		NaturalGrid* computeNaturalGrid();
		void computeAllCellCost();
		void computeAllReachableCellCost();
		std::vector< Eigen::Vector3d > getSortedReachableWSPoint();
		std::vector< std::pair<double,Eigen::Vector3d> > getReachableWSPoint();
	
		
		/**
		 * Basic accesors
		 */
		int getObjectDof() { return m_IndexObjectDof; }
		bool IsHuman() { return m_IsHuman; }
		NaturalGrid* getGrid() { return m_Grid; }
		Robot* getRobot() { return m_Robot; }
		
		
		/**
		 * Basic setters
		 */
		void setGrid(NaturalGrid* grid) { m_Grid = grid; }
		
		
	private:
		bool			m_debug;
		
		int				m_IndexObjectDof;
		
		bool			m_computeNbOfIK;
		
		bool			m_leftArmCost;
		
		bool			m_BestPointsSorted;
		
		Robot*			m_Robot;
		NaturalGrid*	m_Grid;
		
		enum Kinematic 
		{
			Default,
			Justin,
			Achile
		};
		
		bool			m_IsHuman;
		
#ifdef HRI_PLANNER
		HRI_AGENTS*		m_Agents;
#endif
		
		Kinematic		m_KinType;
		
		/***********************************************/
		int m_JOINT_SPINE;
		int m_JOINT_HEAD;
		
		int m_JOINT_ARM_RIGTH_SHOULDER;
		int m_JOINT_ARM_RIGTH_ELBOW;
		int m_JOINT_ARM_RIGTH_WRIST;
		
		int m_JOINT_ARM_LEFT_SHOULDER;
		int m_JOINT_ARM_LEFT_ELBOW;
		int m_JOINT_ARM_LEFT_WRIST; 
		
		/***********************************************/
		int m_CONFIG_INDEX_SPINE;
		int m_CONFIG_INDEX_HEAD;
		
		int m_CONFIG_INDEX_ARM_RIGTH_SHOULDER;
		int m_CONFIG_INDEX_ARM_RIGTH_ELBOW;
		int m_CONFIG_INDEX_ARM_RIGTH_WRIST;
		
		int m_CONFIG_INDEX_ARM_LEFT_SHOULDER;
		int m_CONFIG_INDEX_ARM_LEFT_ELBOW;
		int m_CONFIG_INDEX_ARM_LEFT_WRIST;
		
		
		/**
		 * @brief The Confort configuration
		 */
		std::tr1::shared_ptr<Configuration> m_q_Confort;
		
		/**
		 * @brief Weights associated to confort
		 */
		std::tr1::shared_ptr<Configuration> m_q_ConfortWeigths;
		
		/**
		 * Weigth associated to the joints limits function
		 */
		double								m_G;
		
		/**
		 * Mass times gravity constant = potential energy
		 * @bref Mass (Weigts) associated with each arm
		 */
		std::vector<double>					m_mg;
		
		std::tr1::shared_ptr<Configuration> m_q_Init;
		std::tr1::shared_ptr<Configuration> m_q_Goal;
		
		/**
		 * Sorted Cells
		 */
		std::vector< NaturalCell* >			m_SortedCells;
	};
}

#endif