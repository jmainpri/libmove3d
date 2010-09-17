/*
 * smoothing.hpp
 *
 *  Created on: Jun 26, 2009
 *      Author: jmainpri
 */

#ifndef BASEOPTIMIZATION_HPP_
#define BASEOPTIMIZATION_HPP_

#undef Trajectory

#include "API/Trajectory/trajectory.hpp"
#include "API/Trajectory/smoothing.hpp"

/**
 * @ingroup Trajectory
 * @brief Basic optimization of a trajectory
 */
namespace API 
{
	class Smoothing : public Trajectory 
	{
	public:
		/**
		 * Class constructors and destructors
		 */
		Smoothing();
		Smoothing(const Trajectory& T);
		Smoothing(Robot* R,traj* t);
		
		~Smoothing();
		
		/**
		 * stops the trajectory optimization
		 */
		bool checkStopConditions( unsigned int iter );
			
		/**
		 * gets randomly two random configurations
		 */
		std::vector< std::tr1::shared_ptr<Configuration> > get2RandomConf( double step,
																																			double& secondDist,
																																			double& firstDist);
		/**
		 * One loop of the random shortcut
		 */
		bool oneLoopShortCut( double step );
		
		/**
		 * One loop of the random shortcut 
		 * with recomputation of the trajectory cost
		 */
		bool oneLoopShortCutRecompute();
		
		/**
		 * Go through all nodes in a deterministic manner
		 */
		void removeRedundantNodes();
		
		/**
		 * Get the time spent in optimization
		 */
		double getTime() { return m_time; }
		
		/**
		 * Show the trajectory while being deformed
		 */
		void debugShowTraj(double lPrev,double lNext);
		
		/**
		 * Set the sorted indexes by cost
		 */
		void setSortedIndex();
		
		/**
		 * Get a parameter on the trajectory
		 * which is biased to the high cost parts of the trajectory
		 */
		double getBiasedParamOnTraj();
		
		/**
		 * Compute the gain of the last n succueded iterations
		 * @param last n taken into account iterations
		 */
		double gainOfLastIterations( unsigned int n );
		
		/**
		 * Save the optimization to a file
		 */
		void saveOptimToFile( std::string str );
		
		/**
		 * Runs the shortcut method for a certain number of iterations
		 * @param iterations
		 */
		void runShortCut(int nbIteration, int idRun = 0);
		
	protected:
		std::vector<double>		m_OptimCost;
		std::vector<double>		m_Selected;
		int										m_nbBiased;
		int										m_nbReallyBiased;
		
		double								m_time;
		
		bool									m_IterationSucceded;
		std::vector<double>		m_GainOfIterations;
		unsigned int					m_MaxNumberOfIterations;
		
	private:
		std::vector<uint>			m_IdSorted;
		bool									m_ShortCutBiased;
	};
}

#endif /* SHORTCUT_HPP_ */
