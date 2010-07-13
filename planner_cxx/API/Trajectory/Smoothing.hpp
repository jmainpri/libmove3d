/*
 * Smoothing.hpp
 *
 *  Created on: Jun 26, 2009
 *      Author: jmainpri
 */

#ifndef BASEOPTIMIZATION_HPP_
#define BASEOPTIMIZATION_HPP_

#undef Trajectory

#include "planningAPI.hpp"

/**
 * @ingroup Trajectory
 * @brief Basic optimization of a trajectory
 */
namespace API {
	
	class Smoothing : public Trajectory {
		
	public:
		Smoothing();
		Smoothing(const Trajectory& T);
		Smoothing(Robot* R,traj* t);
		
		~Smoothing();
		
		std::vector< std::tr1::shared_ptr<Configuration> > get2RandomConf(
																																			double& secondDist,
																																			double& firstDist);
		
		bool oneLoopShortCut();
		
		bool oneLoopShortCutRecompute();
		
		void removeRedundantNodes();
		
		void debugShowTraj(double lPrev,double lNext);
		
		void setSortedIndex();
		
		double getBiasedParamOnTraj();
		
		//friend bool costMaj(uint i,uint j);
		
		void saveOptimToFile( std::string str );
		
		void runShortCut(int nbIteration, int idRun = 0);
		
	protected:
		std::vector<double> mOptimCost;
		std::vector<double> mSelected;
		int nbBiased;
		int nbReallyBiased;
		
	private:
		std::vector<uint> mIdSorted;
		bool ShortCutBiased;
		
	};
}

#endif /* SHORTCUT_HPP_ */
