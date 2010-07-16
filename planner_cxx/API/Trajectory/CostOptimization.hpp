/*
 * costOptimization.hpp
 *
 *  Created on: Jun 25, 2009
 *      Author: jmainpri
 */

#ifndef COST_OPTIMIZATION_HPP_
#define COST_OPTIMIZATION_HPP_

#include "planningAPI.hpp"
#include "Smoothing.hpp"

/**
 * @ingroup Trajectory
 * @brief Genera Cost Optimization of a trajectory
 */
namespace API {
	
	class CostOptimization : public Smoothing {
		
	public:
		CostOptimization();
		CostOptimization(const Trajectory& T);
		CostOptimization(Robot* R,p3d_traj* t);
		
		~CostOptimization();
		
		std::vector< std::tr1::shared_ptr<Configuration> > get3RandSuccesConfAlongTraj(
																																									 double& prevDistPt,
																																									 double& randDistPt,
																																									 double& nextDistPt,
																																									 double step);
		
		std::vector< std::tr1::shared_ptr<Configuration> > getClosestConfOnTraj(
																																						double& prevDistPt,
																																						double& randDistPt,
																																						double& nextDistPt,
																																						std::tr1::shared_ptr<Configuration> ptrConf,
																																						double step);
		
		std::tr1::shared_ptr<Configuration> cheat();
		
		bool oneLoopDeform(double step);
		
		bool oneLoopDeformRecompute(double step);
		
		double getMinCost(){return mincost;}
		
		void debugShowTraj(double lPrev,double lNext,std::tr1::shared_ptr<Configuration> qNew , int color);
		
		void printDebugInfo();
		
		bool deformInCollision() {return inCollision;}
		
		void setCheat() { _cheat = true; }
		
		void runDeformation( int nbIteration , int idRun=0 );
		
		
	private:
		
		bool _cheat;
		double mincost;
		uint nbErrors;
		std::vector<double> Errors;
		bool DeformBiased;
		bool inCollision;
		
		//	bool oneLoopShortCut(double step);
		
	};
}

#endif /* COST_OPTIMIZATION_HPP_ */
