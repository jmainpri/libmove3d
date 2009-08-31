/*
 * costOptimization.hpp
 *
 *  Created on: Jun 25, 2009
 *      Author: jmainpri
 */

#ifndef COST_OPTIMIZATION_HPP_
#define COST_OPTIMIZATION_HPP_

#include "BaseOptimization.hpp"

/**
 * @ingroup Trajectory
 * @brief Genera Cost Optimization of a trajectory
 */

class CostOptimization : public BaseOptimization {

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

	bool oneLoopDeform(double step);

	void removeRedundantNodes();

	double getMinCost(){return mincost;}

	void debugShowTraj(double lPrev,double lNext,std::tr1::shared_ptr<Configuration> qNew , int color);

	void printDebugInfo();


private:

	double mincost;
	uint nbErrors;
	std::vector<double> Errors;
	bool DeformBiased;

//	bool oneLoopShortCut(double step);

};

#endif /* COST_OPTIMIZATION_HPP_ */
