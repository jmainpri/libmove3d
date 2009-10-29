/*
 * BaseOptimization.hpp
 *
 *  Created on: Jun 26, 2009
 *      Author: jmainpri
 */

#ifndef BASEOPTIMIZATION_HPP_
#define BASEOPTIMIZATION_HPP_

#include "trajectory.hpp"

/**
 * @ingroup PlanningAPI
 * @defgroup Trajectory The trajectory
 */

/**
 * @ingroup Trajectory
 * @brief Basic optimization of a trajectory
 */

class BaseOptimization : public Trajectory {

public:
	BaseOptimization();
	BaseOptimization(const Trajectory& T);
	BaseOptimization(Robot* R,p3d_traj* t);

	~BaseOptimization();

	std::vector< std::tr1::shared_ptr<Configuration> > get2RandomConf(
			double& secondDist,
			double& firstDist);

	bool oneLoopShortCut();

	bool oneLoopShortCutRecompute();

	void removeRedundantNodes();

	void debugShowTraj(double lPrev,double lNext);

	void setSortedIndex();

	double getBiasedParamOnTraj();

	friend bool costMaj(uint i,uint j);

protected:
	std::vector<double> mSelected;
	int nbBiased;
	int nbReallyBiased;

private:
	std::vector<uint> mIdSorted;
	bool ShortCutBiased;

};

#endif /* SHORTCUT_HPP_ */
