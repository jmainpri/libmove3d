/*
 * testModel.hpp
 *
 *  Created on: Jul 8, 2009
 *      Author: jmainpri
 */

#ifndef TESTMODEL_HPP_
#define TESTMODEL_HPP_

#include "../planning_api/planningAPI.hpp"

class TestModel{

public:

	TestModel();

	int nbOfColisionsPerSeconds();

	int nbOfCostPerSeconds();

	int nbOfLocalPathsPerSeconds();

	void distEnv();

	void runAllTests();

private:
	Robot* modelRobot;

	int nbColisionTest;
	int nbLocalPathTest;

};




#endif /* TESTMODEL_HPP_ */
