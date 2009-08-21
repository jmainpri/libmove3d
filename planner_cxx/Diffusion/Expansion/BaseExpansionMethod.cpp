/*
 * BaseExpansionMethod.cpp
 *
 *  Created on: Jun 12, 2009
 *      Author: jmainpri
 */


#include "BaseExpansionMethod.hpp"

using namespace std;

BaseExpansionMethod::BaseExpansionMethod() :
	ExpansionNodeMethod(NEAREST_EXP_NODE_METH),
	MaxExpandNodeFailure(10),
	kNearestPercent(10),
	ExpansionDirectionMethod(GLOBAL_CS_EXP),
	GoalBias(0.1),
	IsGoalBias(false),
	IsDirSampleWithRlg(false)
	{ cout << "no graph in expansion method" << endl; }

BaseExpansionMethod::BaseExpansionMethod(Graph* ptrGraph) :

	ExpansionNodeMethod(NEAREST_EXP_NODE_METH),
	MaxExpandNodeFailure(10),
	kNearestPercent(10),
	ExpansionDirectionMethod(GLOBAL_CS_EXP),
	GoalBias(0.1),
	IsGoalBias(false),
	IsDirSampleWithRlg(false),
	mGraph(ptrGraph) {}

BaseExpansionMethod::~BaseExpansionMethod(){}

double BaseExpansionMethod::step(){
	 return(p3d_get_env_dmax() * ENV.getDouble(Env::extensionStep));
}
