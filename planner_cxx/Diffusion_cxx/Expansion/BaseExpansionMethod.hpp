/*
 * BaseExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_BASE_EXPANSION_HPP
#define P3D_BASE_EXPANSION_HPP

#include "../../PlanningAPI/planningAPI.hpp"
#include "../../qtWindow/qtBase/env.hpp"

class BaseExpansionMethod {

public:

	BaseExpansionMethod();
	BaseExpansionMethod(Graph* prtGraph);

	~BaseExpansionMethod();

	int getNodeMethod() { return ExpansionNodeMethod; }

	double step();

	void expansionFailed(Node& node);

	bool expandControl(Localpath& path,
			double positionAlongDirection,
			Node& compNode);

	bool nextStep(Localpath& path,
			std::tr1::shared_ptr<Configuration>& directionConfig,
			double& pathDelta,
			std::tr1::shared_ptr<Localpath>& newPath,
			Env::expansionMethod method);

	bool nextStep(Localpath& path,
			Node* directionNode,
			double& pathDelta,
			std::tr1::shared_ptr<Localpath>& newPath,
			Env::expansionMethod method);

protected:

	int ExpansionNodeMethod;
	int MaxExpandNodeFailure;
	int kNearestPercent;

	int ExpansionDirectionMethod; // = GLOBAL_CS_EXP;
	double GoalBias; //= 0.1;
	bool IsGoalBias; //= FALSE;
	bool IsDirSampleWithRlg; //= FALSE;

	Graph* mGraph;
};

#endif
