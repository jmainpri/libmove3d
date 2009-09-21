/*
 * BaseExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_BASE_EXPANSION_HPP
#define P3D_BASE_EXPANSION_HPP

#include "../../planning_api/planningAPI.hpp"

class BaseExpansionMethod {

public:

	/**
	 * Constructor
	 */
	BaseExpansionMethod();

	BaseExpansionMethod(Graph* prtGraph);

	/**
	 * Destructor
	 */
	~BaseExpansionMethod();

	int getNodeMethod() { return ExpansionNodeMethod; }

	/**
	 * Expansion Step (Delta)
	 */
	double step();

	/**
	 * Function called when a node can not be connected
	 * @param the node which has not been connected
	 */
	void expansionFailed(Node& node);

	/**
	 * Adds a node to a connected component
	 */
	Node* addNode(Node* currentNode, LocalPath& path, double pathDelta,
			Node* directionNode, int& nbCreatedNodes);

	/**
	 * Function that balances the ratio of
	 * Exploration towards refinement
	 */
	bool expandControl(LocalPath& path,
			double positionAlongDirection,
			Node& compNode);

	bool nextStep(LocalPath& path,
			std::tr1::shared_ptr<Configuration>& directionConfig,
			double& pathDelta,
			std::tr1::shared_ptr<LocalPath>& newPath,
			Env::expansionMethod method);

	bool nextStep(LocalPath& path,
			Node* directionNode,
			double& pathDelta,
			std::tr1::shared_ptr<LocalPath>& newPath,
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
