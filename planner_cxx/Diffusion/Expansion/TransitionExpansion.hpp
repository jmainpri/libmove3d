/*
 * TranstionExpansion.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TRANSTIONEXPANSION_HPP_
#define TRANSTIONEXPANSION_HPP_

#include "TreeExpansionMethod.hpp"

class TransitionExpansion : public TreeExpansionMethod {

public:
	TransitionExpansion();
	TransitionExpansion(Graph* G);

	~TransitionExpansion();

	/**
	 *
	 */
	bool costTestSucceeded(Node* previousNode, std::tr1::shared_ptr<
			Configuration> currentConfig, double currentCost);

	/**
	 *
	 */
	bool costTestSucceededConf(
			std::tr1::shared_ptr<Configuration>& previousConfig,
			std::tr1::shared_ptr<Configuration>& currentConfig);

	/**
	 *
	 */
	bool expandToGoal(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig);

	/**
	 *
	 */
	bool expandCostConnect(Node& expansionNode, std::tr1::shared_ptr<Configuration> directionConfig,
			Node* directionNode, bool toGoal);

	/**
	 *
	 */
	void adjustTemperature(bool accepted, Node* node);

	/** expandProcess
	 * @param expansionNode
	 * @param directionConfig
	 * @param directionNode
	 * @param method
	 * @return
	 */
	int expandProcess(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig, Node* directionNode,
			Env::expansionMethod method);

};

#endif /* TRANSTIONEXPANSION_HPP_ */
