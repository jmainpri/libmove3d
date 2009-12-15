/*
 * TranstionExpansion.hpp
 *
 *  Created on: Aug 31, 2009
 *      Author: jmainpri
 */

#ifndef TRANSTIONEXPANSION_HPP_
#define TRANSTIONEXPANSION_HPP_

#include "RRTExpansion.h"

class TransitionExpansion : public RRTExpansion {

public:
	TransitionExpansion();
	TransitionExpansion(Graph* G);

	~TransitionExpansion();

        /**
         * Shoots a direction (includes the biasing)
         *
         * @param Expanding component
         * @param Goal Component
         * @param Sampling passive mode
         * @param Direction node
         */
        virtual std::tr1::shared_ptr<Configuration> getExpansionDirection(
            Node* expandComp, Node* goalComp, bool samplePassive,
            Node*& directionNode);


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
	bool transitionTest(Node& fromNode,LocalPath& extensionLocalpath);

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
