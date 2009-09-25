/*
 * TreeExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_TREE_EXPANSION_HPP
#define P3D_TREE_EXPANSION_HPP

#include "BaseExpansion.hpp"

class TreeExpansionMethod: public BaseExpansionMethod
{

public:

	TreeExpansionMethod();
	TreeExpansionMethod(Graph* prtGraph);

	~TreeExpansionMethod();

	/**
	 *
	 */
	std::tr1::shared_ptr<Configuration> getExpansionDirection(Node* fromComp,
			Node* toComp, bool samplePassive, Node*& directionNode);

	/**
	 *
	 */
	std::tr1::shared_ptr<Configuration> selectExpansionDirection(
			Node* expandComp, Node* goalComp, bool samplePassive,
			Node*& directionNode);

	/**
	 *
	 */
	Node* getExpansionNode(Node* compNode,
			std::tr1::shared_ptr<Configuration> direction, int distance);

	/**
	 *
	 */
	Node* selectExpansionNode(Node* compNode, std::tr1::shared_ptr<
			Configuration> direction, int distance);

	/**
	 * Expands towards the goal
	 */
	virtual bool expandToGoal(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig);

	/**
	 * EST Special case
	 */
	Node* getExpansionNode(std::vector<Node*>& nodes);

	/**
	 * EST Special case
	 */
	std::tr1::shared_ptr<Configuration> getExpansionDirection(Node* expansionNode,Node* toComp);

	/**
	 * EST Special case
	 */
	Node* expandProcessEST( Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig,
			int& nbOfNodesAdded);

	/**
	 * EST
	 */
	void printAllNodes(std::vector<Node*>& nodes);

	/**
	 * expandProcess
	 * @param expansionNode
	 * @param directionConfig
	 * @param directionNode
	 * @param method
	 * @return
	 */
	virtual int expandProcess(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig,
			Node* directionNode, Env::expansionMethod method);

};

#endif
