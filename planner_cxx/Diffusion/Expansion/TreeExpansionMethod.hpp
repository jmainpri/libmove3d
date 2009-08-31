/*
 * TreeExpansion.hpp
 *
 *  Created on: Jun 11, 2009
 *      Author: jmainpri
 */
#ifndef P3D_TREE_EXPANSION_HPP
#define P3D_TREE_EXPANSION_HPP

#include "BaseExpansionMethod.hpp"

class TreeExpansionMethod: public BaseExpansionMethod
{

public:

	TreeExpansionMethod();
	TreeExpansionMethod(Graph* prtGraph);

	~TreeExpansionMethod();

	std::tr1::shared_ptr<Configuration> getExpansionDirection(Node* fromComp,
			Node* toComp, bool samplePassive, Node*& directionNode);

	std::tr1::shared_ptr<Configuration> selectExpansionDirection(
			Node* expandComp, Node* goalComp, bool samplePassive,
			Node*& directionNode);

	Node* getExpansionNode(Node* compNode,
			std::tr1::shared_ptr<Configuration> direction, int distance);

	Node* selectExpansionNode(Node* compNode, std::tr1::shared_ptr<
			Configuration> direction, int distance);

	Node* addNode(Node* currentNode, LocalPath& path, double pathDelta,
			Node* directionNode, double currentCost, int& nbCreatedNodes);

	/** expandProcess
	 * @param expansionNode
	 * @param directionConfig
	 * @param directionNode
	 * @param method
	 * @return
	 */
	int expandProcess(Node* expansionNode,
			std::tr1::shared_ptr<Configuration> directionConfig,
			Node* directionNode, Env::expansionMethod method);

};

#endif
